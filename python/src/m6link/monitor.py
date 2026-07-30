"""Read-only live telemetry aggregation and terminal rendering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping, TextIO

from .errors import M6Error
from .faults import decode_safety_faults
from .models import Identity, TelemetrySample
from .registry import REGISTRY, TelemetryStream
from .session import Session


MONITOR_SUBSCRIPTIONS: Mapping[TelemetryStream, int] = {
    # The dashboard is an observer; the firmware wheel loop remains 5 ms.
    # Matching CORE to WHEEL at 25 Hz avoids P2 slot replacement while the
    # page also polls action/parameter RPC state.
    TelemetryStream.CORE: 40,
    TelemetryStream.WHEEL: 40,
    # Keep headroom below the 7000 B/s admission ceiling for escaped wire
    # bytes.  The old 40 ms profile estimated 6689 B/s before escaping and
    # produced real P2 gaps on COM10 even while the vehicle was idle. Wheel
    # tuning does not consume LINE dynamics; T6 uses a dedicated line profile.
    TelemetryStream.LINE: 100,
    TelemetryStream.HEALTH: 1000,
}
_RECONNECT_SILENCE_S = 2.0
_RECONNECT_RETRY_S = 1.0

_STREAM_KEYS = {
    TelemetryStream.CORE: "core",
    TelemetryStream.WHEEL: "wheel",
    TelemetryStream.LINE: "line",
    TelemetryStream.HEALTH: "health",
}

_TREND_SOURCES = {
    "speed_l": (TelemetryStream.WHEEL, "filtered_l_mm_s", 1.0),
    "speed_r": (TelemetryStream.WHEEL, "filtered_r_mm_s", 1.0),
    "line": (TelemetryStream.LINE, "position_x1000", 0.001),
    "yaw": (TelemetryStream.CORE, "yaw_cdeg", 0.01),
}

def _registry_numeric(item: Mapping[str, Any]) -> int:
    if "value_hex" in item:
        return int(str(item["value_hex"]), 16)
    if "value" in item:
        return int(item["value"])
    return 1 << int(item["bit"])


_REGISTRY_NAMES = {
    name: {_registry_numeric(item): str(item["name"]) for item in entries}
    for name, entries in REGISTRY["registries"].items()
}


@dataclass(frozen=True, slots=True)
class MonitorStreamSnapshot:
    stream: TelemetryStream
    sample_id: int
    time_ms: int
    age_ms: int
    stale: bool
    gaps: int
    values: Mapping[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "sample_id": self.sample_id,
            "time_ms": self.time_ms,
            "age_ms": self.age_ms,
            "stale": self.stale,
            "gaps": self.gaps,
            "values": dict(self.values),
        }


@dataclass(frozen=True, slots=True)
class MonitorSnapshot:
    captured_monotonic: float
    samples_received: int
    gaps_total: int
    streams: Mapping[TelemetryStream, MonitorStreamSnapshot]

    def as_dict(self, identity: Identity | None = None) -> dict[str, Any]:
        result: dict[str, Any] = {
            "host_monotonic_s": round(self.captured_monotonic, 6),
            "samples_received": self.samples_received,
            "gaps_total": self.gaps_total,
            "streams": {
                _STREAM_KEYS[stream]: sample.as_dict()
                for stream, sample in self.streams.items()
            },
        }
        if identity is not None:
            result["device"] = {
                "port": identity.port,
                "firmware": identity.firmware_id,
                "wire_version": identity.wire_version,
                "registry_revision": identity.registry_revision,
                "boot_id": identity.boot_id,
                "parameter_revision": identity.active_revision,
                "generation": identity.generation,
            }
        return result


class TelemetryAggregator:
    """Keep the newest sample from each subscribed stream."""

    def __init__(
        self,
        periods_ms: Mapping[TelemetryStream, int] = MONITOR_SUBSCRIPTIONS,
        *,
        clock: Callable[[], float] = time.monotonic,
        stale_periods: int = 5,
    ) -> None:
        if stale_periods < 1:
            raise ValueError("stale_periods must be positive")
        self.periods_ms = {
            TelemetryStream(int(stream)): int(period)
            for stream, period in periods_ms.items()
        }
        if any(period <= 0 for period in self.periods_ms.values()):
            raise ValueError("telemetry periods must be positive")
        self.clock = clock
        self.stale_periods = int(stale_periods)
        self._latest: dict[TelemetryStream, tuple[TelemetrySample, float]] = {}
        self._gaps: dict[TelemetryStream, int] = {
            stream: 0 for stream in self.periods_ms
        }
        self.samples_received = 0
        self.gaps_total = 0

    def ingest(self, sample: TelemetrySample, *, received_at: float | None = None) -> None:
        stream = TelemetryStream(int(sample.stream))
        if stream not in self.periods_ms:
            return
        received_at = self.clock() if received_at is None else float(received_at)
        self._latest[stream] = (sample, received_at)
        self.samples_received += 1
        if sample.gap_before:
            self._gaps[stream] += 1
            self.gaps_total += 1

    def snapshot(self, *, now: float | None = None) -> MonitorSnapshot:
        now = self.clock() if now is None else float(now)
        streams: dict[TelemetryStream, MonitorStreamSnapshot] = {}
        for stream, (sample, received_at) in self._latest.items():
            age_ms = max(0, round((now - received_at) * 1000.0))
            stale = age_ms > self.periods_ms[stream] * self.stale_periods
            streams[stream] = MonitorStreamSnapshot(
                stream=stream,
                sample_id=sample.sample_id,
                time_ms=sample.time_ms,
                age_ms=age_ms,
                stale=stale,
                gaps=self._gaps[stream],
                values=sample.values,
            )
        return MonitorSnapshot(now, self.samples_received, self.gaps_total, streams)


class TrendHistory:
    """Sample selected live values at the terminal refresh cadence."""

    def __init__(self, max_points: int) -> None:
        if max_points < 2:
            raise ValueError("trend history requires at least two points")
        self.max_points = int(max_points)
        self.values: dict[str, deque[float | None]] = {
            name: deque(maxlen=self.max_points) for name in _TREND_SOURCES
        }

    def ingest(self, snapshot: MonitorSnapshot) -> None:
        for name, (stream, field, scale) in _TREND_SOURCES.items():
            current = snapshot.streams.get(stream)
            value: float | None = None
            if current is not None and not current.stale and field in current.values:
                try:
                    value = float(current.values[field]) * scale
                except (TypeError, ValueError):
                    value = None
            self.values[name].append(value)

    def series(self, name: str) -> tuple[float | None, ...]:
        return tuple(self.values[name])


def _registry_name(registry: str, value: Any) -> str:
    try:
        numeric = int(value)
    except (TypeError, ValueError):
        return str(value)
    return _REGISTRY_NAMES.get(registry, {}).get(numeric, str(numeric))


def _stream(snapshot: MonitorSnapshot, stream: TelemetryStream) -> MonitorStreamSnapshot | None:
    return snapshot.streams.get(stream)


def _value(sample: MonitorStreamSnapshot | None, key: str, default: Any = "--") -> Any:
    return sample.values.get(key, default) if sample is not None else default


def _pair(left: Any, right: Any, width: int = 9) -> str:
    return f"{str(left):>{width}} {str(right):>{width}}"


def _scaled(value: Any, divisor: float, places: int) -> str:
    try:
        return f"{float(value) / divisor:.{places}f}"
    except (TypeError, ValueError):
        return "--"


def _stream_status(sample: MonitorStreamSnapshot | None) -> str:
    if sample is None:
        return "WAIT"
    return f"{'STALE' if sample.stale else 'OK'} {sample.age_ms}ms gap={sample.gaps}"


def _channel_bits(mask: int) -> str:
    """Render channels in the same left-to-right order as ADC[0]..ADC[7]."""

    return "".join("1" if (mask & (1 << channel)) else "0" for channel in range(8))


def render_sparkline(values: tuple[float | None, ...], width: int = 48) -> str:
    """Render a compact trend with the newest value at the right edge."""

    if width < 2:
        raise ValueError("sparkline width must be at least two")
    recent = values[-width:]
    if not recent:
        return "waiting"
    valid = [value for value in recent if value is not None]
    if not valid:
        return " " * len(recent)
    low, high = min(valid), max(valid)
    levels = "▁▂▃▄▅▆▇█"
    if high == low:
        return "".join("▄" if value is not None else " " for value in recent)
    return "".join(
        " " if value is None else levels[min(7, int((value - low) * 7.0 / (high - low) + 0.5))]
        for value in recent
    )


def _trend_line(label: str, values: tuple[float | None, ...], unit: str) -> str:
    valid = [value for value in values if value is not None]
    bounds = "waiting" if not valid else f"{min(valid):7.2f}..{max(valid):7.2f} {unit}"
    return f"  {label:<8} {render_sparkline(values)}  {bounds}"


def render_terminal(
    snapshot: MonitorSnapshot,
    identity: Identity,
    trends: TrendHistory | None = None,
) -> str:
    """Render one fixed-width, dependency-free terminal dashboard."""

    core = _stream(snapshot, TelemetryStream.CORE)
    wheel = _stream(snapshot, TelemetryStream.WHEEL)
    line = _stream(snapshot, TelemetryStream.LINE)
    health = _stream(snapshot, TelemetryStream.HEALTH)

    adc = _value(line, "adc", ())
    adc_text = " ".join(f"{int(value):4d}" for value in adc) if adc else "waiting for line telemetry"
    mask = _value(line, "active_mask", None)
    mask_value = int(mask) if mask is not None else None
    mask_text = _channel_bits(mask_value) if mask_value is not None else "--------"
    mask_hex = f"0x{mask_value:02X}" if mask_value is not None else "--"
    fault_bits = int(_value(health, "fault_bits", _value(core, "fault_bits", 0)))
    task_miss = _value(health, "task_miss", ())
    missed_total = sum(int(value) for value in task_miss) if task_miss else 0
    fault_text = "、".join(item["title"] for item in decode_safety_faults(fault_bits))

    port = identity.port or "<auto>"
    lines = [
        f"M6 LIVE MONITOR  port={port}  fw={identity.firmware_id}",
        f"samples={snapshot.samples_received}  gaps={snapshot.gaps_total}  "
        f"core[{_stream_status(core)}] wheel[{_stream_status(wheel)}] "
        f"line[{_stream_status(line)}] health[{_stream_status(health)}]",
        "",
        "MOTOR                         LEFT     RIGHT",
        f"  requested mm/s     {_pair(_value(core, 'request_l_mm_s'), _value(core, 'request_r_mm_s'))}",
        f"  ramped mm/s        {_pair(_value(wheel, 'ramp_l_mm_s'), _value(wheel, 'ramp_r_mm_s'))}",
        f"  raw mm/s           {_pair(_value(wheel, 'raw_l_mm_s'), _value(wheel, 'raw_r_mm_s'))}",
        f"  filtered mm/s      {_pair(_value(wheel, 'filtered_l_mm_s'), _value(wheel, 'filtered_r_mm_s'))}",
        f"  error mm/s         {_pair(_value(wheel, 'error_l_mm_s'), _value(wheel, 'error_r_mm_s'))}",
        f"  P / I / FF left    {_value(wheel, 'p_pwm_l'):>6} / {_value(wheel, 'i_pwm_l'):>6} / {_value(wheel, 'ff_pwm_l'):>6}",
        f"  P / I / FF right   {_value(wheel, 'p_pwm_r'):>6} / {_value(wheel, 'i_pwm_r'):>6} / {_value(wheel, 'ff_pwm_r'):>6}",
        f"  PWM                {_pair(_value(wheel, 'pwm_l', _value(core, 'pwm_l')), _value(wheel, 'pwm_r', _value(core, 'pwm_r')))}",
        f"  encoder delta      {_pair(_value(wheel, 'encoder_delta_l'), _value(wheel, 'encoder_delta_r'))}",
        "",
        f"LINE  adc: {adc_text}",
        f"  active[0..7]={mask_text} mask={mask_hex} "
        f"count={_value(line, 'active_count')} clusters={_value(line, 'cluster_count')} "
        f"state={_registry_name('line_states', _value(line, 'line_state'))}",
        f"  position={_scaled(_value(line, 'position_x1000'), 1000.0, 3)} "
        f"raw={_scaled(_value(line, 'raw_position_x1000'), 1000.0, 3)} "
        f"confidence={_scaled(_value(line, 'confidence_x1000'), 1000.0, 3)}",
        f"  base={_value(line, 'dynamic_base_mm_s')} correction={_value(line, 'correction_mm_s')} "
        f"target={_value(line, 'target_l_mm_s')}/{_value(line, 'target_r_mm_s')} mm/s "
        f"lost={_value(line, 'lost_ms')}ms",
        "",
        f"GYRO  yaw={_scaled(_value(core, 'yaw_cdeg'), 100.0, 2)} deg  "
        f"rate={_scaled(_value(core, 'yaw_rate_cdeg_s'), 100.0, 2)} deg/s  "
        f"source={_registry_name('rate_sources', _value(core, 'rate_source'))}",
        "",
        f"HEALTH  state={_registry_name('system_states', _value(health, 'system_state'))} "
        f"faults=0x{fault_bits:08X} missed={missed_total}",
        f"  fault reason={fault_text or 'none'}",
        f"  uart drop tx/rx={_value(health, 'uart_tx_drop')}/{_value(health, 'uart_rx_drop')} "
        f"telemetry drop p2/p3={_value(health, 'p2_drop')}/{_value(health, 'p3_drop')} "
        f"queue={_value(health, 'telemetry_queue')}",
    ]
    if trends is not None:
        lines.extend(
            [
                "",
                "TRENDS  oldest -> newest",
                _trend_line("speed L", trends.series("speed_l"), "mm/s"),
                _trend_line("speed R", trends.series("speed_r"), "mm/s"),
                _trend_line("line pos", trends.series("line"), "norm"),
                _trend_line("yaw", trends.series("yaw"), "deg"),
            ]
        )
    lines.extend(["", "Read-only monitor. Press Ctrl+C to exit."])
    return "\n".join(lines)


def run_monitor(
    session: Session,
    identity: Identity,
    *,
    refresh_hz: float = 10.0,
    trend_seconds: float = 6.0,
    duration_s: float | None = None,
    jsonl: bool = False,
    output: Path | None = None,
    stdout: TextIO = sys.stdout,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Subscribe to telemetry and render until interrupted or duration expires."""

    if refresh_hz <= 0.0 or refresh_hz > 50.0:
        raise ValueError("refresh_hz must be in (0, 50]")
    if duration_s is not None and duration_s <= 0.0:
        raise ValueError("duration must be positive")
    if trend_seconds < 0.0 or trend_seconds > 60.0:
        raise ValueError("trend_seconds must be in [0, 60]")
    refresh_period = 1.0 / refresh_hz
    aggregator = TelemetryAggregator(clock=clock)
    trends = (
        TrendHistory(max(2, round(refresh_hz * trend_seconds)))
        if trend_seconds > 0.0 and not jsonl
        else None
    )
    started = clock()
    deadline = float("inf") if duration_s is None else started + duration_s
    next_render = started + refresh_period
    last_sample_at = started
    next_reconnect_at = started
    output_stream: TextIO | None = None
    terminal = bool(getattr(stdout, "isatty", lambda: False)()) and not jsonl

    if output is not None:
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output_stream = output.open("w", encoding="utf-8", newline="\n")

    session.subscribe(MONITOR_SUBSCRIPTIONS)
    try:
        while clock() < deadline:
            now = clock()
            if not session.connected or now - last_sample_at >= _RECONNECT_SILENCE_S:
                if now < next_reconnect_at:
                    time.sleep(min(0.05, max(0.0, next_reconnect_at - now)))
                    continue
                try:
                    identity = session.reconnect()
                    session.subscribe(MONITOR_SUBSCRIPTIONS)
                except (M6Error, OSError, ValueError):
                    next_reconnect_at = clock() + _RECONNECT_RETRY_S
                    continue
                aggregator = TelemetryAggregator(clock=clock)
                if trends is not None:
                    trends = TrendHistory(max(2, round(refresh_hz * trend_seconds)))
                last_sample_at = clock()
                next_reconnect_at = last_sample_at
                next_render = last_sample_at + refresh_period
                continue
            wait = min(0.05, max(0.0, next_render - now), max(0.0, deadline - now))
            sample = session.read_telemetry(timeout=wait)
            if sample is not None:
                aggregator.ingest(sample)
                last_sample_at = clock()
            now = clock()
            if now < next_render:
                continue
            snapshot = aggregator.snapshot(now=now)
            if trends is not None:
                trends.ingest(snapshot)
            payload = json.dumps(snapshot.as_dict(identity), ensure_ascii=False, separators=(",", ":"))
            if jsonl:
                stdout.write(payload + "\n")
            else:
                if terminal:
                    stdout.write("\x1b[2J\x1b[H")
                stdout.write(render_terminal(snapshot, identity, trends) + "\n")
                if not terminal:
                    stdout.write("\n")
            stdout.flush()
            if output_stream is not None:
                output_stream.write(payload + "\n")
                output_stream.flush()
            while next_render <= now:
                next_render += refresh_period
    except KeyboardInterrupt:
        return 0
    finally:
        if session.connected:
            try:
                session.subscribe({})
            except Exception:
                pass
        if output_stream is not None:
            output_stream.close()
        if terminal:
            stdout.write("\n")
            stdout.flush()
    return 0


__all__ = [
    "MONITOR_SUBSCRIPTIONS",
    "MonitorSnapshot",
    "MonitorStreamSnapshot",
    "TelemetryAggregator",
    "TrendHistory",
    "render_sparkline",
    "render_terminal",
    "run_monitor",
]
