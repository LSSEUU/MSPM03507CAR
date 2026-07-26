"""Recording/replay adapters with legacy CSV/JSON compatibility."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import queue
import time
import threading
from typing import Any, Iterable, Iterator, Mapping

from .compat import (
    TELEMETRY_FIELDS,
    TELEMETRY_HEADER,
    TELEMETRY_PREFIX,
    TelemetryRecorder,
    load_recording,
    score_samples,
    utc_now,
)
from .errors import SafetyTrip
from .models import GapEvent, Identity, RecordingResult, TelemetrySample
from .registry import TelemetryStream
from .session import Session


def _empty_legacy_row() -> dict[str, int]:
    return {key: 0 for key in TELEMETRY_FIELDS}


def _recording_subscriptions(period_ms: int) -> dict[TelemetryStream, int]:
    """Return a full-stream profile within the frozen r14 admission budget."""

    period_ms = int(period_ms)
    if period_ms < 20 or period_ms > 1000 or period_ms % 20:
        raise ValueError("recording period must be 20..1000 ms in 20 ms steps")
    # At the default 20 ms row cadence this is 6105 B/s: CORE remains 50 Hz,
    # WHEEL runs at 25 Hz, LINE at 16.7 Hz, and HEALTH at 1 Hz.  The margin is
    # reserved for actual escape expansion; slower cadences only reduce load.
    return {
        TelemetryStream.CORE: period_ms,
        TelemetryStream.WHEEL: max(40, period_ms),
        TelemetryStream.LINE: max(60, period_ms),
        TelemetryStream.HEALTH: max(1000, period_ms),
    }


def _consume_transition_sample(
    sample: TelemetrySample, unsettled_streams: set[TelemetryStream]
) -> bool:
    """Consume one baseline sample per changed stream after subscription replace."""

    try:
        stream = TelemetryStream(int(sample.stream))
    except (TypeError, ValueError):
        return False
    if stream not in unsettled_streams:
        return False
    unsettled_streams.remove(stream)
    return True


def _gap_provenance(sample: TelemetrySample) -> str:
    sources = []
    if sample.wire_gap_before:
        sources.append("wire")
    if sample.sequence_gap_before:
        sources.append("sequence")
    return "+".join(sources) or "unknown"


def _seed_diagnostic_counters(session: Session, state: dict[str, int]) -> None:
    """Seed cumulative counters so pre-record history is not scored as a delta."""

    task = session.get_task_stats()
    uart = session.get_uart_stats()
    link = session.get_link_stats()
    state["missed_tasks"] = sum(int(value) for value in task.miss_count)
    state["rx_dropped"] = int(uart.rx_drop)
    state["tx_dropped"] = int(uart.tx_drop)
    state["telemetry_dropped"] = int(link.drops[2])


def _merge_v2_sample(row: dict[str, int], sample: TelemetrySample) -> bool:
    values = sample.values
    stream = int(sample.stream)
    row["time_ms"] = int(values.get("time_ms", row["time_ms"]))
    if stream == int(TelemetryStream.CORE):
        mapping = {
            "parameter_revision": "revision", "request_l_mm_s": "request_l",
            "request_r_mm_s": "request_r", "speed_l_mm_s": "filter_l",
            "speed_r_mm_s": "filter_r", "pwm_l": "pwm_l", "pwm_r": "pwm_r",
            "yaw_cdeg": "yaw_x100", "yaw_rate_cdeg_s": "rate_x100",
            "rate_source": "rate_source", "saturation_bits": "saturation_bits",
            "fault_bits": "fault_bits",
        }
        for source, target in mapping.items():
            row[target] = int(values[source])
        return True
    if stream == int(TelemetryStream.WHEEL):
        mapping = {
            "raw_l_mm_s": "raw_l", "raw_r_mm_s": "raw_r",
            "filtered_l_mm_s": "filter_l", "filtered_r_mm_s": "filter_r",
            "error_l_mm_s": "error_l", "error_r_mm_s": "error_r",
            "p_pwm_l": "p_l", "p_pwm_r": "p_r", "i_pwm_l": "i_l",
            "i_pwm_r": "i_r", "ff_pwm_l": "ff_l", "ff_pwm_r": "ff_r",
            "ramp_l_mm_s": "ramp_l", "ramp_r_mm_s": "ramp_r",
            "pwm_l": "pwm_l", "pwm_r": "pwm_r", "sat_ms_l": "sat_ms_l",
            "sat_ms_r": "sat_ms_r", "dt_us": "dt_us",
            "encoder_delta_l": "delta_l", "encoder_delta_r": "delta_r",
        }
        for source, target in mapping.items():
            row[target] = int(values[source])
    elif stream == int(TelemetryStream.LINE):
        row["gray_mask"] = int(values["active_mask"])
        row["line_info"] = (int(values["active_count"]) & 0x0F) | ((int(values["cluster_count"]) & 0x0F) << 4)
        mapping = {
            "raw_position_x1000": "pos_raw_x1000", "position_x1000": "pos_filter_x1000",
            "derivative_x1000": "derivative_x1000", "dynamic_base_mm_s": "dynamic_base",
            "correction_mm_s": "correction",
        }
        for source, target in mapping.items():
            row[target] = int(values[source])
    elif stream == int(TelemetryStream.HEALTH):
        row["fault_bits"] = int(values["fault_bits"])
        row["missed_tasks"] = sum(int(value) for value in values["task_miss"])
        row["rx_dropped"] = int(values["uart_rx_drop"])
        row["tx_dropped"] = int(values["uart_tx_drop"])
        row["telemetry_dropped"] = int(values["p2_drop"])
    return False


class Recorder:
    """Record V2 events while preserving the existing M6 CSV/JSON shape."""

    def __init__(self, session: Session) -> None:
        self.session = session

    def record(
        self,
        output_base: Path,
        *,
        duration_s: float | None,
        period_ms: int = 20,
        test_mode: str = "wheel",
        requested_speed: float = 0.0,
        identity: Identity | Mapping[str, Any] | None = None,
        parameters: Mapping[str, Any] | None = None,
        stop_event: threading.Event | None = None,
    ) -> RecordingResult:
        if duration_s is not None and duration_s <= 0:
            raise ValueError("recording duration must be positive")
        if duration_s is None and stop_event is None:
            raise ValueError("manual recording requires a stop_event")
        output_base = Path(output_base)
        if output_base.suffix in (".csv", ".json"):
            output_base = output_base.with_suffix("")
        output_base.parent.mkdir(parents=True, exist_ok=True)
        csv_path, json_path = output_base.with_suffix(".csv"), output_base.with_suffix(".json")
        subscriptions = _recording_subscriptions(period_ms)
        previous_subscriptions = dict(self.session._subscriptions)
        subscription_changed = previous_subscriptions != subscriptions
        # The diagnostic RPC burst below can legitimately preempt/coalesce a
        # queued P2 sample even when the requested subscriptions are already
        # active.  Treat one sample per stream as the post-setup recording
        # boundary; every subsequent GAP_BEFORE remains a hard failure.
        unsettled_streams = set(subscriptions)
        rows: list[dict[str, int]] = []
        event_records: list[dict[str, Any]] = []
        state = _empty_legacy_row()
        started = utc_now()
        start_monotonic = time.monotonic()
        deadline = float("inf") if duration_s is None else start_monotonic + duration_s
        abort_reason: str | None = None
        gap_provenance: str | None = None
        capture_error: BaseException | None = None
        try:
            with self.session.event_tap() as event_tap:
                if subscription_changed:
                    self.session.subscribe(subscriptions)
                # HEALTH carries cumulative counters.  Start rows from the
                # current values, after any intentional subscription-boundary
                # clear, so earlier bench/estop history is not mis-scored as a
                # drop during this recording.
                _seed_diagnostic_counters(self.session, state)
                try:
                    while time.monotonic() < deadline and not (stop_event and stop_event.is_set()):
                        wait = 0.2 if deadline == float("inf") else min(0.2, max(0.0, deadline - time.monotonic()))
                        try:
                            event = event_tap.get(timeout=wait)
                        except queue.Empty:
                            event = None
                        if event is None:
                            continue
                        if isinstance(event, TelemetrySample):
                            # Subscription replacement or the baseline RPCs may
                            # mark the next queued P2 sample GAP_BEFORE.  The
                            # first sample of each stream establishes the new
                            # recording boundary; every later gap remains a hard
                            # failure.
                            if _consume_transition_sample(event, unsettled_streams):
                                continue
                            if event.gap_before:
                                gap_provenance = _gap_provenance(event)
                                event_records.append(
                                    {
                                        "elapsed_ms": round(
                                            (time.monotonic() - start_monotonic) * 1000
                                        ),
                                        "firmware_time_ms": event.time_ms,
                                        "event": "TelemetryGap",
                                        "values": {
                                            "stream": int(event.stream),
                                            "sample_id": event.sample_id,
                                            "wire_gap_before": event.wire_gap_before,
                                            "sequence_gap_before": event.sequence_gap_before,
                                            "expected_sequence": event.expected_sequence,
                                            "observed_sequence": event.sequence,
                                        },
                                    }
                                )
                                abort_reason = "telemetry_gap"
                                raise SafetyTrip(0x010A, 0, name="TELEMETRY_GAP")
                            if _merge_v2_sample(state, event):
                                rows.append(dict(state))
                        else:
                            event_records.append(
                                {
                                    "elapsed_ms": round((time.monotonic() - start_monotonic) * 1000),
                                    "firmware_time_ms": getattr(event, "time_ms", state.get("time_ms")),
                                    "event": type(event).__name__,
                                    "values": dict(getattr(event, "values", {})),
                                }
                            )
                            if isinstance(event, GapEvent):
                                gap_provenance = "device_event"
                                abort_reason = "telemetry_gap"
                                raise SafetyTrip(0x010A, 0, name="TELEMETRY_GAP")
                finally:
                    if subscription_changed:
                        try:
                            self.session.subscribe(previous_subscriptions)
                        except Exception:
                            if abort_reason is None:
                                abort_reason = "log_stop_unconfirmed"
        except BaseException as exc:
            capture_error = exc
        linked = identity or self.session.identity
        if isinstance(linked, Identity):
            identity_map = linked.as_legacy_dict()
        else:
            identity_map = dict(linked or {})
        metrics = score_samples(rows, requested_speed=requested_speed, mode=test_mode, expected_period_ms=period_ms)
        if abort_reason and abort_reason not in metrics["hard_failures"]:
            metrics["hard_failures"].append(abort_reason)
        summary = {
            "firmware": identity_map.get("firmware"), "schema": identity_map.get("schema"),
            "revision": identity_map.get("revision"), "source": identity_map.get("source"),
            "generation": identity_map.get("generation"), "test_mode": test_mode,
            "requested_speed": requested_speed, "period_ms": period_ms,
            "start_time": started, "end_time": utc_now(),
            "parameters": dict(parameters or {}), "events": event_records,
            "abort_reason": abort_reason, "gap_provenance": gap_provenance,
            "metrics": metrics,
        }
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(TELEMETRY_HEADER)
            for row in rows:
                writer.writerow([TELEMETRY_PREFIX] + [row[key] for key in TELEMETRY_FIELDS])
        json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        result = RecordingResult(csv_path, json_path, rows, summary)
        if capture_error is not None:
            raise capture_error
        return result


class Replay:
    """Deterministic iterable over a legacy-compatible recording."""

    def __init__(self, path: Path) -> None:
        self.samples, self.metadata = load_recording(path)

    def __iter__(self) -> Iterator[dict[str, int]]:
        return iter(self.samples)

    def events(self) -> Iterable[Mapping[str, Any]]:
        return tuple(self.metadata.get("events", ()))

    def score(self) -> Mapping[str, Any]:
        return score_samples(
            self.samples,
            requested_speed=float(self.metadata["requested_speed"]),
            mode=str(self.metadata["test_mode"]),
            expected_period_ms=int(self.metadata.get("period_ms", 20)),
        )


__all__ = [
    "Recorder", "Replay", "RecordingResult", "TelemetryRecorder",
    "load_recording", "score_samples",
]
