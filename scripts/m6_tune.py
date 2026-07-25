#!/usr/bin/env python3
"""Safe host-side parameter control, telemetry recording, and M6 tuning.

The script deliberately keeps motion commands separate from transport retries:
each transaction frame is written once, then a bounded number of reads is used
to find the response with the matching transaction ID.  Emergency stop is the
existing single byte ``S`` and is never hidden behind the transaction parser.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import statistics
import sys
import time
from typing import Any, Callable, Iterable, Mapping, Sequence


BAUD_RATE = 115200
PARAMETER_SCHEMA = 1
DEFAULT_FIRMWARE_PREFIX = "M6"
MAX_FRAME_BYTES = 60
TELEMETRY_PREFIX = "M6"
TELEMETRY_FIELDS = (
    "time_ms",
    "revision",
    "request_l",
    "request_r",
    "ramp_l",
    "ramp_r",
    "raw_l",
    "raw_r",
    "filter_l",
    "filter_r",
    "error_l",
    "error_r",
    "p_l",
    "p_r",
    "i_l",
    "i_r",
    "ff_l",
    "ff_r",
    "pwm_l",
    "pwm_r",
    "dt_us",
    "delta_l",
    "delta_r",
    "sat_ms_l",
    "sat_ms_r",
    "yaw_x100",
    "rate_x100",
    "rate_source",
    "gray_mask",
    "line_info",
    "pos_raw_x1000",
    "pos_filter_x1000",
    "derivative_x1000",
    "dynamic_base",
    "correction",
    "saturation_bits",
    "fault_bits",
    "missed_tasks",
    "rx_dropped",
    "tx_dropped",
    "telemetry_dropped",
)
TELEMETRY_HEADER = (TELEMETRY_PREFIX,) + TELEMETRY_FIELDS

PARAMETER_KEYS = (
    "wheel.left.kp",
    "wheel.left.ki",
    "wheel.right.kp",
    "wheel.right.ki",
    "wheel.filter_alpha",
    "wheel.accel",
    "wheel.decel",
    "wheel.jerk",
    "wheel.left.ks",
    "wheel.left.kv",
    "wheel.right.ks",
    "wheel.right.kv",
    "wheel.diag_target",
    "heading.straight.kp",
    "heading.straight.kd",
    "heading.turn.kp",
    "heading.turn.kd",
    "heading.rate_alpha",
    "heading.correction_limit",
    "heading.turn_speed",
    "heading.smooth_zone",
    "line.speed_min",
    "line.speed_max",
    "line.position_alpha",
    "line.derivative_alpha",
    "line.kp",
    "line.kd",
    "line.correction_limit",
    "line.correction_slew",
    "line.curve_gain",
    "line.lost_search_speed",
    "remote.speed_low",
    "remote.speed_medium",
    "remote.speed_high",
    "remote.heading_hold",
)

PARAMETER_BOUNDS: dict[str, tuple[float, float]] = {
    "wheel.left.kp": (0.0, 3.0),
    "wheel.left.ki": (0.0, 10.0),
    "wheel.right.kp": (0.0, 3.0),
    "wheel.right.ki": (0.0, 10.0),
    "wheel.filter_alpha": (0.05, 1.0),
    "wheel.accel": (100.0, 2000.0),
    "wheel.decel": (100.0, 2000.0),
    "wheel.jerk": (1000.0, 20000.0),
    "wheel.left.ks": (0.0, 850.0),
    "wheel.left.kv": (0.0, 5.0),
    "wheel.right.ks": (0.0, 850.0),
    "wheel.right.kv": (0.0, 5.0),
    "wheel.diag_target": (-400.0, 400.0),
    "heading.straight.kp": (0.0, 10.0),
    "heading.straight.kd": (0.0, 5.0),
    "heading.turn.kp": (0.0, 10.0),
    "heading.turn.kd": (0.0, 5.0),
    "heading.rate_alpha": (0.05, 1.0),
    "heading.correction_limit": (0.0, 250.0),
    "heading.turn_speed": (50.0, 400.0),
    "heading.smooth_zone": (2.0, 60.0),
    "line.speed_min": (50.0, 400.0),
    "line.speed_max": (50.0, 400.0),
    "line.position_alpha": (0.05, 1.0),
    "line.derivative_alpha": (0.05, 1.0),
    "line.kp": (0.0, 400.0),
    "line.kd": (0.0, 100.0),
    "line.correction_limit": (0.0, 250.0),
    "line.correction_slew": (100.0, 2000.0),
    "line.curve_gain": (0.0, 2.0),
    "line.lost_search_speed": (50.0, 400.0),
    "remote.speed_low": (50.0, 400.0),
    "remote.speed_medium": (50.0, 400.0),
    "remote.speed_high": (50.0, 400.0),
    "remote.heading_hold": (0.0, 1.0),
}

MIN_STEPS: dict[str, float] = {
    "wheel.left.kp": 0.05,
    "wheel.left.ki": 0.25,
    "wheel.right.kp": 0.05,
    "wheel.right.ki": 0.25,
    "wheel.filter_alpha": 0.05,
    "wheel.accel": 50.0,
    "wheel.decel": 50.0,
    "wheel.jerk": 500.0,
    "wheel.left.ks": 5.0,
    "wheel.left.kv": 0.02,
    "wheel.right.ks": 5.0,
    "wheel.right.kv": 0.02,
}

_DECIMAL_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$")


class M6Error(RuntimeError):
    """Base class for expected M6 tool failures."""


class TransportError(M6Error):
    """UART framing, timeout, or disconnection failure."""


class DeviceError(M6Error):
    """Firmware returned an ``err`` transaction response."""

    def __init__(self, request_id: int, code: str, details: Sequence[str] = ()):
        self.request_id = request_id
        self.code = code
        self.details = tuple(details)
        suffix = "," + ",".join(self.details) if self.details else ""
        super().__init__(f"device error {code} for transaction {request_id}{suffix}")


class SafetyAbort(M6Error):
    """An acceptance-invalidating condition was observed."""

    def __init__(self, reason: str, samples: Sequence[Mapping[str, int]] = ()):
        self.reason = reason
        self.samples = list(samples)
        super().__init__(reason)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def decimal_text(value: Any) -> str:
    """Return firmware-compatible finite decimal text without exponent form."""

    if isinstance(value, str):
        text = value.strip()
        if not _DECIMAL_RE.fullmatch(text):
            raise ValueError(f"not a finite plain decimal: {value!r}")
        number = float(text)
    else:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f"not finite: {value!r}")
        text = f"{number:.6f}".rstrip("0").rstrip(".")
        if text in ("-0", "+0", ""):
            text = "0"
    if not math.isfinite(number):
        raise ValueError(f"not finite: {value!r}")
    return text


def numeric_value(text: str) -> float | int | str:
    try:
        value = float(text)
    except ValueError:
        return text
    if not math.isfinite(value):
        return text
    if value.is_integer() and "." not in text:
        return int(value)
    return value


class M6Client:
    """One-write transaction client over a pyserial-compatible object."""

    def __init__(
        self,
        transport: Any,
        *,
        response_timeout: float = 1.0,
        read_attempts: int = 3,
    ) -> None:
        self.transport = transport
        self.response_timeout = max(0.05, float(response_timeout))
        self.read_attempts = max(1, int(read_attempts))
        self._next_id = 1
        self.unsolicited: list[str] = []
        self._rx_buffer = ""

    @property
    def port(self) -> str:
        return str(getattr(self.transport, "port", "<injected>"))

    def close(self) -> None:
        close = getattr(self.transport, "close", None)
        if close:
            close()

    def _allocate_id(self) -> int:
        request_id = self._next_id
        self._next_id = 1 if request_id >= 65535 else request_id + 1
        return request_id

    def _readline(self) -> str:
        try:
            raw = self.transport.readline()
        except Exception as exc:  # pyserial uses several platform exceptions
            raise TransportError(f"serial read failed: {exc}") from exc
        if not raw:
            return ""
        if isinstance(raw, bytes):
            fragment = raw.decode("ascii", errors="replace")
        else:
            fragment = str(raw)
        self._rx_buffer += fragment
        if len(self._rx_buffer) > 1024:
            self._rx_buffer = ""
            raise TransportError("serial line exceeded 1024 buffered characters")
        terminators = [
            index
            for index in (self._rx_buffer.find("\r"), self._rx_buffer.find("\n"))
            if index >= 0
        ]
        if not terminators:
            return ""
        end = min(terminators)
        line = self._rx_buffer[:end]
        consumed = end + 1
        if (
            consumed < len(self._rx_buffer)
            and self._rx_buffer[end] == "\r"
            and self._rx_buffer[consumed] == "\n"
        ):
            consumed += 1
        self._rx_buffer = self._rx_buffer[consumed:]
        return line

    def read_line(self) -> str:
        return self._readline()

    def transaction(self, verb: str, *arguments: str) -> list[str]:
        if not re.fullmatch(r"[a-z]+", verb):
            raise ValueError(f"invalid transaction verb: {verb!r}")
        request_id = self._allocate_id()
        frame = ":" + ",".join((str(request_id), verb, *arguments)) + "\r\n"
        encoded = frame.encode("ascii")
        if len(encoded) > MAX_FRAME_BYTES:
            raise ValueError(f"transaction frame exceeds {MAX_FRAME_BYTES} bytes")
        try:
            written = self.transport.write(encoded)
            if written is not None and written != len(encoded):
                raise TransportError(f"short serial write: {written}/{len(encoded)}")
        except M6Error:
            raise
        except Exception as exc:
            raise TransportError(f"serial write failed: {exc}") from exc

        # Retries are reads only.  The request is never transmitted twice.
        slice_timeout = self.response_timeout / self.read_attempts
        for _ in range(self.read_attempts):
            deadline = time.monotonic() + slice_timeout
            while time.monotonic() < deadline:
                line = self._readline()
                if not line:
                    continue
                fields = line.split(",")
                if (
                    len(fields) >= 2
                    and fields[0] in ("ok", "err")
                    and fields[1].isdigit()
                ):
                    if int(fields[1]) != request_id:
                        self.unsolicited.append(line)
                        continue
                    if fields[0] == "err":
                        code = fields[2] if len(fields) >= 3 else "syntax"
                        raise DeviceError(request_id, code, fields[3:])
                    return fields[2:]
                self.unsolicited.append(line)
        raise TransportError(f"timeout waiting for transaction {request_id}")

    def get(self, key: str) -> tuple[Any, ...]:
        payload = self.transaction("get", key)
        if len(payload) < 2 or payload[0] != key:
            raise TransportError(f"malformed get response for {key}: {payload}")
        return tuple(numeric_value(item) for item in payload[1:])

    def get_one(self, key: str) -> Any:
        values = self.get(key)
        if len(values) != 1:
            raise TransportError(f"expected one value for {key}, received {values}")
        return values[0]

    def set(self, key: str, value: Any) -> list[str]:
        return self.transaction("set", key, decimal_text(value))

    def apply(self) -> list[str]:
        return self.transaction("apply")

    def revert(self) -> list[str]:
        return self.transaction("revert")

    def load(self) -> list[str]:
        return self.transaction("load")

    def defaults(self) -> list[str]:
        return self.transaction("defaults")

    def save(self) -> list[str]:
        return self.transaction("save")

    def log(self, mode: str, period_ms: int) -> list[str]:
        if mode not in ("m6", "off"):
            raise ValueError("log mode must be m6 or off")
        return self.transaction("log", mode, str(int(period_ms)))

    def emergency_stop(self) -> None:
        try:
            written = self.transport.write(b"S")
            if written is not None and written != 1:
                raise TransportError("short emergency-stop write")
        except M6Error:
            raise
        except Exception as exc:
            raise TransportError(f"emergency-stop write failed: {exc}") from exc

    def legacy(self, command: str) -> None:
        if len(command) != 1 or command in ("S", " "):
            raise ValueError("legacy command must be one non-stop character")
        try:
            self.transport.write(command.encode("ascii"))
        except Exception as exc:
            raise TransportError(f"legacy command write failed: {exc}") from exc

    def probe(
        self,
        firmware_prefix: str = DEFAULT_FIRMWARE_PREFIX,
        schema: int = PARAMETER_SCHEMA,
        firmware_id: str | None = None,
    ) -> dict[str, Any]:
        result = {
            "port": self.port,
            "firmware": str(self.get_one("meta.firmware")),
            "schema": int(self.get_one("meta.schema")),
            "revision": int(self.get_one("meta.revision")),
            "source": str(self.get_one("meta.source")),
        }
        try:
            result["generation"] = int(self.get_one("meta.generation"))
        except DeviceError as exc:
            if exc.code not in ("key", "unknown"):
                raise
        if not result["firmware"].startswith(firmware_prefix):
            raise TransportError(
                f"firmware {result['firmware']!r} does not start with {firmware_prefix!r}"
            )
        if firmware_id is not None and result["firmware"] != firmware_id:
            raise TransportError(
                f"firmware {result['firmware']!r} does not match expected {firmware_id!r}"
            )
        if result["schema"] != schema:
            raise TransportError(
                f"parameter schema {result['schema']} does not match expected {schema}"
            )
        return result

    def active_snapshot(self, keys: Iterable[str] = PARAMETER_KEYS) -> dict[str, Any]:
        snapshot: dict[str, Any] = {}
        for key in keys:
            values = self.get(key)
            if not values:
                raise TransportError(f"missing active value for {key}")
            snapshot[key] = values[0]
        return snapshot


def _import_serial() -> tuple[Any, Any]:
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise M6Error("pyserial is required: python -m pip install pyserial") from exc
    return serial, list_ports


def discover_client(
    *,
    port: str | None = None,
    baud: int = BAUD_RATE,
    firmware_prefix: str = DEFAULT_FIRMWARE_PREFIX,
    firmware_id: str | None = None,
    schema: int = PARAMETER_SCHEMA,
    timeout: float = 1.0,
    serial_factory: Callable[..., Any] | None = None,
    port_provider: Callable[[], Iterable[Any]] | None = None,
) -> tuple[M6Client, dict[str, Any]]:
    serial_module: Any = None
    if serial_factory is None or port_provider is None:
        serial_module, list_ports = _import_serial()
        serial_factory = serial_factory or serial_module.Serial
        port_provider = port_provider or list_ports.comports
    candidates = [port] if port else [str(item.device) for item in port_provider()]
    if not candidates:
        raise TransportError("no serial ports are enumerated")
    failures: list[str] = []
    for candidate in candidates:
        transport = None
        try:
            transport = serial_factory(
                candidate,
                baudrate=baud,
                timeout=min(0.10, timeout / 3.0),
                write_timeout=0.5,
            )
            reset = getattr(transport, "reset_input_buffer", None)
            if reset:
                reset()
            client = M6Client(transport, response_timeout=timeout)
            identity = client.probe(firmware_prefix, schema, firmware_id)
            return client, identity
        except Exception as exc:
            failures.append(f"{candidate}: {exc}")
            if transport is not None:
                try:
                    transport.close()
                except Exception:
                    pass
    raise TransportError("no compatible M6 firmware found; " + "; ".join(failures))


def parse_telemetry_line(line: str) -> dict[str, int] | None:
    fields = line.strip().split(",")
    if not fields or fields[0] != TELEMETRY_PREFIX:
        return None
    if tuple(fields) == TELEMETRY_HEADER:
        return None
    if len(fields) != len(TELEMETRY_HEADER):
        raise ValueError(
            f"M6 telemetry has {len(fields)} columns; expected {len(TELEMETRY_HEADER)}"
        )
    sample: dict[str, int] = {}
    for key, value in zip(TELEMETRY_FIELDS, fields[1:]):
        if not re.fullmatch(r"-?\d+", value):
            raise ValueError(f"M6 telemetry field {key} is not an integer: {value!r}")
        sample[key] = int(value)
    return sample


def _delta(samples: Sequence[Mapping[str, int]], key: str) -> int:
    if not samples:
        return 0
    return max(0, int(samples[-1][key]) - int(samples[0][key]))


def _rms(values: Iterable[float]) -> float:
    data = list(values)
    if not data:
        return 0.0
    return math.sqrt(sum(value * value for value in data) / len(data))


def _saturation_duration_ms(samples: Sequence[Mapping[str, int]]) -> int:
    longest = current = 0
    previous_time: int | None = None
    for sample in samples:
        now = int(sample["time_ms"])
        if int(sample["saturation_bits"]) & 0x03:
            if previous_time is not None:
                current += max(0, now - previous_time)
            longest = max(longest, current)
        else:
            current = 0
        previous_time = now
    reported = max(
        (
            max(int(sample.get("sat_ms_l", 0)), int(sample.get("sat_ms_r", 0)))
            for sample in samples
        ),
        default=0,
    )
    return max(longest, reported)


def telemetry_abort_reason(
    samples: Sequence[Mapping[str, int]],
    *,
    mode: str,
    period_ms: int,
) -> str | None:
    if not samples:
        return "telemetry_missing"
    current = samples[-1]
    first = samples[0]
    if int(current["fault_bits"]) != 0:
        return f"firmware_fault_0x{int(current['fault_bits']):X}"
    if int(current["missed_tasks"]) > int(first["missed_tasks"]):
        return "scheduler_deadline_miss"
    if int(current["rx_dropped"]) > int(first["rx_dropped"]):
        return "uart_rx_drop"
    if int(current["tx_dropped"]) > int(first["tx_dropped"]):
        return "uart_tx_drop"
    if int(current["telemetry_dropped"]) > int(first["telemetry_dropped"]):
        return "telemetry_drop"
    if mode == "line" and (int(current["line_info"]) >> 4) == 0:
        return "line_loss"
    if len(samples) >= 2:
        gap = int(samples[-1]["time_ms"]) - int(samples[-2]["time_ms"])
        if gap > max(100, period_ms * 3):
            return "telemetry_gap"
    if _saturation_duration_ms(samples) >= 300:
        return "sustained_pwm_saturation"
    return None


def _rise_time_ms(
    samples: Sequence[Mapping[str, int]], requested_speed: float
) -> float:
    if not samples:
        return math.inf
    first_active = next(
        (
            sample
            for sample in samples
            if int(sample["request_l"]) != 0 or int(sample["request_r"]) != 0
        ),
        None,
    )
    if first_active is None:
        return float(int(samples[-1]["time_ms"]) - int(samples[0]["time_ms"]))
    start = int(first_active["time_ms"])
    hold_ms = 300
    tolerance = max(1.0, abs(requested_speed) * 0.05)
    for index, sample in enumerate(samples):
        active_pairs = [
            (measured, request)
            for measured, request in (
                ("filter_l", "request_l"),
                ("filter_r", "request_r"),
            )
            if int(sample[request]) != 0
        ]
        if not active_pairs:
            continue
        now = int(sample["time_ms"])
        if all(
            abs(abs(float(sample[measured])) - abs(requested_speed)) <= tolerance
            for measured, _ in active_pairs
        ):
            held = True
            end_seen = now
            for later in samples[index:]:
                later_time = int(later["time_ms"])
                if later_time - now > hold_ms:
                    break
                end_seen = later_time
                if not any(
                    int(later[request]) != 0
                    for request in ("request_l", "request_r")
                ):
                    held = False
                    break
                for measured, request in (
                    ("filter_l", "request_l"),
                    ("filter_r", "request_r"),
                ):
                    if int(later[request]) != 0 and abs(
                        abs(float(later[measured])) - abs(requested_speed)
                    ) > tolerance:
                        held = False
                        break
                if not held:
                    break
            if held and end_seen - now >= hold_ms:
                return float(now - start)
    return float(int(samples[-1]["time_ms"]) - start)


def score_samples(
    samples: Sequence[Mapping[str, int]],
    *,
    requested_speed: float,
    mode: str = "wheel",
    expected_period_ms: int = 20,
) -> dict[str, Any]:
    if not samples:
        return {
            "score": 1.0e9,
            "hard_failures": ["telemetry_missing"],
            "sample_count": 0,
        }
    ordered = sorted(samples, key=lambda item: int(item["time_ms"]))
    target = abs(float(requested_speed))
    if target <= 0:
        nonzero = [
            abs(float(sample[key]))
            for sample in ordered
            for key in ("request_l", "request_r")
            if int(sample[key]) != 0
        ]
        target = statistics.median(nonzero) if nonzero else 1.0
    start_ms = int(ordered[0]["time_ms"])
    end_ms = int(ordered[-1]["time_ms"])
    duration_ms = max(1, end_ms - start_ms)
    rise_time_ms = _rise_time_ms(ordered, target)

    active_measurements: list[float] = []
    normalized_errors: list[float] = []
    for sample in ordered:
        for measured, request in (
            ("filter_l", "request_l"),
            ("filter_r", "request_r"),
        ):
            if int(sample[request]) != 0:
                speed = abs(float(sample[measured]))
                active_measurements.append(speed)
                normalized_errors.append(abs(speed - target) / max(target, 1.0))
    overshoot = max(
        (max(0.0, speed - target) / max(target, 1.0) for speed in active_measurements),
        default=0.0,
    )
    active_rows = [
        sample
        for sample in ordered
        if int(sample["request_l"]) != 0 or int(sample["request_r"]) != 0
    ]
    active_end_ms = int(active_rows[-1]["time_ms"]) if active_rows else end_ms
    steady = [
        sample
        for sample in active_rows
        if int(sample["time_ms"]) >= active_end_ms - 1000
    ]
    steady_errors = [
        abs(abs(float(sample[measured])) - target) / max(target, 1.0)
        for sample in steady
        for measured, request in (
            ("filter_l", "request_l"),
            ("filter_r", "request_r"),
        )
        if int(sample[request]) != 0
    ]
    steady_error = statistics.fmean(steady_errors) if steady_errors else 1.0

    zero_crossings = 0
    previous_sign: dict[str, int] = {}
    crossing_start = start_ms + int(rise_time_ms if math.isfinite(rise_time_ms) else 0)
    deadband = target * 0.01
    for sample in ordered:
        if int(sample["time_ms"]) < crossing_start:
            continue
        for measured, request in (
            ("filter_l", "request_l"),
            ("filter_r", "request_r"),
        ):
            if int(sample[request]) == 0:
                continue
            error = target - abs(float(sample[measured]))
            if abs(error) <= deadband:
                continue
            sign = 1 if error > 0 else -1
            if measured in previous_sign and previous_sign[measured] != sign:
                zero_crossings += 1
            previous_sign[measured] = sign

    saturated = sum(1 for sample in ordered if int(sample["saturation_bits"]) & 0x03)
    saturation_ratio = saturated / len(ordered)
    yaw_origin = float(ordered[0]["yaw_x100"]) / 100.0
    yaw_values = [float(sample["yaw_x100"]) / 100.0 - yaw_origin for sample in ordered]
    position_values = [
        float(sample["pos_filter_x1000"]) / 1000.0 for sample in ordered
    ]
    correction_rates: list[float] = []
    for previous, current in zip(ordered, ordered[1:]):
        dt = (int(current["time_ms"]) - int(previous["time_ms"])) / 1000.0
        if dt > 0:
            correction_rates.append(
                (float(current["correction"]) - float(previous["correction"])) / dt
            )
    line_loss_events = 0
    was_valid = True
    for sample in ordered:
        valid = (int(sample["line_info"]) >> 4) != 0
        if was_valid and not valid:
            line_loss_events += 1
        was_valid = valid

    missed_tasks = _delta(ordered, "missed_tasks")
    uart_drops = _delta(ordered, "rx_dropped") + _delta(ordered, "tx_dropped")
    telemetry_drops = _delta(ordered, "telemetry_dropped")
    fault_bits = 0
    for sample in ordered:
        fault_bits |= int(sample["fault_bits"])
    gaps = sum(
        1
        for previous, current in zip(ordered, ordered[1:])
        if int(current["time_ms"]) - int(previous["time_ms"])
        > max(100, expected_period_ms * 3)
    )
    saturation_ms = _saturation_duration_ms(ordered)

    hard_failures: list[str] = []
    if fault_bits:
        hard_failures.append(f"firmware_fault_0x{fault_bits:X}")
    if mode == "line" and line_loss_events:
        hard_failures.append("line_loss")
    if saturation_ms >= 300:
        hard_failures.append("sustained_pwm_saturation")
    if missed_tasks:
        hard_failures.append("scheduler_deadline_miss")
    if uart_drops:
        hard_failures.append("uart_drop")
    if telemetry_drops:
        hard_failures.append("telemetry_drop")
    if gaps:
        hard_failures.append("telemetry_gap")

    yaw_rms = _rms(yaw_values)
    position_rms = _rms(position_values)
    correction_jerk_rms = _rms(correction_rates)
    score = (
        rise_time_ms / 1000.0
        + overshoot * 4.0
        + steady_error * 6.0
        + zero_crossings * 0.05
        + saturation_ratio * 5.0
        + yaw_rms / 10.0
        + position_rms * 2.0
        + correction_jerk_rms / 1000.0
        + line_loss_events * 10.0
        + missed_tasks * 20.0
        + (uart_drops + telemetry_drops + gaps) * 20.0
    )
    return {
        "score": score,
        "hard_failures": hard_failures,
        "sample_count": len(ordered),
        "duration_ms": duration_ms,
        "rise_time_ms": rise_time_ms,
        "overshoot_pct": overshoot * 100.0,
        "steady_error_pct": steady_error * 100.0,
        "error_zero_crossings": zero_crossings,
        "saturation_ratio": saturation_ratio,
        "sustained_saturation_ms": saturation_ms,
        "yaw_rms_deg": yaw_rms,
        "yaw_peak_deg": max((abs(value) for value in yaw_values), default=0.0),
        "line_position_rms": position_rms,
        "correction_jerk_rms": correction_jerk_rms,
        "line_loss_events": line_loss_events,
        "missed_task_delta": missed_tasks,
        "uart_drop_delta": uart_drops,
        "telemetry_drop_delta": telemetry_drops,
        "telemetry_gap_count": gaps,
        "fault_bits": fault_bits,
    }


def evidence_compatible(baseline: Mapping[str, Any], candidate: Mapping[str, Any]) -> bool:
    return all(
        baseline.get(key) == candidate.get(key)
        for key in ("firmware", "schema", "test_mode", "requested_speed")
    )


def candidate_accepted(
    baseline: Mapping[str, Any],
    candidate: Mapping[str, Any],
    *,
    feedforward: bool = False,
) -> tuple[bool, float]:
    if baseline.get("hard_failures") or candidate.get("hard_failures"):
        return False, -1.0
    baseline_score = float(baseline["score"])
    candidate_score = float(candidate["score"])
    improvement = (baseline_score - candidate_score) / max(abs(baseline_score), 1e-9)
    threshold = 0.10 if feedforward else 0.05
    if feedforward:
        required_speeds = [100, 160, 220, 300]
        if (
            sorted(candidate.get("validation_speeds", [])) != required_speeds
            or sorted(baseline.get("validation_speeds", [])) != required_speeds
            or float(candidate.get("overshoot_pct", math.inf))
            > float(baseline.get("overshoot_pct", 0.0))
            or float(candidate.get("saturation_ratio", math.inf))
            > float(baseline.get("saturation_ratio", 0.0))
            or int(candidate.get("sustained_saturation_ms", 0)) >= 300
        ):
            return False, improvement
    return improvement >= threshold, improvement


def aggregate_validation_metrics(metrics: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if not metrics:
        return {"score": 1.0e9, "hard_failures": ["missing_validation_points"]}
    failures = [
        reason
        for metric in metrics
        for reason in metric.get("hard_failures", [])
    ]
    return {
        "score": statistics.fmean(float(metric["score"]) for metric in metrics),
        "hard_failures": failures,
        "overshoot_pct": max(float(metric.get("overshoot_pct", 0.0)) for metric in metrics),
        "saturation_ratio": max(
            float(metric.get("saturation_ratio", 0.0)) for metric in metrics
        ),
        "sustained_saturation_ms": max(
            int(metric.get("sustained_saturation_ms", 0)) for metric in metrics
        ),
        "validation_speeds": [metric.get("requested_speed") for metric in metrics],
    }


def feedforward_evidence_passed(
    baseline_by_speed: Mapping[int, Mapping[str, Any]],
    candidate_by_speed: Mapping[int, Mapping[str, Any]],
) -> tuple[bool, float]:
    required = (100, 160, 220, 300)
    if any(speed not in baseline_by_speed or speed not in candidate_by_speed for speed in required):
        return False, -1.0
    baseline = aggregate_validation_metrics([baseline_by_speed[speed] for speed in required])
    candidate = aggregate_validation_metrics([candidate_by_speed[speed] for speed in required])
    baseline["validation_speeds"] = list(required)
    candidate["validation_speeds"] = list(required)
    return candidate_accepted(baseline, candidate, feedforward=True)


def fit_feedforward(samples_by_speed: Mapping[int, Sequence[Mapping[str, int]]]) -> dict[str, float]:
    """Fit ``PWM = kS + kV*speed`` from settled absolute wheel data."""

    required = (100, 160, 220, 300)
    if any(speed not in samples_by_speed for speed in required):
        raise ValueError("feedforward fit requires 100/160/220/300 mm/s data")
    result: dict[str, float] = {}
    for suffix, pwm_key in (("left", "pwm_l"), ("right", "pwm_r")):
        points: list[tuple[float, float]] = []
        for speed in required:
            samples = list(samples_by_speed[speed])
            if not samples:
                raise ValueError(f"no samples at {speed} mm/s")
            end = int(samples[-1]["time_ms"])
            settled = [sample for sample in samples if int(sample["time_ms"]) >= end - 1000]
            pwm = statistics.fmean(abs(float(sample[pwm_key])) for sample in settled)
            points.append((float(speed), pwm))
        mean_x = statistics.fmean(point[0] for point in points)
        mean_y = statistics.fmean(point[1] for point in points)
        denominator = sum((x - mean_x) ** 2 for x, _ in points)
        kv = sum((x - mean_x) * (y - mean_y) for x, y in points) / denominator
        ks = mean_y - kv * mean_x
        result[f"wheel.{suffix}.ks"] = min(680.0, max(0.0, ks))
        result[f"wheel.{suffix}.kv"] = min(5.0, max(0.0, kv))
        if result[f"wheel.{suffix}.ks"] + result[f"wheel.{suffix}.kv"] * 400 > 680:
            result[f"wheel.{suffix}.kv"] = max(
                0.0, (680.0 - result[f"wheel.{suffix}.ks"]) / 400.0
            )
    return result


@dataclass
class RecordingResult:
    csv_path: Path
    json_path: Path
    samples: list[dict[str, int]]
    summary: dict[str, Any]


class TelemetryRecorder:
    def __init__(self, client: M6Client) -> None:
        self.client = client

    def record(
        self,
        output_base: Path,
        *,
        duration_s: float,
        period_ms: int,
        test_mode: str,
        requested_speed: float,
        identity: Mapping[str, Any],
        parameters: Mapping[str, Any],
        start_action: Callable[[], None] | None = None,
    ) -> RecordingResult:
        if period_ms < 20 or period_ms > 1000 or period_ms % 20:
            raise ValueError("period must be 20-1000 ms in 20 ms multiples")
        if duration_s <= 0:
            raise ValueError("recording duration must be positive")
        output_base = Path(output_base)
        if output_base.suffix in (".csv", ".json"):
            output_base = output_base.with_suffix("")
        output_base.parent.mkdir(parents=True, exist_ok=True)
        csv_path = output_base.with_suffix(".csv")
        json_path = output_base.with_suffix(".json")
        samples: list[dict[str, int]] = []
        events: list[dict[str, Any]] = []
        started = utc_now()
        abort_reason: str | None = None
        self.client.log("m6", period_ms)
        capture_start = time.monotonic()
        last_sample_at = capture_start
        try:
            if start_action:
                start_action()
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                line = self.client.read_line()
                if not line:
                    if time.monotonic() - last_sample_at > max(0.5, period_ms / 1000.0 * 4):
                        abort_reason = "telemetry_timeout"
                        raise SafetyAbort(abort_reason, samples)
                    continue
                try:
                    sample = parse_telemetry_line(line)
                except ValueError:
                    events.append(
                        {
                            "elapsed_ms": round(
                                (time.monotonic() - capture_start) * 1000
                            ),
                            "firmware_time_ms": (
                                int(samples[-1]["time_ms"]) if samples else None
                            ),
                            "line": line,
                        }
                    )
                    abort_reason = "malformed_telemetry"
                    raise SafetyAbort(abort_reason, samples)
                if sample is None:
                    events.append(
                        {
                            "elapsed_ms": round(
                                (time.monotonic() - capture_start) * 1000
                            ),
                            "firmware_time_ms": (
                                int(samples[-1]["time_ms"]) if samples else None
                            ),
                            "line": line,
                        }
                    )
                    continue
                samples.append(sample)
                last_sample_at = time.monotonic()
                abort_reason = telemetry_abort_reason(
                    samples, mode=test_mode, period_ms=period_ms
                )
                if abort_reason:
                    raise SafetyAbort(abort_reason, samples)
        except KeyboardInterrupt as exc:
            abort_reason = "operator_emergency_stop"
            try:
                self.client.emergency_stop()
            finally:
                raise SafetyAbort(abort_reason, samples) from exc
        except SafetyAbort:
            try:
                self.client.emergency_stop()
            except TransportError:
                pass
            raise
        finally:
            try:
                self.client.log("off", 0)
            except M6Error:
                if abort_reason is None:
                    abort_reason = "log_stop_unconfirmed"
            metrics = score_samples(
                samples,
                requested_speed=requested_speed,
                mode=test_mode,
                expected_period_ms=period_ms,
            )
            if abort_reason and abort_reason not in metrics["hard_failures"]:
                metrics["hard_failures"].append(abort_reason)
            summary = {
                "firmware": identity.get("firmware"),
                "schema": identity.get("schema"),
                "revision": identity.get("revision"),
                "source": identity.get("source"),
                "generation": identity.get("generation"),
                "test_mode": test_mode,
                "requested_speed": requested_speed,
                "period_ms": period_ms,
                "start_time": started,
                "end_time": utc_now(),
                "parameters": dict(parameters),
                "events": events,
                "abort_reason": abort_reason,
                "metrics": metrics,
            }
            with csv_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle, lineterminator="\n")
                writer.writerow(TELEMETRY_HEADER)
                for sample in samples:
                    writer.writerow(
                        [TELEMETRY_PREFIX]
                        + [sample[key] for key in TELEMETRY_FIELDS]
                    )
            json_path.write_text(
                json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                encoding="utf-8",
            )
        result = RecordingResult(csv_path, json_path, samples, summary)
        if result.summary["metrics"]["hard_failures"]:
            try:
                self.client.emergency_stop()
            except TransportError:
                pass
            raise SafetyAbort(
                str(result.summary["metrics"]["hard_failures"][0]), samples
            )
        return result


def load_recording(path: Path) -> tuple[list[dict[str, int]], dict[str, Any]]:
    path = Path(path)
    csv_path = path if path.suffix == ".csv" else path.with_suffix(".csv")
    json_path = path if path.suffix == ".json" else path.with_suffix(".json")
    if path.suffix == ".json":
        csv_path = path.with_suffix(".csv")
    samples: list[dict[str, int]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.reader(handle):
            sample = parse_telemetry_line(",".join(row))
            if sample is not None:
                samples.append(sample)
    metadata = json.loads(json_path.read_text(encoding="utf-8"))
    return samples, metadata


@dataclass
class SearchResult:
    accepted: dict[str, float]
    metrics: dict[str, Any]
    rounds: int
    audit: list[dict[str, Any]] = field(default_factory=list)
    aborted: bool = False
    pending_rollback: bool = False
    abort_reason: str | None = None


class CoordinateSearch:
    """At most three rounds; every ordinary candidate changes exactly one key."""

    def __init__(
        self,
        client: M6Client,
        evaluator: Callable[[Mapping[str, float], str], Mapping[str, Any]],
    ) -> None:
        self.client = client
        self.evaluator = evaluator

    def _set_apply(self, values: Mapping[str, float]) -> None:
        for key, value in values.items():
            self.client.set(key, value)
        self.client.apply()

    def _rollback(self, accepted: Mapping[str, float]) -> bool:
        try:
            for key, value in accepted.items():
                self.client.set(key, value)
            self.client.apply()
            return False
        except (DeviceError, TransportError):
            return True

    def search(
        self,
        initial: Mapping[str, float],
        baseline_metrics: Mapping[str, Any],
        *,
        keys: Sequence[str],
        max_rounds: int = 3,
    ) -> SearchResult:
        if max_rounds < 1 or max_rounds > 3:
            raise ValueError("coordinate search is limited to one through three rounds")
        if len(set(keys)) != len(keys):
            raise ValueError("duplicate tuning key")
        accepted = {key: float(value) for key, value in initial.items()}
        best_metrics = dict(baseline_metrics)
        audit: list[dict[str, Any]] = []
        rounds_completed = 0
        try:
            for round_index in range(max_rounds):
                rounds_completed = round_index + 1
                round_improved = False
                for key in keys:
                    if key not in accepted or key not in PARAMETER_BOUNDS:
                        raise ValueError(f"unsupported tuning key: {key}")
                    origin = accepted[key]
                    step = max(abs(origin) * 0.10, MIN_STEPS.get(key, 0.01))
                    low, high = PARAMETER_BOUNDS[key]
                    for direction in (1.0, -1.0):
                        candidate_value = min(high, max(low, origin + direction * step))
                        if math.isclose(candidate_value, accepted[key], abs_tol=1e-9):
                            continue
                        candidate = dict(accepted)
                        candidate[key] = candidate_value
                        # A candidate differs from the accepted state in one key,
                        # therefore it cannot cross parameter-family boundaries.
                        self._set_apply({key: candidate_value})
                        metrics = dict(
                            self.evaluator(candidate, f"r{round_index + 1}-{key}-{candidate_value:g}")
                        )
                        feedforward_candidate = key in (
                            "wheel.left.ks",
                            "wheel.left.kv",
                            "wheel.right.ks",
                            "wheel.right.kv",
                        )
                        accepted_now, improvement = candidate_accepted(
                            best_metrics, metrics, feedforward=feedforward_candidate
                        )
                        audit.append(
                            {
                                "round": round_index + 1,
                                "key": key,
                                "value": candidate_value,
                                "improvement": improvement,
                                "accepted": accepted_now,
                                "hard_failures": metrics.get("hard_failures", []),
                            }
                        )
                        if metrics.get("hard_failures"):
                            raise SafetyAbort(str(metrics["hard_failures"][0]))
                        if accepted_now:
                            accepted = candidate
                            best_metrics = metrics
                            origin = candidate_value
                            round_improved = True
                        else:
                            self._set_apply({key: accepted[key]})
                if not round_improved:
                    break
        except (SafetyAbort, TransportError, DeviceError) as exc:
            try:
                self.client.emergency_stop()
            except TransportError:
                pass
            pending = self._rollback(accepted)
            return SearchResult(
                accepted=accepted,
                metrics=best_metrics,
                rounds=rounds_completed,
                audit=audit,
                aborted=True,
                pending_rollback=pending,
                abort_reason=str(exc),
            )
        return SearchResult(
            accepted=accepted,
            metrics=best_metrics,
            rounds=rounds_completed,
            audit=audit,
        )


def _response_json(payload: Sequence[Any]) -> str:
    return json.dumps(list(payload), ensure_ascii=False)


def _create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port; omitted means probe every port")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--firmware-prefix", default=DEFAULT_FIRMWARE_PREFIX)
    parser.add_argument("--firmware-id", help="optional exact build identity")
    parser.add_argument("--schema", type=int, default=PARAMETER_SCHEMA)
    parser.add_argument("--timeout", type=float, default=1.0)
    sub = parser.add_subparsers(dest="command", required=True)

    probe = sub.add_parser("probe", help="verify firmware identity and parameter schema")
    probe.add_argument("--json", action="store_true")
    get = sub.add_parser("get")
    get.add_argument("key")
    set_command = sub.add_parser("set")
    set_command.add_argument("key")
    set_command.add_argument("value")
    for name in ("apply", "revert", "load", "defaults", "save"):
        sub.add_parser(name)
    log = sub.add_parser("log")
    log.add_argument("mode", choices=("m6", "off"))
    log.add_argument("period_ms", type=int)
    record = sub.add_parser("record")
    record.add_argument("output", type=Path)
    record.add_argument("--duration", type=float, required=True)
    record.add_argument("--period", type=int, default=20)
    record.add_argument("--mode", choices=("wheel", "ground", "line"), required=True)
    record.add_argument("--speed", type=float, required=True)
    score = sub.add_parser("score")
    score.add_argument("recording", type=Path)
    score.add_argument("--baseline", type=Path)
    tune = sub.add_parser("tune-wheel")
    tune.add_argument("output_dir", type=Path)
    tune.add_argument("--speed", type=float, default=160.0)
    tune.add_argument("--duration", type=float, default=7.0)
    tune.add_argument("--period", type=int, default=20)
    tune.add_argument("--rounds", type=int, choices=(1, 2, 3), default=3)
    tune.add_argument(
        "--key",
        action="append",
        dest="keys",
        choices=tuple(MIN_STEPS),
        help="coordinate to tune; repeat as needed",
    )
    tune.add_argument("--confirm-lifted", action="store_true")
    tune.add_argument("--save-final", action="store_true")
    return parser


def _run_record_command(
    client: M6Client,
    identity: Mapping[str, Any],
    args: argparse.Namespace,
) -> RecordingResult:
    parameters = client.active_snapshot()
    recorder = TelemetryRecorder(client)
    return recorder.record(
        args.output,
        duration_s=args.duration,
        period_ms=args.period,
        test_mode=args.mode,
        requested_speed=args.speed,
        identity=identity,
        parameters=parameters,
    )


def _run_tune_wheel(
    client: M6Client,
    identity: Mapping[str, Any],
    args: argparse.Namespace,
) -> SearchResult:
    if not args.confirm_lifted:
        raise M6Error(
            "tune-wheel refused: raise both wheels and pass --confirm-lifted"
        )
    if args.duration < 7.0:
        raise M6Error("tune-wheel duration must be at least 7 s for run and ramp-down")
    if not 50.0 <= args.speed <= 400.0:
        raise M6Error("wheel diagnostic speed must be in [50, 400] mm/s")
    keys = args.keys or [
        "wheel.left.kp",
        "wheel.left.ki",
        "wheel.right.kp",
        "wheel.right.ki",
    ]
    complete_snapshot = client.active_snapshot()
    initial_raw = {key: complete_snapshot[key] for key in (*keys, "wheel.diag_target")}
    initial = {key: float(initial_raw[key]) for key in keys}
    client.set("wheel.diag_target", args.speed)
    client.apply()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    recorder = TelemetryRecorder(client)
    feedforward_keys = {
        "wheel.left.ks",
        "wheel.left.kv",
        "wheel.right.ks",
        "wheel.right.kv",
    }
    multi_speed_evidence = any(key in feedforward_keys for key in keys)

    def run_point(
        parameters: Mapping[str, float], label: str, speed: float
    ) -> Mapping[str, Any]:
        client.set("wheel.diag_target", speed)
        client.apply()
        current_identity = client.probe(
            args.firmware_prefix, args.schema, args.firmware_id
        )

        def start_action() -> None:
            client.legacy("b")
            time.sleep(0.25)
            client.legacy("A")

        result = recorder.record(
            args.output_dir / f"{label}-{int(speed)}",
            duration_s=args.duration,
            period_ms=args.period,
            test_mode="wheel",
            requested_speed=speed,
            identity=current_identity,
            parameters={
                **complete_snapshot,
                **parameters,
                "wheel.diag_target": speed,
            },
            start_action=start_action,
        )
        metrics = dict(result.summary["metrics"])
        metrics.update(
            {
                "firmware": current_identity["firmware"],
                "schema": current_identity["schema"],
                "test_mode": "wheel",
                "requested_speed": speed,
            }
        )
        return metrics

    def evaluate(parameters: Mapping[str, float], label: str) -> Mapping[str, Any]:
        if not multi_speed_evidence:
            return run_point(parameters, label, args.speed)
        point_metrics = [
            run_point(parameters, label, float(speed))
            for speed in (100, 160, 220, 300)
        ]
        aggregate = aggregate_validation_metrics(point_metrics)
        aggregate.update(
            {
                "firmware": point_metrics[0]["firmware"],
                "schema": point_metrics[0]["schema"],
                "test_mode": "wheel-feedforward",
                "requested_speed": "100/160/220/300",
                "validation_speeds": [100, 160, 220, 300],
            }
        )
        return aggregate

    try:
        baseline_metrics = dict(evaluate(initial, "baseline"))
    except (SafetyAbort, TransportError, DeviceError) as exc:
        try:
            client.emergency_stop()
        except TransportError:
            pass
        pending = False
        try:
            for key, value in initial.items():
                client.set(key, value)
            client.set("wheel.diag_target", initial_raw["wheel.diag_target"])
            client.apply()
        except (TransportError, DeviceError):
            pending = True
        return SearchResult(
            accepted=initial,
            metrics={"score": 1.0e9, "hard_failures": [str(exc)]},
            rounds=0,
            aborted=True,
            pending_rollback=pending,
            abort_reason=str(exc),
        )
    if baseline_metrics.get("hard_failures"):
        try:
            client.emergency_stop()
        except TransportError:
            pass
        return SearchResult(
            accepted=initial,
            metrics=baseline_metrics,
            rounds=0,
            aborted=True,
            pending_rollback=True,
            abort_reason=str(baseline_metrics["hard_failures"][0]),
        )
    search = CoordinateSearch(client, evaluate)
    result = search.search(
        initial,
        baseline_metrics,
        keys=keys,
        max_rounds=args.rounds,
    )
    restore_target = (
        float(initial_raw["wheel.diag_target"]) if result.aborted else float(args.speed)
    )
    try:
        client.set("wheel.diag_target", restore_target)
        client.apply()
    except (TransportError, DeviceError):
        if result.aborted:
            result.pending_rollback = True
        else:
            raise
    report_path = args.output_dir / "search.json"
    report_path.write_text(
        json.dumps(result.__dict__, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    if args.save_final:
        if result.aborted or result.metrics.get("hard_failures"):
            raise M6Error("refusing --save-final because validation did not pass")
        client.save()
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = _create_parser()
    args = parser.parse_args(argv)
    client: M6Client | None = None
    try:
        if args.command == "score":
            samples, metadata = load_recording(args.recording)
            metrics = score_samples(
                samples,
                requested_speed=float(metadata["requested_speed"]),
                mode=str(metadata["test_mode"]),
                expected_period_ms=int(metadata.get("period_ms", 20)),
            )
            output: dict[str, Any] = {"metrics": metrics}
            if args.baseline:
                _, baseline_metadata = load_recording(args.baseline)
                baseline_metrics = baseline_metadata["metrics"]
                comparable = evidence_compatible(baseline_metadata, metadata)
                accepted, improvement = (
                    candidate_accepted(baseline_metrics, metrics)
                    if comparable
                    else (False, -1.0)
                )
                output.update(
                    {
                        "comparable": comparable,
                        "accepted": accepted,
                        "improvement": improvement,
                    }
                )
            print(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False))
            return 0
        client, identity = discover_client(
            port=args.port,
            baud=args.baud,
            firmware_prefix=args.firmware_prefix,
            firmware_id=args.firmware_id,
            schema=args.schema,
            timeout=args.timeout,
        )
        if args.command == "probe":
            if args.json:
                print(json.dumps(identity, ensure_ascii=False, sort_keys=True))
            else:
                display_identity = dict(identity)
                display_identity.setdefault("generation", "unknown")
                print(
                    "port={port} firmware={firmware} schema={schema} revision={revision} "
                    "source={source} generation={generation}".format(**display_identity)
                )
        elif args.command == "get":
            print(_response_json(client.get(args.key)))
        elif args.command == "set":
            print(_response_json(client.set(args.key, args.value)))
        elif args.command in ("apply", "revert", "load", "defaults", "save"):
            print(_response_json(getattr(client, args.command)()))
        elif args.command == "log":
            print(_response_json(client.log(args.mode, args.period_ms)))
        elif args.command == "record":
            result = _run_record_command(client, identity, args)
            print(result.json_path)
        elif args.command == "tune-wheel":
            result = _run_tune_wheel(client, identity, args)
            print(json.dumps(result.__dict__, indent=2, ensure_ascii=False, allow_nan=False))
            return 3 if result.aborted else 0
        return 0
    except SafetyAbort as exc:
        print(f"M6 safety abort: {exc.reason}; rollback is pending until safe IDLE", file=sys.stderr)
        return 3
    except (M6Error, ValueError, OSError, KeyError) as exc:
        print(f"M6 error: {exc}", file=sys.stderr)
        return 2
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
