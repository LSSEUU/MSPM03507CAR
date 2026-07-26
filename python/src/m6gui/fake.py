"""Deterministic fake ``m6link.Session`` for GUI development without a car."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from enum import IntEnum, IntFlag
import json
import math
from pathlib import Path
import time
from typing import Any, Mapping


class Capability(IntFlag):
    WIRE_R1 = 1 << 0
    RAW_ESTOP_S_SPACE = 1 << 1
    SESSION_DEDUP = 1 << 2
    PRIORITY_TX = 1 << 3
    RESET_DIAG = 1 << 4
    HEALTH_STATS = 1 << 5
    PARAM_READ = 1 << 6
    PARAM_STAGE = 1 << 7
    PARAM_BOUNDARY_APPLY = 1 << 8
    PARAM_PERSIST = 1 << 9
    TELEMETRY_SUBSCRIBE = 1 << 10
    ACTION_TWO_PHASE = 1 << 11
    LEGACY_V1 = 1 << 13
    HANDSHAKE_RESYNC = 1 << 14
    FAULT_CLEAR = 1 << 15
    WHEEL_DIAG_ACTION = 1 << 16
    HEADING_ACTION = 1 << 17
    HEADING_CALIBRATION = 1 << 20
    CONFIGURABLE_TURN = 1 << 22


FAKE_CAPABILITIES = (
    Capability.WIRE_R1
    | Capability.RAW_ESTOP_S_SPACE
    | Capability.SESSION_DEDUP
    | Capability.PRIORITY_TX
    | Capability.RESET_DIAG
    | Capability.HEALTH_STATS
    | Capability.PARAM_READ
    | Capability.PARAM_STAGE
    | Capability.PARAM_BOUNDARY_APPLY
    | Capability.PARAM_PERSIST
    | Capability.TELEMETRY_SUBSCRIBE
    | Capability.ACTION_TWO_PHASE
    | Capability.LEGACY_V1
    | Capability.HANDSHAKE_RESYNC
    | Capability.FAULT_CLEAR
    | Capability.WHEEL_DIAG_ACTION
    | Capability.HEADING_ACTION
    | Capability.HEADING_CALIBRATION
    | Capability.CONFIGURABLE_TURN
)


class ActionKind(IntEnum):
    NONE = 0
    WHEEL_SPEED_TEST = 1
    HEADING_HOLD = 2
    HEADING_TURN = 3


@dataclass(frozen=True)
class ActionSpec:
    kind: ActionKind | str
    wheel_mask: int = 0
    target_l_mm_s: int = 0
    target_r_mm_s: int = 0
    angle_mdeg: int = 0
    yaw_rate_mdeg_s: int = 0
    duration_ms: int = 0
    expected_active_revision: int = 0


@dataclass(frozen=True)
class ArmedAction:
    token: int
    expires_in_ms: int = 10_000


@dataclass(frozen=True)
class Identity:
    firmware_id: str
    wire_version: int
    registry_revision: int
    parameter_schema: int
    active_revision: int
    parameter_source: str
    boot_id: int
    enabled_capabilities: Capability


@dataclass(frozen=True)
class TelemetrySample:
    time_ms: int
    speed_l_mm_s: int
    speed_r_mm_s: int
    yaw_cdeg: int
    yaw_rate_cdeg_s: int
    adc: tuple[int, ...]
    position_x1000: int


@dataclass(frozen=True)
class ParameterValue:
    active: float
    staged: float
    persisted: float
    minimum: float
    maximum: float
    unit: str
    writable: bool = True


class FakeSession:
    """A fake that records all safety-relevant calls for assertions."""

    def __init__(
        self,
        port: str | None = None,
        *,
        baudrate: int = 115_200,
        legacy: bool = False,
        serial_factory: Any = None,
        timeout: float = 1.0,
        clock: Any = time.monotonic,
    ) -> None:
        del serial_factory, timeout
        self.port = port or "FAKE0"
        self.baudrate = int(baudrate)
        self.legacy = bool(legacy)
        self.clock = clock
        self.mode = "V1" if legacy else "V2_ESTABLISHED"
        self.capabilities = Capability.LEGACY_V1 if legacy else FAKE_CAPABILITIES
        self.identity: Identity | None = None
        self.status: dict[str, Any] = {}
        self.calls: list[tuple[Any, ...]] = []
        self.closed = False
        self._start = clock()
        self._last_sample = -math.inf
        self._sample_id = 0
        self._token = 1000
        self._armed: ActionSpec | Any | None = None
        self._recording_path: Path | None = None
        self._recorded: list[dict[str, Any]] = []
        self._parameters = {
            "wheel.left.kp": ParameterValue(0.70, 0.70, 0.70, 0.0, 3.0, "gain"),
            "wheel.left.ki": ParameterValue(2.50, 2.50, 2.50, 0.0, 10.0, "gain"),
            "wheel.right.kp": ParameterValue(0.70, 0.70, 0.70, 0.0, 3.0, "gain"),
            "heading.straight.kp": ParameterValue(3.0, 3.0, 3.0, 0.0, 10.0, "gain"),
            "heading.turn_speed": ParameterValue(150, 150, 150, 50, 400, "mm/s"),
        }
        FakeSession.last_created = self

    last_created: "FakeSession | None" = None

    @staticmethod
    def discover_ports() -> list[str]:
        return ["FAKE0", "FAKE1"]

    def connect(self, port: str | None = None, *, legacy: bool | None = None) -> Identity:
        if port:
            self.port = port
        if legacy is not None:
            self.legacy = bool(legacy)
            self.mode = "V1" if self.legacy else "V2_ESTABLISHED"
            self.capabilities = Capability.LEGACY_V1 if self.legacy else FAKE_CAPABILITIES
        self.identity = Identity(
            firmware_id="M6-runtime-20260726-r14-fake",
            wire_version=1 if self.legacy else 2,
            registry_revision=1,
            parameter_schema=1,
            active_revision=12,
            parameter_source="defaults",
            boot_id=42,
            enabled_capabilities=self.capabilities,
        )
        self.status = {
            "system_state": "IDLE",
            "action_state": "IDLE",
            "fault_bits": 0,
            "active_revision": 12,
            "source": "defaults",
            "dirty": False,
        }
        self.calls.append(("connect", self.port, self.legacy))
        return self.identity

    def close(self) -> None:
        self.closed = True
        self.calls.append(("close",))

    def get_identity(self) -> Identity | None:
        return self.identity

    def get_status(self) -> Mapping[str, Any]:
        return dict(self.status)

    def get_reset_info(self) -> Mapping[str, Any]:
        return {
            "raw_cause": 0x0C,
            "reset_class": "external_boot",
            "boot_id": 42,
            "breadcrumb": "IDLE/none",
            "previous_checkpoint": "main_loop",
        }

    def get_task_stats(self) -> Mapping[str, Any]:
        return {"miss_5ms": 0, "miss_10ms": 0, "miss_20ms": 0, "miss_100ms": 0}

    def get_uart_stats(self) -> Mapping[str, Any]:
        return {"rx_dropped": 0, "tx_dropped": 0, "rx_high_water": 4}

    def get_link_stats(self) -> Mapping[str, Any]:
        return {
            "mode": self.mode,
            "p0_drop": 0,
            "p1_drop": 0,
            "p2_drop": 0,
            "p3_drop": 0,
            "telemetry_dropped": 0,
        }

    def subscribe(self, entries: Any) -> None:
        self.calls.append(("subscribe", entries))

    def read_event(self, timeout: float | None = None) -> TelemetrySample | None:
        del timeout
        now = self.clock()
        if now - self._last_sample < 0.04:
            return None
        self._last_sample = now
        self._sample_id += 1
        elapsed = now - self._start
        left = int(round(180.0 + 40.0 * math.sin(elapsed * 2.4)))
        right = int(round(180.0 + 35.0 * math.sin(elapsed * 2.4 + 0.2)))
        centroid = 1.8 * math.sin(elapsed * 1.3)
        adc = tuple(
            int(450 + 2600 * math.exp(-((index - 3.5 - centroid) ** 2) / 1.3))
            for index in range(8)
        )
        sample = TelemetrySample(
            time_ms=int(elapsed * 1000),
            speed_l_mm_s=left,
            speed_r_mm_s=right,
            yaw_cdeg=int(2000 * math.sin(elapsed * 0.25)),
            yaw_rate_cdeg_s=int(500 * math.cos(elapsed * 0.25)),
            adc=adc,
            position_x1000=int(centroid * 1000),
        )
        if self._recording_path is not None:
            self._recorded.append(asdict(sample))
        return sample

    def get_parameters(self) -> Mapping[str, ParameterValue]:
        return dict(self._parameters)

    def stage_parameter(self, key: str, value: Any) -> ParameterValue:
        old = self._parameters[key]
        number = float(value)
        if not old.minimum <= number <= old.maximum:
            raise ValueError(f"{key} must be within {old.minimum}..{old.maximum}")
        updated = ParameterValue(
            old.active,
            number,
            old.persisted,
            old.minimum,
            old.maximum,
            old.unit,
            old.writable,
        )
        self._parameters[key] = updated
        self.calls.append(("stage_parameter", key, number))
        return updated

    def apply_parameters(self) -> None:
        self._parameters = {
            key: ParameterValue(
                value.staged,
                value.staged,
                value.persisted,
                value.minimum,
                value.maximum,
                value.unit,
                value.writable,
            )
            for key, value in self._parameters.items()
        }
        self.calls.append(("apply_parameters",))

    def revert_parameters(self) -> None:
        self._parameters = {
            key: ParameterValue(
                value.active,
                value.active,
                value.persisted,
                value.minimum,
                value.maximum,
                value.unit,
                value.writable,
            )
            for key, value in self._parameters.items()
        }
        self.calls.append(("revert_parameters",))

    def load_parameters(self) -> None:
        self._parameters = {
            key: ParameterValue(
                value.persisted,
                value.persisted,
                value.persisted,
                value.minimum,
                value.maximum,
                value.unit,
                value.writable,
            )
            for key, value in self._parameters.items()
        }
        self.calls.append(("load_parameters",))

    def default_parameters(self) -> None:
        self.calls.append(("default_parameters",))

    def save_parameters(self) -> None:
        self._parameters = {
            key: ParameterValue(
                value.active,
                value.staged,
                value.active,
                value.minimum,
                value.maximum,
                value.unit,
                value.writable,
            )
            for key, value in self._parameters.items()
        }
        self.calls.append(("save_parameters",))

    def arm_action(self, spec: ActionSpec | Any) -> ArmedAction:
        self._token += 1
        self._armed = spec
        self.calls.append(("arm_action", spec))
        return ArmedAction(self._token)

    def confirm_action(self, token: int) -> Mapping[str, Any]:
        if self._armed is None:
            raise RuntimeError("not armed")
        self.calls.append(("confirm_action", int(token)))
        spec, self._armed = self._armed, None
        self.status = {**self.status, "action_state": "RUNNING"}
        return {"state": "RUNNING", "token": int(token), "kind": str(spec.kind)}

    def cancel_action(self, action_id: int = 0) -> Mapping[str, Any]:
        self.calls.append(("cancel_action", int(action_id)))
        self._armed = None
        self.status = {**self.status, "action_state": "IDLE"}
        return {"state": "IDLE", "reason": "HOST_CANCEL"}

    def emergency_stop(self) -> Mapping[str, Any]:
        self.calls.append(("emergency_stop",))
        self._armed = None
        self.status = {**self.status, "system_state": "FAULT", "fault_bits": 1}
        return {"state": "FAULT", "reason": "EMERGENCY"}

    def start_recording(self, path: str) -> None:
        self._recording_path = Path(path)
        self._recorded = []
        self.calls.append(("start_recording", str(path)))

    def stop_recording(self) -> Path | None:
        path, self._recording_path = self._recording_path, None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.suffix.lower() == ".json":
                path.write_text(json.dumps(self._recorded, ensure_ascii=False, indent=2), encoding="utf-8")
            else:
                with path.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(TelemetrySample.__dataclass_fields__))
                    writer.writeheader()
                    for row in self._recorded:
                        encoded = dict(row)
                        encoded["adc"] = ";".join(str(item) for item in encoded["adc"])
                        writer.writerow(encoded)
        self.calls.append(("stop_recording",))
        return path

    def replay(self, path: str) -> Mapping[str, Any]:
        data = load_recording(path)
        self.calls.append(("replay", str(path)))
        return {"path": str(path), "samples": data, "sample_count": len(data)}

    def score(self, path: str) -> Mapping[str, Any]:
        score = score_samples(load_recording(path))
        self.calls.append(("score", str(path)))
        return score


Session = FakeSession


def discover_ports() -> list[str]:
    return FakeSession.discover_ports()


def load_recording(path: str | Path) -> list[dict[str, Any]]:
    source = Path(path)
    if not source.exists():
        return []
    if source.suffix.lower() == ".json":
        data = json.loads(source.read_text(encoding="utf-8"))
        return list(data) if isinstance(data, list) else []
    with source.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def score_samples(samples: list[Mapping[str, Any]]) -> Mapping[str, Any]:
    if not samples:
        return {"sample_count": 0, "score": 0.0, "summary": "无可评分样本"}
    errors: list[float] = []
    for row in samples:
        try:
            errors.append(abs(float(row["speed_l_mm_s"]) - float(row["speed_r_mm_s"])))
        except (KeyError, TypeError, ValueError):
            continue
    mean_error = sum(errors) / len(errors) if errors else 0.0
    score = max(0.0, 100.0 - mean_error)
    return {
        "sample_count": len(samples),
        "mean_wheel_delta_mm_s": round(mean_error, 3),
        "score": round(score, 1),
        "summary": "fake 平均左右轮速差评分",
    }
