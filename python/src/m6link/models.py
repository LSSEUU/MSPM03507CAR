"""Public immutable data models shared by host-side consumers."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

from .registry import (
    ActionKind,
    ActionReason,
    ActionState,
    Capability,
    ParameterSource,
    ParameterType,
    TelemetryStream,
)


@dataclass(frozen=True, slots=True)
class Identity:
    firmware_id: str
    wire_version: int
    registry_revision: int
    parameter_schema: int
    active_revision: int
    stage_revision: int
    generation: int
    boot_id: int
    source: ParameterSource | int
    capabilities: Capability
    enabled_capabilities: Capability = Capability(0)
    image_flags: int = 0
    port: str | None = None

    @property
    def firmware(self) -> str:
        return self.firmware_id

    @property
    def schema(self) -> int:
        return self.parameter_schema

    @property
    def revision(self) -> int:
        return self.active_revision

    def as_legacy_dict(self) -> dict[str, Any]:
        return {
            "port": self.port,
            "firmware": self.firmware_id,
            "schema": self.parameter_schema,
            "revision": self.active_revision,
            "source": getattr(self.source, "name", self.source).lower()
            if isinstance(getattr(self.source, "name", None), str)
            else self.source,
            "generation": self.generation,
        }


DeviceIdentity = Identity


@dataclass(frozen=True, slots=True)
class Status:
    time_ms: int
    system_state: int
    action_state: ActionState | int
    action_kind: ActionKind | int
    fault_bits: int
    first_fault_ms: int
    last_fault_ms: int
    fault_count: int
    required_sensor_mask: int
    action_deadline_ms: int
    requested_pwm: tuple[int, int]
    applied_pwm: tuple[int, int]
    target_mm_s: tuple[int, int]
    measured_mm_s: tuple[int, int]
    active_revision: int
    stage_revision: int
    source: ParameterSource | int
    flags: int = 0


@dataclass(frozen=True, slots=True)
class Breadcrumb:
    checkpoint: int
    boot_id: int
    time_ms: int
    last_wwdt_ms: int
    miss_total: int
    fault_bits: int
    uart_tx_drop: int
    uart_rx_drop: int
    telemetry_drop: int
    pending_mask: int
    missed_mask: int
    detail_checkpoint: int
    system_state: int
    telemetry_queue: int
    health_mask: int


@dataclass(frozen=True, slots=True)
class ResetInfo:
    raw_cause: int
    reset_class: int
    current_checkpoint: int
    boot_id: int
    flags: int
    previous: Breadcrumb
    hardfault_pc: int
    hardfault_lr: int
    hardfault_xpsr: int


@dataclass(frozen=True, slots=True)
class TaskStats:
    task_count: int
    scheduler_healthy: bool
    pending_mask: int
    missed_mask: int
    health_mask: int
    release_count: tuple[int, ...]
    completion_count: tuple[int, ...]
    miss_count: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class UartStats:
    tx_bytes: int
    rx_bytes: int
    format_count: int
    tx_drop: int
    rx_drop: int
    emergency_epoch: int
    rx_queued: int
    rx_high_water: int
    tx_queued: int
    tx_high_water: int


@dataclass(frozen=True, slots=True)
class LinkStats:
    mode: int
    control_window: int
    replay_used: int
    flags: int
    session: int
    next_host_sequence: int
    next_async_sequence: int
    rx_frames: int
    tx_frames: int
    crc_errors: int
    frame_errors: int
    timeout_errors: int
    session_mismatch: int
    sequence_errors: int
    replay_count: int
    busy_count: int
    drops: tuple[int, int, int, int]
    control_error_drop: int
    high_water: tuple[int, int, int, int]


@dataclass(frozen=True, slots=True)
class ParameterDescriptor:
    parameter_id: int
    key: str
    type: ParameterType | int
    flags: int
    minimum: float | int | bool
    maximum: float | int | bool
    default: float | int | bool
    domain_rule: int
    relation_group: int
    unit: int


@dataclass(frozen=True, slots=True)
class ParameterValue:
    parameter_id: int
    key: str
    type: ParameterType | int
    active: float | int | bool
    staged: float | int | bool
    active_source: ParameterSource | int
    staged_source: ParameterSource | int
    active_revision: int
    stage_revision: int
    generation: int


@dataclass(frozen=True, slots=True)
class ParameterSnapshot:
    parameter_schema: int
    active_revision: int
    stage_revision: int
    generation: int
    active_source: ParameterSource | int
    values: Mapping[str, ParameterValue] = field(default_factory=dict)
    flags: int = 0
    slot: int = 0
    storage_health: int = 0
    save_failures: int = 0


@dataclass(frozen=True, slots=True)
class ActionSpec:
    kind: ActionKind | int
    wheel_mask: int = 3
    target_l_mm_s: int = 0
    target_r_mm_s: int = 0
    angle_mdeg: int = 0
    yaw_rate_mdeg_s: int = 0
    duration_ms: int = 1500
    expected_active_revision: int | None = None


@dataclass(frozen=True, slots=True)
class ArmedAction:
    token: int
    expires_ms: int
    spec: ActionSpec
    armed_at: float

    @property
    def expires_at(self) -> float:
        return self.armed_at + 10.0


@dataclass(frozen=True, slots=True)
class ActionStatus:
    action_id: int
    state: ActionState | int
    kind: ActionKind | int = 0
    reason: ActionReason | int = 0
    start_ms: int = 0
    deadline_ms: int = 0
    stopped_ms: int = 0


@dataclass(frozen=True, slots=True)
class Subscription:
    stream: TelemetryStream | int
    period_ms: int
    flags: int = 0


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    stream: TelemetryStream | int
    sample_id: int
    time_ms: int
    values: Mapping[str, Any]
    gap_before: bool = False
    sequence: int = 0
    wire_gap_before: bool = False
    sequence_gap_before: bool = False
    expected_sequence: int = 0


@dataclass(frozen=True, slots=True)
class LinkEvent:
    code: int
    severity: int
    time_ms: int
    occurrence_count: int
    values: Mapping[str, Any]
    gap_before: bool = False
    sequence: int = 0
    wire_gap_before: bool = False
    sequence_gap_before: bool = False
    expected_sequence: int = 0


@dataclass(frozen=True, slots=True)
class FaultEvent(LinkEvent):
    pass


@dataclass(frozen=True, slots=True)
class ResetEvent(LinkEvent):
    pass


@dataclass(frozen=True, slots=True)
class GapEvent(LinkEvent):
    pass


@dataclass(frozen=True, slots=True)
class RecordingManifest:
    firmware: str | None
    schema: int | None
    revision: int | None
    source: str | int | None
    generation: int | None
    test_mode: str
    requested_speed: float
    period_ms: int
    start_time: str
    end_time: str
    parameters: Mapping[str, Any]
    events: tuple[Mapping[str, Any], ...]
    abort_reason: str | None


@dataclass(frozen=True, slots=True)
class ScoreReport:
    metrics: Mapping[str, Any]
    comparable: bool | None = None
    accepted: bool | None = None
    improvement: float | None = None


@dataclass(frozen=True, slots=True)
class RecordingResult:
    csv_path: Path
    json_path: Path
    samples: list[dict[str, int]]
    summary: dict[str, Any]
