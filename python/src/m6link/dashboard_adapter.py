"""Convert asynchronous dashboard sessions into deterministic score evidence."""

from __future__ import annotations

import csv
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from .compat import TELEMETRY_FIELDS, TELEMETRY_HEADER, TELEMETRY_PREFIX
from .dashboard_recording import FORMAT_VERSION
from .models import RecordingResult, TelemetrySample
from .registry import ActionReason, ActionState, TelemetryStream


_STREAMS = {
    "core": TelemetryStream.CORE,
    "wheel": TelemetryStream.WHEEL,
    "line": TelemetryStream.LINE,
    "health": TelemetryStream.HEALTH,
}
_STREAM_ORDER = {"health": 0, "line": 1, "wheel": 2, "core": 3}
_ACTION_MODES = {
    "WHEEL_SPEED_TEST": "wheel",
    "HEADING_HOLD": "ground",
    "HEADING_TURN": "ground",
    "HEADING_RATE": "ground",
    "LINE_FOLLOW": "line",
}


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _enum_name(value: Any) -> str | None:
    if isinstance(value, Mapping):
        name = value.get("name")
        return str(name) if name is not None else None
    return str(value) if isinstance(value, str) else None


def _append_failure(failures: list[str], reason: str) -> None:
    if reason and reason not in failures:
        failures.append(reason)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"dashboard recording file is unreadable: {path.name}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"dashboard recording file must contain an object: {path.name}")
    return value


def _read_timeline(path: Path, failures: list[str]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    previous_index = 0
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ValueError("dashboard timeline is unreadable") from exc
    for line in lines:
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            _append_failure(failures, "malformed_timeline_json")
            continue
        if not isinstance(record, dict):
            _append_failure(failures, "malformed_timeline_record")
            continue
        try:
            record_index = int(record["record_index"])
        except (KeyError, TypeError, ValueError):
            _append_failure(failures, "malformed_timeline_record")
            continue
        if (not records and record_index != 1) or (
            previous_index and record_index != previous_index + 1
        ):
            _append_failure(failures, "timeline_record_gap")
        if record_index <= previous_index:
            _append_failure(failures, "timeline_record_order")
        previous_index = max(previous_index, record_index)
        records.append(record)
    return records


def _action_context(
    manifest: Mapping[str, Any], records: Sequence[Mapping[str, Any]]
) -> tuple[str | None, Mapping[str, Any], list[str]]:
    failures: list[str] = []
    action = _mapping(manifest.get("action"))
    spec = _mapping(action.get("spec"))
    kind_name = _enum_name(spec.get("kind")) or _enum_name(action.get("kind"))
    arm_request: Mapping[str, Any] = {}
    for record in records:
        if record.get("record_type") != "dashboard_event" or record.get("event") != "control_arm":
            continue
        candidate = _mapping(_mapping(record.get("values")).get("request"))
        if candidate:
            arm_request = candidate
    if not spec and arm_request:
        spec = arm_request
    if kind_name is None:
        kind_name = _enum_name(spec.get("kind")) or _enum_name(arm_request.get("kind"))
    if manifest.get("trigger") == "action" and not spec:
        _append_failure(failures, "missing_action_spec")
    return kind_name, spec, failures


def _derive_mode_and_speed(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    *,
    test_mode: str | None,
    requested_speed: float | None,
) -> tuple[str, float, Mapping[str, Any], list[str]]:
    kind_name, spec, failures = _action_context(manifest, records)
    mode = str(test_mode) if test_mode is not None else _ACTION_MODES.get(kind_name or "", "")
    if mode not in {"wheel", "ground", "line"}:
        raise ValueError("cannot infer test mode; pass --mode wheel, ground, or line")
    if requested_speed is not None:
        speed = float(requested_speed)
    elif mode == "line" or kind_name in {"HEADING_TURN", "HEADING_RATE"}:
        speed = 0.0
    else:
        targets = []
        for key in ("target_l_mm_s", "target_r_mm_s"):
            try:
                target = abs(float(spec.get(key, 0)))
            except (TypeError, ValueError):
                target = 0.0
            if target > 0.0:
                targets.append(target)
        speed = max(targets, default=0.0)
        if len({round(value, 6) for value in targets}) > 1:
            _append_failure(failures, "asymmetric_requested_speed")
    if mode != "line" and speed <= 0.0:
        _append_failure(failures, "missing_requested_speed")
    return mode, speed, spec, failures


def _signed_u32_delta(value: int, origin: int) -> int:
    return ((int(value) - int(origin) + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def _record_device_time(record: Mapping[str, Any]) -> int | None:
    raw_time = record.get("device_time_ms")
    if raw_time is None:
        raw_time = _mapping(record.get("values")).get("time_ms")
    try:
        return int(raw_time)
    except (TypeError, ValueError):
        return None


def _telemetry_sort_key(record: Mapping[str, Any], origin: int) -> tuple[int, int, int]:
    raw_time = _record_device_time(record)
    stream = str(record.get("stream", ""))
    return (
        _signed_u32_delta(raw_time if raw_time is not None else origin, origin),
        _STREAM_ORDER.get(stream, 4),
        int(record.get("record_index", 0)),
    )


def _event_failures(
    manifest: Mapping[str, Any],
    summary: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
) -> list[str]:
    failures: list[str] = []
    stop_reason = str(manifest.get("stop_reason") or summary.get("stop_reason") or "")
    if not bool(manifest.get("complete", False)) or manifest.get("status") != "complete":
        _append_failure(failures, f"recording_incomplete:{stop_reason or 'unknown'}")
    if manifest.get("trigger") == "action" and stop_reason != "action_complete":
        _append_failure(failures, stop_reason or "action_not_complete")
    try:
        if int(_mapping(manifest.get("gaps")).get("total", 0)) > 0:
            _append_failure(failures, "telemetry_gap")
    except (TypeError, ValueError):
        _append_failure(failures, "malformed_gap_summary")
    for record in records:
        if record.get("gap") is True:
            _append_failure(failures, "telemetry_gap")
        event = str(record.get("event") or "")
        values = _mapping(record.get("values"))
        if event == "TELEMETRY_GAP":
            _append_failure(failures, "telemetry_gap")
        elif event == "RESET":
            _append_failure(failures, "device_restarted")
        elif event == "identity_changed":
            previous_boot = values.get("previous_boot_id")
            current_boot = values.get("boot_id")
            if previous_boot is not None and current_boot is not None and previous_boot != current_boot:
                _append_failure(failures, "device_restarted")
        elif event == "SAFETY_FAULT":
            try:
                bits = int(values.get("fault_bits", values.get("new_fault_bits", 0)))
            except (TypeError, ValueError):
                bits = 0
            _append_failure(failures, f"firmware_fault_0x{bits:X}" if bits else "firmware_fault")
        elif event in {"EMERGENCY_STOP", "emergency_stop_requested"}:
            _append_failure(failures, "operator_emergency_stop")
        elif event == "STORAGE_FAULT":
            _append_failure(failures, "storage_fault")
        elif event == "connection_lost":
            _append_failure(failures, "connection_lost")
        elif event == "control_cancel":
            _append_failure(failures, "action_cancelled")
        elif event == "ACTION_STATE":
            try:
                state = ActionState(int(values.get("state", -1)))
            except (TypeError, ValueError):
                state = None
            if state == ActionState.REJECTED:
                _append_failure(failures, "action_rejected")
            elif state == ActionState.FAULT:
                _append_failure(failures, "action_fault")
            elif state == ActionState.COMPLETE:
                try:
                    reason = ActionReason(int(values.get("reason", -1)))
                except (TypeError, ValueError):
                    reason = None
                if reason != ActionReason.COMPLETE:
                    name = reason.name.lower() if reason is not None else "unknown"
                    _append_failure(failures, f"action_complete_reason:{name}")
    return failures


def _merge_telemetry(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    failures: list[str],
) -> list[dict[str, int]]:
    from .recording import _empty_legacy_row, _merge_v2_sample

    telemetry = [
        record
        for record in records
        if record.get("record_type") == "telemetry" and str(record.get("stream")) in _STREAMS
    ]
    if not telemetry:
        _append_failure(failures, "telemetry_missing")
        return []
    first_time = next(
        (value for value in (_record_device_time(record) for record in telemetry) if value is not None),
        None,
    )
    if first_time is None:
        _append_failure(failures, "telemetry_time_missing")
        return []
    origin = int(first_time)
    telemetry.sort(key=lambda record: _telemetry_sort_key(record, origin))

    required = {
        name
        for name in _STREAMS
        if _mapping(_mapping(manifest.get("sampling")).get(name)).get("period_ms") is not None
    }
    if not required:
        required = set(_STREAMS)
    seen: set[str] = set()
    last_sample_id: dict[str, int] = {}
    state = _empty_legacy_row()
    rows: list[dict[str, int]] = []
    first_emitted_time: int | None = None
    for record in telemetry:
        stream_name = str(record.get("stream"))
        values = _mapping(record.get("values"))
        raw_time = _record_device_time(record)
        try:
            device_time = int(raw_time)
            sample_id = int(record["sample_id"])
        except (KeyError, TypeError, ValueError):
            _append_failure(failures, "malformed_dashboard_telemetry")
            continue
        if "time_ms" in values:
            try:
                if int(values["time_ms"]) != device_time:
                    _append_failure(failures, "telemetry_time_mismatch")
            except (TypeError, ValueError):
                _append_failure(failures, "malformed_dashboard_telemetry")
                continue
        previous = last_sample_id.get(stream_name)
        if previous is not None:
            delta = (sample_id - previous) & 0xFFFFFFFF
            if delta == 0 or delta > 0x80000000:
                _append_failure(failures, "telemetry_sample_order")
            elif delta > 1:
                _append_failure(failures, "telemetry_gap")
        last_sample_id[stream_name] = sample_id
        seen.add(stream_name)
        try:
            emits_row = _merge_v2_sample(
                state,
                TelemetrySample(
                    stream=_STREAMS[stream_name],
                    sample_id=sample_id,
                    time_ms=device_time,
                    values=values,
                    gap_before=bool(record.get("gap", False)),
                ),
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            _append_failure(failures, "malformed_dashboard_telemetry")
            continue
        if emits_row and required <= seen:
            relative_time = _signed_u32_delta(device_time, origin)
            if first_emitted_time is None:
                first_emitted_time = relative_time
            state["time_ms"] = max(0, relative_time - first_emitted_time)
            rows.append(dict(state))
    missing = sorted(required - seen)
    for stream_name in missing:
        _append_failure(failures, f"telemetry_stream_missing:{stream_name}")
    if not rows:
        _append_failure(failures, "telemetry_missing")
    return rows


def _identity_metadata(manifest: Mapping[str, Any]) -> dict[str, Any]:
    identity = _mapping(manifest.get("firmware_identity"))
    return {
        "firmware": identity.get("firmware_id", identity.get("firmware")),
        "schema": identity.get("parameter_schema", identity.get("schema")),
        "revision": identity.get("active_revision", identity.get("revision")),
        "source": _enum_name(identity.get("source")) or identity.get("source"),
        "generation": identity.get("generation"),
        "boot_id": manifest.get("boot_id", identity.get("boot_id")),
    }


def load_dashboard_recording(
    path: Path,
    *,
    test_mode: str | None = None,
    requested_speed: float | None = None,
) -> tuple[list[dict[str, int]], dict[str, Any]]:
    """Load and merge one ``m6-dashboard-session-v1`` directory."""

    directory = Path(path).resolve()
    manifest = _read_json(directory / "manifest.json")
    if manifest.get("format_version") != FORMAT_VERSION:
        raise ValueError("unsupported dashboard recording format")
    summary_path = directory / "summary.json"
    summary = _read_json(summary_path) if summary_path.exists() else {}
    failures: list[str] = []
    records = _read_timeline(directory / "timeline.jsonl", failures)
    mode, speed, spec, context_failures = _derive_mode_and_speed(
        manifest,
        records,
        test_mode=test_mode,
        requested_speed=requested_speed,
    )
    for reason in context_failures:
        _append_failure(failures, reason)
    for reason in _event_failures(manifest, summary, records):
        _append_failure(failures, reason)
    rows = _merge_telemetry(manifest, records, failures)
    identity = _identity_metadata(manifest)
    parameters = _mapping(manifest.get("parameters"))
    metadata: dict[str, Any] = {
        **identity,
        "format_version": "m6-dashboard-score-adapter-v1",
        "source_format": FORMAT_VERSION,
        "source_session": str(directory),
        "session_id": manifest.get("session_id", directory.name),
        "test_mode": mode,
        "requested_speed": speed,
        "period_ms": int(
            _mapping(_mapping(manifest.get("sampling")).get("core")).get("period_ms") or 20
        ),
        "parameters": _mapping(parameters.get("active")) or parameters,
        "action_spec": dict(spec),
        "events": [record for record in records if record.get("record_type") != "telemetry"],
        "adapter_hard_failures": failures,
        "abort_reason": None,
    }
    from .recording import score_loaded_recording

    metadata["metrics"] = score_loaded_recording(rows, metadata)
    return rows, metadata


def convert_dashboard_recording(
    path: Path,
    output_base: Path,
    *,
    test_mode: str | None = None,
    requested_speed: float | None = None,
) -> RecordingResult:
    """Write adapter rows and metadata without mutating the source session."""

    rows, metadata = load_dashboard_recording(
        path,
        test_mode=test_mode,
        requested_speed=requested_speed,
    )
    target = Path(output_base)
    if target.suffix in {".csv", ".json"}:
        target = target.with_suffix("")
    target.parent.mkdir(parents=True, exist_ok=True)
    csv_path = target.with_suffix(".csv")
    json_path = target.with_suffix(".json")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(TELEMETRY_HEADER)
        for row in rows:
            writer.writerow([TELEMETRY_PREFIX] + [row[key] for key in TELEMETRY_FIELDS])
    json_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return RecordingResult(csv_path, json_path, rows, metadata)


__all__ = ["convert_dashboard_recording", "load_dashboard_recording"]
