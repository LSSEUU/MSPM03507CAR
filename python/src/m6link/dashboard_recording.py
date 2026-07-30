"""Streaming dashboard-session recording and indexed offline playback support."""

from __future__ import annotations

from collections import Counter, deque
from collections.abc import Callable, Mapping
from contextlib import suppress
from datetime import datetime, timezone
from enum import Enum
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import tempfile
import threading
import time
from typing import Any
import uuid
import zipfile

from .models import LinkEvent, TelemetrySample, VisionUartChunk
from .registry import (
    ActionReason,
    ActionState,
    EventCode,
    LAYOUTS,
    REGISTRY,
    TELEMETRY,
    TelemetryStream,
)


FORMAT_VERSION = "m6-dashboard-session-v1"
MIN_FREE_BYTES = 512 * 1024 * 1024
PRE_ROLL_SECONDS = 5.0
POST_ROLL_SECONDS = 2.0
_SESSION_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_TERMINAL_ACTION_STATES = {
    int(ActionState.COMPLETE),
    int(ActionState.REJECTED),
    int(ActionState.FAULT),
}


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso_utc(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _json_safe(value: Any) -> Any:
    if isinstance(value, Enum):
        return {"value": int(value), "name": value.name}
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if hasattr(value, "__dataclass_fields__"):
        return {
            key: _json_safe(getattr(value, key))
            for key in value.__dataclass_fields__
        }
    return str(value)


def _write_json(path: Path, value: Any) -> None:
    payload = json.dumps(
        _json_safe(value), ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False
    ).encode("utf-8") + b"\n"
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(payload)
    temporary.replace(path)


def _field_unit(name: str) -> tuple[float, str]:
    suffixes = (
        ("_cdeg_s", 0.01, "deg/s"),
        ("_cdeg", 0.01, "deg"),
        ("_mdeg_s", 0.001, "deg/s"),
        ("_mdeg", 0.001, "deg"),
        ("_x1000", 0.001, "normalized"),
        ("_mm_s", 1.0, "mm/s"),
        ("_ms", 1.0, "ms"),
        ("_us", 1.0, "us"),
        ("_adc", 1.0, "ADC count"),
    )
    for suffix, scale, unit in suffixes:
        if name.endswith(suffix):
            return scale, unit
    if "pwm" in name:
        return 1.0, "PWM count"
    if "mask" in name or "flags" in name or "fault" in name:
        return 1.0, "bitmask"
    return 1.0, "raw"


def dashboard_schema() -> dict[str, Any]:
    streams: dict[str, Any] = {}
    for descriptor in TELEMETRY["streams"]:
        layout = LAYOUTS[str(descriptor["body_layout"])]
        fields = []
        for field in layout["fields"]:
            if field.get("reserved"):
                continue
            scale, unit = _field_unit(str(field["name"]))
            fields.append(
                {
                    "name": field["name"],
                    "type": field["kind"],
                    "scale": scale,
                    "unit": unit,
                    **({"registry": field["registry"]} if "registry" in field else {}),
                }
            )
        streams[str(descriptor["name"]).lower()] = {
            "stream_id": int(descriptor["value"]),
            "schema": int(descriptor["schema"]),
            "nominal_period_min_ms": int(descriptor["period_min_ms"]),
            "fields": fields,
        }
    events = {
        str(item["name"]): {
            "event_code": int(item["value"]),
            "schema": int(item["schema"]),
            "body_layout": item["body_layout"],
        }
        for item in REGISTRY["registries"]["event_codes"]
    }
    return {
        "format_version": FORMAT_VERSION,
        "record_fields": {
            "record_index": "monotonic integer",
            "elapsed_ms": "milliseconds since the first record in this session",
            "host_time_utc": "ISO-8601 UTC",
            "device_time_ms": "device monotonic milliseconds, when available",
            "record_type": "telemetry | uart_data | device_event | dashboard_event",
            "stream": "core | wheel | line | health | vision_uart_rx, or null",
            "event": "stable event name, or null",
            "sample_id": "per-stream sample id, or null",
            "gap": "whether a transport/sample gap preceded this record",
            "values": "raw decoded protocol values",
        },
        "streams": streams,
        "uart_streams": {
            "vision_uart_rx": {
                "transport": "UART1",
                "direction": "camera-to-MCU RX",
                "baud": 115200,
                "format": "8N1",
                "mcu_pin": "PB7",
                "data_encoding": "lowercase hexadecimal in values.data_hex",
                "fields": [
                    "sequence", "status_flags", "rx_bytes", "tx_bytes",
                    "rx_overflow", "tx_overflow", "hardware_errors",
                    "bridge_dropped", "data_hex",
                ],
            }
        },
        "events": events,
    }


class _SessionWriter:
    def __init__(
        self,
        directory: Path,
        manifest: dict[str, Any],
        *,
        origin_monotonic: float,
        origin_utc: datetime,
    ) -> None:
        self.directory = directory
        self.manifest = manifest
        self.origin_monotonic = origin_monotonic
        self.origin_utc = origin_utc
        self.timeline_path = directory / "timeline.jsonl"
        self._timeline = self.timeline_path.open("ab", buffering=0)
        self.index: list[dict[str, int]] = []
        self.counts: Counter[str] = Counter()
        self.gaps: Counter[str] = Counter()
        self.faults: list[dict[str, Any]] = []
        self.markers: list[dict[str, Any]] = []
        self.signal_stats: dict[str, dict[str, float | int]] = {}
        self.record_index = 0
        self.last_elapsed_ms = 0
        self.last_index_second = -1
        self.last_flush = time.monotonic()
        self.bytes_written = 0

    def append(self, captured: dict[str, Any]) -> dict[str, Any]:
        elapsed_ms = max(
            self.last_elapsed_ms,
            0,
            round((float(captured.pop("_captured_at")) - self.origin_monotonic) * 1000),
        )
        self.record_index += 1
        record = {
            "record_index": self.record_index,
            "elapsed_ms": elapsed_ms,
            "host_time_utc": captured["host_time_utc"],
            "device_time_ms": captured.get("device_time_ms"),
            "record_type": captured["record_type"],
            "stream": captured.get("stream"),
            "event": captured.get("event"),
            "sample_id": captured.get("sample_id"),
            "gap": bool(captured.get("gap", False)),
            "values": _json_safe(captured.get("values", {})),
        }
        offset = self._timeline.tell()
        second = elapsed_ms // 1000
        if second > self.last_index_second:
            for indexed_second in range(self.last_index_second + 1, second + 1):
                self.index.append({"elapsed_ms": indexed_second * 1000, "record_index": self.record_index, "byte_offset": offset})
            self.last_index_second = second
        encoded = json.dumps(record, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
        self._timeline.write(encoded)
        self.bytes_written += len(encoded)
        self.last_elapsed_ms = elapsed_ms
        key = record["stream"] or record["event"] or record["record_type"]
        self.counts[str(key)] += 1
        if record["gap"]:
            self.gaps[str(key)] += 1
        if record["record_type"] == "device_event" and record["event"] == "TELEMETRY_GAP":
            stream_id = record["values"].get("stream_id", "unknown")
            try:
                drop_delta = max(1, int(record["values"].get("drop_delta", 1)))
            except (TypeError, ValueError):
                drop_delta = 1
            self.gaps[f"stream_{stream_id}"] += drop_delta
        if record["record_type"] == "device_event" and record["event"] in {"SAFETY_FAULT", "EMERGENCY_STOP", "STORAGE_FAULT"}:
            self.faults.append({"elapsed_ms": elapsed_ms, "event": record["event"], "values": record["values"]})
        if record["event"] == "marker":
            self.markers.append({"elapsed_ms": elapsed_ms, **record["values"]})
        self._update_stats(record)
        now = time.monotonic()
        if now - self.last_flush >= 1.0:
            self._timeline.flush()
            self.last_flush = now
        return record

    def _update_stats(self, record: Mapping[str, Any]) -> None:
        if record["record_type"] != "telemetry":
            return
        stream = str(record["stream"])
        common = {
            "core": {"speed_l_mm_s", "speed_r_mm_s", "yaw_cdeg", "yaw_rate_cdeg_s", "fault_bits"},
            "wheel": {"filtered_l_mm_s", "filtered_r_mm_s", "pwm_l", "pwm_r"},
            "line": {"position_x1000", "raw_position_x1000", "confidence_x1000", "correction_mm_s"},
            "health": {"task_missed", "uart_rx_drop", "uart_tx_drop", "telemetry_p2_drop", "telemetry_p3_drop"},
        }.get(stream, set())
        for key, raw in record["values"].items():
            if key not in common or isinstance(raw, bool) or not isinstance(raw, (int, float)):
                continue
            value = float(raw)
            name = f"{stream}.{key}"
            stats = self.signal_stats.setdefault(name, {"minimum": value, "maximum": value, "sum": 0.0, "count": 0})
            stats["minimum"] = min(float(stats["minimum"]), value)
            stats["maximum"] = max(float(stats["maximum"]), value)
            stats["sum"] = float(stats["sum"]) + value
            stats["count"] = int(stats["count"]) + 1

    def close(self, *, complete: bool, reason: str, action_result: Any = None) -> None:
        with suppress(OSError):
            self._timeline.flush()
        with suppress(OSError):
            self._timeline.close()
        ended = _utc_now()
        duration_ms = self.last_elapsed_ms
        stats = {
            key: {
                "minimum": value["minimum"],
                "maximum": value["maximum"],
                "mean": float(value["sum"]) / max(1, int(value["count"])),
                "count": value["count"],
            }
            for key, value in self.signal_stats.items()
        }
        summary = {
            "format_version": FORMAT_VERSION,
            "duration_ms": duration_ms,
            "records": self.record_index,
            "samples": dict(self.counts),
            "gaps": {"total": sum(self.gaps.values()), "by_stream_or_event": dict(self.gaps)},
            "faults": self.faults,
            "markers": self.markers,
            "signal_ranges": stats,
            "action_result": _json_safe(action_result),
            "stop_reason": reason,
            "complete": bool(complete),
        }
        self.manifest.update(
            end_time_utc=_iso_utc(ended),
            duration_ms=duration_ms,
            counts=dict(self.counts),
            gaps=summary["gaps"],
            faults=self.faults,
            stop_reason=reason,
            status="complete" if complete else "incomplete",
            complete=bool(complete),
            bytes=self._directory_size(),
        )
        _write_json(self.directory / "timeline.index.json", {"version": 1, "entries": self.index})
        _write_json(self.directory / "summary.json", summary)
        _write_json(self.directory / "manifest.json", self.manifest)

    def _directory_size(self) -> int:
        total = self.bytes_written
        for path in self.directory.iterdir():
            if path != self.timeline_path:
                with suppress(OSError):
                    total += path.stat().st_size
        return total


class DashboardRecordingManager:
    """Maintain pre-roll, stream sessions to disk, and serve indexed recordings."""

    def __init__(
        self,
        session: Any,
        identity: Any,
        recordings_dir: str | Path = "recordings",
        *,
        parameter_provider: Callable[[], Any] | None = None,
        connected_provider: Callable[[], bool] | None = None,
        sampling_periods: Mapping[str, int] | None = None,
        clock: Callable[[], float] = time.monotonic,
        utc_clock: Callable[[], datetime] = _utc_now,
        minimum_free_bytes: int = MIN_FREE_BYTES,
    ) -> None:
        self.session = session
        self.root = Path(recordings_dir).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._identity = _json_safe(identity)
        self._parameter_provider = parameter_provider
        self._connected_provider = connected_provider or (lambda: bool(getattr(session, "connected", True)))
        self._clock = clock
        self._utc_clock = utc_clock
        self._minimum_free_bytes = minimum_free_bytes
        self._sampling_periods = {str(key): int(value) for key, value in (sampling_periods or {}).items()}
        self._prebuffer: deque[dict[str, Any]] = deque()
        self._writer: _SessionWriter | None = None
        self._trigger: str | None = None
        self._action_id: int | None = None
        self._action_result: Any = None
        self._action_motion_seen = False
        self._local_line_active = False
        self._gap_abort_requested = False
        self._post_deadline: float | None = None
        self._disconnect_deadline: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._lock = threading.RLock()
        self._last_error: str | None = None
        self._recover_stale_sessions()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        if not callable(getattr(self.session, "event_tap", None)):
            return
        self._stop.clear()
        self._ready.clear()
        self._thread = threading.Thread(target=self._event_worker, name="m6-dashboard-recorder", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=1.0)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        with self._lock:
            if self._writer is not None:
                self._finish_locked(complete=False, reason="host_shutdown")

    def ingest(self, event: Any, *, captured_at: float | None = None) -> None:
        now = self._clock() if captured_at is None else captured_at
        captured = self._capture(event, now)
        if captured is None:
            return
        abort_for_gap = False
        with self._lock:
            self._add_prebuffer(captured, now)
            self._append_active(dict(captured))
            if isinstance(event, LinkEvent):
                self._handle_link_event(event, now)
            elif isinstance(event, TelemetrySample):
                self._handle_local_line_telemetry(event, now)
                self._handle_action_telemetry(event, now)
                if (
                    event.gap_before and self._local_line_active
                    and not self._gap_abort_requested
                    and callable(getattr(self.session, "abort_h_mission", None))
                ):
                    self._gap_abort_requested = True
                    abort_for_gap = True
            self._tick_locked(now)
        if abort_for_gap:
            try:
                result = self.session.abort_h_mission(telemetry_gap=True)
            except Exception as exc:
                self.record_event("h_mission_gap_abort_failed", {"error": str(exc)})
            else:
                self.record_event("h_mission_gap_abort", {"result": _json_safe(result)})

    def record_event(self, event: str, values: Mapping[str, Any] | None = None) -> None:
        now = self._clock()
        captured = {
            "_captured_at": now,
            "host_time_utc": _iso_utc(self._utc_clock()),
            "device_time_ms": None,
            "record_type": "dashboard_event",
            "stream": None,
            "event": str(event),
            "sample_id": None,
            "gap": False,
            "values": _json_safe(values or {}),
        }
        with self._lock:
            self._add_prebuffer(captured, now)
            self._append_active(dict(captured))
            if event == "connection_lost" and self._trigger == "action":
                self._disconnect_deadline = now + 3.0
            elif event == "connection_restored":
                self._disconnect_deadline = None
            elif event == "control_cancel" and self._trigger == "action":
                self._set_action_terminal(
                    now,
                    state=ActionState.COMPLETE,
                    reason=ActionReason.HOST_CANCEL,
                    source="control_cancel",
                )
            elif event == "emergency_stop_requested" and self._trigger == "action":
                self._set_action_terminal(
                    now,
                    state=ActionState.FAULT,
                    reason=ActionReason.EMERGENCY,
                    source="emergency_stop_requested",
                )

    def update_identity(self, identity: Any) -> None:
        value = _json_safe(identity)
        with self._lock:
            old_boot = self._identity.get("boot_id") if isinstance(self._identity, Mapping) else None
            new_boot = value.get("boot_id") if isinstance(value, Mapping) else None
            self._identity = value
        self.record_event("identity_changed", {"previous_boot_id": old_boot, "boot_id": new_boot, "identity": value})
        with self._lock:
            if self._trigger == "action" and old_boot is not None and new_boot != old_boot:
                self._finish_locked(complete=False, reason="device_restarted")

    def start_manual(self, name: str = "", notes: str = "") -> dict[str, Any]:
        return self._start(name=name, notes=notes, trigger="manual", include_preroll=False, action=None)

    def start_action(self, *, name: str = "", notes: str = "", action: Any = None) -> dict[str, Any]:
        return self._start(name=name, notes=notes, trigger="action", include_preroll=True, action=action)

    def _start(self, *, name: str, notes: str, trigger: str, include_preroll: bool, action: Any) -> dict[str, Any]:
        if not self._connected_provider():
            raise ValueError("车辆未在线，不能开始录制")
        if shutil.disk_usage(self.root).free < self._minimum_free_bytes:
            raise OSError("录制目录剩余空间不足 512 MB")
        now = self._clock()
        started_utc = self._utc_clock()
        with self._lock:
            if self._writer is not None:
                if trigger == "action":
                    self.record_event("action_recording_attached", {"action": _json_safe(action)})
                    return self.status()
                raise ValueError("已有录制正在进行")
            self._last_error = None
            title = str(name).strip()[:120] or ("动作联动录制" if trigger == "action" else "手动录制")
            safe_slug = re.sub(r"[^A-Za-z0-9_-]+", "-", title).strip("-")[:24] or "session"
            session_id = f"{started_utc.strftime('%Y%m%dT%H%M%S')}-{started_utc.microsecond // 1000:03d}Z-{safe_slug}-{uuid.uuid4().hex[:6]}"
            directory = self.root / started_utc.strftime("%Y-%m-%d") / session_id
            directory.mkdir(parents=True, exist_ok=False)
            parameters = self._parameter_snapshot()
            pre = [dict(item) for item in self._prebuffer if now - float(item["_captured_at"]) <= PRE_ROLL_SECONDS] if include_preroll else []
            origin = float(pre[0]["_captured_at"]) if pre else now
            origin_utc = started_utc
            if pre:
                try:
                    origin_utc = datetime.fromisoformat(str(pre[0]["host_time_utc"]).replace("Z", "+00:00"))
                except ValueError:
                    pass
            manifest = {
                "format_version": FORMAT_VERSION,
                "session_id": session_id,
                "name": title,
                "notes": str(notes).strip()[:4000],
                "start_time_utc": _iso_utc(origin_utc),
                "recording_started_utc": _iso_utc(started_utc),
                "trigger": trigger,
                "pre_roll_ms": round((now - origin) * 1000) if pre else 0,
                "post_roll_ms": round(POST_ROLL_SECONDS * 1000) if trigger == "action" else 0,
                "action": _json_safe(action),
                "firmware_identity": self._identity,
                "boot_id": self._identity.get("boot_id") if isinstance(self._identity, Mapping) else None,
                "parameters": parameters,
                "sampling": {
                    str(item["name"]).lower(): {
                        "period_ms": self._sampling_periods.get(str(item["name"]).lower()),
                        "protocol_min_ms": item["period_min_ms"],
                    }
                    for item in TELEMETRY["streams"]
                },
                "status": "recording",
                "complete": False,
            }
            _write_json(directory / "manifest.json", manifest)
            _write_json(directory / "schema.json", dashboard_schema())
            (directory / "README.md").write_text(self._readme(session_id), encoding="utf-8", newline="\n")
            self._writer = _SessionWriter(directory, manifest, origin_monotonic=origin, origin_utc=origin_utc)
            self._trigger = trigger
            self._post_deadline = None
            self._disconnect_deadline = None
            self._action_result = action
            self._action_id = self._extract_action_id(action)
            self._action_motion_seen = False
            for item in pre:
                self._append_active(item)
            self._append_active({
                "_captured_at": now,
                "host_time_utc": _iso_utc(started_utc),
                "device_time_ms": None,
                "record_type": "dashboard_event",
                "stream": None,
                "event": "recording_started",
                "sample_id": None,
                "gap": False,
                "values": {"trigger": trigger, "pre_roll_ms": manifest["pre_roll_ms"]},
            })
            return self.status()

    def stop(self, *, reason: str = "manual_stop") -> dict[str, Any]:
        with self._lock:
            if self._writer is None:
                raise ValueError("当前没有正在进行的录制")
            self._finish_locked(complete=True, reason=reason)
            return self.status()

    def add_marker(self, label: str) -> dict[str, Any]:
        text = str(label).strip()
        if not text:
            raise ValueError("标记说明不能为空")
        with self._lock:
            if self._writer is None:
                raise ValueError("当前没有正在进行的录制")
        self.record_event("marker", {"label": text[:500]})
        return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            writer = self._writer
            if writer is None:
                return {"ok": True, "recording": False, "last_error": self._last_error}
            elapsed = max(writer.last_elapsed_ms, round((self._clock() - writer.origin_monotonic) * 1000))
            return {
                "ok": True,
                "recording": True,
                "session_id": writer.manifest["session_id"],
                "name": writer.manifest["name"],
                "trigger": self._trigger,
                "elapsed_ms": elapsed,
                "bytes": writer._directory_size(),
                "post_roll": self._post_deadline is not None,
            }

    def list_sessions(self) -> dict[str, Any]:
        sessions = []
        for manifest_path in self.root.glob("*/*/manifest.json"):
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                directory = manifest_path.parent
                sessions.append({
                    "session_id": manifest.get("session_id", directory.name),
                    "name": manifest.get("name", directory.name),
                    "notes": manifest.get("notes", ""),
                    "start_time_utc": manifest.get("start_time_utc"),
                    "duration_ms": int(manifest.get("duration_ms", 0)),
                    "trigger": manifest.get("trigger", "unknown"),
                    "action": manifest.get("action"),
                    "bytes": int(manifest.get("bytes", self._path_size(directory))),
                    "status": manifest.get("status", "incomplete"),
                    "complete": bool(manifest.get("complete", False)),
                    "stop_reason": manifest.get("stop_reason"),
                })
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                continue
        sessions.sort(key=lambda item: str(item.get("start_time_utc") or ""), reverse=True)
        return {"ok": True, "recordings": sessions}

    def manifest(self, session_id: str) -> dict[str, Any]:
        path = self._session_directory(session_id) / "manifest.json"
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError("录制会话不可读取") from exc

    def timeline(self, session_id: str, *, from_ms: int = 0, after: int = 0, limit: int = 2000) -> dict[str, Any]:
        directory = self._session_directory(session_id)
        timeline_path = directory / "timeline.jsonl"
        limit = min(max(int(limit), 1), 10_000)
        from_ms = max(0, int(from_ms))
        after = max(0, int(after))
        offset = 0
        if from_ms:
            with suppress(OSError, json.JSONDecodeError, TypeError, ValueError):
                index = json.loads((directory / "timeline.index.json").read_text(encoding="utf-8"))["entries"]
                candidates = [item for item in index if int(item["elapsed_ms"]) <= from_ms]
                if candidates:
                    offset = int(candidates[-1]["byte_offset"])
        records: list[dict[str, Any]] = []
        last_index = after
        has_more = False
        try:
            with timeline_path.open("rb") as source:
                source.seek(offset)
                for line in source:
                    record = json.loads(line)
                    record_index = int(record["record_index"])
                    if record_index <= after or int(record["elapsed_ms"]) < from_ms:
                        continue
                    if len(records) >= limit:
                        has_more = True
                        break
                    records.append(record)
                    last_index = record_index
        except (OSError, json.JSONDecodeError, KeyError, ValueError) as exc:
            raise ValueError("录制时间线不可读取") from exc
        return {"ok": True, "records": records, "after": last_index, "has_more": has_more}

    def create_download(self, session_id: str) -> Path:
        directory = self._session_directory(session_id)
        handle, filename = tempfile.mkstemp(prefix=f"m6-{session_id}-", suffix=".zip")
        os.close(handle)
        Path(filename).unlink(missing_ok=True)
        target = Path(filename)
        try:
            with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
                for path in sorted(directory.iterdir()):
                    if path.is_file() and not path.name.endswith(".tmp"):
                        archive.write(path, arcname=f"{session_id}/{path.name}")
        except Exception:
            target.unlink(missing_ok=True)
            raise
        return target

    def tick(self, now: float | None = None) -> None:
        with self._lock:
            self._tick_locked(self._clock() if now is None else now)

    def _event_worker(self) -> None:
        try:
            with self.session.event_tap() as tap:
                self._ready.set()
                while not self._stop.is_set():
                    try:
                        event = tap.get(timeout=0.2)
                    except queue.Empty:
                        self.tick()
                        continue
                    self.ingest(event)
        except Exception as exc:
            self._ready.set()
            self._last_error = f"录制事件线程停止：{exc}"
            with self._lock:
                if self._writer is not None:
                    self._finish_locked(complete=False, reason="event_tap_failed")

    def _capture(self, event: Any, now: float) -> dict[str, Any] | None:
        common = {
            "_captured_at": now,
            "host_time_utc": _iso_utc(self._utc_clock()),
            "device_time_ms": getattr(event, "time_ms", None),
            "sample_id": None,
            "gap": bool(getattr(event, "gap_before", False)),
        }
        if isinstance(event, TelemetrySample):
            return {
                **common,
                "record_type": "telemetry",
                "stream": str(getattr(event.stream, "name", event.stream)).lower(),
                "event": None,
                "sample_id": int(event.sample_id),
                "values": _json_safe(event.values),
            }
        if isinstance(event, VisionUartChunk):
            return {
                **common,
                "record_type": "uart_data",
                "stream": "vision_uart_rx",
                "event": None,
                "values": {
                    "direction": "rx",
                    "sequence": int(event.sequence),
                    "status_flags": int(event.status_flags),
                    "rx_bytes": int(event.rx_bytes),
                    "tx_bytes": int(event.tx_bytes),
                    "rx_overflow": int(event.rx_overflow),
                    "tx_overflow": int(event.tx_overflow),
                    "hardware_errors": int(event.hardware_errors),
                    "bridge_dropped": int(event.bridge_dropped),
                    "data_hex": event.data.hex(),
                },
            }
        if isinstance(event, LinkEvent):
            try:
                name = EventCode(int(event.code)).name
            except ValueError:
                name = f"EVENT_{int(event.code)}"
            return {
                **common,
                "record_type": "device_event",
                "stream": None,
                "event": name,
                "values": {
                    "event_code": int(event.code),
                    "severity": int(event.severity),
                    "occurrence_count": int(event.occurrence_count),
                    **_json_safe(event.values),
                },
            }
        return None

    def _handle_link_event(self, event: LinkEvent, now: float) -> None:
        if int(event.code) == int(EventCode.RESET) and self._trigger == "action":
            self._finish_locked(complete=False, reason="device_restarted")
            return
        if int(event.code) == int(EventCode.SAFETY_FAULT) and self._trigger == "action":
            self._set_action_terminal(
                now,
                state=ActionState.FAULT,
                reason=ActionReason.SAFETY_FAULT,
                source="safety_fault",
                device_time_ms=event.time_ms,
            )
            return
        if int(event.code) == int(EventCode.EMERGENCY_STOP) and self._trigger == "action":
            self._set_action_terminal(
                now,
                state=ActionState.FAULT,
                reason=ActionReason.EMERGENCY,
                source="emergency_stop",
                device_time_ms=event.time_ms,
            )
            return
        if int(event.code) != int(EventCode.ACTION_STATE) or self._trigger != "action":
            return
        state_value = event.values.get("state")
        try:
            state = int(state_value)
        except (TypeError, ValueError):
            return
        event_action_id = int(event.values.get("action_id", 0))
        if self._action_id not in (None, 0) and event_action_id not in (0, self._action_id):
            return
        if state in _TERMINAL_ACTION_STATES:
            self._action_result = _json_safe(event.values)
            self._post_deadline = now + POST_ROLL_SECONDS

    def _handle_action_telemetry(self, sample: TelemetrySample, now: float) -> None:
        """Infer a normal terminal edge when older firmware omits ACTION_STATE."""

        if (
            self._trigger != "action"
            or self._post_deadline is not None
            or int(sample.stream) != int(TelemetryStream.CORE)
        ):
            return
        values = sample.values
        try:
            active = int(values.get("request_l_mm_s", 0)) != 0 or int(
                values.get("request_r_mm_s", 0)
            ) != 0
            fault_bits = int(values.get("fault_bits", 0))
        except (TypeError, ValueError):
            return
        if active:
            self._action_motion_seen = True
            return
        if not self._action_motion_seen:
            return
        self._set_action_terminal(
            now,
            state=ActionState.FAULT if fault_bits else ActionState.COMPLETE,
            reason=ActionReason.SAFETY_FAULT if fault_bits else ActionReason.COMPLETE,
            source="core_target_zero",
            device_time_ms=sample.time_ms,
        )

    def _handle_local_line_telemetry(self, sample: TelemetrySample, now: float) -> None:
        if int(sample.stream) != int(TelemetryStream.LINE):
            return
        try:
            running = int(sample.values.get("app_state", 0)) == 2
        except (TypeError, ValueError):
            return
        if running and not self._local_line_active:
            self._local_line_active = True
            self._gap_abort_requested = False
            if self._writer is None:
                try:
                    self._start(
                        name="K1 HMission 自动录制",
                        notes="由车载本地按键启动边沿自动触发；包含前5秒和后2秒。",
                        trigger="action",
                        include_preroll=True,
                        action={"source": "local_key", "kind": "h_mission"},
                    )
                except (OSError, ValueError) as exc:
                    self._last_error = str(exc)
            return
        if not running and self._local_line_active:
            self._local_line_active = False
            self._gap_abort_requested = False
            self._set_action_terminal(
                now,
                state=ActionState.COMPLETE,
                reason=ActionReason.COMPLETE,
                source="line_app_idle",
                device_time_ms=sample.time_ms,
            )

    def _set_action_terminal(
        self,
        now: float,
        *,
        state: ActionState,
        reason: ActionReason,
        source: str,
        device_time_ms: int | None = None,
    ) -> None:
        if self._trigger != "action" or self._post_deadline is not None:
            return
        result = {
            "action_id": self._action_id or 0,
            "state": int(state),
            "reason": int(reason),
            "stopped_ms": device_time_ms,
            "inferred": True,
            "source": source,
        }
        self._action_result = result
        self._post_deadline = now + POST_ROLL_SECONDS
        self._append_active(
            {
                "_captured_at": now,
                "host_time_utc": _iso_utc(self._utc_clock()),
                "device_time_ms": device_time_ms,
                "record_type": "dashboard_event",
                "stream": None,
                "event": "action_completion_inferred",
                "sample_id": None,
                "gap": False,
                "values": result,
            }
        )

    def _tick_locked(self, now: float) -> None:
        while self._prebuffer and now - float(self._prebuffer[0]["_captured_at"]) > PRE_ROLL_SECONDS:
            self._prebuffer.popleft()
        if self._post_deadline is not None and now >= self._post_deadline and self._writer is not None:
            state = self._action_result.get("state") if isinstance(self._action_result, Mapping) else None
            try:
                state_name = ActionState(int(state)).name.lower()
            except (ValueError, TypeError):
                state_name = "ended"
            self._finish_locked(complete=True, reason=f"action_{state_name}")
        elif self._disconnect_deadline is not None and now >= self._disconnect_deadline and self._writer is not None:
            self._finish_locked(complete=False, reason="action_connection_lost")

    def _add_prebuffer(self, captured: dict[str, Any], now: float) -> None:
        self._prebuffer.append(dict(captured))
        while self._prebuffer and now - float(self._prebuffer[0]["_captured_at"]) > PRE_ROLL_SECONDS:
            self._prebuffer.popleft()

    def _append_active(self, captured: dict[str, Any]) -> None:
        writer = self._writer
        if writer is None:
            return
        try:
            writer.append(captured)
        except (OSError, ValueError, TypeError) as exc:
            self._last_error = f"录制写盘失败：{exc}"
            self._finish_locked(complete=False, reason="disk_write_failed")

    def _finish_locked(self, *, complete: bool, reason: str) -> None:
        writer = self._writer
        if writer is None:
            return
        self._writer = None
        try:
            writer.close(complete=complete, reason=reason, action_result=self._action_result)
        except OSError as exc:
            self._last_error = f"录制收尾失败：{exc}"
        self._trigger = None
        self._action_id = None
        self._action_result = None
        self._action_motion_seen = False
        self._gap_abort_requested = False
        self._post_deadline = None
        self._disconnect_deadline = None

    def _parameter_snapshot(self) -> dict[str, Any] | None:
        if self._parameter_provider is None:
            return None
        try:
            state = _json_safe(self._parameter_provider())
            if not isinstance(state, Mapping):
                return None
            return {
                "summary": state.get("summary"),
                "active": {
                    item["key"]: item.get("active")
                    for item in state.get("parameters", [])
                    if isinstance(item, Mapping) and "key" in item
                },
            }
        except Exception as exc:
            return {"error": str(exc)}

    def _session_directory(self, session_id: str) -> Path:
        if not _SESSION_ID.fullmatch(str(session_id)):
            raise ValueError("非法录制会话 ID")
        matches = [path for path in self.root.glob(f"*/{session_id}") if path.is_dir()]
        if len(matches) != 1:
            raise ValueError("录制会话不存在")
        resolved = matches[0].resolve()
        if self.root not in resolved.parents:
            raise ValueError("非法录制会话路径")
        return resolved

    def _recover_stale_sessions(self) -> None:
        for manifest_path in self.root.glob("*/*/manifest.json"):
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                if manifest.get("status") != "recording":
                    continue
                count = 0
                duration = 0
                entries = []
                last_second = -1
                valid_end = 0
                with (manifest_path.parent / "timeline.jsonl").open("r+b") as source:
                    while True:
                        offset = source.tell()
                        line = source.readline()
                        if not line:
                            break
                        try:
                            record = json.loads(line)
                        except json.JSONDecodeError:
                            source.truncate(valid_end)
                            break
                        count += 1
                        duration = int(record.get("elapsed_ms", duration))
                        valid_end = source.tell()
                        second = duration // 1000
                        if second > last_second:
                            for indexed_second in range(last_second + 1, second + 1):
                                entries.append({"elapsed_ms": indexed_second * 1000, "record_index": int(record.get("record_index", count)), "byte_offset": offset})
                            last_second = second
                manifest.update(status="incomplete", complete=False, stop_reason="host_interrupted", duration_ms=duration, bytes=self._path_size(manifest_path.parent))
                _write_json(manifest_path.parent / "timeline.index.json", {"version": 1, "entries": entries})
                _write_json(manifest_path.parent / "summary.json", {"format_version": FORMAT_VERSION, "duration_ms": duration, "records": count, "stop_reason": "host_interrupted", "complete": False})
                _write_json(manifest_path, manifest)
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                continue

    @staticmethod
    def _extract_action_id(action: Any) -> int | None:
        if isinstance(action, Mapping):
            value = action.get("action_id")
            if value is not None:
                with suppress(TypeError, ValueError):
                    return int(value)
        return None

    @staticmethod
    def _path_size(directory: Path) -> int:
        total = 0
        for path in directory.glob("*"):
            if path.is_file():
                with suppress(OSError):
                    total += path.stat().st_size
        return total

    @staticmethod
    def _readme(session_id: str) -> str:
        return f"""# M6 dashboard recording `{session_id}`

This directory is one self-contained UTF-8 telemetry session.

- `manifest.json`: identity, parameters, trigger, timing, integrity and stop reason.
- `timeline.jsonl`: canonical ordered records; one strict JSON object per line.
- `schema.json`: stable record fields plus protocol stream fields, scale and units.
- `summary.json`: counts, gaps, faults, markers, common signal ranges and action result.
- `timeline.index.json`: one byte offset per second for seek operations.

For AI analysis, read `manifest.json` and `schema.json` first, use `summary.json` for orientation,
then stream only the required time range from `timeline.jsonl`. Raw integer protocol values are
preserved; apply the scale declared in `schema.json` when engineering units are needed. UART1
camera bytes use `record_type: "uart_data"` and lossless lowercase HEX in `values.data_hex`.
"""


__all__ = ["DashboardRecordingManager", "FORMAT_VERSION", "dashboard_schema"]
