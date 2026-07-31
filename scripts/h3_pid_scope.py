"""Local H3 PID oscilloscope and RAM-only Kp/Kd controller.

The background owner is the only code that touches the serial Session.  The
HTTP threads exchange commands with it through a queue, so telemetry, PID
updates, heartbeat and stop cannot race each other on COM10.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import queue
import secrets
import struct
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlsplit
import webbrowser


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link import FrameType, H3TuneParameter, Session  # noqa: E402
from m6link.h3_tune_registry import CAPABILITY_MASK as H3_TUNE_CAPABILITY  # noqa: E402


FIRMWARE_ID = "M6-runtime-20260801-r158-stop"
WEB_ROOT = ROOT / "python" / "src" / "m6link" / "web"
H3_RECORD = struct.Struct("<BBBBIII7hiiIHBBbBh")
H3_FIELDS = (
    "kind", "schema", "state", "flags", "time_ms", "run_id",
    "profile_revision", "x_mm_x10", "vx_mm_s", "target_mm_x10",
    "error_mm_x10", "raw_pd_mdeg", "forced_tilt_mdeg", "tilt_mdeg",
    "actuator_counts", "actuator_target_counts", "fault", "vision_age_ms",
    "vision_seq", "actuator_state", "actuator_direction", "actuator_rpm",
    "integral_mdeg",
)
CSV_FIELDS = ("cursor", "received_at") + H3_FIELDS
PROFILE_FIELDS = (
    "plus_start_mdeg", "plus_max_mdeg", "minus_start_mdeg",
    "minus_max_mdeg", "kp_mdeg_per_mm", "kd_mdeg_per_mm_s",
    "normal_slew_mdeg_per_20ms", "breakaway_slew_mdeg_per_20ms",
    "brake_slew_mdeg_per_20ms", "tracking_rpm", "plus_launch_rpm",
    "minus_launch_rpm",
)
PARAMETERS = {
    "kp_mdeg_per_mm": (H3TuneParameter.kp_mdeg_per_mm, 20, 300),
    "kd_mdeg_per_mm_s": (H3TuneParameter.kd_mdeg_per_mm_s, 0, 120),
}
STATE_NAMES = {
    0: "WAIT_READY", 1: "RUN_PLUS", 2: "HOLD_PLUS", 3: "RUN_MINUS",
    4: "HOLD_MINUS", 5: "RETURNING", 6: "FAULT", 7: "CENTER_HOLD",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def decode_h3(payload: bytes) -> dict[str, int]:
    if len(payload) != H3_RECORD.size:
        raise ValueError(f"H3 telemetry length {len(payload)} != {H3_RECORD.size}")
    row = dict(zip(H3_FIELDS, H3_RECORD.unpack(payload), strict=True))
    if row["kind"] != 3 or row["schema"] != 3:
        raise ValueError("not H3 schema 3 telemetry")
    return row


def h3_row(event: object) -> dict[str, int] | None:
    payload = getattr(event, "payload", b"")
    if (getattr(event, "type", -1) != int(FrameType.BULK) or
            len(payload) != H3_RECORD.size or payload[:2] != b"\x03\x03"):
        return None
    return decode_h3(payload)


def profile_dict(profile: object) -> dict[str, int]:
    return {name: int(getattr(profile, name)) for name in PROFILE_FIELDS}


class FlickAnalyzer:
    """Deterministic metrics for a hand flick around the fixed 0 mm target."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.run_started_ms: int | None = None
        self.flick_started_ms: int | None = None
        self.initial_sign = 0
        self.last_sign = 0
        self.peak_x10 = 0
        self.opposite_peak_x10 = 0
        self.max_speed = 0
        self.max_tilt = 0
        self.max_actuator_error = 0
        self.zero_crossings = 0
        self.settle_started_ms: int | None = None
        self.settle_ms: int | None = None
        self.sample_count = 0
        self.gap_count = 0
        self.max_gap_ms = 0
        self.last_time_ms: int | None = None

    def feed(self, row: dict[str, int]) -> None:
        now = row["time_ms"]
        self.sample_count += 1
        if self.run_started_ms is None:
            self.run_started_ms = now
        if self.last_time_ms is not None:
            gap = (now - self.last_time_ms) & 0xFFFFFFFF
            if gap > 100:
                self.gap_count += 1
                self.max_gap_ms = max(self.max_gap_ms, gap)
        self.last_time_ms = now
        x = row["x_mm_x10"]
        vx = row["vx_mm_s"]
        self.max_speed = max(self.max_speed, abs(vx))
        self.max_tilt = max(self.max_tilt, abs(row["tilt_mdeg"]))
        self.max_actuator_error = max(
            self.max_actuator_error,
            abs(row["actuator_target_counts"] - row["actuator_counts"]),
        )
        if self.flick_started_ms is None and (abs(x) >= 50 or abs(vx) >= 40):
            self.flick_started_ms = now
            direction = x if abs(x) >= 10 else vx
            self.initial_sign = 1 if direction >= 0 else -1
            self.last_sign = self.initial_sign
        if self.flick_started_ms is None:
            return
        directed = self.initial_sign * x
        self.peak_x10 = max(self.peak_x10, directed)
        self.opposite_peak_x10 = max(self.opposite_peak_x10, -directed)
        sign = 1 if x > 10 else (-1 if x < -10 else 0)
        if sign and self.last_sign and sign != self.last_sign:
            self.zero_crossings += 1
        if sign:
            self.last_sign = sign
        in_window = abs(x) <= 30 and abs(vx) <= 5
        if in_window:
            if self.settle_started_ms is None:
                self.settle_started_ms = now
            if self.settle_ms is None and now - self.settle_started_ms >= 1000:
                self.settle_ms = now - self.flick_started_ms
        else:
            self.settle_started_ms = None

    def result(self) -> dict[str, Any]:
        return {
            "detected": self.flick_started_ms is not None,
            "direction": "+" if self.initial_sign > 0 else (
                "-" if self.initial_sign < 0 else "waiting"
            ),
            "peak_mm": round(self.peak_x10 / 10.0, 1),
            "opposite_overshoot_mm": round(self.opposite_peak_x10 / 10.0, 1),
            "settled": self.settle_ms is not None,
            "settle_ms": self.settle_ms,
            "zero_crossings": self.zero_crossings,
            "max_speed_mm_s": self.max_speed,
            "max_tilt_deg": round(self.max_tilt / 1000.0, 2),
            "max_actuator_error_counts": self.max_actuator_error,
            "samples": self.sample_count,
            "gap_count": self.gap_count,
            "max_gap_ms": self.max_gap_ms,
        }


@dataclass
class OwnerCommand:
    name: str
    payload: dict[str, Any]
    done: threading.Event
    result: Any = None
    error: str = ""


class ScopeOwner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.commands: queue.Queue[OwnerCommand] = queue.Queue()
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, name="h3-pid-scope-owner", daemon=True)
        self.lock = threading.Lock()
        self.samples: deque[dict[str, Any]] = deque(maxlen=args.max_samples)
        self.cursor = 0
        self.analyzer = FlickAnalyzer()
        self.session: Session | None = None
        self.snapshot: Any = None
        self.token = 0
        self.prior_monitor = False
        self.last_sample_received: float | None = None
        self.center_since: float | None = None
        self.trial_deadline: float | None = None
        self.capture_dir = args.output / datetime.now().strftime("%Y%m%d-%H%M%S")
        self.capture_path = self.capture_dir / "telemetry.csv"
        self.events_path = self.capture_dir / "events.jsonl"
        self.state_value: dict[str, Any] = {
            "connected": False, "phase": "connecting", "wait_reason": "opening serial",
            "error": "", "port": args.port, "firmware_id": "", "boot_id": 0,
            "profile": {}, "active_revision": 0, "stage_revision": 0,
            "authorized": False, "remaining_runs": 0, "run_id": 0,
            "ready": False, "centered": False, "center_stable_ms": 0,
            "h3_state": 0, "h3_state_name": STATE_NAMES[0], "h3_fault": 0,
            "heartbeat_count": 0, "heartbeat_error": "",
            "telemetry_age_ms": None,
            "latest": {}, "metrics": self.analyzer.result(), "cursor": 0,
            "capture_dir": str(self.capture_dir), "trial_duration_s": args.duration,
            "message": "", "updated_at": utc_now(),
        }

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5.0)

    def submit(self, name: str, payload: dict[str, Any] | None = None,
               timeout: float = 4.0) -> Any:
        command = OwnerCommand(name, payload or {}, threading.Event())
        self.commands.put(command)
        if not command.done.wait(timeout):
            raise TimeoutError(f"serial owner did not finish {name}")
        if command.error:
            raise RuntimeError(command.error)
        return command.result

    def state(self) -> dict[str, Any]:
        with self.lock:
            result = json.loads(json.dumps(self.state_value))
            received = self.last_sample_received
        age_ms = None if received is None else int(
            max(0.0, time.monotonic() - received) * 1000
        )
        result["telemetry_age_ms"] = age_ms
        if age_ms is None or age_ms > 250:
            result["ready"] = False
            result["centered"] = False
            result["center_stable_ms"] = 0
        return result

    def samples_after(self, after: int) -> dict[str, Any]:
        with self.lock:
            rows = [dict(item) for item in self.samples if item["cursor"] > after]
            oldest = self.samples[0]["cursor"] if self.samples else self.cursor + 1
            return {
                "ok": True, "after": after, "oldest": oldest,
                "cursor": self.cursor, "truncated": bool(after and after < oldest - 1),
                "samples": rows,
            }

    def export_csv(self) -> bytes:
        with self.lock:
            rows = [dict(item) for item in self.samples]
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        return stream.getvalue().encode("utf-8-sig")

    def _update(self, **values: Any) -> None:
        with self.lock:
            self.state_value.update(values)
            self.state_value["updated_at"] = utc_now()

    def _event(self, kind: str, **values: Any) -> None:
        record = {"at": utc_now(), "kind": kind, **values}
        with self.events_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _snapshot_state(self, snapshot: Any) -> None:
        self.snapshot = snapshot
        self._update(
            profile=profile_dict(snapshot.active),
            active_revision=int(snapshot.active_revision),
            stage_revision=int(snapshot.stage_revision),
            authorized=bool(snapshot.authorized),
            remaining_runs=int(snapshot.remaining_runs),
            run_id=int(snapshot.run_id),
            ready=bool(snapshot.ready),
            h3_state=int(snapshot.h3_state),
            h3_state_name=STATE_NAMES.get(int(snapshot.h3_state), "UNKNOWN"),
            h3_fault=int(snapshot.h3_fault),
        )

    def _handle(self, command: OwnerCommand) -> None:
        assert self.session is not None
        name = command.name
        if name == "start":
            local = self.state()
            telemetry_age_ms = local.get("telemetry_age_ms")
            if telemetry_age_ms is None or int(telemetry_age_ms) > 250:
                raise RuntimeError("H3 遥测已中断；拒绝启动，请重新连接控制板")
            if not local.get("centered", False):
                latest = local.get("latest", {})
                if not bool(int(latest.get("flags", 0)) & 1):
                    raise RuntimeError("视觉未给出有效球坐标；请确认球在相机视野内")
                x_mm = float(latest.get("x_mm_x10", 0)) / 10.0
                raise RuntimeError(
                    f"球尚未在中心稳定 1 秒（当前 {x_mm:+.1f} mm）"
                )
            snapshot = self.session.get_h3_tune()
            self._snapshot_state(snapshot)
            if self.cursor == 0:
                raise RuntimeError("尚未收到 H3 schema 3 遥测，拒绝启动")
            if not snapshot.ready:
                raise RuntimeError("H3 NOT READY：设备执行器或 Safety 预检未通过")
            if snapshot.host_run_active or snapshot.h3_active:
                raise RuntimeError("H3 正在运行，不能重复启动")
            if snapshot.h3_fault:
                raise RuntimeError(f"H3 fault=0x{snapshot.h3_fault:02X}；示波器不会自动清 fault")
            if snapshot.dirty:
                snapshot = self.session.revert_h3_tune(snapshot=snapshot)
            if not snapshot.authorized:
                snapshot = self.session.authorize_h3_tune(
                    8, ttl_ms=1_800_000, run_timeout_ms=12_000,
                    nonce=secrets.randbits(32) or 1, snapshot=snapshot,
                )
            elif self.token and snapshot.auth_token != self.token:
                raise RuntimeError("设备存在其他 H3 授权，拒绝接管")
            self.token = int(snapshot.auth_token)
            self.analyzer.reset()
            snapshot = self.session.start_h3_center_hold(self.token, snapshot=snapshot)
            self.trial_deadline = time.monotonic() + min(11.5, self.args.duration)
            self._snapshot_state(snapshot)
            self._update(phase="running", wait_reason="请轻拨小球约 ±10 mm",
                         metrics=self.analyzer.result(), message="0 mm 保持测试已启动")
            self._event("start", run_id=snapshot.run_id,
                        profile_revision=snapshot.active_revision,
                        profile=profile_dict(snapshot.active))
            command.result = {"ok": True, "state": self.state()}
        elif name == "stop":
            state = self.state()
            if state.get("phase") == "running" and self.token:
                self.session.stop_h3_tune_nowait(self.token)
                self._update(phase="returning", wait_reason="支架正在回水平",
                             message="已请求正常停止")
                self._event("stop", reason="operator")
            else:
                self._update(phase="idle", wait_reason="点击开始后轻拨小球",
                             message="当前未运行")
            self.trial_deadline = None
            command.result = {"ok": True, "state": self.state()}
        elif name == "parameter":
            parameter = str(command.payload.get("parameter", ""))
            if parameter not in PARAMETERS:
                raise ValueError("只允许独立修改 Kp 或 Kd")
            try:
                value = int(command.payload["value"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("参数值必须是整数") from exc
            parameter_id, minimum, maximum = PARAMETERS[parameter]
            if not minimum <= value <= maximum:
                raise ValueError(f"{parameter} 范围为 {minimum}..{maximum}")
            snapshot = self.session.get_h3_tune()
            if snapshot.host_run_active or snapshot.h3_active:
                raise RuntimeError("请先正常停止并等待回水平，再更新参数")
            if snapshot.dirty:
                snapshot = self.session.revert_h3_tune(snapshot=snapshot)
            old = int(getattr(snapshot.active, parameter))
            snapshot = self.session.stage_h3_tune_field(
                parameter_id, value, snapshot=snapshot,
            )
            snapshot = self.session.apply_h3_tune(snapshot=snapshot)
            self._snapshot_state(snapshot)
            self._update(phase="idle", wait_reason="参数已在 RAM 生效；可开始下一次测试",
                         message=f"{parameter}: {old} → {value}（RAM only）")
            self._event("parameter", parameter=parameter, old=old, new=value,
                        profile_revision=snapshot.active_revision)
            command.result = {"ok": True, "state": self.state()}
        elif name == "marker":
            label = str(command.payload.get("label", "手动标记")).strip()[:80] or "手动标记"
            self._event("marker", label=label, cursor=self.cursor)
            self._update(message=f"已标记：{label}")
            command.result = {"ok": True}
        elif name == "estop":
            self.session.emergency_stop()
            self.trial_deadline = None
            self._update(phase="emergency", wait_reason="已发送原始 S；请检查设备",
                         message="急停已发送；不会自动清 fault")
            self._event("emergency_stop", cursor=self.cursor)
            command.result = {"ok": True, "state": self.state()}
        else:
            raise ValueError(f"unknown command {name}")

    def _drain_commands(self) -> None:
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                self._handle(command)
            except Exception as exc:  # surfaced verbatim to the local operator
                command.error = str(exc)
                self._update(message=str(exc))
                self._event("command_error", command=command.name, error=str(exc))
            finally:
                command.done.set()

    def _ingest(self, row: dict[str, int], writer: csv.DictWriter,
                stream: Any) -> None:
        self.last_sample_received = time.monotonic()
        center_now = time.monotonic()
        in_center_window = (
            row["state"] == 0 and bool(row["flags"] & 1) and
            abs(row["x_mm_x10"]) <= 30 and abs(row["vx_mm_s"]) <= 5
        )
        if in_center_window:
            if self.center_since is None:
                self.center_since = center_now
            center_stable_ms = int((center_now - self.center_since) * 1000)
        else:
            self.center_since = None
            center_stable_ms = 0
        centered = center_stable_ms >= 1000
        self.cursor += 1
        item: dict[str, Any] = {"cursor": self.cursor, "received_at": utc_now(), **row}
        writer.writerow(item)
        stream.flush()
        if row["state"] == 7 and row["target_mm_x10"] == 0:
            self.analyzer.feed(row)
        metrics = self.analyzer.result()
        with self.lock:
            self.samples.append(item)
            self.state_value.update({
                "cursor": self.cursor, "latest": row, "metrics": metrics,
                "h3_state": row["state"],
                "h3_state_name": STATE_NAMES.get(row["state"], "UNKNOWN"),
                "ready": bool(row["flags"] & 2),
                "centered": centered,
                "center_stable_ms": min(center_stable_ms, 1000),
                "h3_fault": row["fault"], "run_id": row["run_id"],
                "active_revision": row["profile_revision"],
                "updated_at": utc_now(),
            })
        if row["fault"]:
            self.trial_deadline = None
            self._update(phase="fault", wait_reason="检测到真实 fault；请检查设备",
                         message=f"fault=0x{row['fault']:08X}，未自动清除")

    def _poll_control(self, now: float) -> None:
        assert self.session is not None
        if self.trial_deadline is not None and now >= self.trial_deadline:
            try:
                if self.token:
                    self.session.stop_h3_tune_nowait(self.token)
                self._update(phase="returning", wait_reason="测试到时，支架正在回水平",
                             message="本轮已正常停止")
                self._event("stop", reason="trial_timeout", metrics=self.analyzer.result())
            finally:
                self.trial_deadline = None
        local = self.state()
        latest = local.get("latest", {})
        host_active = bool(int(latest.get("flags", 0)) & 128)
        terminal_hint = bool(
            self.snapshot is not None and
            int(latest.get("run_id", -1)) == int(self.snapshot.run_id) and
            not host_active and int(latest.get("state", -1)) in (0, 6)
        )
        active_hint = bool(
            host_active or (self.snapshot is not None and
                            self.snapshot.host_run_active)
        )
        telemetry_age_ms = local.get("telemetry_age_ms")
        telemetry_stale = (
            telemetry_age_ms is None or int(telemetry_age_ms) > 250
        )
        if active_hint and telemetry_stale:
            if local.get("phase") != "telemetry_stale":
                self.trial_deadline = None
                if self.token:
                    self.session.stop_h3_tune_nowait(self.token)
                self._update(
                    phase="telemetry_stale",
                    wait_reason="H3 遥测中断；已停止发送心跳并请求正常停止",
                    message="遥测中断，拒绝继续闭环",
                )
                self._event(
                    "telemetry_stale", age_ms=telemetry_age_ms,
                    run_id=int(latest.get("run_id", 0)),
                )
            return
        if terminal_hint:
            try:
                self.snapshot = self.session.get_h3_tune()
                self._snapshot_state(self.snapshot)
            except Exception as exc:
                self._update(heartbeat_error=str(exc))
        elif self.token and active_hint:
            try:
                self.session.heartbeat_h3_tune_nowait(self.token)
                count = int(self.state().get("heartbeat_count", 0)) + 1
                self._update(heartbeat_count=count, heartbeat_error="")
            except Exception as exc:
                self.trial_deadline = None
                self._update(phase="heartbeat_error", heartbeat_error=str(exc),
                             wait_reason="心跳发送失败，正在安全停止")
                self._event("heartbeat_error", error=str(exc))
                self.session.emergency_stop()
        if self.snapshot is not None and not self.snapshot.host_run_active:
            phase = self.state().get("phase")
            if phase == "returning":
                self._update(phase="idle", wait_reason="可修改一个参数或开始下一轮",
                             message="支架已回水平")

    def _safe_close(self) -> None:
        if self.session is None:
            return
        try:
            snapshot = self.session.get_h3_tune()
            if snapshot.host_run_active and snapshot.auth_token:
                token = int(snapshot.auth_token)
                try:
                    self.session.stop_h3_tune(token)
                    deadline = time.monotonic() + 3.0
                    while time.monotonic() < deadline:
                        time.sleep(0.1)
                        snapshot = self.session.get_h3_tune()
                        if not snapshot.host_run_active:
                            break
                        self.session.heartbeat_h3_tune(token)
                except Exception:
                    self.session.emergency_stop()
            snapshot = self.session.get_h3_tune()
            if snapshot.authorized and not snapshot.host_run_active:
                self.session.revoke_h3_tune(int(snapshot.auth_token))
        except Exception:
            pass
        try:
            self.session.set_vision_uart_monitor(self.prior_monitor)
        except Exception:
            pass
        self.session.close()

    def _run(self) -> None:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        try:
            self.session = Session(
                port=self.args.port, timeout=0.8, firmware_id=self.args.firmware_id,
            )
            identity = self.session.connect()
            if not int(identity.enabled_capabilities) & H3_TUNE_CAPABILITY:
                raise RuntimeError("固件没有 m6-h3-tune-v1 capability bit 31")
            self.snapshot = self.session.get_h3_tune()
            self.prior_monitor = self.session.get_vision_uart_monitor().receive_enabled
            if self.prior_monitor:
                self.session.set_vision_uart_monitor(False)
            self._snapshot_state(self.snapshot)
            self._update(
                connected=True, phase="idle", wait_reason="点击开始后轻拨小球约 ±10 mm",
                firmware_id=identity.firmware_id, boot_id=int(identity.boot_id),
                message="串口与 H3 schema 3 遥测已连接",
            )
            self._event("connected", firmware_id=identity.firmware_id,
                        boot_id=int(identity.boot_id), port=self.args.port)
            self.ready.set()
            with self.capture_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
                writer.writeheader(); stream.flush()
                with self.session.event_tap() as events:
                    next_control = time.monotonic()
                    while not self.stop_event.is_set():
                        self._drain_commands()
                        try:
                            event = events.get(timeout=0.02)
                        except queue.Empty:
                            event = None
                        if event is not None:
                            row = h3_row(event)
                            if row is not None:
                                self._ingest(row, writer, stream)
                        now = time.monotonic()
                        if now >= next_control:
                            self._poll_control(now)
                            next_control = now + 0.05
        except Exception as exc:
            self._update(connected=False, phase="error", error=str(exc),
                         wait_reason="串口所有权或固件预检失败", message=str(exc))
            try:
                self._event("owner_error", error=str(exc))
            except OSError:
                pass
            self.ready.set()
        finally:
            self._safe_close()
            self._update(connected=False)


ASSETS = {
    "/": ("h3_pid_scope.html", "text/html; charset=utf-8"),
    "/h3_pid_scope.css": ("h3_pid_scope.css", "text/css; charset=utf-8"),
    "/h3_pid_scope.js": ("h3_pid_scope.js", "text/javascript; charset=utf-8"),
}


class ScopeHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], owner: ScopeOwner) -> None:
        self.owner = owner
        super().__init__(address, ScopeHandler)


class ScopeHandler(BaseHTTPRequestHandler):
    server: ScopeHTTPServer

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def _bytes(self, status: int, body: bytes, content_type: str,
               extra: dict[str, str] | None = None) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if extra:
            for name, value in extra.items():
                self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value: Any) -> None:
        self._bytes(status, json.dumps(value, ensure_ascii=False).encode("utf-8"),
                    "application/json; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlsplit(self.path)
        if parsed.path == "/api/state":
            self._json(200, {"ok": True, "state": self.server.owner.state()})
            return
        if parsed.path == "/api/samples":
            try:
                after = max(0, int(parse_qs(parsed.query).get("after", ["0"])[0]))
            except ValueError:
                after = 0
            self._json(200, self.server.owner.samples_after(after))
            return
        if parsed.path == "/api/export.csv":
            body = self.server.owner.export_csv()
            self._bytes(200, body, "text/csv; charset=utf-8",
                        {"Content-Disposition": "attachment; filename=h3_pid_scope.csv"})
            return
        asset = ASSETS.get(parsed.path)
        if asset is None:
            self._bytes(404, b"Not found\n", "text/plain; charset=utf-8")
            return
        filename, content_type = asset
        try:
            body = (WEB_ROOT / filename).read_bytes()
        except OSError:
            self._bytes(500, b"Scope asset missing\n", "text/plain; charset=utf-8")
            return
        self._bytes(200, body, content_type)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlsplit(self.path)
        routes = {
            "/api/start": "start", "/api/stop": "stop",
            "/api/parameter": "parameter", "/api/marker": "marker",
            "/api/estop": "estop",
        }
        command = routes.get(parsed.path)
        if command is None:
            self._json(405, {"ok": False, "error": "method not allowed"})
            return
        origin = self.headers.get("Origin", "")
        host = self.headers.get("Host", "")
        if origin and not origin.endswith(host):
            self._json(403, {"ok": False, "error": "cross-origin control rejected"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                raise ValueError("request too large")
            raw = self.rfile.read(length) if length else b"{}"
            body = json.loads(raw.decode("utf-8"))
            if not isinstance(body, dict):
                raise ValueError("JSON object required")
            result = self.server.owner.submit(command, body)
            self._json(200, result)
        except Exception as exc:
            self._json(409, {"ok": False, "error": str(exc)})


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H3 PID serial waveform UI")
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--firmware-id", default=FIRMWARE_ID)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--max-samples", type=int, default=12_000)
    parser.add_argument("--output", type=Path, default=ROOT / "captures" / "h3_pid_scope")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)
    if not 2.0 <= args.duration <= 11.5:
        parser.error("--duration must be between 2 and 11.5 seconds")
    if args.max_samples < 100:
        parser.error("--max-samples must be at least 100")
    args.output = args.output.resolve()
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    owner = ScopeOwner(args)
    owner.start()
    owner.ready.wait(5.0)
    state = owner.state()
    if state["phase"] == "error":
        owner.close()
        print(f"H3 PID Scope 启动失败：{state['error']}", file=sys.stderr)
        print("如果 COM10 被 h3_autotune 占用，请先在原终端按 Ctrl+C。", file=sys.stderr)
        return 2
    server = ScopeHTTPServer((args.host, args.http_port), owner)
    url = f"http://{args.host}:{args.http_port}/"
    print(f"H3 PID Scope: {url}")
    print(f"capture: {owner.capture_dir}")
    print("Ctrl+C 将先正常停止、回水平、撤销授权，再关闭串口。")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        owner.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
