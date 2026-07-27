"""Read-only local web dashboard for live M6 telemetry."""

from __future__ import annotations

import json
import secrets
import threading
import time
import webbrowser
from collections import deque
from collections.abc import Mapping
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlsplit

from .dashboard_control import DashboardController
from .dashboard_recording import DashboardRecordingManager
from .errors import M6Error
from .monitor import MONITOR_SUBSCRIPTIONS
from .models import TelemetrySample


_WEB_ROOT = Path(__file__).with_name("web")
_ASSETS = {
    "/": ("dashboard.html", "text/html; charset=utf-8"),
    "/dashboard.css": ("dashboard.css", "text/css; charset=utf-8"),
    "/dashboard.js": ("dashboard.js", "text/javascript; charset=utf-8"),
}
_RECONNECT_SILENCE_S = 2.0
_RECONNECT_RETRY_S = 1.0


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


class DashboardBuffer:
    """Thread-safe cursor buffer that preserves every decoded telemetry sample."""

    def __init__(
        self,
        identity: Mapping[str, Any],
        *,
        max_events: int = 12_000,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_events < 1:
            raise ValueError("max_events must be positive")
        self._identity = _json_safe(identity)
        self._events: deque[dict[str, Any]] = deque(maxlen=max_events)
        self._latest: dict[str, dict[str, Any]] = {}
        self._counts: dict[str, int] = {}
        self._gaps: dict[str, int] = {}
        self._cursor = 0
        self._clock = clock
        self._started_at = clock()
        self._connected = True
        self._reconnecting = False
        self._connection_message = "串口数据在线"
        self._reconnect_count = 0
        self._lock = threading.Lock()

    def ingest(self, sample: TelemetrySample, *, received_at: float | None = None) -> int:
        now = self._clock() if received_at is None else received_at
        stream = sample.stream.name.lower()
        with self._lock:
            self._cursor += 1
            event = {
                "cursor": self._cursor,
                "received_ms": max(0, round((now - self._started_at) * 1000)),
                "stream": stream,
                "stream_id": int(sample.stream),
                "sample_id": sample.sample_id,
                "time_ms": sample.time_ms,
                "gap": sample.gap_before,
                "missing": 1 if sample.gap_before else 0,
                "values": _json_safe(sample.values),
            }
            self._events.append(event)
            self._latest[stream] = event
            self._counts[stream] = self._counts.get(stream, 0) + 1
            self._gaps[stream] = self._gaps.get(stream, 0) + (1 if sample.gap_before else 0)
            return self._cursor

    def set_connected(
        self,
        connected: bool,
        *,
        reconnecting: bool = False,
        message: str | None = None,
    ) -> None:
        with self._lock:
            self._connected = connected
            self._reconnecting = reconnecting
            if message is not None:
                self._connection_message = message

    def set_identity(self, identity: Mapping[str, Any], *, reconnected: bool = False) -> None:
        with self._lock:
            self._identity = _json_safe(identity)
            if reconnected:
                self._reconnect_count += 1

    def read_since(self, after: int = 0, *, limit: int = 2_000) -> dict[str, Any]:
        limit = min(max(limit, 1), 10_000)
        with self._lock:
            earliest = self._events[0]["cursor"] if self._events else self._cursor + 1
            reset = after < earliest - 1 or after > self._cursor
            if reset:
                after = earliest - 1
            events = [event for event in self._events if event["cursor"] > after][:limit]
            next_cursor = events[-1]["cursor"] if events else max(after, self._cursor if not self._events else after)
            has_more = next_cursor < self._cursor
            return {
                "cursor": next_cursor,
                "server_cursor": self._cursor,
                "reset": reset,
                "has_more": has_more,
                "connected": self._connected,
                "reconnecting": self._reconnecting,
                "connection_message": self._connection_message,
                "reconnect_count": self._reconnect_count,
                "uptime_ms": round((self._clock() - self._started_at) * 1000),
                "identity": self._identity,
                "counts": dict(self._counts),
                "gaps": dict(self._gaps),
                "latest": dict(self._latest),
                "events": events,
            }


class DashboardHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        buffer: DashboardBuffer,
        controller: DashboardController | None = None,
        recording_manager: DashboardRecordingManager | None = None,
    ) -> None:
        self.dashboard_buffer = buffer
        self.dashboard_controller = controller
        self.recording_manager = recording_manager
        super().__init__(address, DashboardRequestHandler)


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server: DashboardHTTPServer

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlsplit(self.path)
        if parsed.path == "/api/state":
            self._send_state(parse_qs(parsed.query))
            return
        if parsed.path == "/api/control":
            self._send_controller_state()
            return
        if parsed.path == "/api/parameters":
            self._send_parameter_state(parse_qs(parsed.query))
            return
        if parsed.path == "/api/recordings":
            manager = self.server.recording_manager
            self._send_json(200, manager.list_sessions() if manager else {"ok": True, "recordings": []})
            return
        if parsed.path == "/api/recordings/status":
            manager = self.server.recording_manager
            self._send_json(200, manager.status() if manager else {"ok": True, "recording": False})
            return
        if parsed.path.startswith("/api/recordings/"):
            self._send_recording_resource(parsed.path, parse_qs(parsed.query))
            return
        asset = _ASSETS.get(parsed.path)
        if asset is None:
            self._send_bytes(404, b"Not found\n", "text/plain; charset=utf-8")
            return
        filename, content_type = asset
        try:
            content = (_WEB_ROOT / filename).read_bytes()
        except OSError:
            self._send_bytes(500, b"Dashboard asset missing\n", "text/plain; charset=utf-8")
            return
        self._send_bytes(200, content, content_type)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlsplit(self.path)
        manager = self.server.recording_manager
        recording_routes = {
            "/api/recordings/start": lambda body: manager.start_manual(str(body.get("name", "")), str(body.get("notes", ""))) if manager else None,
            "/api/recordings/stop": lambda _body: manager.stop() if manager else None,
            "/api/recordings/marker": lambda body: manager.add_marker(str(body.get("label", ""))) if manager else None,
        }
        routes = {
            "/api/control/arm": lambda controller, body: controller.arm(body),
            "/api/control/confirm": lambda controller, body: controller.confirm(body),
            "/api/control/cancel": lambda controller, _body: controller.cancel(),
            "/api/control/estop": lambda controller, _body: controller.emergency_stop(),
            "/api/control/clear-fault": lambda controller, body: controller.clear_fault(body),
            "/api/control/manual/enable": lambda controller, _body: controller.manual_enable(),
            "/api/control/manual/command": lambda controller, body: controller.manual_command(body),
            "/api/control/manual/disable": lambda controller, body: controller.manual_disable(body),
            "/api/parameters/stage": lambda controller, body: controller.parameter_command("stage", body),
            "/api/parameters/apply": lambda controller, body: controller.parameter_command("apply", body),
            "/api/parameters/revert": lambda controller, body: controller.parameter_command("revert", body),
            "/api/parameters/load": lambda controller, body: controller.parameter_command("load", body),
            "/api/parameters/defaults": lambda controller, body: controller.parameter_command("defaults", body),
            "/api/parameters/save": lambda controller, body: controller.parameter_command("save", body),
        }
        recording_route = recording_routes.get(parsed.path)
        route = routes.get(parsed.path)
        if route is None and recording_route is None:
            self._send_bytes(405, b"Method not allowed\n", "text/plain; charset=utf-8")
            return
        controller = self.server.dashboard_controller
        if controller is None:
            self._send_json(404, {"ok": False, "error": {"message": "控制功能未启用"}})
            return
        if not self._authorized(controller):
            self._send_json(403, {"ok": False, "error": {"message": "控制令牌无效"}})
            return
        try:
            body = self._read_json_body()
            if recording_route is not None:
                if manager is None:
                    self._send_json(404, {"ok": False, "error": {"message": "录制功能未启用"}})
                    return
                result = recording_route(body)
            else:
                previous_revision = None
                if parsed.path.startswith("/api/parameters/"):
                    previous_revision = controller.parameter_state().get("summary", {})
                result = route(controller, body)
                if manager is not None:
                    event_name = {
                        "/api/control/arm": "control_arm",
                        "/api/control/confirm": "control_confirm",
                        "/api/control/cancel": "control_cancel",
                        "/api/control/estop": "emergency_stop_requested",
                        "/api/control/clear-fault": "fault_clear_requested",
                        "/api/control/manual/enable": "manual_drive_enabled",
                        "/api/control/manual/command": "manual_drive_input_changed",
                        "/api/control/manual/disable": "manual_drive_disabled",
                    }.get(parsed.path, f"parameter_{parsed.path.rsplit('/', 1)[-1]}")
                    event_values: dict[str, Any] = {"request": body, "result": result}
                    if previous_revision is not None:
                        event_values["revision_before"] = previous_revision
                        event_values["revision_after"] = controller.parameter_state(force=True).get("summary", {})
                    if parsed.path != "/api/control/manual/command" or result.get("changed") is True:
                        manager.record_event(event_name, event_values)
                    recording = body.get("recording")
                    if parsed.path == "/api/control/confirm" and isinstance(recording, Mapping) and recording.get("auto") is True:
                        try:
                            result["recording"] = manager.start_action(
                                name=str(recording.get("name", "")),
                                notes=str(recording.get("notes", "")),
                                action=result.get("action"),
                            )
                        except (OSError, ValueError) as exc:
                            # Motion was already confirmed. Report the recording failure
                            # without making the client believe Confirm itself failed.
                            result["recording_error"] = str(exc)
        except (ValueError, KeyError, TypeError) as exc:
            self._send_json(400, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})
            return
        except M6Error as exc:
            self._send_json(409, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})
            return
        except OSError as exc:
            self._send_json(503, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})
            return
        self._send_json(200, result)

    def _send_recording_resource(self, path: str, query: dict[str, list[str]]) -> None:
        manager = self.server.recording_manager
        if manager is None:
            self._send_json(404, {"ok": False, "error": {"message": "录制功能未启用"}})
            return
        parts = path.strip("/").split("/")
        if len(parts) != 4 or parts[:2] != ["api", "recordings"]:
            self._send_json(404, {"ok": False, "error": {"message": "录制资源不存在"}})
            return
        session_id, resource = parts[2], parts[3]
        try:
            if resource == "manifest":
                self._send_json(200, manager.manifest(session_id))
                return
            if resource == "timeline":
                from_ms = int(query.get("from_ms", ["0"])[0])
                after = int(query.get("after", ["0"])[0])
                limit = int(query.get("limit", ["2000"])[0])
                self._send_json(200, manager.timeline(session_id, from_ms=from_ms, after=after, limit=limit))
                return
            if resource == "download":
                archive = manager.create_download(session_id)
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/zip")
                    self.send_header("Content-Disposition", f'attachment; filename="m6-{session_id}.zip"')
                    self.send_header("Content-Length", str(archive.stat().st_size))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.end_headers()
                    with archive.open("rb") as source:
                        while chunk := source.read(1024 * 1024):
                            self.wfile.write(chunk)
                finally:
                    archive.unlink(missing_ok=True)
                return
            self._send_json(404, {"ok": False, "error": {"message": "录制资源不存在"}})
        except (ValueError, TypeError) as exc:
            self._send_json(400, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})
        except OSError as exc:
            self._send_json(503, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})

    def _send_state(self, query: dict[str, list[str]]) -> None:
        try:
            after = max(0, int(query.get("after", ["0"])[0]))
            limit = int(query.get("limit", ["2000"])[0])
        except ValueError:
            self._send_bytes(400, b"Invalid cursor\n", "text/plain; charset=utf-8")
            return
        self._send_json(200, self.server.dashboard_buffer.read_since(after, limit=limit))

    def _send_controller_state(self) -> None:
        controller = self.server.dashboard_controller
        if controller is None:
            self._send_json(404, {"ok": False, "error": {"message": "控制功能未启用"}})
            return
        try:
            result = controller.control_state()
        except M6Error as exc:
            self._send_json(503, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})
            return
        self._send_json(200, result)

    def _send_parameter_state(self, query: dict[str, list[str]]) -> None:
        controller = self.server.dashboard_controller
        if controller is None:
            self._send_json(404, {"ok": False, "error": {"message": "参数功能未启用"}})
            return
        force = query.get("refresh", ["0"])[0] == "1"
        try:
            result = controller.parameter_state(force=force)
        except M6Error as exc:
            self._send_json(503, {"ok": False, "error": {"type": type(exc).__name__, "message": str(exc)}})
            return
        self._send_json(200, result)

    def _authorized(self, controller: DashboardController) -> bool:
        supplied = self.headers.get("X-M6-Control-Token", "")
        if not supplied or not secrets.compare_digest(supplied, controller.token):
            return False
        origin = self.headers.get("Origin")
        if not origin:
            return True
        port = self.server.server_address[1]
        return origin in {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}

    def _read_json_body(self) -> dict[str, Any]:
        content_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
        if content_type != "application/json":
            raise ValueError("请求必须使用 application/json")
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError as exc:
            raise ValueError("缺少有效的 Content-Length") from exc
        if length < 0 or length > 8192:
            raise ValueError("请求体过大")
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("请求 JSON 无效") from exc
        if not isinstance(value, dict):
            raise ValueError("请求 JSON 必须是对象")
        return value

    def _send_json(self, status: int, value: Any) -> None:
        payload = json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=str).encode("utf-8")
        self._send_bytes(status, payload, "application/json; charset=utf-8")

    def _send_bytes(self, status: int, content: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self'; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, _format: str, *_args: Any) -> None:
        return


def create_dashboard_server(
    buffer: DashboardBuffer,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    controller: DashboardController | None = None,
    recording_manager: DashboardRecordingManager | None = None,
) -> DashboardHTTPServer:
    return DashboardHTTPServer((host, port), buffer, controller, recording_manager)


def _identity_mapping(identity: Any) -> dict[str, Any]:
    if isinstance(identity, Mapping):
        return dict(identity)
    fields = (
        "firmware_id", "wire_version", "registry_revision", "parameter_schema",
        "active_revision", "generation", "boot_id", "source", "capabilities",
        "enabled_capabilities", "image_flags", "port",
    )
    return {field: getattr(identity, field) for field in fields if hasattr(identity, field)}


def run_dashboard(
    session: Any,
    identity: Any,
    *,
    port_name: str = "",
    http_port: int = 8765,
    duration_s: float | None = None,
    open_browser: bool = True,
    recordings_dir: str | Path = "recordings",
) -> int:
    """Run a local HTTP dashboard while consuming telemetry from ``session``."""

    identity_data = _identity_mapping(identity)
    if port_name:
        identity_data["port"] = port_name
    buffer = DashboardBuffer(identity_data)
    controller = DashboardController(session, identity)
    session.subscribe(MONITOR_SUBSCRIPTIONS)
    recording_manager = DashboardRecordingManager(
        session,
        identity_data,
        recordings_dir,
        parameter_provider=lambda: controller.parameter_state(),
        connected_provider=lambda: bool(getattr(session, "connected", True)),
        sampling_periods={stream.name.lower(): period for stream, period in MONITOR_SUBSCRIPTIONS.items()},
    )
    recording_manager.start()
    server: DashboardHTTPServer | None = None
    server_thread: threading.Thread | None = None
    try:
        server = create_dashboard_server(
            buffer, port=http_port, controller=controller, recording_manager=recording_manager
        )
        server_thread = threading.Thread(
            target=server.serve_forever, name="m6-dashboard-http", daemon=True
        )
        server_thread.start()
        actual_port = server.server_address[1]
        url = f"http://127.0.0.1:{actual_port}/"
        print(f"M6 dashboard: {url}")
        print("Motion requires Arm/Confirm; emergency stop remains available. Press Ctrl+C to exit.")
        if open_browser:
            webbrowser.open(url)

        started_at = time.monotonic()
        last_sample_at = started_at
        next_reconnect_at = started_at
        reconnect_announced = False
        while duration_s is None or time.monotonic() - started_at < duration_s:
            now = time.monotonic()
            transport_alive = bool(getattr(session, "connected", True))
            silent = now - last_sample_at >= _RECONNECT_SILENCE_S
            if not transport_alive or silent:
                if not reconnect_announced:
                    recording_manager.record_event(
                        "connection_lost",
                        {"transport_alive": transport_alive, "silence_ms": round((now - last_sample_at) * 1000)},
                    )
                    reconnect_announced = True
                buffer.set_connected(
                    False,
                    reconnecting=True,
                    message="车辆无数据，正在等待重新上电并自动连接…",
                )
                if now < next_reconnect_at:
                    time.sleep(min(0.05, next_reconnect_at - now))
                    continue
                try:
                    identity = controller.reconnect(MONITOR_SUBSCRIPTIONS)
                except (M6Error, OSError, ValueError) as exc:
                    buffer.set_connected(
                        False,
                        reconnecting=True,
                        message=f"自动重连等待中：{exc}",
                    )
                    next_reconnect_at = time.monotonic() + _RECONNECT_RETRY_S
                    continue
                buffer.set_identity(_identity_mapping(identity), reconnected=True)
                buffer.set_connected(True, message="车辆已重新连接")
                recording_manager.update_identity(_identity_mapping(identity))
                recording_manager.record_event("connection_restored", {"port": port_name})
                last_sample_at = time.monotonic()
                next_reconnect_at = last_sample_at
                reconnect_announced = False
                continue
            sample = session.read_telemetry(timeout=0.1)
            if sample is not None:
                buffer.ingest(sample)
                last_sample_at = time.monotonic()
    except KeyboardInterrupt:
        return 0
    finally:
        buffer.set_connected(False, message="串口服务已停止")
        recording_manager.close()
        controller.close()
        try:
            session.subscribe({})
        except Exception:
            pass
        if server is not None:
            if server_thread is not None and server_thread.is_alive():
                server.shutdown()
                server_thread.join(timeout=1.0)
            server.server_close()
    return 0
