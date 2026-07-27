from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from m6link.cli import _create_parser
from m6link.dashboard import DashboardBuffer, create_dashboard_server, run_dashboard
from m6link.dashboard_control import DashboardController
from m6link.dashboard_recording import DashboardRecordingManager
from m6link.errors import Disconnected
from m6link.models import (
    ActionStatus,
    ArmedAction,
    ManualDriveSnapshot,
    ParameterSnapshot,
    ParameterValue,
    Status,
    TelemetrySample,
)
from m6link.registry import (
    REGISTRY,
    ActionKind,
    ActionState,
    Capability,
    ManualDriveSource,
    ManualDriveStopReason,
    ParameterSource,
    ParameterType,
    TelemetryStream,
)


def _sample(sample_id: int, *, gap: bool = False) -> TelemetrySample:
    return TelemetrySample(
        TelemetryStream.CORE,
        sample_id,
        sample_id * 20,
        {"speed_l_mm_s": sample_id, "adc": (1, 2, 3)},
        gap_before=gap,
    )


class FakeControlSession:
    def __init__(self) -> None:
        self.capabilities = (
            Capability.FAULT_CLEAR
            | Capability.WHEEL_DIAG_ACTION
            | Capability.HEADING_ACTION
            | Capability.HEADING_RATE_ACTION
            | Capability.LINE_ACTION
            | Capability.MOTOR_SELF_TEST
            | Capability.MANUAL_DRIVE
        )
        self.fault_bits = 0
        self.armed_specs = []
        self.confirmed = []
        self.cancelled = 0
        self.estops = 0
        self.cleared_faults = []
        self.parameter_commands = []
        self.manual_commands = []
        self.manual_snapshot = ManualDriveSnapshot(
            0, 0, (0, 0), ManualDriveSource.NONE, False,
            ManualDriveStopReason.NONE,
        )
        self._stage_revision = 1
        self.active_values = {
            str(spec["key"]): spec["default"] for spec in REGISTRY["parameters"]
        }
        self.staged_values = dict(self.active_values)

    def get_status(self) -> Status:
        return Status(
            time_ms=100,
            system_state=1,
            action_state=ActionState.IDLE,
            action_kind=ActionKind.NONE,
            fault_bits=self.fault_bits,
            first_fault_ms=0,
            last_fault_ms=0,
            fault_count=0,
            required_sensor_mask=0,
            action_deadline_ms=0,
            requested_pwm=(0, 0),
            applied_pwm=(0, 0),
            target_mm_s=(0, 0),
            measured_mm_s=(0, 0),
            active_revision=1,
            stage_revision=self._stage_revision,
            source=ParameterSource.DEFAULTS,
        )

    def arm_action(self, spec):
        self.armed_specs.append(spec)
        return ArmedAction(12345, 10_000, spec, time.monotonic())

    def confirm_action(self, armed):
        self.confirmed.append(armed)
        return ActionStatus(7, ActionState.RUNNING, armed.spec.kind, start_ms=100)

    def cancel_action(self, _action_id=0):
        self.cancelled += 1
        return ActionStatus(7, ActionState.COMPLETE)

    def emergency_stop(self):
        self.estops += 1

    def clear_fault(self, observed_fault_bits):
        assert observed_fault_bits == self.fault_bits
        self.cleared_faults.append(observed_fault_bits)
        self.fault_bits = 0
        return {
            "boot_id": 7,
            "cleared_fault_bits": observed_fault_bits,
            "system_state": 1,
        }

    def get_manual_drive(self):
        return self.manual_snapshot

    def set_manual_drive(self, left, right, *, lease_ms, takeover, command_id):
        self.manual_commands.append(("set", left, right, lease_ms, takeover, command_id))
        self.manual_snapshot = ManualDriveSnapshot(
            command_id, 100 + lease_ms, (left, right),
            ManualDriveSource.HOST_RPC, True, ManualDriveStopReason.NONE,
        )
        return self.manual_snapshot

    def stop_manual_drive(self, *, command_id):
        self.manual_commands.append(("stop", command_id))
        self.manual_snapshot = ManualDriveSnapshot(
            command_id, 0, (0, 0), ManualDriveSource.NONE, False,
            ManualDriveStopReason.RELEASE,
        )
        return self.manual_snapshot

    def get_parameters(self) -> ParameterSnapshot:
        values = {}
        for spec in REGISTRY["parameters"]:
            parameter_type = ParameterType[str(spec["type"])]
            values[str(spec["key"])] = ParameterValue(
                int(spec["id"]),
                str(spec["key"]),
                parameter_type,
                self.active_values[str(spec["key"])],
                self.staged_values[str(spec["key"])],
                ParameterSource.DEFAULTS,
                ParameterSource.DEFAULTS,
                1,
                self._stage_revision,
                0,
            )
        return ParameterSnapshot(1, 1, self._stage_revision, 0, ParameterSource.DEFAULTS, values)

    def stage_parameter(self, key, value):
        self.parameter_commands.append(("stage", key, value))
        self.staged_values[key] = value
        self._stage_revision += 1
        return self.get_parameters().values[key]

    def _parameter_command(self, name):
        self.parameter_commands.append((name,))
        return {"command": name}

    def apply_parameters(self):
        self.active_values = dict(self.staged_values)
        return self._parameter_command("apply")

    def revert_parameters(self):
        self.staged_values = dict(self.active_values)
        return self._parameter_command("revert")
    def load_parameters(self): return self._parameter_command("load")
    def default_parameters(self): return self._parameter_command("defaults")
    def save_parameters(self): return self._parameter_command("save")


def test_dashboard_buffer_preserves_all_samples_between_browser_polls() -> None:
    now = [100.0]
    buffer = DashboardBuffer({"firmware_id": "test"}, clock=lambda: now[0])
    for sample_id in range(1, 6):
        now[0] += 0.02
        buffer.ingest(_sample(sample_id), received_at=now[0])

    first = buffer.read_since(0, limit=2)
    assert [event["sample_id"] for event in first["events"]] == [1, 2]
    assert first["cursor"] == 2
    assert first["has_more"] is True

    second = buffer.read_since(first["cursor"])
    assert [event["sample_id"] for event in second["events"]] == [3, 4, 5]
    assert second["cursor"] == 5
    assert second["has_more"] is False
    assert second["latest"]["core"]["sample_id"] == 5
    assert second["latest"]["core"]["values"]["adc"] == [1, 2, 3]


def test_dashboard_buffer_reports_overflow_reset_and_gap() -> None:
    buffer = DashboardBuffer({}, max_events=2)
    buffer.ingest(_sample(1))
    buffer.ingest(_sample(2, gap=True))
    buffer.ingest(_sample(3))

    state = buffer.read_since(0)
    assert state["reset"] is True
    assert [event["sample_id"] for event in state["events"]] == [2, 3]
    assert state["gaps"] == {"core": 1}


def test_dashboard_buffer_recovers_from_cursor_created_by_old_server() -> None:
    buffer = DashboardBuffer({})
    buffer.ingest(_sample(1))
    state = buffer.read_since(9999)
    assert state["reset"] is True
    assert [event["sample_id"] for event in state["events"]] == [1]
    assert state["cursor"] == 1


def test_dashboard_buffer_exposes_reconnect_state_and_new_identity() -> None:
    buffer = DashboardBuffer({"firmware_id": "before", "port": "COM10"})
    buffer.set_connected(
        False,
        reconnecting=True,
        message="车辆无数据，正在等待重新上电并自动连接…",
    )
    waiting = buffer.read_since(0)
    assert waiting["connected"] is False
    assert waiting["reconnecting"] is True
    assert "自动连接" in waiting["connection_message"]
    buffer.set_identity({"firmware_id": "after", "port": "COM10"}, reconnected=True)
    buffer.set_connected(True, message="车辆已重新连接")
    online = buffer.read_since(0)
    assert online["identity"]["firmware_id"] == "after"
    assert online["reconnect_count"] == 1
    assert online["connection_message"] == "车辆已重新连接"


def test_dashboard_http_serves_assets_state_and_rejects_writes() -> None:
    buffer = DashboardBuffer({"firmware_id": "fw-test", "port": "COM1"})
    buffer.ingest(_sample(7))
    server = create_dashboard_server(buffer, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(base + "/", timeout=2) as response:
            html = response.read().decode("utf-8")
            assert response.status == 200
            assert "M6 实时遥测" in html
            assert 'id="layoutCockpitButton"' in html
            assert 'id="parameterDialog"' in html
            assert 'id="startRecordingButton"' in html
            assert 'id="recordingLibraryDialog"' in html
            assert 'id="playbackBar"' in html
            assert 'id="manualDrivePanel"' in html
            assert 'id="manualEnableButton"' in html
        with urlopen(base + "/dashboard.js", timeout=2) as response:
            javascript = response.read()
            assert b"requestAnimationFrame" in javascript
            assert b"m6.dashboard.layout.v1" in javascript
            assert b"drawLineVisual" in javascript
            assert b"drawGyroVisual" in javascript
            assert b"openPlayback" in javascript
            assert b"seekPlayback" in javascript
            assert b"applyPlaybackLock" in javascript
            assert b"sendManualCommand" in javascript
            assert b"m6.dashboard.manual-drive.v1" in javascript
        with urlopen(base + "/dashboard.css", timeout=2) as response:
            stylesheet = response.read()
            assert b'data-layout="cockpit"' in stylesheet
            assert b"100dvh" in stylesheet
            assert b"playback-bar" in stylesheet
            assert b"recording-card" in stylesheet
            assert b"wasd-keys" in stylesheet
        with urlopen(base + "/api/state?after=0", timeout=2) as response:
            state = json.load(response)
            assert state["events"][0]["sample_id"] == 7
            assert response.headers["Cache-Control"] == "no-store"
        request = Request(base + "/api/state", method="POST")
        try:
            urlopen(request, timeout=2)
            raise AssertionError("POST should be rejected")
        except HTTPError as exc:
            assert exc.code == 405
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def test_dashboard_controller_requires_safety_ack_and_two_phase_confirmation() -> None:
    session = FakeControlSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    request = {
        "kind": "WHEEL_SPEED_TEST",
        "target_l_mm_s": 120,
        "target_r_mm_s": 120,
        "duration_ms": 1500,
    }
    try:
        controller.arm(request)
        raise AssertionError("safety acknowledgement should be required")
    except ValueError as exc:
        assert "安全" in str(exc)

    armed = controller.arm({**request, "safety_ack": True})["armed"]
    assert session.armed_specs[0].target_l_mm_s == 120
    try:
        controller.confirm({"arm_token": armed["arm_token"] + 1})
        raise AssertionError("wrong arm token should be rejected")
    except ValueError:
        pass
    confirmed = controller.confirm({"arm_token": armed["arm_token"]})
    assert confirmed["action"]["action_id"] == 7
    assert len(session.confirmed) == 1


def test_manual_drive_mixing_lease_and_heartbeat_coalescing() -> None:
    session = FakeControlSession()
    controller = DashboardController(
        session, SimpleNamespace(enabled_capabilities=session.capabilities)
    )
    try:
        assert controller._mix_manual_targets(1, 0, 160, 65) == (160, 160)
        assert controller._mix_manual_targets(0, 1, 160, 65) == (-104, 104)
        assert controller._mix_manual_targets(1, 1, 160, 65) == (34, 160)
        controller_id = controller.manual_enable()["controller_id"]
        first = controller.manual_command({
            "controller_id": controller_id,
            "sequence": 1,
            "throttle": 1,
            "steering": 1,
            "max_speed_mm_s": 160,
            "steering_percent": 65,
        })
        assert first["target_mm_s"] == [34, 160]
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not session.manual_commands:
            time.sleep(0.005)
        assert session.manual_commands[-1] == ("set", 34, 160, 300, True, 1)

        heartbeat = controller.manual_command({
            "controller_id": controller_id,
            "sequence": 2,
            "throttle": 1,
            "steering": 1,
            "max_speed_mm_s": 160,
            "steering_percent": 65,
        })
        assert heartbeat["changed"] is False
        controller.manual_disable({"controller_id": controller_id})
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and session.manual_commands[-1][0] != "stop":
            time.sleep(0.005)
        assert session.manual_commands[-1][0] == "stop"
    finally:
        controller.close()


def test_dashboard_continuous_mode_is_line_only_and_uses_zero_duration() -> None:
    session = FakeControlSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    armed = controller.arm({
        "kind": "LINE_FOLLOW",
        "duration_ms": 1500,
        "continuous": True,
        "safety_ack": True,
    })["armed"]
    assert session.armed_specs[-1].duration_ms == 0
    assert armed["spec"]["duration_ms"] == 0
    try:
        controller.arm({
            "kind": "HEADING_HOLD",
            "target_l_mm_s": 100,
            "target_r_mm_s": 100,
            "continuous": True,
            "safety_ack": True,
        })
        raise AssertionError("continuous mode must remain line-only")
    except ValueError:
        pass


def test_dashboard_controller_blocks_motor_self_test_and_faulted_arm() -> None:
    session = FakeControlSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    try:
        controller.arm({"kind": "MOTOR_SELF_TEST", "duration_ms": 3000, "safety_ack": True})
        raise AssertionError("motor self test must stay unavailable")
    except ValueError as exc:
        assert "未在网页" in str(exc)
    session.fault_bits = 1
    try:
        controller.arm(
            {
                "kind": "LINE_FOLLOW",
                "duration_ms": 1000,
                "safety_ack": True,
            }
        )
        raise AssertionError("faulted action must be blocked")
    except ValueError as exc:
        assert "Safety" in str(exc)


def test_dashboard_decodes_and_clears_only_the_observed_faults() -> None:
    session = FakeControlSession()
    session.fault_bits = 1 | 256
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    state = controller.control_state(force=True)
    assert [item["name"] for item in state["fault_details"]] == [
        "EMERGENCY_STOP",
        "LINE_LOST",
    ]
    assert state["fault_details"][0]["title"] == "急停已触发"
    try:
        controller.clear_fault({
            "confirm": "CLEAR_OBSERVED_FAULTS",
            "observed_fault_bits": 1,
        })
        raise AssertionError("stale observed fault bits must be rejected")
    except ValueError as exc:
        assert "已变化" in str(exc)
    result = controller.clear_fault({
        "confirm": "CLEAR_OBSERVED_FAULTS",
        "observed_fault_bits": 257,
    })
    assert result["cleared_fault_bits"] == 257
    assert session.cleared_faults == [257]


def test_dashboard_parameter_commands_keep_persistence_explicit() -> None:
    session = FakeControlSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    state = controller.parameter_state()
    assert len(state["parameters"]) == len(REGISTRY["parameters"])
    assert state["parameters"][0]["key"] == "wheel.left.kp"
    controller.parameter_command("stage", {"key": "wheel.left.kp", "value": 0.8})
    controller.parameter_command("apply", {})
    for command, confirmation in (
        ("defaults", "LOAD_DEFAULTS_TO_STAGE"),
        ("save", "SAVE_ACTIVE_PARAMETERS"),
    ):
        try:
            controller.parameter_command(command, {})
            raise AssertionError(f"{command} should require confirmation")
        except ValueError:
            pass
        controller.parameter_command(command, {"confirm": confirmation})
    assert ("stage", "wheel.left.kp", 0.8) in session.parameter_commands
    assert ("save",) in session.parameter_commands


def test_control_http_requires_token_and_accepts_authenticated_estop() -> None:
    session = FakeControlSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    server = create_dashboard_server(DashboardBuffer({}), port=0, controller=controller)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(base + "/api/control", timeout=2) as response:
            state = json.load(response)
            token = state["control_token"]
            assert state["status"]["action_state_name"] == "IDLE"
        body = b"{}"
        unauthorized = Request(
            base + "/api/control/estop",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            urlopen(unauthorized, timeout=2)
            raise AssertionError("missing token should be rejected")
        except HTTPError as exc:
            assert exc.code == 403
        authorized = Request(
            base + "/api/control/estop",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json", "X-M6-Control-Token": token},
        )
        with urlopen(authorized, timeout=2) as response:
            assert json.load(response)["accepted"] is True
        assert session.estops == 1
        enable_manual = Request(
            base + "/api/control/manual/enable", data=body, method="POST",
            headers={"Content-Type": "application/json", "X-M6-Control-Token": token},
        )
        with urlopen(enable_manual, timeout=2) as response:
            controller_id = json.load(response)["controller_id"]
        manual_body = json.dumps({
            "controller_id": controller_id, "sequence": 1,
            "throttle": 1, "steering": -1,
            "max_speed_mm_s": 200, "steering_percent": 50,
        }).encode("utf-8")
        manual_command = Request(
            base + "/api/control/manual/command", data=manual_body, method="POST",
            headers={"Content-Type": "application/json", "X-M6-Control-Token": token},
        )
        with urlopen(manual_command, timeout=2) as response:
            assert json.load(response)["target_mm_s"] == [200, 67]
        session.fault_bits = 1
        clear_fault = Request(
            base + "/api/control/clear-fault",
            data=json.dumps({
                "confirm": "CLEAR_OBSERVED_FAULTS",
                "observed_fault_bits": 1,
            }).encode("utf-8"),
            method="POST",
            headers={
                "Content-Type": "application/json",
                "X-M6-Control-Token": token,
            },
        )
        with urlopen(clear_fault, timeout=2) as response:
            assert json.load(response)["cleared_fault_bits"] == 1
        assert session.cleared_faults == [1]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)
        controller.close()


def test_recording_http_api_requires_token_and_supports_library_download(tmp_path) -> None:
    session = FakeControlSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    recording = DashboardRecordingManager(
        session,
        {"firmware_id": "M6-test", "boot_id": 7},
        tmp_path / "recordings",
        parameter_provider=controller.parameter_state,
        minimum_free_bytes=0,
    )
    server = create_dashboard_server(
        DashboardBuffer({}), port=0, controller=controller, recording_manager=recording
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(base + "/api/control", timeout=2) as response:
            token = json.load(response)["control_token"]

        def post(path, payload, *, authorized=True):
            headers = {"Content-Type": "application/json"}
            if authorized:
                headers["X-M6-Control-Token"] = token
            request = Request(
                base + path,
                data=json.dumps(payload).encode("utf-8"),
                method="POST",
                headers=headers,
            )
            with urlopen(request, timeout=2) as response:
                return json.load(response)

        try:
            post("/api/recordings/start", {"name": "blocked"}, authorized=False)
            raise AssertionError("recording mutations require the control token")
        except HTTPError as exc:
            assert exc.code == 403
        started = post("/api/recordings/start", {"name": "手动", "notes": "HTTP test"})
        assert started["recording"] is True
        post("/api/recordings/marker", {"label": "起步"})
        post("/api/recordings/stop", {})
        with urlopen(base + "/api/recordings", timeout=2) as response:
            listing = json.load(response)["recordings"]
        session_id = listing[0]["session_id"]
        with urlopen(base + f"/api/recordings/{session_id}/manifest", timeout=2) as response:
            assert json.load(response)["name"] == "手动"
        with urlopen(base + f"/api/recordings/{session_id}/timeline?limit=1", timeout=2) as response:
            assert len(json.load(response)["records"]) == 1
        with urlopen(base + f"/api/recordings/{session_id}/download", timeout=2) as response:
            assert response.headers["Content-Type"] == "application/zip"
            assert response.read(2) == b"PK"
        armed = post("/api/control/arm", {
            "kind": "LINE_FOLLOW",
            "duration_ms": 1500,
            "safety_ack": True,
        })["armed"]
        confirmed = post("/api/control/confirm", {
            "arm_token": armed["arm_token"],
            "recording": {"auto": True, "name": "自动循迹"},
        })
        assert confirmed["action"]["action_id"] == 7
        assert confirmed["recording"]["recording"] is True
        assert confirmed["recording"]["trigger"] == "action"
        post("/api/recordings/stop", {})
        try:
            urlopen(base + "/api/recordings/..%2F..%2Fbad/manifest", timeout=2)
            raise AssertionError("path traversal must be rejected")
        except HTTPError as exc:
            assert exc.code in (400, 404)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def test_status_failure_still_exposes_estop_token_but_not_armable_state() -> None:
    class StatusFailureSession(FakeControlSession):
        def get_status(self):
            raise Disconnected("status unavailable")

    session = StatusFailureSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    state = controller.control_state()
    assert state["control_token"]
    assert state["status"] == {}
    assert state["status_error"] == "status unavailable"
    assert controller.emergency_stop()["accepted"] is True
    assert session.estops == 1


def test_dashboard_cli_options() -> None:
    args = _create_parser().parse_args(
        ["--port", "COM10", "dashboard", "--http-port", "9876", "--no-open"]
    )
    assert args.command == "dashboard"
    assert args.http_port == 9876
    assert args.no_open is True
    assert args.recordings_dir == Path("recordings")


def test_run_dashboard_subscribes_and_unsubscribes() -> None:
    class FakeSession:
        def __init__(self) -> None:
            self.subscriptions = []

        def subscribe(self, entries):
            self.subscriptions.append(dict(entries))
            return (), 0

        def read_telemetry(self, timeout=None):
            raise AssertionError("zero duration must not read telemetry")

    session = FakeSession()
    run_dashboard(
        session,
        {"firmware_id": "test"},
        http_port=0,
        duration_s=0,
        open_browser=False,
    )
    assert session.subscriptions[0]
    assert session.subscriptions[-1] == {}


def test_run_dashboard_reconnects_a_live_com_port_after_vehicle_reboot(monkeypatch) -> None:
    import m6link.dashboard as dashboard_module

    class ReconnectingSession:
        def __init__(self) -> None:
            self.connected = False
            self.capabilities = Capability.FAULT_CLEAR
            self.subscriptions = []
            self.reconnect_count = 0
            self.sample_id = 0

        def subscribe(self, entries):
            self.subscriptions.append(dict(entries))
            return (), 0

        def reconnect(self):
            self.reconnect_count += 1
            self.connected = True
            return SimpleNamespace(
                firmware_id="M6-after-reboot",
                enabled_capabilities=self.capabilities,
                port="COM10",
            )

        def read_telemetry(self, timeout=None):
            self.sample_id += 1
            return _sample(self.sample_id)

    monkeypatch.setattr(dashboard_module, "_RECONNECT_SILENCE_S", 0.01)
    monkeypatch.setattr(dashboard_module, "_RECONNECT_RETRY_S", 0.01)
    session = ReconnectingSession()
    run_dashboard(
        session,
        {"firmware_id": "M6-before-reboot", "port": "COM10"},
        http_port=0,
        duration_s=0.03,
        open_browser=False,
    )
    assert session.reconnect_count == 1
    assert session.subscriptions[0]
    assert session.subscriptions[1]
    assert session.subscriptions[-1] == {}
