from __future__ import annotations

import json
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link.dashboard import DashboardBuffer, create_dashboard_server
from m6link.dashboard_control import DashboardController
from m6link.errors import DeviceRejected, M6Error
from m6link.models import StepperSnapshot
from m6link.registry import Capability, RpcResult
from m6link.stepper_registry import CAPABILITY_MASK, StepperNotReady


def snapshot(**changes: int) -> StepperSnapshot:
    values = dict(
        time_ms=100, response_sequence=1, response_age_ms=10,
        last_function=0x3A, last_response=7, valid_flags=0x0F,
        speed_rpm=-60, motor_status=1, homing_status=3,
        position_raw=-65536, tx_bytes=20, rx_bytes=16,
        tx_overflow=0, rx_overflow=0, hardware_errors=0,
        invalid_frames=0, checksum_errors=0, discarded_rx_bytes=0,
        preempted_frames=0, command_id=0, test_id=0,
        remaining_ms=0, state=0, kind=0, direction=0,
        acceleration=0, requested_speed_rpm=0, position_mode=0,
        stop_reason=8, pulses=0,
    )
    values.update(changes)
    return StepperSnapshot(**values)


class FakeStepperSession:
    def __init__(self) -> None:
        self.capabilities = Capability(CAPABILITY_MASK)
        self.connected = True
        self.calls: list[tuple] = []
        self.estops = 0
        self.current = snapshot()
        self.motor_refresh_pending = False
        self.motor_refresh_delay_reads = 0
        self.motor_refresh_never = False

    def get_stepper(self, refresh_mask=0):
        mask = int(refresh_mask)
        self.calls.append(("get", mask))
        if mask & 4:
            self.motor_refresh_pending = True
            return self.current
        if mask == 0 and self.motor_refresh_pending and not self.motor_refresh_never:
            if self.motor_refresh_delay_reads > 0:
                self.motor_refresh_delay_reads -= 1
                self.current = snapshot(
                    response_sequence=self.current.response_sequence + 1,
                    last_function=0x35,
                )
            else:
                self.current = snapshot(
                    response_sequence=self.current.response_sequence + 1,
                    response_age_ms=5,
                    last_function=0x3A,
                    last_response=7,
                    valid_flags=0x0F,
                    motor_status=1,
                )
                self.motor_refresh_pending = False
        return self.current

    def enable_stepper(self):
        self.calls.append(("enable",)); return self.current

    def stop_stepper(self):
        self.calls.append(("stop",)); return self.current

    def disable_stepper(self):
        self.calls.append(("disable",)); return self.current

    def start_stepper_velocity_test(self, **values):
        self.calls.append(("velocity", values));
        self.current = snapshot(state=2, kind=1, remaining_ms=values["duration_ms"])
        return self.current

    def start_stepper_position_test(self, **values):
        self.calls.append(("position", values)); return self.current

    def emergency_stop(self):
        self.estops += 1


def test_stepper_controller_is_safety_independent_and_requires_confirmation() -> None:
    session = FakeStepperSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    try:
        state = controller.stepper_state(force=True, refresh=True)
        assert state["available"] is True
        assert state["safety_independent"] is True
        assert state["snapshot"]["speed_rpm"] == -60
        assert state["snapshot"]["position_revolutions"] == -1.0
        assert state["snapshot"]["motor_flags"] == ["已使能"]
        try:
            controller.stepper_enable({})
            raise AssertionError("enable confirmation must be required")
        except ValueError:
            pass
        controller.stepper_enable({"confirm": "ENABLE_ZDT_STEPPER"})
        controller.stepper_velocity({
            "confirm": "START_ZDT_VELOCITY_TEST", "direction": 0,
            "speed_rpm": 60, "acceleration": 20, "duration_ms": 1000,
        })
        velocity_index = next(
            index for index, call in enumerate(session.calls)
            if call[0] == "velocity"
        )
        assert session.calls[velocity_index - 2:velocity_index] == [
            ("get", int(4)), ("get", 0),
        ]
        before = list(session.calls)
        controller.emergency_stop()
        assert session.estops == 1
        assert session.calls == before
    finally:
        controller.close()
    assert not any(call[0] == "stop" for call in session.calls)


def test_stepper_http_auth_origin_and_endpoints() -> None:
    session = FakeStepperSession()
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=session.capabilities))
    server = create_dashboard_server(DashboardBuffer({}), port=0, controller=controller)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urlopen(base + "/api/stepper?refresh=1", timeout=2) as response:
            state = json.load(response)
        assert state["available"] is True
        assert session.calls[-1] == ("get", 15)
        with urlopen(base + "/api/stepper?refresh=4", timeout=2) as response:
            assert json.load(response)["available"] is True
        assert session.calls[-1] == ("get", 4)
        try:
            urlopen(base + "/api/stepper?refresh=16", timeout=2)
            raise AssertionError("invalid stepper refresh mask must fail")
        except HTTPError as exc:
            assert exc.code == 400
            exc.close()

        def request(path: str, payload: dict, *, token: str | None = None,
                    origin: str | None = None):
            headers = {"Content-Type": "application/json"}
            if token is not None: headers["X-M6-Control-Token"] = token
            if origin is not None: headers["Origin"] = origin
            return Request(base + path, data=json.dumps(payload).encode(), method="POST", headers=headers)

        try:
            urlopen(request("/api/stepper/stop", {}), timeout=2)
            raise AssertionError("missing control token must fail")
        except HTTPError as exc:
            assert exc.code == 403
            exc.close()
        try:
            urlopen(request("/api/stepper/stop", {}, token=controller.token,
                            origin="https://evil.invalid"), timeout=2)
            raise AssertionError("foreign Origin must fail")
        except HTTPError as exc:
            assert exc.code == 403
            exc.close()
        try:
            urlopen(request("/api/stepper/enable", {}, token=controller.token), timeout=2)
            raise AssertionError("missing fixed confirmation must fail")
        except HTTPError as exc:
            assert exc.code == 405
            exc.close()
        try:
            urlopen(request(
                "/api/stepper/enable", {"confirm": "ENABLE_ZDT_STEPPER"},
                token=controller.token, origin=base,
            ), timeout=2)
            raise AssertionError("raw stepper enable route must remain closed")
        except HTTPError as exc:
            assert exc.code == 405
            exc.close()
        with urlopen(request("/api/stepper/stop", {}, token=controller.token), timeout=2) as response:
            assert json.load(response)["ok"] is True
        assert ("enable",) not in session.calls and ("stop",) in session.calls
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=1)
        controller.close()


def test_missing_capability_only_disables_stepper_panel() -> None:
    session = FakeStepperSession()
    session.capabilities = Capability(0)
    controller = DashboardController(session, SimpleNamespace(enabled_capabilities=Capability(0)))
    try:
        state = controller.stepper_state(force=True, refresh=True)
        assert state["available"] is False
        assert state["snapshot"] is None
        assert not session.calls
    finally:
        controller.close()


def test_wrong_state_detail_is_translated_for_the_dashboard() -> None:
    session = FakeStepperSession()
    controller = DashboardController(
        session, SimpleNamespace(enabled_capabilities=session.capabilities)
    )
    detail = int(
        StepperNotReady.MOTOR_STATUS_STALE | StepperNotReady.MOTOR_DISABLED
    )

    def reject(**_values):
        raise DeviceRejected(
            int(RpcResult.WRONG_STATE), detail, opcode=0x8308,
            name="WRONG_STATE",
        )

    session.start_stepper_velocity_test = reject
    try:
        with unittest.TestCase().assertRaisesRegex(
            M6Error, "超过 500 ms 未更新.*电机未使能.*00000006"
        ):
            controller.stepper_velocity({
                "confirm": "START_ZDT_VELOCITY_TEST", "direction": 0,
                "speed_rpm": 60, "acceleration": 20, "duration_ms": 1000,
            })
    finally:
        controller.close()


def test_motion_preflight_waits_for_exact_new_motor_status() -> None:
    session = FakeStepperSession()
    session.motor_refresh_delay_reads = 2
    controller = DashboardController(
        session, SimpleNamespace(enabled_capabilities=session.capabilities),
        sleeper=lambda _seconds: None,
    )
    try:
        controller.stepper_velocity({
            "confirm": "START_ZDT_VELOCITY_TEST", "direction": 0,
            "speed_rpm": 60, "acceleration": 20, "duration_ms": 1000,
        })
        velocity_index = next(
            index for index, call in enumerate(session.calls)
            if call[0] == "velocity"
        )
        assert session.calls[velocity_index - 4:velocity_index] == [
            ("get", 4), ("get", 0), ("get", 0), ("get", 0),
        ]
    finally:
        controller.close()


def test_motion_preflight_timeout_never_submits_motion() -> None:
    session = FakeStepperSession()
    session.motor_refresh_never = True
    now = [0.0]

    def clock() -> float:
        return now[0]

    def sleeper(seconds: float) -> None:
        now[0] += seconds

    controller = DashboardController(
        session, SimpleNamespace(enabled_capabilities=session.capabilities),
        clock=clock, sleeper=sleeper,
    )
    try:
        with unittest.TestCase().assertRaisesRegex(
            M6Error, "未在 750 ms 内收到新的电机状态.*未发送运动命令"
        ):
            controller.stepper_velocity({
                "confirm": "START_ZDT_VELOCITY_TEST", "direction": 0,
                "speed_rpm": 60, "acceleration": 20, "duration_ms": 1000,
            })
        assert not any(call[0] == "velocity" for call in session.calls)
    finally:
        controller.close()


def load_tests(_loader, _tests, _pattern):
    suite = unittest.TestSuite()
    for function in (
        test_stepper_controller_is_safety_independent_and_requires_confirmation,
        test_stepper_http_auth_origin_and_endpoints,
        test_missing_capability_only_disables_stepper_panel,
        test_wrong_state_detail_is_translated_for_the_dashboard,
        test_motion_preflight_waits_for_exact_new_motor_status,
        test_motion_preflight_timeout_never_submits_motion,
    ):
        suite.addTest(unittest.FunctionTestCase(function))
    return suite
