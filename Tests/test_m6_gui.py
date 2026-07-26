"""Headless safety and controller tests for the Tk GUI."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
PYTHON_SRC = ROOT / "python" / "src"
if str(PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(PYTHON_SRC))

from m6gui.controller import (  # noqa: E402
    Controller,
    GuiInputError,
    arc_action,
    feature_visibility,
    is_emergency_key,
    normalize_capabilities,
    telemetry_point,
    turn_action,
    wheel_action,
    wizard_drafts,
)
from m6gui.fake import Capability, FakeSession  # noqa: E402


def pump(controller: Controller, predicate=None, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        controller.poll()
        if predicate is None or predicate():
            return
        time.sleep(0.005)
    controller.poll()
    if predicate is not None and not predicate():
        raise AssertionError(f"condition not reached; state={controller.state}")


class ControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        FakeSession.last_created = None
        self.controller = Controller(FakeSession)

    def tearDown(self) -> None:
        self.controller.shutdown()

    def connect(self, *, legacy: bool = False) -> FakeSession:
        self.assertTrue(self.controller.connect("FAKE0", legacy=legacy))
        pump(self.controller, lambda: self.controller.state.connected)
        session = FakeSession.last_created
        self.assertIsNotNone(session)
        return session  # type: ignore[return-value]

    def test_default_connection_requires_v2_and_queries_full_diagnostics(self) -> None:
        session = self.connect()
        self.assertEqual(self.controller.state.protocol_version, 2)
        self.assertFalse(self.controller.state.legacy)
        self.assertEqual(session.calls[0], ("connect", "FAKE0", False))
        self.assertEqual(
            set(self.controller.state.diagnostics),
            {"status", "reset", "tasks", "uart", "link"},
        )
        self.assertIn("breadcrumb", self.controller.state.diagnostics["reset"])
        self.assertTrue(self.controller.has_capability("PRIORITY_TX", "RAW_ESTOP_S_SPACE"))

    def test_explicit_legacy_is_allowed_but_never_defaulted(self) -> None:
        session = self.connect(legacy=True)
        self.assertEqual(self.controller.state.protocol_version, 1)
        self.assertEqual(session.calls[0], ("connect", "FAKE0", True))
        self.assertFalse(self.controller.has_capability("ACTION_TWO_PHASE"))

    def test_arm_never_auto_confirms(self) -> None:
        session = self.connect()
        draft = wheel_action(160, 0, 1.5)
        self.assertTrue(self.controller.arm_action(draft))
        pump(self.controller, lambda: self.controller.state.armed is not None)
        names = [call[0] for call in session.calls]
        self.assertIn("arm_action", names)
        self.assertNotIn("confirm_action", names)
        self.assertEqual(self.controller.state.armed.draft.wheel_mask, 1)  # type: ignore[union-attr]

        self.assertTrue(self.controller.confirm_action())
        pump(self.controller, lambda: "confirm_action" in [call[0] for call in session.calls])
        self.assertEqual([call[0] for call in session.calls].count("confirm_action"), 1)

    def test_each_wizard_step_requires_a_fresh_arm(self) -> None:
        session = self.connect()
        drafts = wizard_drafts(160, 1.5, 0.7)
        self.assertEqual(len(drafts), 8)
        self.controller.arm_action(drafts[0])
        pump(self.controller, lambda: self.controller.state.armed is not None)
        with self.assertRaises(GuiInputError):
            self.controller.arm_action(drafts[1])
        self.controller.confirm_action()
        pump(self.controller, lambda: self.controller.state.armed is None)
        self.controller.arm_action(drafts[1])
        pump(
            self.controller,
            lambda: [call[0] for call in session.calls].count("arm_action") == 2,
        )
        self.assertEqual([call[0] for call in session.calls].count("confirm_action"), 1)

    def test_emergency_stop_uses_separate_operation_and_clears_lease(self) -> None:
        session = self.connect()
        self.controller.arm_action(wheel_action(160, 160, 1.0))
        pump(self.controller, lambda: self.controller.state.armed is not None)
        self.assertTrue(self.controller.emergency_stop())
        pump(self.controller, lambda: ("emergency_stop",) in session.calls)
        self.assertIsNone(self.controller.state.armed)
        self.assertEqual(session.status["system_state"], "FAULT")

    def test_heading_calibration_is_independent_and_capability_gated(self) -> None:
        session = self.connect()
        self.assertTrue(self.controller.calibrate_heading())
        pump(self.controller, lambda: ("calibrate_heading",) in session.calls)
        names = [call[0] for call in session.calls]
        self.assertNotIn("arm_action", names)
        self.assertNotIn("confirm_action", names)
        self.assertIn("heading_calibration", self.controller.state.diagnostics)

        self.controller.state.capabilities = frozenset(
            name
            for name in self.controller.state.capabilities
            if name != "HEADING_CALIBRATION"
        )
        with self.assertRaises(GuiInputError):
            self.controller.calibrate_heading()

    def test_parameter_transaction_refreshes_three_values(self) -> None:
        session = self.connect()
        before = self.controller.state.parameters["wheel.left.kp"]
        self.assertEqual((before.active, before.staged, before.persisted), (0.7, 0.7, 0.7))
        self.controller.stage_parameter("wheel.left.kp", "0.9")
        pump(self.controller, lambda: ("stage_parameter", "wheel.left.kp", 0.9) in session.calls)
        pump(
            self.controller,
            lambda: self.controller.state.parameters["wheel.left.kp"].staged == 0.9,
        )
        self.assertEqual(self.controller.state.parameters["wheel.left.kp"].active, 0.7)
        self.controller.apply_parameters()
        pump(self.controller, lambda: ("apply_parameters",) in session.calls)
        pump(
            self.controller,
            lambda: self.controller.state.parameters["wheel.left.kp"].active == 0.9,
        )
        self.controller.save_parameters()
        pump(self.controller, lambda: ("save_parameters",) in session.calls)
        pump(
            self.controller,
            lambda: self.controller.state.parameters["wheel.left.kp"].persisted == 0.9,
        )

    def test_after_style_tick_ingests_telemetry_without_tk(self) -> None:
        self.connect()
        for _ in range(8):
            self.controller.tick()
            pump(self.controller)
            time.sleep(0.045)
        self.controller.tick()
        pump(self.controller)
        self.assertGreater(len(self.controller.state.telemetry), 2)
        latest = self.controller.state.telemetry[-1]
        self.assertFalse(math.isnan(latest.left_mm_s))
        self.assertEqual(len(latest.grayscale), 8)

    def test_record_replay_and_score_are_session_operations(self) -> None:
        session = self.connect()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fake.json"
            self.controller.start_recording(path)
            pump(self.controller, lambda: self.controller.state.recording)
            for _ in range(3):
                time.sleep(0.045)
                self.controller.tick()
                pump(self.controller)
            self.controller.stop_recording()
            pump(self.controller, lambda: not self.controller.state.recording)
            self.assertTrue(path.exists())
            self.controller.replay(path)
            pump(self.controller, lambda: self.controller.state.replay_result is not None)
            self.controller.score(path)
            pump(self.controller, lambda: self.controller.state.score_result is not None)
            self.assertGreater(self.controller.state.score_result["sample_count"], 0)
            names = [call[0] for call in session.calls]
            self.assertIn("replay", names)
            self.assertIn("score", names)


class PureModelTest(unittest.TestCase):
    def test_registry_names_come_from_intflag_not_gui_numbers(self) -> None:
        names = normalize_capabilities(
            Capability.ACTION_TWO_PHASE | Capability.WHEEL_DIAG_ACTION
        )
        self.assertEqual(names, {"ACTION_TWO_PHASE", "WHEEL_DIAG_ACTION"})

    def test_capability_filter_hides_unpublished_controls(self) -> None:
        visible = feature_visibility(
            {"WIRE_R1", "PARAM_READ", "ACTION_TWO_PHASE", "WHEEL_DIAG_ACTION"},
            connected=True,
            protocol=2,
        )
        self.assertTrue(visible["parameter_tab"])
        self.assertTrue(visible["action_tab"])
        self.assertFalse(visible["parameter_stage"])
        self.assertFalse(visible["parameter_persist"])
        self.assertFalse(visible["heading"])
        self.assertFalse(visible["configurable_turn"])
        self.assertFalse(visible["heading_calibration"])
        self.assertFalse(visible["recording"])

        calibration_only = feature_visibility(
            {"WIRE_R1", "HEADING_CALIBRATION"}, connected=True, protocol=2
        )
        self.assertTrue(calibration_only["action_tab"])
        self.assertTrue(calibration_only["heading_calibration"])
        self.assertFalse(calibration_only["wheel_actions"])
        self.assertFalse(calibration_only["wizard_tab"])

    def test_m6link_nested_telemetry_model_is_supported(self) -> None:
        try:
            from m6link import TelemetrySample, TelemetryStream
        except ImportError:
            self.skipTest("m6link package is not present")
        point = telemetry_point(
            TelemetrySample(
                TelemetryStream.CORE,
                1,
                100,
                {"time_ms": 100, "speed_l_mm_s": 12, "speed_r_mm_s": 13},
            )
        )
        self.assertEqual(point.left_mm_s, 12)  # type: ignore[union-attr]

    def test_action_safety_bounds_and_default_arc(self) -> None:
        left_arc = arc_action(200, left=True)
        self.assertEqual((left_arc.target_l_mm_s, left_arc.target_r_mm_s), (140, 200))
        self.assertEqual(left_arc.duration_ms, 1500)
        with self.assertRaises(GuiInputError):
            wheel_action(401, 0, 1.0)
        with self.assertRaises(GuiInputError):
            wheel_action(20, 0, 1.0)
        with self.assertRaises(GuiInputError):
            wheel_action(0, 0, 1.0)

    def test_relative_turn_range_is_one_to_360_degrees(self) -> None:
        self.assertEqual(turn_action(-360, 3.0).angle_mdeg, -360_000)
        self.assertEqual(turn_action(1, 0.5).angle_mdeg, 1000)
        for value in (0.9, 361):
            with self.assertRaises(GuiInputError):
                turn_action(value, 1.0)

    def test_keyboard_emergency_predicate(self) -> None:
        self.assertTrue(is_emergency_key("space"))
        self.assertTrue(is_emergency_key("S", "S"))
        self.assertTrue(is_emergency_key("s", "s"))
        self.assertFalse(is_emergency_key("Return", "\r"))

    def test_telemetry_accepts_frozen_wire_field_names(self) -> None:
        point = telemetry_point(
            {
                "time_ms": 250,
                "speed_l_mm_s": 101,
                "speed_r_mm_s": 99,
                "yaw_cdeg": -125,
                "yaw_rate_cdeg_s": 250,
                "adc": list(range(8)),
                "position_x1000": -500,
            }
        )
        self.assertIsNotNone(point)
        self.assertEqual(point.time_s, 0.25)  # type: ignore[union-attr]
        self.assertEqual(point.yaw_deg, -1.25)  # type: ignore[union-attr]
        self.assertEqual(point.centroid, -0.5)  # type: ignore[union-attr]


if __name__ == "__main__":
    unittest.main()
