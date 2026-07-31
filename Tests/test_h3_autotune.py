from pathlib import Path
import struct
import sys
import threading
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "python" / "src"))

from scripts.h3_autotune import (  # noqa: E402
    H3_RECORD, H3HeartbeatPump, analyze_rows, atomic_json, candidate,
    candidate_is_single_bounded, decode_h3, host_launch_pulse_required,
    host_soft_stop_required, run_frame_state,
)
from m6link.errors import DeviceRejected  # noqa: E402


PROFILE = {
    "plus_start_mdeg": 8000, "plus_max_mdeg": 18000,
    "minus_start_mdeg": 8000, "minus_max_mdeg": 18000,
    "kp_mdeg_per_mm": 105, "kd_mdeg_per_mm_s": 36,
    "normal_slew_mdeg_per_20ms": 200,
    "breakaway_slew_mdeg_per_20ms": 1000,
    "brake_slew_mdeg_per_20ms": 1500, "tracking_rpm": 1,
    "plus_launch_rpm": 5, "minus_launch_rpm": 5,
    "ki_mdeg_per_mm_s": 0,
}


def row(time_ms: int, *, state: int = 1, flags: int = 0, x: int = 0,
        target: int = 500, fault: int = 0, counts: int = 5000, vx: int = 0,
        target_counts: int = 5000, forced: int = 18000) -> dict[str, int]:
    return {
        "kind": 3, "schema": 3, "state": state, "flags": flags,
        "time_ms": time_ms, "run_id": 1, "profile_revision": 1,
        "x_mm_x10": x, "vx_mm_s": vx, "target_mm_x10": target,
        "error_mm_x10": target - x, "raw_pd_mdeg": 5000,
        "forced_tilt_mdeg": forced, "tilt_mdeg": forced,
        "actuator_counts": counts, "actuator_target_counts": target_counts,
        "fault": fault, "vision_age_ms": 10, "vision_seq": time_ms // 50,
        "actuator_state": 0, "actuator_direction": 1, "actuator_rpm": 5,
        "integral_mdeg": 0,
    }


class H3AutotuneTests(unittest.TestCase):
    def test_atomic_json_retries_transient_windows_reader_lock(self) -> None:
        import tempfile
        from unittest.mock import patch

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "state.json"
            real_replace = __import__("os").replace
            calls = 0

            def transient_replace(source, destination):
                nonlocal calls
                calls += 1
                if calls < 3:
                    raise PermissionError("simulated sharing violation")
                return real_replace(source, destination)

            with patch("scripts.h3_autotune.os.replace", transient_replace):
                atomic_json(target, {"phase": "waiting_center"})
            self.assertEqual(calls, 3)
            self.assertEqual(target.read_text(encoding="utf-8").strip(),
                             '{\n  "phase": "waiting_center"\n}')

    def test_stale_preroll_frame_cannot_finish_new_run(self) -> None:
        preroll = row(100, state=0, flags=3)
        preroll["run_id"] = 0
        started, terminal = run_frame_state(preroll, 7, False)
        self.assertFalse(started)
        self.assertFalse(terminal)

        active = row(150, state=1, flags=131)
        active["run_id"] = 7
        started, terminal = run_frame_state(active, 7, started)
        self.assertTrue(started)
        self.assertFalse(terminal)

        stopped = row(200, state=0, flags=3)
        stopped["run_id"] = 7
        started, terminal = run_frame_state(stopped, 7, started)
        self.assertTrue(started)
        self.assertTrue(terminal)

    def test_live_runner_uses_confirmed_independent_heartbeat(self) -> None:
        source = (ROOT / "scripts" / "h3_autotune.py").read_text(encoding="utf-8")
        self.assertIn("H3HeartbeatPump", source)
        self.assertNotIn("heartbeat_h3_tune_nowait", source)
        self.assertNotIn("stop_h3_tune_nowait", source)
        self.assertIn("initial_breakaway_cap_mdeg", source)
        self.assertIn("post_run_coast", source)
        self.assertNotIn("self.session.set_vision_uart_monitor(True)", source)

    def test_host_guard_stops_outward_ball_only_when_actuator_lags(self) -> None:
        dangerous = row(100, x=530, vx=15, counts=5930, target_counts=7210,
                        forced=0)
        dangerous["tilt_mdeg"] = 4000
        dangerous["actuator_direction"] = 0
        self.assertTrue(host_soft_stop_required(dangerous))
        self.assertFalse(host_soft_stop_required(
            row(100, x=73, vx=15, counts=5940, target_counts=5930, forced=0)
        ))
        self.assertFalse(host_soft_stop_required(
            row(100, x=73, vx=-15, counts=5930, target_counts=7210,
                forced=0)
        ))
        self.assertFalse(host_soft_stop_required(
            row(100, x=73, vx=15, counts=7210, target_counts=4396,
                forced=0)
        ))
        # r102 was already withdrawing toward a smaller positive target.  It
        # was ahead, not lagging, so the host must leave H3 closed-loop.
        withdrawing = row(
            100, x=332, vx=15, counts=5355, target_counts=4619, forced=0,
        )
        withdrawing["tilt_mdeg"] = 1224
        self.assertFalse(host_soft_stop_required(withdrawing))
        # r109 was below the +40 mm pass window and the encoder was already
        # moving toward the requested positive recovery target.  Neither a
        # 0.1-mm unit mistake nor ordinary following error may issue STOP.
        recovering = row(
            100, x=356, vx=17, counts=3428, target_counts=4547, forced=0,
        )
        recovering["tilt_mdeg"] = 900
        recovering["actuator_direction"] = 1
        self.assertFalse(host_soft_stop_required(recovering))

        # Captured r156 handoff: RUN_MINUS still has a transient positive
        # ramp target.  Stage direction is negative, so positive x/v cannot
        # be classified as negative-stage outward motion and trigger STOP.
        handoff = row(
            100, state=3, x=500, vx=18, target=200,
            counts=2882, target_counts=4942, forced=3000,
        )
        handoff["tilt_mdeg"] = 3000
        handoff["actuator_direction"] = 0
        self.assertFalse(host_soft_stop_required(handoff))

        # The same guard remains active for a genuinely outward-moving ball
        # on the negative side, even while the ramp target is still positive.
        negative_danger = row(
            120, state=3, x=-530, vx=-15, target=200,
            counts=3000, target_counts=2000, forced=-3000,
        )
        negative_danger["tilt_mdeg"] = -3000
        negative_danger["actuator_direction"] = 0
        self.assertTrue(host_soft_stop_required(negative_danger))

    def test_launch_pulse_uses_encoder_before_ball_displacement(self) -> None:
        horizontal = 4396
        # Sixteen degrees is 2913 counts.  The guard fires without requiring
        # ball displacement or velocity once the forced launch reaches it.
        self.assertTrue(host_launch_pulse_required(
            row(100, x=0, vx=0, counts=7310, target_counts=7673,
                forced=18000), horizontal, 16000
        ))
        self.assertFalse(host_launch_pulse_required(
            row(100, x=0, vx=0, counts=7200, target_counts=7673,
                forced=18000), horizontal, 16000
        ))
        self.assertFalse(host_launch_pulse_required(
            row(100, x=0, vx=0, counts=7310, target_counts=7673,
                forced=0), horizontal, 16000
        ))

    def test_heartbeat_pump_confirms_one_request_at_a_time(self) -> None:
        class FakeSession:
            def __init__(self) -> None:
                self.calls = 0
                self.active = 0
                self.overlapped = False
                self.lock = threading.Lock()

            def heartbeat_h3_tune(self, token: int, *, timeout: float | None = None):
                self.assertions = (token, timeout)
                with self.lock:
                    self.active += 1
                    self.overlapped |= self.active > 1
                time.sleep(0.001)
                with self.lock:
                    self.calls += 1
                    self.active -= 1

        fake = FakeSession()
        pump = H3HeartbeatPump(fake, 123, interval=0.003, timeout=0.02)  # type: ignore[arg-type]
        pump.start()
        time.sleep(0.025)
        pump.stop()
        pump.raise_if_failed()
        self.assertGreaterEqual(fake.calls, 2)
        self.assertFalse(fake.overlapped)
        self.assertEqual(fake.assertions, (123, 0.02))

    def test_heartbeat_timeout_stays_below_firmware_watchdog(self) -> None:
        class FakeSession:
            pass

        pump = H3HeartbeatPump(FakeSession(), 123)  # type: ignore[arg-type]
        self.assertEqual(pump.timeout, 0.25)
        with self.assertRaisesRegex(ValueError, "below 300 ms"):
            H3HeartbeatPump(FakeSession(), 123, timeout=0.30)  # type: ignore[arg-type]

    def test_heartbeat_not_armed_after_terminal_is_not_capture_failure(self) -> None:
        class TerminalSession:
            def heartbeat_h3_tune(self, token: int, *, timeout: float | None = None):
                raise DeviceRejected(
                    0x10C, 6, opcode=0x830B, name="NOT_ARMED",
                )

        pump = H3HeartbeatPump(
            TerminalSession(), 123, interval=0.003, timeout=0.02,  # type: ignore[arg-type]
        )
        pump.start()
        time.sleep(0.01)
        pump.stop()
        pump.raise_if_failed()

    def test_schema_3_decode(self) -> None:
        payload = H3_RECORD.pack(3, 3, 1, 0, 100, 7, 9,
                                 0, 0, 500, 500, 5250, 8000, 8000,
                                 4396, 5000, 0, 10, 2, 1, 1, 5, 250)
        decoded = decode_h3(payload)
        self.assertEqual(decoded["run_id"], 7)
        self.assertEqual(decoded["forced_tilt_mdeg"], 8000)
        self.assertEqual(decoded["integral_mdeg"], 250)

    def test_no_roll_followed_increases_only_positive_max(self) -> None:
        metrics = analyze_rows([row(0), row(1000), row(1050)])
        decision = candidate(PROFILE, metrics)
        self.assertEqual(decision["parameter"], "plus_max_mdeg")
        self.assertEqual(decision["new"], 22000)
        self.assertTrue(candidate_is_single_bounded(PROFILE, decision))

    def test_rolled_but_stuck_before_target_increases_ki(self) -> None:
        rows = [row(0, flags=0, x=0), row(50, flags=32, x=10),
                row(100, flags=32, x=250),
                row(150, state=0, flags=0, x=250, fault=8)]
        metrics = analyze_rows(rows)
        self.assertFalse(metrics["hard_failure"])
        self.assertIn(8, metrics["observed_faults"])
        decision = candidate(PROFILE, metrics)
        self.assertEqual(decision["parameter"], "ki_mdeg_per_mm_s")
        self.assertEqual(decision["new"], 10)
        self.assertTrue(candidate_is_single_bounded(PROFILE, decision))

    def test_pass_through_plus_and_stable_minus_meets_five_second_task(self) -> None:
        rows = [
            row(0, state=1, x=0, vx=0),
            row(900, state=1, flags=32, x=410, vx=35),
            row(950, state=3, x=430, vx=30, target=-500),
            row(2500, state=3, flags=32, x=-450, vx=-12, target=-500),
            row(2800, state=4, flags=16 | 32, x=-520, vx=-4,
                target=-500),
            row(3000, state=5, x=-530, vx=-2, target=-500, fault=8),
        ]
        metrics = analyze_rows(rows)
        self.assertTrue(metrics["plus_passed"])
        self.assertTrue(metrics["complete"])
        self.assertTrue(metrics["success"])
        self.assertLessEqual(metrics["completion_ms"], 5000)
        self.assertLessEqual(metrics["steady_error_mm"], 10.0)

    def test_plus_brake_timing_records_pd_window_and_encoder_events(self) -> None:
        rows = [
            row(0, state=1, x=300, vx=40, counts=5000,
                target_counts=5000, forced=0),
            row(50, state=1, x=380, vx=45, counts=5000,
                target_counts=4900, forced=0),
            row(100, state=3, x=405, vx=40, counts=5000,
                target_counts=4800, forced=0),
            row(150, state=3, x=430, vx=30, counts=4988,
                target_counts=4700, forced=0),
        ]
        rows[0]["raw_pd_mdeg"] = 100
        rows[1]["raw_pd_mdeg"] = -100
        rows[1]["tilt_mdeg"] = -100
        rows[2]["raw_pd_mdeg"] = -600
        rows[2]["tilt_mdeg"] = -600
        rows[2]["actuator_direction"] = -1
        rows[3]["raw_pd_mdeg"] = -2000
        rows[3]["tilt_mdeg"] = -2000
        rows[3]["actuator_direction"] = -1
        timing = analyze_rows(rows)["plus_brake_timing"]
        self.assertEqual(timing["pd_negative"]["time_ms"], 50)
        self.assertEqual(timing["entered_plus_40"]["time_ms"], 100)
        self.assertEqual(timing["actuator_reverse"]["time_ms"], 150)
        self.assertEqual(timing["brake_request_to_actuator_ms"], 100)
        self.assertEqual(timing["pd_to_actuator_ms"], 100)
        self.assertEqual(timing["plus_40_to_actuator_ms"], 50)

    def test_missing_plus_window_cannot_pass_even_if_minus_holds(self) -> None:
        rows = [
            row(0, state=1, x=0, vx=0),
            row(900, state=1, flags=32, x=350, vx=20),
            row(2500, state=4, flags=16, x=-500, vx=0, target=-500),
        ]
        metrics = analyze_rows(rows)
        self.assertFalse(metrics["plus_passed"])
        self.assertFalse(metrics["complete"])

    def test_slow_launch_recovery_increases_tracking_before_gains(self) -> None:
        rows = [
            row(0, x=0, vx=0, forced=16000),
            row(50, flags=32, x=10, vx=5, forced=0),
            row(150, flags=32, x=80, vx=20, forced=0),
            row(250, flags=32, x=180, vx=30, forced=0),
            row(350, flags=32, x=240, vx=10, forced=0),
            row(400, flags=32, x=250, vx=0, forced=0, fault=8, state=0),
        ]
        # Model the commanded launch angle remaining well above raw PD for the
        # complete 300 ms recovery window.
        for item in rows[1:5]:
            item["tilt_mdeg"] = 12000
            item["raw_pd_mdeg"] = 4000
        metrics = analyze_rows(rows)
        self.assertTrue(metrics["control_recovery_slow"])
        decision = candidate(PROFILE, metrics)
        self.assertEqual(decision["parameter"], "tracking_rpm")
        self.assertEqual(decision["new"], 2)

    def test_no_follow_raises_rpm_and_never_exceeds_limits(self) -> None:
        rows = [row(0, counts=4000, target_counts=6000),
                row(50, counts=4000, target_counts=6000)]
        metrics = analyze_rows(rows)
        decision = candidate(PROFILE, metrics)
        self.assertEqual(decision["parameter"], "plus_launch_rpm")
        self.assertEqual(decision["new"], 6)
        saturated = dict(PROFILE, plus_max_mdeg=30000, plus_launch_rpm=8)
        decision = candidate(saturated, metrics)
        self.assertEqual(decision["parameter"], "breakaway_slew_mdeg_per_20ms")
        self.assertLessEqual(decision["new"], 1500)

    def test_gap_fault_and_visual_contradiction_are_hard(self) -> None:
        rows = [row(0), row(300, flags=32, x=10)]
        metrics = analyze_rows(rows, [{"result": "no_roll"}])
        self.assertTrue(metrics["hard_failure"])
        self.assertIn("vision_telemetry_contradiction", metrics["hard_reasons"])

    def test_visual_roll_agrees_with_motion_during_early_return(self) -> None:
        rows = [
            row(0, state=1, x=0, vx=0),
            row(50, state=5, x=4, vx=1, forced=0, fault=8),
            row(150, state=5, x=100, vx=20, forced=0, fault=8),
            row(250, state=0, x=120, vx=5, forced=0, fault=8),
        ]
        metrics = analyze_rows(rows, [{"result": "rolled"}])
        self.assertTrue(metrics["telemetry_rolled"])
        self.assertNotIn(
            "vision_telemetry_contradiction", metrics["hard_reasons"]
        )

    def test_breakaway_active_flag_is_not_ball_motion(self) -> None:
        rows = [
            row(0, flags=64, x=0, vx=0),
            row(50, flags=64, x=1, vx=0),
            row(100, state=0, flags=0, x=1, vx=0, fault=8),
        ]
        metrics = analyze_rows(rows, [{"result": "no_roll"}])
        self.assertFalse(metrics["telemetry_rolled"])
        self.assertNotIn(
            "vision_telemetry_contradiction", metrics["hard_reasons"]
        )


if __name__ == "__main__":
    unittest.main()
