from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import struct
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = spec_from_file_location("h3_pid_scope", ROOT / "scripts" / "h3_pid_scope.py")
assert SPEC and SPEC.loader
SCOPE = module_from_spec(SPEC)
sys.modules[SPEC.name] = SCOPE
SPEC.loader.exec_module(SCOPE)


def row(time_ms: int, x10: int, vx: int = 0, tilt: int = 0) -> dict[str, int]:
    return {
        "time_ms": time_ms, "x_mm_x10": x10, "vx_mm_s": vx,
        "tilt_mdeg": tilt, "actuator_target_counts": 4400,
        "actuator_counts": 4390,
    }


class H3PidScopeTests(unittest.TestCase):
    def test_schema_three_decoder(self) -> None:
        values = (3, 3, 7, 129, 1000, 2, 4, 100, -20, 0, -100,
                  -1770, 0, -200, 4396, 4370, 0, 25, 9, 1, -1, 1, 300)
        payload = struct.pack("<BBBBIII7hiiIHBBbBh", *values)
        decoded = SCOPE.decode_h3(payload)
        self.assertEqual(decoded["state"], 7)
        self.assertEqual(decoded["target_mm_x10"], 0)
        self.assertEqual(decoded["forced_tilt_mdeg"], 0)
        self.assertEqual(decoded["integral_mdeg"], 300)

    def test_hand_flick_metrics(self) -> None:
        analyzer = SCOPE.FlickAnalyzer()
        trajectory = [
            row(0, 0), row(50, 20, 45), row(100, 100, 70, -800),
            row(150, 80, -30, -600), row(200, 40, -20, -300),
            row(250, -20, -8, 100), row(300, -10, 0, 80),
        ]
        trajectory.extend(row(t, 0, 0, 0) for t in range(350, 1401, 50))
        for sample in trajectory:
            analyzer.feed(sample)
        result = analyzer.result()
        self.assertTrue(result["detected"])
        self.assertTrue(result["settled"])
        self.assertEqual(result["peak_mm"], 10.0)
        self.assertEqual(result["opposite_overshoot_mm"], 2.0)
        self.assertEqual(result["zero_crossings"], 1)
        self.assertEqual(result["gap_count"], 0)

    def test_gap_detection_is_explicit(self) -> None:
        analyzer = SCOPE.FlickAnalyzer()
        analyzer.feed(row(10, 0))
        analyzer.feed(row(210, 60, 45))
        self.assertEqual(analyzer.result()["gap_count"], 1)
        self.assertEqual(analyzer.result()["max_gap_ms"], 200)

    def test_stale_telemetry_clears_ready_and_center_gate(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            owner = SCOPE.ScopeOwner(SimpleNamespace(
                max_samples=20, output=Path(folder), duration=8.0,
                port="COM10",
            ))
            owner.state_value.update(ready=True, centered=True,
                                     center_stable_ms=1000)
            owner.last_sample_received = time.monotonic() - 0.4
            state = owner.state()
        self.assertGreaterEqual(state["telemetry_age_ms"], 350)
        self.assertFalse(state["ready"])
        self.assertFalse(state["centered"])
        self.assertEqual(state["center_stable_ms"], 0)

    def test_web_assets_are_self_contained_and_have_four_waveforms(self) -> None:
        web = ROOT / "python" / "src" / "m6link" / "web"
        html = (web / "h3_pid_scope.html").read_text(encoding="utf-8")
        js = (web / "h3_pid_scope.js").read_text(encoding="utf-8")
        css = (web / "h3_pid_scope.css").read_text(encoding="utf-8")
        self.assertEqual(html.count("<canvas"), 4)
        self.assertIn("/api/parameter", js)
        self.assertIn("/api/estop", js)
        self.assertIn("ResizeObserver", js)
        self.assertNotIn("https://", html + js + css)


if __name__ == "__main__":
    unittest.main()
