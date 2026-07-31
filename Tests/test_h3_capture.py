from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import struct
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = spec_from_file_location("h3_capture", ROOT / "scripts" / "h3_capture.py")
assert SPEC and SPEC.loader
CAPTURE = module_from_spec(SPEC)
sys.modules[SPEC.name] = CAPTURE
SPEC.loader.exec_module(CAPTURE)


class H3CaptureTests(unittest.TestCase):
    def test_callback_metrics_cover_rate_gap_and_small_motion(self) -> None:
        times = [index * 0.05 for index in range(21)]
        metrics = CAPTURE.callback_metrics(times, [-10, -5, 0, 5, 10])
        self.assertAlmostEqual(metrics["rate_hz"], 20.0)
        self.assertAlmostEqual(metrics["max_gap_ms"], 50.0)
        self.assertAlmostEqual(metrics["movement_mm"], 2.0)

    def test_schema_three_decode(self) -> None:
        values = (3, 3, 0, 3, 1000, 0, 1, 12, -3, 0, -12,
                  -96, 0, -96, 4396, 4378, 0, 15, 9, 0, 0, 0, 125)
        decoded = CAPTURE.decode(struct.pack("<BBBBIII7hiiIHBBbBh", *values))
        self.assertEqual(decoded["x_mm_x10"], 12)
        self.assertEqual(decoded["vx_mm_s"], -3)
        self.assertEqual(decoded["integral_mdeg"], 125)


if __name__ == "__main__":
    unittest.main()
