import importlib.util
import json
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "calibrate_grayscale.py"
SPEC = importlib.util.spec_from_file_location("calibrate_grayscale", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def samples(base, offsets=range(-4, 4), count=64):
    result = []
    offsets = tuple(offsets)
    for index in range(count):
        delta = offsets[index % len(offsets)]
        result.append(
            MODULE.GraySample(tuple(value + delta for value in base), index + 1, 0)
        )
    return result


class GrayscaleCalibrationTests(unittest.TestCase):
    def test_parses_firmware_adc_line(self):
        parsed = MODULE.parse_gray_line(
            "GRAY adc=[3000,3001,3002,3003,3004,3005,3006,3007] "
            "scans=42 errors=0 pending=0 power=1"
        )
        self.assertEqual(parsed.adc[0], 3000)
        self.assertEqual(parsed.adc[7], 3007)
        self.assertEqual(parsed.scan_count, 42)
        self.assertEqual(parsed.error_count, 0)

    def test_accepts_stable_well_separated_stages_and_formats_config(self):
        white = MODULE.summarize_stage(samples([3000] * 8))
        black = MODULE.summarize_stage(samples([800] * 8))
        self.assertEqual(MODULE.validate_calibration(white, black), [])
        output = MODULE.format_config(white, black)
        self.assertIn("GRAYSCALE_CALIBRATION_VERIFIED          1U", output)
        self.assertIn("3000U", output)
        self.assertIn("800U", output)

        stage_payload = json.loads(MODULE.format_stage_json("white", white))
        self.assertEqual(stage_payload["label"], "white")
        self.assertEqual(stage_payload["count"], 64)
        self.assertEqual(stage_payload["mean"], [3000] * 8)

    def test_rejects_short_reversed_low_span_noisy_and_railed_capture(self):
        short_white = MODULE.summarize_stage(samples([3000] * 8, count=8))
        short_black = MODULE.summarize_stage(samples([800] * 8, count=8))
        self.assertTrue(MODULE.validate_calibration(short_white, short_black))

        reversed_white = MODULE.summarize_stage(samples([700] * 8))
        reversed_black = MODULE.summarize_stage(samples([900] * 8))
        self.assertTrue(MODULE.validate_calibration(reversed_white, reversed_black))

        low_white = MODULE.summarize_stage(samples([1200] * 8))
        low_black = MODULE.summarize_stage(samples([900] * 8))
        self.assertTrue(MODULE.validate_calibration(low_white, low_black))

        noisy_white = MODULE.summarize_stage(
            samples([3000] * 8, offsets=(-200, 200))
        )
        stable_black = MODULE.summarize_stage(samples([800] * 8))
        self.assertTrue(MODULE.validate_calibration(noisy_white, stable_black))

        rail_white = MODULE.summarize_stage(samples([4095] * 8, offsets=(0,)))
        self.assertTrue(MODULE.validate_calibration(rail_white, stable_black))

    def test_rejects_malformed_or_out_of_range_lines(self):
        with self.assertRaises(ValueError):
            MODULE.parse_gray_line("GRAY adc=[1,2] scans=1 errors=0")
        with self.assertRaises(ValueError):
            MODULE.parse_gray_line(
                "GRAY adc=[5000,2,3,4,5,6,7,8] scans=1 errors=0"
            )


if __name__ == "__main__":
    unittest.main()
