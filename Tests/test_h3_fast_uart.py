from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
DEMO_UART = ROOT / "demo_uart"
sys.path.insert(0, str(DEMO_UART))

from ball_position import (  # noqa: E402
    AdaptiveAlphaBetaFilter,
    CalibrationTable,
    MULTIPOINT_CALIBRATION_MM,
    THREE_POINT_CALIBRATION_MM,
    measurement_is_valid,
)
from ball_wire import crc8_atm, decode_frame, encode_frame  # noqa: E402


class H3FastWireTests(unittest.TestCase):
    VALID_GOLDEN = bytes.fromhex("A5 5A 2A 01 F4 01 EC FF 73")
    INVALID_GOLDEN = bytes.fromhex("A5 5A 2B 00 00 00 00 00 44")

    def test_crc8_atm_and_frozen_mcu_vectors(self):
        self.assertEqual(crc8_atm(b"123456789"), 0xF4)
        self.assertEqual(encode_frame(42, True, 500, -20), self.VALID_GOLDEN)
        self.assertEqual(encode_frame(43, False, 500, -20), self.INVALID_GOLDEN)

        decoded = decode_frame(self.VALID_GOLDEN)
        self.assertEqual(
            decoded,
            {
                "seq": 42,
                "flags": 1,
                "valid": True,
                "x_mm_x10": 500,
                "vx_mm_s": -20,
            },
        )
        self.assertEqual(decode_frame(self.INVALID_GOLDEN)["valid"], False)

    def test_signed_little_endian_boundaries_and_sequence_values(self):
        frame = encode_frame(255, True, -32768, 32767)
        self.assertEqual(frame[2:8], bytes.fromhex("FF 01 00 80 FF 7F"))
        decoded = decode_frame(frame)
        self.assertEqual(decoded["seq"], 255)
        self.assertEqual(decoded["x_mm_x10"], -32768)
        self.assertEqual(decoded["vx_mm_s"], 32767)
        self.assertEqual(decode_frame(encode_frame(0, True, 1, -1))["seq"], 0)

        with self.assertRaises(ValueError):
            encode_frame(256, True, 0, 0)
        with self.assertRaises(ValueError):
            encode_frame(0, True, 32768, 0)
        with self.assertRaises(TypeError):
            encode_frame(0, 1, 0, 0)

    def test_decoder_rejects_bad_length_header_flags_crc_and_stale_payload(self):
        with self.assertRaises(ValueError):
            decode_frame(self.VALID_GOLDEN[:-1])

        bad_header = bytearray(self.VALID_GOLDEN)
        bad_header[0] = 0
        with self.assertRaises(ValueError):
            decode_frame(bad_header)

        bad_flags = bytearray(self.VALID_GOLDEN)
        bad_flags[3] = 0x02
        bad_flags[8] = crc8_atm(bad_flags[:8])
        with self.assertRaises(ValueError):
            decode_frame(bad_flags)

        bad_crc = bytearray(self.VALID_GOLDEN)
        bad_crc[8] ^= 0x01
        with self.assertRaises(ValueError):
            decode_frame(bad_crc)

        stale_invalid = bytearray(self.INVALID_GOLDEN)
        stale_invalid[4] = 1
        stale_invalid[8] = crc8_atm(stale_invalid[:8])
        with self.assertRaises(ValueError):
            decode_frame(stale_invalid)


class H3FastMeasurementTests(unittest.TestCase):
    def test_multi_and_current_three_point_calibrations_use_mm(self):
        self.assertEqual(len(MULTIPOINT_CALIBRATION_MM), 25)
        multi = CalibrationTable(MULTIPOINT_CALIBRATION_MM)
        three = CalibrationTable(THREE_POINT_CALIBRATION_MM)

        for pixel, expected_mm in ((127, -50.0), (218, 0.0), (331, 50.0)):
            self.assertAlmostEqual(multi.to_mm(pixel), expected_mm)
        for pixel, expected_mm in ((142, 50.0), (247, 0.0), (345, -50.0)):
            self.assertAlmostEqual(three.to_mm(pixel), expected_mm)
        self.assertLess(multi.to_mm(205), multi.to_mm(248))
        self.assertGreater(three.to_mm(142), three.to_mm(345))
        self.assertAlmostEqual(three.to_mm(194.5), 25.0)
        self.assertAlmostEqual(three.to_mm(37), 100.0)

        reversed_installation = CalibrationTable(
            ((100, 50.0), (200, 0.0), (300, -50.0))
        )
        self.assertAlmostEqual(reversed_installation.to_mm(100), 50.0)
        self.assertAlmostEqual(reversed_installation.to_mm(300), -50.0)

    def test_filter_velocity_state_is_physical_mm_per_second(self):
        position_filter = AdaptiveAlphaBetaFilter()
        self.assertEqual(position_filter.update(0.0, 0), (0.0, 0.0))
        position_mm, velocity_mm_s = position_filter.update(10.0, 100)
        self.assertAlmostEqual(position_mm, 9.5)
        self.assertAlmostEqual(velocity_mm_s, 12.0)

        position_filter.mark_missing(349)
        self.assertIsNotNone(position_filter.x_mm)
        position_filter.mark_missing(350)
        self.assertIsNone(position_filter.x_mm)
        self.assertEqual(position_filter.update(5.0, 400), (5.0, 0.0))

    def test_all_quality_gates_invalidate_bad_measurements(self):
        base = {
            "score": 0.75,
            "box_width": 20.0,
            "box_height": 20.0,
            "center": (218.0, 80.0),
            "axis_start": (5.0, 80.0),
            "axis_end": (475.0, 80.0),
            "position_mm": 0.0,
        }

        def is_valid(**changes):
            values = dict(base)
            values.update(changes)
            return measurement_is_valid(**values)[0]

        self.assertTrue(is_valid())
        rejected = (
            {"score": 0.49},
            {"center": (218.0, 100.1)},
            {"box_width": 5.9},
            {"box_height": 64.1},
            {"box_width": 40.1, "box_height": 20.0},
            {"box_width": 9.9, "box_height": 20.0},
            {"position_mm": -115.1},
            {"position_mm": 115.1},
            {"position_mm": float("nan")},
        )
        for changes in rejected:
            with self.subTest(changes=changes):
                self.assertFalse(is_valid(**changes))


class H3FastReplayTests(unittest.TestCase):
    def test_replay_dry_run_encodes_csv_and_wraps_sequence(self):
        csv_text = (
            "delay_ms,valid,x_mm,vx_mm_s\n"
            "0,1,50.0,-20\n"
            "40,0,99,99\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "scenario.csv"
            scenario.write_text(csv_text, encoding="utf-8")

            golden = subprocess.run(
                [
                    sys.executable,
                    str(DEMO_UART / "replay_serial.py"),
                    str(scenario),
                    "--dry-run",
                    "--seq",
                    "42",
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(golden.returncode, 0, golden.stderr)
            self.assertEqual(
                golden.stdout.splitlines(),
                [
                    "A5 5A 2A 01 F4 01 EC FF 73",
                    "A5 5A 2B 00 00 00 00 00 44",
                ],
            )

            wrapped = subprocess.run(
                [
                    sys.executable,
                    str(DEMO_UART / "replay_serial.py"),
                    str(scenario),
                    "--dry-run",
                    "--seq",
                    "255",
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(wrapped.returncode, 0, wrapped.stderr)
            expected = [
                " ".join("{:02X}".format(byte) for byte in encode_frame(255, True, 500, -20)),
                " ".join("{:02X}".format(byte) for byte in encode_frame(0, False)),
            ]
            self.assertEqual(wrapped.stdout.splitlines(), expected)


if __name__ == "__main__":
    unittest.main()
