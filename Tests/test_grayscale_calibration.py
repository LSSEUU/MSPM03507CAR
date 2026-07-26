import importlib.util
import json
import pathlib
import sys
import unittest
from collections import deque
from contextlib import redirect_stdout
from io import StringIO
from unittest import mock


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


class FakeGraySerial:
    def __init__(self):
        self._transactions = deque(
            (
                (
                    b"GRAY scan requested (non-blocking).\r\n",
                    b"GRAY adc=[1000,1001,1002,1003,",
                    b"1004,1005,1006,1007] scans=41 errors=0\r\n",
                    b"GRAY ADC pending=0 power=1 enabled=0 started=0 "
                    b"status=0x00000000 flags=0x00000100\r\n",
                ),
                (
                    b"GRAY scan requested (non-blocking).\r\n",
                    b"GRAY adc=[2000,2001,2002,2003,",
                    b"2004,2005,2006,2007] scans=42 errors=0\r\n",
                    b"GRAY ADC pending=0 power=1 enabled=0 started=0 "
                    b"status=0x00000000 flags=0x00000100\r\n",
                ),
            )
        )
        self._chunks = deque()
        self.writes = []

    def write(self, command):
        if self._chunks:
            raise AssertionError(
                "next q issued before the prior GRAY ADC terminator was consumed"
            )
        self.writes.append(command)
        self._chunks.extend(self._transactions.popleft())

    def flush(self):
        pass

    def readline(self):
        line = bytearray()
        while self._chunks:
            line.extend(self._chunks.popleft())
            if line.endswith(b"\n"):
                break
        return bytes(line)


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

    def test_consecutive_q_transactions_consume_report_terminators(self):
        serial_port = FakeGraySerial()

        first = MODULE._read_sample(serial_port, 0.1)
        second = MODULE._read_sample(serial_port, 0.1)

        self.assertEqual(first.scan_count, 41)
        self.assertEqual(second.scan_count, 42)
        self.assertEqual(serial_port.writes, [b"q", b"q"])

    def test_stage_retries_one_transient_timeout(self):
        readings = (
            TimeoutError("transient UART loss"),
            MODULE.GraySample((3000,) * 8, 100, 0),
            MODULE.GraySample((3001,) * 8, 101, 0),
        )
        with mock.patch.object(MODULE, "SAMPLES_PER_STAGE", 2), mock.patch.object(
            MODULE, "_read_sample", side_effect=readings
        ) as read_sample, redirect_stdout(StringIO()):
            stats = MODULE.capture_stage(object(), "WHITE", 0.1, prompt=False)

        self.assertEqual(stats.count, 2)
        self.assertEqual(stats.mean, (3001,) * 8)
        self.assertEqual(read_sample.call_count, 3)

    def test_stage_rejects_three_consecutive_timeouts(self):
        with mock.patch.object(MODULE, "SAMPLES_PER_STAGE", 1), mock.patch.object(
            MODULE,
            "_read_sample",
            side_effect=[TimeoutError("lost 1"), TimeoutError("lost 2"),
                         TimeoutError("lost 3")],
        ) as read_sample, redirect_stdout(StringIO()):
            with self.assertRaises(TimeoutError):
                MODULE.capture_stage(object(), "WHITE", 0.1, prompt=False)

        self.assertEqual(read_sample.call_count, 3)

    def test_production_entry_delegates_uart_to_m6link_session(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("import serial", source)
        self.assertNotIn("serial.Serial", source)
        self.assertIn("LegacySessionPort", source)
        self.assertIn("Session(", source)


if __name__ == "__main__":
    unittest.main()
