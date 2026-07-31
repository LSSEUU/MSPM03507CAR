from pathlib import Path
import json
import os
import struct
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link.lever_calibration_registry import (  # noqa: E402
    EXTENSION,
    LEVER_CALIBRATION,
    LEVER_CALIBRATION_COMMAND,
    LEVER_CALIBRATION_STATUS_GET,
)
from m6link.session import Session  # noqa: E402


class M6LeverExtensionTests(unittest.TestCase):
    def test_frozen_contract_bindings(self):
        env = dict(os.environ)
        env["PYTHONPATH"] = str(ROOT / "python" / "src")
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "check_m6_lever_extension.py")],
            cwd=ROOT, env=env, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(LEVER_CALIBRATION, 1 << 30)
        self.assertEqual(LEVER_CALIBRATION_STATUS_GET, 16)
        self.assertEqual(LEVER_CALIBRATION_COMMAND, 33546)

        h_mission = json.loads(
            (ROOT / "Tests/vectors/m6_h_mission_ext_v1.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertFalse(
            set(EXTENSION["rpc_opcodes"].values()) &
            set(h_mission["rpc_opcodes"].values())
        )

    def test_snapshot_pages_decode(self):
        summary = bytearray(88)
        struct.pack_into("<IIIIIII", summary, 0, 100, 200, 7, 7, 0xABCDEF01,
                         0x12345678, 0)
        summary[28:32] = bytes((3, 0, 0x0F, 0))
        struct.pack_into("<HHBBBBHHHHH", summary, 32, 12, 54, 3, 0, 1, 0,
                         65000, 12, 8, 100, 200)
        struct.pack_into("<iiiii", summary, 52, 500, -6400, 6400, -640, 640)
        struct.pack_into("<iiii", summary, 72, 10, 20, 0, 123)
        decoded = Session._decode_lever_calibration(bytes(summary))
        self.assertEqual(decoded.progress, 12)
        self.assertEqual(decoded.reference_encoder, 65000)
        self.assertEqual(decoded.last_capture_counts, 123)

        angles = bytearray(summary)
        angles[31] = 1
        for index in range(9):
            struct.pack_into("<i", angles, 32 + index * 4, -8000 + index * 2000)
        decoded_angles = Session._decode_lever_calibration(bytes(angles))
        self.assertEqual(decoded_angles.angles_mdeg[4], 0)


if __name__ == "__main__":
    unittest.main()
