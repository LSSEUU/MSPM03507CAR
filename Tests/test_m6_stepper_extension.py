from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
from types import MethodType, SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link.models import StepperSnapshot
from m6link.registry import Capability
from m6link.session import Session
from m6link.stepper_registry import CAPABILITY_MASK, EXTENSION_SHA256, StepperNotReady


def canonical(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")).hexdigest()


class M6StepperExtensionTests(unittest.TestCase):
    def test_frozen_base_and_extension_digests(self) -> None:
        base = json.loads((ROOT / "Tests/vectors/m6_v2_wire_v1.json").read_text(encoding="utf-8"))
        extension = json.loads((ROOT / "Tests/vectors/m6_zdt_stepper_ext_v1.json").read_text(encoding="utf-8"))
        self.assertEqual(canonical(base), "724d64dfd91a613b5ed7a6e297730623623a69859b047a4a72433abf0a0d410d")
        self.assertEqual(canonical(extension), EXTENSION_SHA256)
        self.assertEqual(CAPABILITY_MASK, 1 << 28)
        self.assertEqual(int(StepperNotReady.NO_MOTOR_STATUS), 1 << 0)
        self.assertEqual(int(StepperNotReady.MOTOR_STATUS_STALE), 1 << 1)
        self.assertEqual(int(StepperNotReady.INTERNAL_REJECT), 1 << 8)

    def test_generated_binding_checker(self) -> None:
        result = subprocess.run([sys.executable, "scripts/check_m6_stepper_extension.py"], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_snapshot_signed_values_and_command_layouts(self) -> None:
        body = bytearray(88)
        struct.pack_into("<III", body, 0, 1234, 9, 15)
        body[12:14] = bytes((0x36, 7))
        struct.pack_into("<HhBBq", body, 14, 0x0F, -60, 0x03, 0x05, -65536)
        struct.pack_into("<IIIIIIIII", body, 28, 1, 2, 3, 4, 5, 6, 7, 8, 9)
        struct.pack_into("<III", body, 64, 10, 11, 120)
        struct.pack_into("<BBBBHBBI", body, 76, 2, 1, 1, 20, 60, 0, 0, 100)
        snapshot = Session._decode_stepper(bytes(body))
        self.assertIsInstance(snapshot, StepperSnapshot)
        self.assertEqual(snapshot.speed_rpm, -60)
        self.assertEqual(snapshot.position_raw, -65536)
        self.assertEqual(snapshot.position_revolutions, -1.0)
        self.assertEqual(snapshot.position_degrees, -360.0)

        session = Session.__new__(Session)
        session.identity = SimpleNamespace(boot_id=0x11223344, enabled_capabilities=Capability(CAPABILITY_MASK))
        session._next_stepper_command_id = 1
        captured: list[tuple[int, bytes]] = []
        def rpc(_self, opcode: int, request: bytes = b"", **_kwargs: object) -> bytes:
            captured.append((int(opcode), bytes(request)))
            return bytes(body)
        session._rpc = MethodType(rpc, session)
        session.start_stepper_velocity_test(direction=1, speed_rpm=60, acceleration=20, duration_ms=1000)
        self.assertEqual(len(captured[-1][1]), 24)
        self.assertEqual(captured[-1][1], struct.pack("<IIBBBBHHIB3x", 0x11223344, 1, 4, 1, 1, 20, 60, 1000, 0, 0))
        session.start_stepper_position_test(direction=0, speed_rpm=30, acceleration=10, pulses=100, timeout_ms=5000)
        self.assertEqual(captured[-1][1], struct.pack("<IIBBBBHHIB3x", 0x11223344, 2, 5, 1, 0, 10, 30, 5000, 100, 2))


if __name__ == "__main__":
    unittest.main()
