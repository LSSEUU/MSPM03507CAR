from pathlib import Path
import json
import struct
from types import SimpleNamespace
import unittest

from m6link.h3_maneuver_registry import (
    CAPABILITY_MASK, EXTENSION_SHA256, PARAMETER_IDS, RPC,
)
from m6link.h3_tune_registry import CAPABILITY_MASK as TUNE_CAPABILITY
from m6link.registry import canonical_digest
from m6link.session import Session


ROOT = Path(__file__).resolve().parents[1]


def tune_snapshot() -> bytes:
    profile = (8000, 18000, 8000, 18000, 105, 36,
               200, 1000, 1500, 3, 5, 5)
    body = bytearray(88)
    struct.pack_into("<8I", body, 0, 100, 99, 4, 7, 0, 0, 0, 0)
    struct.pack_into("<12H", body, 40, *profile)
    struct.pack_into("<12H", body, 64, *profile)
    return bytes(body)


class H3ManeuverExtensionTests(unittest.TestCase):
    def test_registry_digest_and_allocations(self) -> None:
        path = ROOT / "Tests" / "vectors" / "m6_h3_maneuver_ext_v1.json"
        value = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(canonical_digest(value), EXTENSION_SHA256)
        self.assertEqual(CAPABILITY_MASK, 1 << 33)
        self.assertEqual(int(RPC["H3_MANEUVER_STATUS_GET"]), 0x0013)
        self.assertEqual(dict(PARAMETER_IDS), {
            "plus_topup_mdeg": 14,
            "plus_brake_mdeg": 15,
            "minus_brake_trigger_x10": 16,
            "minus_brake_mdeg": 17,
            "plus_brake_trigger_x10": 18,
            "plus_topup_brake_trigger_x10": 19,
            "minus_brake_return_predict_ms": 20,
            "minus_return_rpm": 21,
            "plus_topup_max_speed_mm_s": 22,
        })

    def test_session_enriches_frozen_profile(self) -> None:
        session = object.__new__(Session)
        session.identity = SimpleNamespace(
            boot_id=99,
            enabled_capabilities=TUNE_CAPABILITY | CAPABILITY_MASK,
        )

        def rpc(opcode, body):
            self.assertEqual(body, b"")
            if opcode == 0x0011:
                return tune_snapshot()
            if opcode == 0x0013:
                return struct.pack(
                    "<II18H", 4, 7, 8000, 9000, 8000, 7000,
                    200, 250, 8000, 9000, 450, 500, 250, 260,
                    75, 100, 3, 4, 25, 20,
                )
            self.fail(f"unexpected opcode {opcode:#x}")

        session._rpc = rpc
        snapshot = session.get_h3_tune()
        self.assertEqual(snapshot.active.plus_topup_mdeg, 8000)
        self.assertEqual(snapshot.staged.plus_topup_mdeg, 9000)
        self.assertEqual(snapshot.active.minus_brake_trigger_x10, 200)
        self.assertEqual(snapshot.staged.minus_brake_trigger_x10, 250)
        self.assertEqual(snapshot.active.plus_brake_trigger_x10, 450)
        self.assertEqual(snapshot.staged.plus_brake_trigger_x10, 500)
        self.assertEqual(snapshot.active.plus_topup_brake_trigger_x10, 250)
        self.assertEqual(snapshot.staged.plus_topup_brake_trigger_x10, 260)
        self.assertEqual(snapshot.active.minus_brake_return_predict_ms, 75)
        self.assertEqual(snapshot.staged.minus_brake_return_predict_ms, 100)
        self.assertEqual(snapshot.active.minus_return_rpm, 3)
        self.assertEqual(snapshot.staged.minus_return_rpm, 4)
        self.assertEqual(snapshot.active.plus_topup_max_speed_mm_s, 25)
        self.assertEqual(snapshot.staged.plus_topup_max_speed_mm_s, 20)


if __name__ == "__main__":
    unittest.main()
