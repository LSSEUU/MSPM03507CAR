from pathlib import Path
import json
import struct
from types import SimpleNamespace
import unittest

from m6link.h3_ki_registry import (
    CAPABILITY_MASK as KI_CAPABILITY, EXTENSION_SHA256, PARAMETER_ID, RPC,
)
from m6link.h3_tune_registry import CAPABILITY_MASK as TUNE_CAPABILITY
from m6link.registry import canonical_digest
from m6link.session import Session


ROOT = Path(__file__).resolve().parents[1]


def tune_snapshot() -> bytes:
    profile = (8000, 18000, 8000, 18000, 105, 36,
               200, 1000, 1500, 1, 5, 5)
    body = bytearray(88)
    struct.pack_into("<8I", body, 0, 100, 99, 4, 7, 0, 0, 0, 0)
    struct.pack_into("<12H", body, 40, *profile)
    struct.pack_into("<12H", body, 64, *profile)
    return bytes(body)


class H3KiExtensionTests(unittest.TestCase):
    def test_registry_digest_and_allocations(self) -> None:
        path = ROOT / "Tests" / "vectors" / "m6_h3_ki_ext_v1.json"
        value = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(canonical_digest(value), EXTENSION_SHA256)
        self.assertEqual(KI_CAPABILITY, 1 << 32)
        self.assertEqual(int(RPC["H3_KI_STATUS_GET"]), 0x0012)
        self.assertEqual(PARAMETER_ID, 13)

    def test_session_enriches_frozen_v1_profile_with_ki(self) -> None:
        session = object.__new__(Session)
        session.identity = SimpleNamespace(
            boot_id=99,
            enabled_capabilities=TUNE_CAPABILITY | KI_CAPABILITY,
        )

        def rpc(opcode, body):
            self.assertEqual(body, b"")
            if opcode == 0x0011:
                return tune_snapshot()
            if opcode == 0x0012:
                return struct.pack("<IIHHHH", 4, 7, 20, 30, 3000, 0)
            self.fail(f"unexpected opcode {opcode:#x}")

        session._rpc = rpc
        snapshot = session.get_h3_tune()
        self.assertEqual(snapshot.active.ki_mdeg_per_mm_s, 20)
        self.assertEqual(snapshot.staged.ki_mdeg_per_mm_s, 30)


if __name__ == "__main__":
    unittest.main()
