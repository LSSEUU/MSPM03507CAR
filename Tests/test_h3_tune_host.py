from pathlib import Path
import json
import struct
from types import SimpleNamespace
import threading
import unittest

from m6link.h3_tune_registry import (
    CAPABILITY_MASK, EXTENSION_SHA256, H3TuneCommand, H3TuneParameter, RPC,
)
from m6link.registry import canonical_digest
from m6link.session import Session


ROOT = Path(__file__).resolve().parents[1]


def snapshot_bytes(*, active_revision: int = 4, stage_revision: int = 7,
                   token: int = 1234) -> bytes:
    profile = (8000, 18000, 8000, 18000, 105, 36,
               200, 1000, 1500, 1, 5, 5)
    body = bytearray(88)
    struct.pack_into("<8I", body, 0, 100, 99, active_revision, stage_revision,
                     2, token, 10000, 12000)
    struct.pack_into("<H7B", body, 32, 0x12, 8, 0, 0, 0, 0, 0, 0)
    struct.pack_into("<12H", body, 40, *profile)
    struct.pack_into("<12H", body, 64, *profile)
    return bytes(body)


class H3TuneHostTests(unittest.TestCase):
    def test_registry_digest_and_wire_allocations(self) -> None:
        path = ROOT / "Tests" / "vectors" / "m6_h3_tune_ext_v1.json"
        value = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(canonical_digest(value), EXTENSION_SHA256)
        self.assertEqual(CAPABILITY_MASK, 1 << 31)
        self.assertEqual(int(RPC["H3_TUNE_STATUS_GET"]), 17)
        self.assertEqual(int(RPC["H3_TUNE_COMMAND"]), 0x830B)

    def test_snapshot_decode_and_single_field_request(self) -> None:
        snapshot = Session._decode_h3_tune(snapshot_bytes())
        self.assertEqual(snapshot.active_revision, 4)
        self.assertEqual(snapshot.active.plus_max_mdeg, 18000)

        session = object.__new__(Session)
        session.identity = SimpleNamespace(
            boot_id=99, enabled_capabilities=CAPABILITY_MASK
        )
        captured = {}

        def rpc(opcode, body):
            captured["opcode"] = opcode
            captured["body"] = body
            return snapshot_bytes(active_revision=4, stage_revision=8)

        session._rpc = rpc
        result = session.stage_h3_tune_field(
            H3TuneParameter.plus_max_mdeg, 22000, snapshot=snapshot
        )
        self.assertEqual(captured["opcode"], 0x830B)
        self.assertEqual(len(captured["body"]), 28)
        values = struct.unpack("<IIIBBHIII", captured["body"])
        self.assertEqual(values[:3], (99, 4, 7))
        self.assertEqual(values[3], 1)
        self.assertEqual(values[4], int(H3TuneParameter.plus_max_mdeg))
        self.assertEqual(values[6], 22000)
        self.assertEqual(result.stage_revision, 8)

    def test_center_hold_start_uses_dedicated_command(self) -> None:
        session = object.__new__(Session)
        session.identity = SimpleNamespace(
            boot_id=99, enabled_capabilities=CAPABILITY_MASK
        )
        captured = {}

        def command(command, **kwargs):
            captured["command"] = command
            captured["token"] = kwargs["value"]
            return Session._decode_h3_tune(snapshot_bytes())

        session._h3_tune_command = command
        session.start_h3_center_hold(1234)
        self.assertEqual(captured["command"], H3TuneCommand.START_CENTER)
        self.assertEqual(captured["token"], 1234)

    def test_time_critical_heartbeat_does_not_wait_for_rpc_response(self) -> None:
        session = object.__new__(Session)
        session.identity = SimpleNamespace(
            boot_id=99, enabled_capabilities=CAPABILITY_MASK
        )
        session.mode = "v2"
        session._session_id = 7
        session._outcome_unknown = None
        session._rpc_lock = threading.Lock()
        session._next_sequence = 11
        captured = {}

        def queue_write_nowait(wire, *, priority):
            captured.update(wire=wire, priority=priority)

        session._queue_write_nowait = queue_write_nowait
        session.heartbeat_h3_tune_nowait(1234)
        self.assertGreater(len(captured["wire"]), 28)
        self.assertEqual(captured["priority"], 5)
        self.assertEqual(session._next_sequence, 12)


if __name__ == "__main__":
    unittest.main()
