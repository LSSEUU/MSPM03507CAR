from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link import ActionKind, ActionSpec, Session, TelemetrySample  # noqa: E402
from m6link.protocol.framing import Frame, FrameDecoder, encode_frame  # noqa: E402
from m6link.recording import _consume_transition_sample, _recording_subscriptions  # noqa: E402
from m6link.registry import CAPABILITY_MASKS, FrameType, REGISTRY, RpcOpcode, TelemetryStream  # noqa: E402
from m6link.session import _async_gap_details  # noqa: E402


class V2FakeSerial:
    def __init__(self) -> None:
        self.port = "COMV2"
        self.responses: deque[bytes] = deque()
        self.writes: list[bytes] = []
        self.io_threads: set[int] = set()
        self.closed = False
        self.decoder = FrameDecoder()
        self.session_id = 0x12345678
        self.opcodes: list[int] = []

    @property
    def in_waiting(self) -> int:
        return len(self.responses[0]) if self.responses else 0

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def read(self, size: int) -> bytes:
        self.io_threads.add(threading.get_ident())
        if not self.responses:
            return b""
        data = self.responses.popleft()
        if len(data) > size:
            self.responses.appendleft(data[size:])
            return data[:size]
        return data

    def write(self, data: bytes) -> int:
        self.io_threads.add(threading.get_ident())
        data = bytes(data)
        self.writes.append(data)
        if data == b"S":
            return 1
        if data.startswith(b":"):
            request = data.decode("ascii").strip().split(",")
            request_id = int(request[0][1:])
            verb = request[1]
            if verb == "get":
                values = {
                    "meta.firmware": "M6-runtime-test",
                    "meta.schema": "1",
                    "meta.revision": "12",
                    "meta.source": "defaults",
                    "meta.generation": "3",
                    "meta.capabilities": f"{CAPABILITY_MASKS['ALL_DEFINED']:016x}",
                }
                self.responses.append(
                    f"ok,{request_id},{request[2]},{values[request[2]]}\r\n".encode()
                )
            elif verb == "mode":
                self.responses.append(
                    f"ok,{request_id},v2-ready,{request[3]},2,1,96,32,1\r\n".encode()
                )
            return len(data)
        frames = self.decoder.feed(data)
        assert len(frames) == 1
        frame = frames[0]
        if frame.type == int(FrameType.HELLO):
            challenge = struct.unpack_from("<Q", frame.payload, 8)[0]
            ready = struct.pack(
                "<IHHIQIIIIH2sQQHHB3s",
                0x59443252,
                1,
                0,
                0,
                challenge,
                self.session_id,
                7,
                0x0C,
                2,
                1,
                b"\0\0",
                CAPABILITY_MASKS["ALL_DEFINED"],
                CAPABILITY_MASKS["ALL_DEFINED"],
                96,
                32,
                1,
                b"\0\0\0",
            )
            self.responses.append(encode_frame(Frame(2, int(FrameType.READY), 0, 0, 0, ready)))
            return len(data)
        opcode = struct.unpack_from("<H", frame.payload)[0]
        self.opcodes.append(opcode)
        request_body = frame.payload[2:]
        if opcode == int(RpcOpcode.IDENTITY_GET):
            firmware = b"M6-runtime-test"
            success = struct.pack(
                "<BBHHBBIIIIQ",
                2,
                0,
                1,
                1,
                0,
                len(firmware),
                12,
                4,
                3,
                7,
                CAPABILITY_MASKS["ALL_DEFINED"],
            ) + firmware
        elif opcode == int(RpcOpcode.ACTION_ARM):
            success = struct.pack("<II", 0xAABBCCDD, 10000) + request_body
        elif opcode == int(RpcOpcode.ACTION_CONFIRM):
            success = struct.pack("<IIIBBH", 9, 100, 1600, 2, 1, 0)
        elif opcode == int(RpcOpcode.HEADING_CALIBRATE):
            success = struct.pack("<II", 321, 20)
        elif opcode == int(RpcOpcode.TASK_STATS_GET):
            success = bytearray(64)
            success[0] = 4
        elif opcode == int(RpcOpcode.UART_STATS_GET):
            success = bytes(32)
        elif opcode == int(RpcOpcode.LINK_STATS_GET):
            success = bytearray(76)
            success[0] = 3
            success[1] = 1
            struct.pack_into("<I", success, 4, self.session_id)
            struct.pack_into("<I", success, 60, 5)
        elif opcode == int(RpcOpcode.SUBSCRIPTION_SET):
            count = request_body[1]
            entries = request_body[4 : 4 + count * 4]
            success = struct.pack("<B3xI", count, 0) + entries
        elif opcode == int(RpcOpcode.SESSION_CLOSE):
            success = b""
        else:
            raise AssertionError(f"unexpected opcode {opcode:#x}")
        payload = struct.pack("<HHI", opcode, 0, 0) + success
        self.responses.append(
            encode_frame(
                Frame(2, int(FrameType.RPC_RESPONSE), 0, self.session_id, frame.sequence, payload)
            )
        )
        return len(data)


class M6LinkCodecTests(unittest.TestCase):
    def test_encoder_matches_all_frozen_frame_vectors(self):
        registry = json.loads((ROOT / "Tests" / "vectors" / "m6_v2_wire_v1.json").read_text(encoding="utf-8"))
        frame_types = {item["name"]: item["value"] for item in registry["registries"]["frame_types"]}
        for vector in registry["vectors"]["frames"]:
            frame_type = frame_types.get(vector["type"], 0x7F)
            actual = encode_frame(
                Frame(
                    2,
                    frame_type,
                    vector["flags"],
                    vector["session"],
                    vector["sequence"],
                    bytes.fromhex(vector["payload_hex"]),
                )
            )
            self.assertEqual(actual, bytes.fromhex(vector["expected"]["wire_hex"]), vector["id"])

    def test_decoder_accepts_fragmented_golden_frame(self):
        registry = json.loads((ROOT / "Tests" / "vectors" / "m6_v2_wire_v1.json").read_text(encoding="utf-8"))
        wire = bytes.fromhex(registry["vectors"]["frames"][0]["expected"]["wire_hex"])
        decoder = FrameDecoder()
        frames = []
        for byte in wire:
            frames.extend(decoder.feed(bytes((byte,)), 1))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, bytes.fromhex(registry["vectors"]["frames"][0]["payload_hex"]))


class M6LinkSessionTests(unittest.TestCase):
    def test_async_gap_provenance_keeps_wire_and_sequence_sources_separate(self):
        self.assertEqual(_async_gap_details(0, 7, 0), (False, False, 0))
        self.assertEqual(_async_gap_details(7, 8, 2), (True, False, 0))
        self.assertEqual(_async_gap_details(7, 9, 0), (False, True, 8))
        self.assertEqual(_async_gap_details(7, 9, 2), (True, True, 8))

    def test_default_recording_profile_is_50_hz_and_within_admission_budget(self):
        subscriptions = _recording_subscriptions(20)
        self.assertEqual(
            subscriptions,
            {
                TelemetryStream.CORE: 20,
                TelemetryStream.WHEEL: 40,
                TelemetryStream.LINE: 60,
                TelemetryStream.HEALTH: 1000,
            },
        )
        rules = {item["value"]: item for item in REGISTRY["telemetry"]["streams"]}
        estimated = sum(
            ((1000 + period - 1) // period)
            * (17 + int(rules[int(stream)]["payload_size"]))
            for stream, period in subscriptions.items()
        )
        self.assertEqual(estimated, 6105)
        with self.assertRaises(ValueError):
            _recording_subscriptions(25)

    def test_recording_consumes_only_the_first_boundary_sample_per_stream(self):
        unsettled = {TelemetryStream.CORE, TelemetryStream.WHEEL}
        core = TelemetrySample(TelemetryStream.CORE, 1, 10, {}, gap_before=True)
        wheel = TelemetrySample(TelemetryStream.WHEEL, 1, 10, {}, gap_before=True)
        self.assertTrue(_consume_transition_sample(core, unsettled))
        self.assertFalse(_consume_transition_sample(core, unsettled))
        self.assertTrue(_consume_transition_sample(wheel, unsettled))
        self.assertEqual(unsettled, set())

    def test_event_tap_fans_out_without_consuming_controller_queue(self):
        session = Session()
        marker = object()
        with session.event_tap() as tap:
            session._publish_event(marker)
            self.assertIs(tap.get_nowait(), marker)
            self.assertIs(session.read_event(), marker)

    def test_session_negotiates_v2_and_never_auto_confirms_arm(self):
        fake = V2FakeSerial()
        session = Session(transport=fake, timeout=0.5)
        identity = session.connect()
        self.assertEqual(identity.firmware_id, "M6-runtime-test")
        legacy_writes = [item for item in fake.writes if item.startswith(b":")]
        self.assertTrue(legacy_writes)
        self.assertEqual(legacy_writes[0], b":1,get,meta.capabilities\r\n")
        armed = session.arm_action(
            ActionSpec(
                kind=ActionKind.WHEEL_SPEED_TEST,
                target_l_mm_s=160,
                target_r_mm_s=160,
                duration_ms=1500,
            )
        )
        self.assertEqual(armed.token, 0xAABBCCDD)
        self.assertEqual(fake.opcodes, [int(RpcOpcode.IDENTITY_GET), int(RpcOpcode.ACTION_ARM)])
        session.confirm_action(armed)
        self.assertEqual(fake.opcodes[-1], int(RpcOpcode.ACTION_CONFIRM))
        session.close()

    def test_only_session_worker_touches_injected_transport(self):
        fake = V2FakeSerial()
        caller = threading.get_ident()
        session = Session(transport=fake, timeout=0.5)
        session.connect()
        session.emergency_stop()
        session.close()
        self.assertNotIn(caller, fake.io_threads)
        self.assertEqual(len(fake.io_threads), 1)
        self.assertIn(b"S", fake.writes)

    def test_heading_calibration_is_an_explicit_non_action_rpc(self):
        fake = V2FakeSerial()
        session = Session(transport=fake, timeout=0.5)
        session.connect()
        result = session.calibrate_heading()
        self.assertEqual(result, {"started_ms": 321, "calibration_count_before": 20})
        self.assertEqual(fake.opcodes[-1], int(RpcOpcode.HEADING_CALIBRATE))
        self.assertNotIn(int(RpcOpcode.ACTION_ARM), fake.opcodes)
        self.assertNotIn(int(RpcOpcode.ACTION_CONFIRM), fake.opcodes)
        session.close()

    def test_empty_subscription_response_decodes_as_an_empty_array(self):
        fake = V2FakeSerial()
        session = Session(transport=fake, timeout=0.5)
        session.connect()
        subscriptions, estimated_bps = session.subscribe({})
        self.assertEqual(subscriptions, ())
        self.assertEqual(estimated_bps, 0)
        session.close()

    def test_manual_recording_can_stop_without_consuming_controller_events(self):
        fake = V2FakeSerial()
        session = Session(transport=fake, timeout=0.5)
        session.connect()
        with tempfile.TemporaryDirectory() as directory:
            thread = session.start_recording(
                Path(directory) / "manual",
                duration_s=None,
                period_ms=20,
                test_mode="wheel",
                requested_speed=160,
            )
            self.assertTrue(thread.is_alive())
            self.assertIsNotNone(session._recording_ready)
            self.assertTrue(session._recording_ready.is_set())
            time.sleep(0.05)
            result = session.stop_recording()
            self.assertIsNotNone(result)
            self.assertTrue(result.csv_path.exists())
            self.assertTrue(result.json_path.exists())
        self.assertEqual(fake.opcodes.count(int(RpcOpcode.SUBSCRIPTION_SET)), 2)
        self.assertEqual(fake.opcodes.count(int(RpcOpcode.TASK_STATS_GET)), 1)
        self.assertEqual(fake.opcodes.count(int(RpcOpcode.UART_STATS_GET)), 1)
        self.assertEqual(fake.opcodes.count(int(RpcOpcode.LINK_STATS_GET)), 1)
        session.close()


if __name__ == "__main__":
    unittest.main()
