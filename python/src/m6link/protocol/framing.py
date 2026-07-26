"""M6 V2 byte framing driven by the frozen registry constants."""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import Iterable

from ..errors import IntegrityError
from ..registry import FRAMING, PARSER


@dataclass(frozen=True, slots=True)
class Frame:
    version: int
    type: int
    flags: int
    session: int
    sequence: int
    payload: bytes


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    crc = int(FRAMING["crc16"]["init"])
    polynomial = int(FRAMING["crc16"]["poly"])
    for byte in data:
        crc ^= int(byte) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ polynomial) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc ^ int(FRAMING["crc16"]["xorout"])


def encode_frame(frame: Frame) -> bytes:
    payload = bytes(frame.payload)
    if len(payload) > int(FRAMING["max_payload"]):
        raise ValueError(f"payload exceeds {FRAMING['max_payload']} bytes")
    if not 0 <= frame.version <= 0xFF or not 0 <= frame.type <= 0xFF or not 0 <= frame.flags <= 0xFF:
        raise ValueError("frame version/type/flags must fit u8")
    if not 0 <= frame.session <= 0xFFFFFFFF or not 0 <= frame.sequence <= 0xFFFFFFFF:
        raise ValueError("frame session/sequence must fit u32")
    header = struct.pack(
        "<BBBIIH",
        frame.version,
        frame.type,
        frame.flags,
        frame.session,
        frame.sequence,
        len(payload),
    )
    decoded = header + payload
    decoded += struct.pack("<H", crc16_ccitt_false(decoded))
    must_escape = set(int(value) for value in FRAMING["must_escape"])
    escape = int(FRAMING["escape"])
    escape_xor = int(FRAMING["escape_xor"])
    body = bytearray()
    for byte in decoded:
        if byte in must_escape:
            body.extend((escape, byte ^ escape_xor))
        else:
            body.append(byte)
    wire = bytes((int(FRAMING["delimiter"]),)) + bytes(body) + bytes((int(FRAMING["delimiter"]),))
    if len(wire) > int(FRAMING["max_wire_frame"]):
        raise ValueError(f"encoded frame exceeds {FRAMING['max_wire_frame']} bytes")
    return wire


def decode_frame(decoded: bytes) -> Frame:
    minimum = int(FRAMING["header_size"]) + int(FRAMING["crc_size"])
    if len(decoded) < minimum:
        raise IntegrityError("short V2 frame")
    version, frame_type, flags, session, sequence, payload_length = struct.unpack_from("<BBBIIH", decoded)
    expected = int(FRAMING["header_size"]) + payload_length + int(FRAMING["crc_size"])
    if payload_length > int(FRAMING["max_payload"]) or len(decoded) != expected:
        raise IntegrityError("V2 payload length mismatch")
    received_crc = struct.unpack_from("<H", decoded, len(decoded) - 2)[0]
    calculated_crc = crc16_ccitt_false(decoded[:-2])
    if received_crc != calculated_crc:
        raise IntegrityError("V2 CRC mismatch")
    return Frame(version, frame_type, flags, session, sequence, decoded[13:-2])


class FrameDecoder:
    """Streaming delimiter/escape parser with the frozen timeout boundaries."""

    COUNTERS = (
        "crc_error",
        "short_frame",
        "length_error",
        "escape_error",
        "decoded_overflow",
        "wire_overflow",
        "interbyte_timeout",
        "total_timeout",
        "epoch_abort",
    )

    def __init__(self) -> None:
        self.counters = {name: 0 for name in self.COUNTERS}
        self.state = "HUNT"
        self._decoded = bytearray()
        self._wire_count = 0
        self._started = False
        self._start_ms = 0
        self._last_ms = 0

    @staticmethod
    def _now_ms() -> int:
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    @staticmethod
    def _elapsed(now: int, then: int) -> int:
        return (now - then) & 0xFFFFFFFF

    def _empty_collect(self) -> None:
        self.state = "COLLECT"
        self._decoded.clear()
        self._wire_count = 1
        self._started = False

    def _hunt(self) -> None:
        self.state = "HUNT"
        self._decoded.clear()
        self._wire_count = 0
        self._started = False

    def _expire(self, now: int) -> None:
        if self.state not in ("COLLECT", "ESCAPED", "DISCARD") or not self._started:
            return
        if self._elapsed(now, self._start_ms) >= int(PARSER["total_timeout_ms"]):
            self.counters["total_timeout"] += 1
            self._hunt()
        elif self._elapsed(now, self._last_ms) >= int(PARSER["interbyte_timeout_ms"]):
            self.counters["interbyte_timeout"] += 1
            self._hunt()

    def tick(self, now_ms: int | None = None) -> None:
        self._expire(self._now_ms() if now_ms is None else now_ms & 0xFFFFFFFF)

    def epoch(self) -> None:
        if self.state != "HUNT" and (self._started or self.state in ("ESCAPED", "DISCARD")):
            self.counters["epoch_abort"] += 1
        self._hunt()

    def feed(self, data: bytes | bytearray | memoryview, now_ms: int | None = None) -> list[Frame]:
        now = self._now_ms() if now_ms is None else now_ms & 0xFFFFFFFF
        frames: list[Frame] = []
        for byte in data:
            frame = self._feed_byte(int(byte), now)
            if frame is not None:
                frames.append(frame)
        return frames

    def _feed_byte(self, byte: int, now: int) -> Frame | None:
        self._expire(now)
        delimiter = int(FRAMING["delimiter"])
        if byte == delimiter:
            result: Frame | None = None
            if self.state == "COLLECT" and self._started:
                result = self._finalize()
            elif self.state == "ESCAPED":
                self.counters["escape_error"] += 1
            self._empty_collect()
            return result
        if self.state == "HUNT":
            return None
        if self.state == "DISCARD":
            self._last_ms = now
            return None
        if self._wire_count + 2 > int(FRAMING["max_wire_frame"]):
            self.counters["wire_overflow"] += 1
            self.state = "DISCARD"
            self._last_ms = now
            return None
        if not self._started:
            self._started = True
            self._start_ms = now
        self._last_ms = now
        self._wire_count += 1
        if self.state == "COLLECT":
            if byte == int(FRAMING["escape"]):
                self.state = "ESCAPED"
            else:
                self._append(byte)
            return None
        allowed = {int(value) ^ int(FRAMING["escape_xor"]) for value in FRAMING["must_escape"]}
        if byte not in allowed:
            self.counters["escape_error"] += 1
            self.state = "DISCARD"
            return None
        self._append(byte ^ int(FRAMING["escape_xor"]))
        return None

    def _append(self, byte: int) -> None:
        if len(self._decoded) >= int(FRAMING["max_decoded_frame"]):
            self.counters["decoded_overflow"] += 1
            self.state = "DISCARD"
            return
        self._decoded.append(byte)
        self.state = "COLLECT"

    def _finalize(self) -> Frame | None:
        decoded = bytes(self._decoded)
        minimum = int(FRAMING["header_size"]) + int(FRAMING["crc_size"])
        if len(decoded) < minimum:
            self.counters["short_frame"] += 1
            return None
        payload_length = struct.unpack_from("<H", decoded, 11)[0]
        expected = int(FRAMING["header_size"]) + payload_length + int(FRAMING["crc_size"])
        if payload_length > int(FRAMING["max_payload"]) or len(decoded) != expected:
            self.counters["length_error"] += 1
            return None
        received_crc = struct.unpack_from("<H", decoded, len(decoded) - 2)[0]
        if received_crc != crc16_ccitt_false(decoded[:-2]):
            self.counters["crc_error"] += 1
            return None
        return decode_frame(decoded)
