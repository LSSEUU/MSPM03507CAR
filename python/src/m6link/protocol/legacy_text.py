"""V1 ASCII transaction framing used for safe probing and explicit legacy mode."""

from __future__ import annotations

import re


MAX_FRAME_BYTES = 60


def encode_transaction(request_id: int, verb: str, *arguments: str) -> bytes:
    if not 1 <= request_id <= 65535:
        raise ValueError("legacy request id must be 1..65535")
    if not re.fullmatch(r"[a-z]+", verb):
        raise ValueError(f"invalid transaction verb: {verb!r}")
    frame = ":" + ",".join((str(request_id), verb, *arguments)) + "\r\n"
    encoded = frame.encode("ascii")
    if len(encoded) > MAX_FRAME_BYTES:
        raise ValueError(f"transaction frame exceeds {MAX_FRAME_BYTES} bytes")
    return encoded


class LegacyLineDecoder:
    def __init__(self, *, limit: int = 1024) -> None:
        self.limit = limit
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[str]:
        lines: list[str] = []
        self.buffer.extend(data)
        if len(self.buffer) > self.limit:
            self.buffer.clear()
            raise ValueError(f"legacy line exceeded {self.limit} buffered bytes")
        while True:
            positions = [position for position in (self.buffer.find(b"\r"), self.buffer.find(b"\n")) if position >= 0]
            if not positions:
                break
            end = min(positions)
            raw = bytes(self.buffer[:end])
            consumed = end + 1
            if consumed < len(self.buffer) and self.buffer[end] == 13 and self.buffer[consumed] == 10:
                consumed += 1
            del self.buffer[:consumed]
            lines.append(raw.decode("ascii", errors="replace"))
        return lines
