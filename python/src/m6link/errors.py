"""Typed failures exposed by :mod:`m6link`."""

from __future__ import annotations

from typing import Sequence


class M6Error(RuntimeError):
    """Base class for expected host-link failures."""


class Disconnected(M6Error):
    """The serial transport is not connected or stopped responding."""


class IncompatibleFirmware(M6Error):
    """The device identity, registry, or required capability is incompatible."""


class IntegrityError(M6Error):
    """A frame failed a frozen wire integrity invariant."""


class UnsupportedCapability(M6Error):
    """The connected session did not negotiate a required capability."""

    def __init__(self, capability: object):
        self.capability = capability
        super().__init__(f"capability is not enabled: {capability}")


class TimeoutBeforeSend(M6Error):
    """A queued operation timed out before any byte was handed to the transport."""


class OutcomeUnknown(M6Error):
    """Bytes were sent, but the result cannot be recovered safely."""

    def __init__(self, message: str, *, sequence: int | None = None, frame: bytes | None = None):
        self.sequence = sequence
        self.frame = frame
        super().__init__(message)


class DeviceRejected(M6Error):
    """The firmware returned a deterministic non-OK RPC result."""

    def __init__(
        self,
        result: int,
        detail: int = 0,
        *,
        opcode: int | None = None,
        name: str | None = None,
    ) -> None:
        self.result = int(result)
        self.detail = int(detail)
        self.opcode = opcode
        self.name = name or f"0x{self.result:04x}"
        suffix = f" for opcode 0x{opcode:04x}" if opcode is not None else ""
        super().__init__(f"device rejected{suffix}: {self.name} (detail=0x{self.detail:08x})")


class SafetyTrip(DeviceRejected):
    """A Safety fault rejected or interrupted an operation."""


class DeviceError(M6Error):
    """Compatibility error returned by the legacy V1 text protocol."""

    def __init__(self, request_id: int, code: str, details: Sequence[str] = ()):
        self.request_id = request_id
        self.code = code
        self.details = tuple(details)
        suffix = "," + ",".join(self.details) if self.details else ""
        super().__init__(f"device error {code} for transaction {request_id}{suffix}")


class TransportError(Disconnected):
    """Backward-compatible name for a serial transport failure."""


class SafetyAbort(M6Error):
    """A recording or acceptance run observed a hard safety condition."""

    def __init__(self, reason: str, samples: Sequence[object] = ()):
        self.reason = reason
        self.samples = list(samples)
        super().__init__(reason)
