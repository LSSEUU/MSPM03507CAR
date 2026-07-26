"""Single-owner serial Session for legacy probing and V2 operation."""

from __future__ import annotations

from dataclasses import replace
import itertools
import queue
import re
import secrets
import struct
import threading
import time
from typing import Any, Callable, Iterable, Mapping
from contextlib import contextmanager

from .errors import (
    DeviceError,
    DeviceRejected,
    Disconnected,
    IncompatibleFirmware,
    IntegrityError,
    OutcomeUnknown,
    SafetyTrip,
    TimeoutBeforeSend,
    TransportError,
    UnsupportedCapability,
)
from .models import (
    ActionSpec,
    ActionStatus,
    ArmedAction,
    Breadcrumb,
    FaultEvent,
    GapEvent,
    Identity,
    LinkEvent,
    LinkStats,
    ParameterDescriptor,
    ParameterSnapshot,
    ParameterValue,
    ResetEvent,
    ResetInfo,
    Status,
    Subscription,
    TaskStats,
    TelemetrySample,
    UartStats,
)
from .protocol.framing import Frame, FrameDecoder, encode_frame
from .protocol.legacy_text import LegacyLineDecoder, encode_transaction
from .protocol.v2_binary import (
    bits_to_value,
    decode_event,
    decode_layout,
    decode_ready,
    decode_rpc_response,
    decode_telemetry,
    encode_hello,
    encode_rpc,
    pack_subscription,
    parameter_type_for_id,
    value_to_bits,
)
from .registry import (
    ActionKind,
    ActionReason,
    ActionState,
    CAPABILITY_MASKS,
    Capability,
    FrameFlag,
    FrameType,
    PARAMETER_BY_ID,
    PARAMETER_BY_KEY,
    ParameterSource,
    ParameterType,
    REGISTRY,
    RPC_OPCODE_INFO,
    RPC_RESULT_NAMES,
    RpcOpcode,
    RpcResult,
    TelemetryStream,
)


BAUD_RATE = 115200
PARAMETER_SCHEMA = 1
DEFAULT_FIRMWARE_PREFIX = "M6"


class _Write:
    __slots__ = ("data", "started", "done", "error")

    def __init__(self, data: bytes) -> None:
        self.data = bytes(data)
        self.started = threading.Event()
        self.done = threading.Event()
        self.error: BaseException | None = None


class _PendingLine:
    __slots__ = ("event", "line")

    def __init__(self) -> None:
        self.event = threading.Event()
        self.line: str | None = None


class Session:
    """Own a serial port with one background read/write thread.

    Construction is side-effect free.  :meth:`connect` performs safe V1 text
    identity/capability probes, then establishes V2 unless ``legacy=True`` was
    explicitly selected.  No RPC is automatically retransmitted.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        baudrate: int = BAUD_RATE,
        legacy: bool = False,
        timeout: float = 1.0,
        serial_factory: Callable[..., Any] | None = None,
        port_provider: Callable[[], Iterable[Any]] | None = None,
        transport: Any | None = None,
        firmware_prefix: str = DEFAULT_FIRMWARE_PREFIX,
        firmware_id: str | None = None,
        parameter_schema: int = PARAMETER_SCHEMA,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.legacy_requested = bool(legacy)
        self.timeout = max(0.05, float(timeout))
        self.serial_factory = serial_factory
        self.port_provider = port_provider
        self._injected_transport = transport
        self.firmware_prefix = firmware_prefix
        self.expected_firmware_id = firmware_id
        self.expected_parameter_schema = int(parameter_schema)

        self.mode = "disconnected"
        self.identity: Identity | None = None
        self.status: Status | None = None
        self._transport: Any | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker_error: BaseException | None = None
        self._writes: queue.PriorityQueue[tuple[int, int, _Write]] = queue.PriorityQueue()
        self._write_counter = itertools.count()
        self._legacy_decoder = LegacyLineDecoder()
        self._v2_decoder = FrameDecoder()
        self._route_mode = "legacy"
        self._legacy_lock = threading.Lock()
        self._rpc_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._next_legacy_id = 1
        self._next_sequence = 1
        self._session_id = 0
        self._outcome_unknown: OutcomeUnknown | None = None
        self._pending_lines: dict[int, _PendingLine] = {}
        self._pending_frames: dict[int, queue.Queue[Frame]] = {}
        self._orphan_lines: dict[int, str] = {}
        self.unsolicited: list[str] = []
        self._line_events: queue.Queue[str] = queue.Queue()
        self._events: queue.Queue[object] = queue.Queue()
        self._telemetry: queue.Queue[TelemetrySample] = queue.Queue()
        self._event_taps: set[queue.Queue[object]] = set()
        self._subscriptions: dict[TelemetryStream, int] = {}
        self._recording_thread: threading.Thread | None = None
        self._recording_stop: threading.Event | None = None
        self._recording_result: object | None = None
        self._recording_error: BaseException | None = None
        self._last_async_sequence = 0

    @property
    def capabilities(self) -> Capability:
        return self.identity.enabled_capabilities if self.identity else Capability(0)

    @property
    def connected(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and self.mode != "disconnected"

    def __enter__(self) -> "Session":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @staticmethod
    def discover_ports(port_provider: Callable[[], Iterable[Any]] | None = None) -> list[str]:
        if port_provider is None:
            try:
                from serial.tools import list_ports  # type: ignore
            except ImportError as exc:
                raise Disconnected("pyserial is required: python -m pip install pyserial") from exc
            port_provider = list_ports.comports
        return [str(getattr(item, "device", item)) for item in port_provider()]

    def connect(self, port: str | None = None, *, legacy: bool | None = None) -> Identity:
        if self.connected:
            assert self.identity is not None
            return self.identity
        if legacy is not None:
            self.legacy_requested = bool(legacy)
        candidates = [port or self.port] if (port or self.port) else self.discover_ports(self.port_provider)
        if self._injected_transport is not None:
            candidates = [str(getattr(self._injected_transport, "port", port or self.port or "<injected>"))]
        if not candidates:
            raise Disconnected("no serial ports are enumerated")
        failures: list[str] = []
        for candidate in candidates:
            try:
                self._open_candidate(str(candidate))
                legacy_identity = self._probe_v1_identity()
                if self.legacy_requested:
                    self.identity = legacy_identity
                    self.mode = "legacy"
                    return legacy_identity
                capabilities = self._probe_v1_capabilities()
                required = int(CAPABILITY_MASKS["HANDSHAKE_REQUIRED"])
                if capabilities & required != required:
                    raise IncompatibleFirmware(
                        f"firmware lacks V2 handshake capabilities 0x{required & ~capabilities:016x}"
                    )
                self._establish_v2(capabilities)
                identity = self.get_identity()
                if identity.parameter_schema != self.expected_parameter_schema:
                    raise IncompatibleFirmware(
                        f"parameter schema {identity.parameter_schema} does not match {self.expected_parameter_schema}"
                    )
                if not identity.firmware_id.startswith(self.firmware_prefix):
                    raise IncompatibleFirmware(f"unexpected firmware identity {identity.firmware_id!r}")
                if self.expected_firmware_id and identity.firmware_id != self.expected_firmware_id:
                    raise IncompatibleFirmware(f"firmware does not match {self.expected_firmware_id!r}")
                self.identity = identity
                self.mode = "v2"
                return identity
            except Exception as exc:
                failures.append(f"{candidate}: {exc}")
                self._shutdown_transport()
                if self._injected_transport is not None:
                    break
        raise IncompatibleFirmware("no compatible M6 firmware found; " + "; ".join(failures))

    @classmethod
    def attach_legacy_transport(cls, transport: Any, *, timeout: float = 1.0) -> "Session":
        """Attach an injected V1 transport without probing it (test/compat use)."""
        session = cls(
            transport=transport,
            legacy=True,
            timeout=timeout,
            port=str(getattr(transport, "port", "<injected>")),
        )
        session._open_candidate(str(getattr(transport, "port", "<injected>")))
        session.mode = "legacy"
        return session

    def _open_candidate(self, candidate: str) -> None:
        if self._injected_transport is not None:
            transport = self._injected_transport
        else:
            factory = self.serial_factory
            if factory is None:
                try:
                    import serial  # type: ignore
                except ImportError as exc:
                    raise Disconnected("pyserial is required: python -m pip install pyserial") from exc
                factory = serial.Serial
            transport = factory(
                candidate,
                baudrate=self.baudrate,
                timeout=min(0.02, self.timeout / 4.0),
                write_timeout=0.5,
            )
        reset = getattr(transport, "reset_input_buffer", None)
        if reset:
            reset()
        self.port = str(getattr(transport, "port", candidate))
        self._transport = transport
        self._stop.clear()
        self._worker_error = None
        self._route_mode = "legacy"
        self._next_legacy_id = 1
        self._legacy_decoder = LegacyLineDecoder()
        with self._state_lock:
            self._pending_lines.clear()
            self._pending_frames.clear()
            self._orphan_lines.clear()
        self.mode = "connecting"
        self._thread = threading.Thread(target=self._worker, name=f"m6link:{self.port}", daemon=True)
        self._thread.start()

    def _worker(self) -> None:
        assert self._transport is not None
        try:
            while not self._stop.is_set():
                wrote = False
                try:
                    _, _, request = self._writes.get_nowait()
                except queue.Empty:
                    request = None
                if request is not None:
                    wrote = True
                    request.started.set()
                    try:
                        written = self._transport.write(request.data)
                        if written is not None and int(written) != len(request.data):
                            raise OSError(f"short serial write: {written}/{len(request.data)}")
                    except BaseException as exc:
                        request.error = exc
                        raise
                    finally:
                        request.done.set()
                data = self._read_transport()
                if data:
                    self._route_bytes(data)
                elif not wrote:
                    self._stop.wait(0.001)
                if self._route_mode == "v2":
                    self._v2_decoder.tick()
        except BaseException as exc:
            self._worker_error = exc
            self._wake_pending()
        finally:
            try:
                self._transport.close()
            except Exception:
                pass

    def _read_transport(self) -> bytes:
        assert self._transport is not None
        read = getattr(self._transport, "read", None)
        if read is not None:
            waiting = getattr(self._transport, "in_waiting", 0)
            size = max(1, min(256, int(waiting or 1)))
            raw = read(size)
        else:
            raw = self._transport.readline()
        if not raw:
            return b""
        return bytes(raw) if not isinstance(raw, str) else raw.encode("ascii", errors="replace")

    def _route_bytes(self, data: bytes) -> None:
        if self._route_mode == "legacy":
            for line in self._legacy_decoder.feed(data):
                self._route_line(line)
            return
        for frame in self._v2_decoder.feed(data):
            self._route_frame(frame)

    def _route_line(self, line: str) -> None:
        fields = line.split(",")
        if len(fields) >= 2 and fields[0] in ("ok", "err") and fields[1].isdigit():
            request_id = int(fields[1])
            with self._state_lock:
                pending = self._pending_lines.get(request_id)
                if pending is None:
                    self._orphan_lines[request_id] = line
                    self.unsolicited.append(line)
                else:
                    pending.line = line
                    pending.event.set()
            return
        self.unsolicited.append(line)
        self._line_events.put(line)

    def _route_frame(self, frame: Frame) -> None:
        if frame.version != 2:
            return
        if frame.type == int(FrameType.READY) and frame.session == 0 and frame.sequence == 0:
            pending = self._pending_frames.get(0)
            if pending is not None:
                pending.put(frame)
            return
        if not self._session_id or frame.session != self._session_id:
            return
        if frame.type in (int(FrameType.RPC_RESPONSE), int(FrameType.PROTOCOL_ERROR)):
            pending = self._pending_frames.get(frame.sequence)
            if pending is not None:
                pending.put(frame)
            return
        if frame.type not in (int(FrameType.EVENT), int(FrameType.TELEMETRY), int(FrameType.BULK)):
            return
        gap = bool(frame.flags & int(FrameFlag.GAP_BEFORE))
        if self._last_async_sequence and frame.sequence != self._last_async_sequence + 1:
            gap = True
        self._last_async_sequence = frame.sequence
        try:
            if frame.type == int(FrameType.TELEMETRY):
                stream, sample_id, values = decode_telemetry(frame.payload)
                sample = TelemetrySample(
                    stream=TelemetryStream(stream),
                    sample_id=sample_id,
                    time_ms=int(values.get("time_ms", 0)),
                    values=values,
                    gap_before=gap,
                    sequence=frame.sequence,
                )
                self._telemetry.put(sample)
                self._publish_event(sample)
            elif frame.type == int(FrameType.EVENT):
                common, values = decode_event(frame.payload)
                event_class: type[LinkEvent]
                event_class = LinkEvent
                if int(common["event_code"]) == 1:
                    event_class = ResetEvent
                elif int(common["event_code"]) in (2, 3):
                    event_class = FaultEvent
                elif int(common["event_code"]) == 6:
                    event_class = GapEvent
                event = event_class(
                    code=int(common["event_code"]),
                    severity=int(common["severity"]),
                    time_ms=int(common["time_ms"]),
                    occurrence_count=int(common["occurrence_count"]),
                    values=values,
                    gap_before=gap,
                    sequence=frame.sequence,
                )
                self._publish_event(event)
            else:
                self._publish_event(frame)
        except IntegrityError as exc:
            self._publish_event(exc)

    def _publish_event(self, event: object) -> None:
        self._events.put(event)
        with self._state_lock:
            taps = tuple(self._event_taps)
        for tap in taps:
            tap.put(event)

    def _wake_pending(self) -> None:
        with self._state_lock:
            for pending in self._pending_lines.values():
                pending.event.set()
            for pending in self._pending_frames.values():
                pending.put(Frame(0, 0, 0, 0, 0, b""))

    def _check_worker(self) -> None:
        if self._worker_error is not None:
            raise Disconnected(f"serial worker failed: {self._worker_error}") from self._worker_error
        if self._thread is None or not self._thread.is_alive():
            raise Disconnected("serial Session is not connected")

    def _queue_write(self, data: bytes, *, priority: int, timeout: float | None = None) -> None:
        self._check_worker()
        request = _Write(data)
        self._writes.put((priority, next(self._write_counter), request))
        wait = self.timeout if timeout is None else timeout
        if not request.done.wait(wait):
            if request.started.is_set():
                raise OutcomeUnknown("serial write completion is unknown", frame=data)
            raise TimeoutBeforeSend("serial write did not begin before timeout")
        if request.error is not None:
            if request.started.is_set():
                raise OutcomeUnknown(f"serial write failed after starting: {request.error}", frame=data)
            raise TimeoutBeforeSend(f"serial write failed before starting: {request.error}")

    def _allocate_legacy_id(self) -> int:
        request_id = self._next_legacy_id
        self._next_legacy_id = 1 if request_id >= 65535 else request_id + 1
        return request_id

    def legacy_transaction(
        self,
        verb: str,
        *arguments: str,
        request_id: int | None = None,
        timeout: float | None = None,
    ) -> list[str]:
        if self._route_mode != "legacy":
            raise IncompatibleFirmware("legacy transactions are disabled in an established V2 session")
        with self._legacy_lock:
            actual_id = self._allocate_legacy_id() if request_id is None else int(request_id)
            pending = _PendingLine()
            with self._state_lock:
                self._pending_lines[actual_id] = pending
                orphan = self._orphan_lines.pop(actual_id, None)
                if orphan is not None:
                    try:
                        self.unsolicited.remove(orphan)
                    except ValueError:
                        pass
                    pending.line = orphan
                    pending.event.set()
            encoded = encode_transaction(actual_id, verb, *arguments)
            started = time.monotonic()
            try:
                self._queue_write(encoded, priority=10, timeout=timeout)
                remaining = max(0.0, (self.timeout if timeout is None else timeout) - (time.monotonic() - started))
                while not pending.event.wait(min(0.05, remaining) if remaining else 0):
                    self._check_worker()
                    remaining = max(0.0, remaining - 0.05)
                    if remaining <= 0:
                        raise OutcomeUnknown(
                            f"timeout waiting for legacy transaction {actual_id}", frame=encoded
                        )
                self._check_worker()
                if pending.line is None:
                    raise Disconnected("serial worker stopped before legacy response")
                fields = pending.line.split(",")
                if fields[0] == "err":
                    raise DeviceError(actual_id, fields[2] if len(fields) > 2 else "syntax", fields[3:])
                return fields[2:]
            finally:
                with self._state_lock:
                    self._pending_lines.pop(actual_id, None)

    def legacy_get(self, key: str) -> tuple[Any, ...]:
        response = self.legacy_transaction("get", key)
        if len(response) < 2 or response[0] != key:
            raise IntegrityError(f"malformed legacy get response for {key}: {response}")
        return tuple(_numeric_value(item) for item in response[1:])

    def _probe_v1_identity(self) -> Identity:
        firmware = str(self.legacy_get("meta.firmware")[0])
        schema = int(self.legacy_get("meta.schema")[0])
        revision = int(self.legacy_get("meta.revision")[0])
        source_text = str(self.legacy_get("meta.source")[0]).upper()
        try:
            generation = int(self.legacy_get("meta.generation")[0])
        except DeviceError as exc:
            if exc.code not in ("key", "unknown"):
                raise
            generation = 0
        if not firmware.startswith(self.firmware_prefix):
            raise IncompatibleFirmware(f"firmware {firmware!r} does not start with {self.firmware_prefix!r}")
        if self.expected_firmware_id and firmware != self.expected_firmware_id:
            raise IncompatibleFirmware(f"firmware does not match {self.expected_firmware_id!r}")
        if schema != self.expected_parameter_schema:
            raise IncompatibleFirmware(f"parameter schema {schema} does not match {self.expected_parameter_schema}")
        source = ParameterSource.__members__.get(source_text, 0)
        return Identity(
            firmware_id=firmware,
            wire_version=1,
            registry_revision=0,
            parameter_schema=schema,
            active_revision=revision,
            stage_revision=revision,
            generation=generation,
            boot_id=0,
            source=source,
            capabilities=Capability(0),
            enabled_capabilities=Capability(0),
            port=self.port,
        )

    def _probe_v1_capabilities(self) -> int:
        try:
            value = str(self.legacy_get("meta.capabilities")[0])
        except DeviceError as exc:
            raise IncompatibleFirmware("firmware has no V2 capability probe") from exc
        if not re.fullmatch(r"[0-9a-f]{16}", value):
            raise IncompatibleFirmware(f"invalid V2 capability mask {value!r}")
        return int(value, 16)

    def _establish_v2(self, advertised_capabilities: int) -> None:
        challenge = 0
        while challenge == 0:
            challenge = secrets.randbits(64)
        ready = self.legacy_transaction(
            "mode", "v2", f"{challenge:016x}", request_id=42, timeout=1.0
        )
        expected = ["v2-ready", f"{challenge:016x}", "2", "1", "96", "32", "1"]
        if ready != expected:
            raise IncompatibleFirmware(f"malformed V2 mode ready: {ready}")
        self._route_mode = "v2"
        self.mode = "v2-armed"
        required = int(CAPABILITY_MASKS["HANDSHAKE_REQUIRED"])
        optional = int(CAPABILITY_MASKS["HELLO_OPTIONAL"]) & ~required
        payload = encode_hello(challenge, required, optional)
        frame = encode_frame(Frame(2, int(FrameType.HELLO), 0, 0, 0, payload))
        pending: queue.Queue[Frame] = queue.Queue(maxsize=1)
        self._pending_frames[0] = pending
        try:
            self._queue_write(frame, priority=10, timeout=0.5)
            try:
                response = pending.get(timeout=0.5)
            except queue.Empty as exc:
                raise OutcomeUnknown("HELLO result is unknown; reset the device before reconnecting", frame=frame) from exc
        finally:
            self._pending_frames.pop(0, None)
        if response.type != int(FrameType.READY):
            raise IntegrityError("HELLO did not receive READY")
        values = decode_ready(response.payload)
        if int(values["ready_magic"]) != int(REGISTRY["semantics"]["handshake"]["ready_magic_u32"]):
            raise IntegrityError("READY magic mismatch")
        if int(values["challenge_echo"]) != challenge:
            raise IntegrityError("READY challenge mismatch")
        result = int(values["result"])
        if result != int(RpcResult.OK):
            raise DeviceRejected(result, int(values["detail"]), name=RPC_RESULT_NAMES.get(result))
        session_id = int(values["assigned_session"])
        enabled = int(values["enabled_capabilities"])
        if (
            int(values["selected_registry"]) != 1
            or not session_id
            or enabled & required != required
            or int(values["selected_max_payload"]) != 96
            or int(values["selected_max_rpc_request"]) != 32
            or int(values["device_rx_window"]) != 1
        ):
            raise IncompatibleFirmware("READY success fields violate registry revision 1")
        if enabled & ~advertised_capabilities:
            raise IncompatibleFirmware("READY enabled unadvertised capabilities")
        self._session_id = session_id
        self._next_sequence = 1
        self._last_async_sequence = 0
        self._negotiated_capabilities = Capability(enabled)
        self.mode = "v2"

    def _rpc(self, opcode: RpcOpcode | int, body: bytes = b"", *, timeout: float | None = None) -> bytes:
        if self.mode != "v2" or not self._session_id:
            raise IncompatibleFirmware("operation requires an established V2 session")
        if self._outcome_unknown is not None:
            raise OutcomeUnknown(
                "the previous RPC outcome is unknown; reset or explicitly recover the session",
                sequence=self._outcome_unknown.sequence,
                frame=self._outcome_unknown.frame,
            )
        with self._rpc_lock:
            sequence = self._next_sequence
            if sequence > 0xFFFFFFFE:
                raise IncompatibleFirmware("host sequence is exhausted; reconnect before another RPC")
            opcode_value = int(opcode)
            wire = encode_frame(
                Frame(
                    2,
                    int(FrameType.RPC_REQUEST),
                    0,
                    self._session_id,
                    sequence,
                    encode_rpc(opcode_value, body),
                )
            )
            pending: queue.Queue[Frame] = queue.Queue(maxsize=1)
            self._pending_frames[sequence] = pending
            rpc_timeout = self.timeout if timeout is None else timeout
            started = time.monotonic()
            try:
                self._queue_write(wire, priority=10, timeout=rpc_timeout)
                remaining = max(0.0, rpc_timeout - (time.monotonic() - started))
                try:
                    response = pending.get(timeout=remaining)
                except queue.Empty as exc:
                    unknown = OutcomeUnknown(
                        f"RPC 0x{opcode_value:04x} sequence {sequence} timed out after send",
                        sequence=sequence,
                        frame=wire,
                    )
                    self._outcome_unknown = unknown
                    raise unknown from exc
            finally:
                self._pending_frames.pop(sequence, None)
            self._check_worker()
            if response.type == 0:
                raise Disconnected("serial worker stopped before RPC response")
            if response.type == int(FrameType.PROTOCOL_ERROR):
                if len(response.payload) != 8:
                    raise IntegrityError("malformed protocol-error response")
                result, _, _, detail = struct.unpack("<HBBI", response.payload)
                raise DeviceRejected(result, detail, opcode=opcode_value, name=RPC_RESULT_NAMES.get(result))
            if response.type != int(FrameType.RPC_RESPONSE):
                raise IntegrityError("unexpected frame while waiting for RPC response")
            response_opcode, result, detail, success = decode_rpc_response(response.payload)
            if response_opcode != opcode_value:
                raise IntegrityError("RPC response opcode mismatch")
            if result != int(RpcResult.BUSY):
                self._next_sequence += 1
            if result != int(RpcResult.OK):
                error_class = SafetyTrip if result == int(RpcResult.SAFETY_FAULT) else DeviceRejected
                raise error_class(result, detail, opcode=opcode_value, name=RPC_RESULT_NAMES.get(result))
            return success

    def require_capability(self, capability: Capability) -> None:
        if self.capabilities & capability != capability:
            raise UnsupportedCapability(capability)

    def get_identity(self) -> Identity:
        if self.mode == "legacy":
            assert self.identity is not None
            return self.identity
        body = self._rpc(RpcOpcode.IDENTITY_GET)
        if not 32 <= len(body) <= 63:
            raise IntegrityError("IDENTITY_GET success length is invalid")
        values = decode_layout("rpc_identity_get_success_body_v1", body, exact=False)
        length = int(values["firmware_id_length"])
        raw = bytes(values.get("firmware_id_ascii_max", b""))
        if length > 31 or len(raw) < length or any(raw[length:]):
            raise IntegrityError("IDENTITY_GET firmware string is malformed")
        try:
            firmware = raw[:length].decode("ascii")
        except UnicodeDecodeError as exc:
            raise IntegrityError("IDENTITY_GET firmware string is not ASCII") from exc
        identity = Identity(
            firmware_id=firmware,
            wire_version=int(values["wire_version"]),
            registry_revision=int(values["registry_revision"]),
            parameter_schema=int(values["parameter_schema"]),
            active_revision=int(values["active_revision"]),
            stage_revision=int(values["stage_revision"]),
            generation=int(values["generation"]),
            boot_id=int(values["boot_id"]),
            source=ParameterSource(int(values["source"])),
            capabilities=Capability(int(values["supported_capabilities"])),
            enabled_capabilities=getattr(self, "_negotiated_capabilities", Capability(0)),
            image_flags=int(values["image_flags"]),
            port=self.port,
        )
        self.identity = identity
        return identity

    def get_status(self) -> Status:
        values = decode_layout("rpc_status_get_success_body_v1", self._rpc(RpcOpcode.STATUS_GET))
        status = Status(
            time_ms=int(values["time_ms"]),
            system_state=int(values["system_state"]),
            action_state=_enum_or_int(ActionState, values["action_state"]),
            action_kind=_enum_or_int(ActionKind, values["action_kind"]),
            fault_bits=int(values["fault_bits"]),
            first_fault_ms=int(values["first_fault_ms"]),
            last_fault_ms=int(values["last_fault_ms"]),
            fault_count=int(values["fault_count"]),
            required_sensor_mask=int(values["required_sensor_mask"]),
            action_deadline_ms=int(values["action_deadline_ms"]),
            requested_pwm=(int(values["requested_pwm_l"]), int(values["requested_pwm_r"])),
            applied_pwm=(int(values["applied_pwm_l"]), int(values["applied_pwm_r"])),
            target_mm_s=(int(values["target_l_mm_s"]), int(values["target_r_mm_s"])),
            measured_mm_s=(int(values["measured_l_mm_s"]), int(values["measured_r_mm_s"])),
            active_revision=int(values["active_revision"]),
            stage_revision=int(values["stage_revision"]),
            source=ParameterSource(int(values["source"])),
            flags=int(values["flags"]),
        )
        self.status = status
        return status

    def get_reset_info(self) -> ResetInfo:
        values = decode_layout("reset_snapshot_v1", self._rpc(RpcOpcode.RESET_GET))
        previous = Breadcrumb(
            checkpoint=int(values["previous_checkpoint"]),
            boot_id=int(values["previous_boot_id"]),
            time_ms=int(values["previous_time_ms"]),
            last_wwdt_ms=int(values["previous_last_wwdt_ms"]),
            miss_total=int(values["previous_miss_total"]),
            fault_bits=int(values["previous_fault_bits"]),
            uart_tx_drop=int(values["previous_uart_tx_drop"]),
            uart_rx_drop=int(values["previous_uart_rx_drop"]),
            telemetry_drop=int(values["previous_telemetry_drop"]),
            pending_mask=int(values["previous_pending_mask"]),
            missed_mask=int(values["previous_missed_mask"]),
            detail_checkpoint=int(values["previous_detail_checkpoint"]),
            system_state=int(values["previous_system_state"]),
            telemetry_queue=int(values["previous_telemetry_queue"]),
            health_mask=int(values["previous_health_mask"]),
        )
        return ResetInfo(
            raw_cause=int(values["raw_cause"]),
            reset_class=int(values["reset_class"]),
            current_checkpoint=int(values["current_checkpoint"]),
            boot_id=int(values["boot_id"]),
            flags=int(values["flags"]),
            previous=previous,
            hardfault_pc=int(values["hardfault_pc"]),
            hardfault_lr=int(values["hardfault_lr"]),
            hardfault_xpsr=int(values["hardfault_xpsr"]),
        )

    def get_task_stats(self) -> TaskStats:
        values = decode_layout("rpc_task_stats_get_success_body_v1", self._rpc(RpcOpcode.TASK_STATS_GET))
        return TaskStats(
            int(values["task_count"]),
            bool(values["scheduler_healthy"]),
            int(values["pending_mask"]),
            int(values["missed_mask"]),
            int(values["health_mask"]),
            tuple(map(int, values["release_count"])),
            tuple(map(int, values["completion_count"])),
            tuple(map(int, values["miss_count"])),
        )

    def get_uart_stats(self) -> UartStats:
        values = decode_layout("rpc_uart_stats_get_success_body_v1", self._rpc(RpcOpcode.UART_STATS_GET))
        return UartStats(**{name: int(values[name]) for name in UartStats.__dataclass_fields__})

    def get_link_stats(self) -> LinkStats:
        values = decode_layout("rpc_link_stats_get_success_body_v1", self._rpc(RpcOpcode.LINK_STATS_GET))
        return LinkStats(
            mode=int(values["mode"]), control_window=int(values["control_window"]),
            replay_used=int(values["replay_used"]), flags=int(values["flags"]),
            session=int(values["session"]), next_host_sequence=int(values["next_host_sequence"]),
            next_async_sequence=int(values["next_async_sequence"]), rx_frames=int(values["rx_frames"]),
            tx_frames=int(values["tx_frames"]), crc_errors=int(values["crc_errors"]),
            frame_errors=int(values["frame_errors"]), timeout_errors=int(values["timeout_errors"]),
            session_mismatch=int(values["session_mismatch"]), sequence_errors=int(values["sequence_errors"]),
            replay_count=int(values["replay_count"]), busy_count=int(values["busy_count"]),
            drops=tuple(int(values[f"p{i}_drop"]) for i in range(4)),
            control_error_drop=int(values["control_error_drop"]),
            high_water=tuple(int(values[f"p{i}_high_water"]) for i in range(4)),
        )

    def get_parameter_descriptor(self, parameter: str | int) -> ParameterDescriptor:
        parameter_id = _parameter_id(parameter)
        body = self._rpc(RpcOpcode.PARAM_DESCRIPTOR_GET, struct.pack("<B3x", parameter_id))
        if not 20 <= len(body) <= 51:
            raise IntegrityError("parameter descriptor length is invalid")
        values = decode_layout("rpc_param_descriptor_get_success_body_v1", body, exact=False)
        key_length = int(values["key_length"])
        raw_key = bytes(values.get("key_ascii_max", b""))
        if key_length > 31 or len(raw_key) < key_length or any(raw_key[key_length:]):
            raise IntegrityError("parameter descriptor key is malformed")
        key = raw_key[:key_length].decode("ascii")
        parameter_type = ParameterType(int(values["type"]))
        return ParameterDescriptor(
            parameter_id=parameter_id,
            key=key,
            type=parameter_type,
            flags=int(values["descriptor_flags"]),
            minimum=bits_to_value(parameter_type, int(values["minimum_bits"])),
            maximum=bits_to_value(parameter_type, int(values["maximum_bits"])),
            default=bits_to_value(parameter_type, int(values["default_bits"])),
            domain_rule=int(values["domain_rule"]),
            relation_group=int(values["relation_group"]),
            unit=int(values["unit"]),
        )

    def get_parameter(self, parameter: str | int) -> ParameterValue:
        parameter_id = _parameter_id(parameter)
        key = str(PARAMETER_BY_ID[parameter_id]["key"])
        values = decode_layout(
            "rpc_param_value_get_success_body_v1",
            self._rpc(RpcOpcode.PARAM_VALUE_GET, struct.pack("<B3x", parameter_id)),
        )
        parameter_type = ParameterType(int(values["type"]))
        if int(values["parameter_id"]) != parameter_id:
            raise IntegrityError("PARAM_VALUE_GET parameter id mismatch")
        return ParameterValue(
            parameter_id=parameter_id,
            key=key,
            type=parameter_type,
            active=bits_to_value(parameter_type, int(values["active_value_bits"])),
            staged=bits_to_value(parameter_type, int(values["staged_value_bits"])),
            active_source=ParameterSource(int(values["active_source"])),
            staged_source=ParameterSource(int(values["staged_source"])),
            active_revision=int(values["active_revision"]),
            stage_revision=int(values["stage_revision"]),
            generation=int(values["generation"]),
        )

    def get_parameters(self) -> ParameterSnapshot:
        summary = decode_layout("rpc_param_summary_get_success_body_v1", self._rpc(RpcOpcode.PARAM_SUMMARY_GET))
        values = {str(item["key"]): self.get_parameter(int(item["id"])) for item in REGISTRY["parameters"]}
        return ParameterSnapshot(
            parameter_schema=int(summary["parameter_schema"]),
            active_revision=int(summary["active_revision"]),
            stage_revision=int(summary["stage_revision"]),
            generation=int(summary["generation"]),
            active_source=ParameterSource(int(summary["active_source"])),
            values=values,
            flags=int(summary["flags"]), slot=int(summary["slot"]),
            storage_health=int(summary["storage_health"]), save_failures=int(summary["save_failures"]),
        )

    def stage_parameter(self, parameter: str | int, value: Any) -> ParameterValue:
        identity = self._require_identity()
        parameter_id = _parameter_id(parameter)
        parameter_type = parameter_type_for_id(parameter_id)
        bits = value_to_bits(parameter_type, value)
        body = struct.pack("<IBBHI", identity.stage_revision, parameter_id, int(parameter_type), 0, bits)
        response = decode_layout("rpc_param_stage_set_success_body_v1", self._rpc(RpcOpcode.PARAM_STAGE_SET, body))
        self.identity = replace(identity, stage_revision=int(response["stage_revision"]))
        return self.get_parameter(parameter_id)

    def apply_parameters(self) -> Mapping[str, Any]:
        identity = self._require_identity()
        body = struct.pack("<III", identity.active_revision, identity.stage_revision, identity.generation)
        values = decode_layout("rpc_param_apply_success_body_v1", self._rpc(RpcOpcode.PARAM_APPLY, body))
        self.identity = replace(
            identity,
            active_revision=int(values["active_revision"]),
            stage_revision=int(values["stage_revision"]),
            source=ParameterSource(int(values["source"])),
        )
        return values

    def revert_parameters(self) -> Mapping[str, Any]:
        identity = self._require_identity()
        values = decode_layout(
            "rpc_param_revert_success_body_v1",
            self._rpc(RpcOpcode.PARAM_REVERT, struct.pack("<II", identity.active_revision, identity.stage_revision)),
        )
        self.identity = replace(identity, stage_revision=int(values["stage_revision"]))
        return values

    def load_parameters(self) -> Mapping[str, Any]:
        identity = self._require_identity()
        values = decode_layout(
            "rpc_param_load_success_body_v1",
            self._rpc(RpcOpcode.PARAM_LOAD, struct.pack("<II", identity.stage_revision, identity.generation)),
        )
        self.identity = replace(identity, stage_revision=int(values["stage_revision"]), generation=int(values["generation"]))
        return values

    def default_parameters(self) -> Mapping[str, Any]:
        identity = self._require_identity()
        values = decode_layout(
            "rpc_param_defaults_success_body_v1",
            self._rpc(RpcOpcode.PARAM_DEFAULTS, struct.pack("<I", identity.stage_revision)),
        )
        self.identity = replace(identity, stage_revision=int(values["stage_revision"]))
        return values

    def save_parameters(self) -> Mapping[str, Any]:
        identity = self._require_identity()
        values = decode_layout(
            "rpc_param_save_success_body_v1",
            self._rpc(RpcOpcode.PARAM_SAVE, struct.pack("<II", identity.active_revision, identity.generation), timeout=max(2.0, self.timeout)),
        )
        self.identity = replace(identity, generation=int(values["generation"]))
        return values

    def clear_fault(self, observed_fault_bits: int | None = None) -> Mapping[str, Any]:
        identity = self._require_identity()
        if observed_fault_bits is None:
            observed_fault_bits = self.get_status().fault_bits
        values = decode_layout(
            "rpc_fault_clear_success_body_v1",
            self._rpc(
                RpcOpcode.FAULT_CLEAR,
                struct.pack("<II", identity.boot_id, int(observed_fault_bits)),
            ),
        )
        return values

    def subscribe(self, entries: Mapping[TelemetryStream | int, int] | Iterable[Subscription | tuple[int, int]]) -> tuple[tuple[Subscription, ...], int]:
        if isinstance(entries, Mapping):
            pairs = [(int(stream), int(period)) for stream, period in entries.items()]
        else:
            pairs = []
            for item in entries:
                if isinstance(item, Subscription):
                    if item.flags:
                        raise ValueError("subscription flags must be zero in registry revision 1")
                    pairs.append((int(item.stream), int(item.period_ms)))
                else:
                    stream, period = item
                    pairs.append((int(stream), int(period)))
        values = decode_layout("rpc_subscription_get_success_body_v1", self._rpc(RpcOpcode.SUBSCRIPTION_SET, pack_subscription(pairs)), exact=False)
        result = tuple(
            Subscription(TelemetryStream(int(item["stream_id"])), int(item["period_ms"]), int(item["flags"]))
            for item in values["entries_max"]
        )
        self._subscriptions = {TelemetryStream(int(item.stream)): int(item.period_ms) for item in result}
        return result, int(values["estimated_bps"])

    def arm_action(self, spec: ActionSpec) -> ArmedAction:
        normalized = self._validate_action(spec)
        body = struct.pack(
            "<BBHhhiiII",
            int(normalized.kind), normalized.wheel_mask, 0,
            normalized.target_l_mm_s, normalized.target_r_mm_s,
            normalized.angle_mdeg, normalized.yaw_rate_mdeg_s,
            normalized.duration_ms, int(normalized.expected_active_revision),
        )
        values = decode_layout("rpc_action_arm_success_body_v1", self._rpc(RpcOpcode.ACTION_ARM, body))
        token, expires_ms = int(values["token"]), int(values["expires_ms"])
        if token == 0 or expires_ms != 10000:
            raise IntegrityError("ACTION_ARM returned an invalid token or expiry")
        returned = values["normalized_request"]
        returned_spec = ActionSpec(
            kind=_enum_or_int(ActionKind, returned["kind"]), wheel_mask=int(returned["wheel_mask"]),
            target_l_mm_s=int(returned["target_l_mm_s"]), target_r_mm_s=int(returned["target_r_mm_s"]),
            angle_mdeg=int(returned["angle_mdeg"]), yaw_rate_mdeg_s=int(returned["yaw_rate_mdeg_s"]),
            duration_ms=int(returned["duration_ms"]), expected_active_revision=int(returned["expected_active_revision"]),
        )
        return ArmedAction(token, expires_ms, returned_spec, time.monotonic())

    def confirm_action(self, action: ArmedAction | int) -> ActionStatus:
        if isinstance(action, ArmedAction):
            if time.monotonic() >= action.expires_at:
                raise ValueError("armed action has expired locally; arm again")
            token = action.token
        else:
            token = int(action)
        identity = self._require_identity()
        values = decode_layout(
            "rpc_action_confirm_success_body_v1",
            self._rpc(RpcOpcode.ACTION_CONFIRM, struct.pack("<II", identity.boot_id, token)),
        )
        return ActionStatus(
            action_id=int(values["action_id"]), state=_enum_or_int(ActionState, values["state"]),
            kind=_enum_or_int(ActionKind, values["kind"]), start_ms=int(values["start_ms"]),
            deadline_ms=int(values["deadline_ms"]),
        )

    def cancel_action(self, action_id: int = 0) -> ActionStatus:
        values = decode_layout(
            "rpc_action_cancel_success_body_v1",
            self._rpc(RpcOpcode.ACTION_CANCEL, struct.pack("<I", int(action_id))),
        )
        return ActionStatus(
            action_id=int(values["action_id"]), state=_enum_or_int(ActionState, values["state"]),
            reason=_enum_or_int(ActionReason, values["reason"]), stopped_ms=int(values["stopped_ms"]),
        )

    def _validate_action(self, spec: ActionSpec) -> ActionSpec:
        identity = self._require_identity()
        try:
            kind = ActionKind(int(spec.kind))
        except ValueError as exc:
            raise ValueError("unknown action kind") from exc
        if kind == ActionKind.NONE:
            raise ValueError("NONE cannot be armed")
        rule = next(item for item in REGISTRY["semantics"]["actions"]["kinds"] if item["name"] == kind.name)
        for capability_name in rule["required_capabilities"]:
            self.require_capability(Capability[capability_name])
        if spec.wheel_mask not in rule["wheel_mask_allowed"]:
            raise ValueError("wheel mask is invalid for this action")
        if not int(rule["duration_min_ms"]) <= spec.duration_ms <= int(rule["duration_max_ms"]):
            raise ValueError("action duration is out of range")
        for field in rule["zero_fields"]:
            if int(getattr(spec, field)) != 0:
                raise ValueError(f"{field} must be zero for {kind.name}")
        targets = (int(spec.target_l_mm_s), int(spec.target_r_mm_s))
        relation = rule["target_relation"]
        if relation == "selected-abs-range":
            for index, target in enumerate(targets):
                selected = bool(spec.wheel_mask & (1 << index))
                if not selected and target != 0:
                    raise ValueError("unselected wheel target must be zero")
                if selected and not int(rule["target_abs_min_mm_s"]) <= abs(target) <= int(rule["target_abs_max_mm_s"]):
                    raise ValueError("selected wheel target is out of range")
        elif relation == "left-equals-right-nonzero":
            if targets[0] != targets[1] or not int(rule["target_abs_min_mm_s"]) <= abs(targets[0]) <= int(rule["target_abs_max_mm_s"]):
                raise ValueError("heading-hold wheel targets must match and be nonzero")
        elif relation == "all-zero" and any(targets):
            raise ValueError("wheel targets must be zero for this action")
        if "angle_abs_min_mdeg" in rule:
            if not int(rule["angle_abs_min_mdeg"]) <= abs(spec.angle_mdeg) <= int(rule["angle_abs_max_mdeg"]):
                raise ValueError("turn angle is out of range")
            extension = rule.get("angle_extension_capability")
            if extension and not self.capabilities & Capability[extension] and spec.angle_mdeg not in rule["without_configurable_turn_angles_mdeg"]:
                raise UnsupportedCapability(Capability[extension])
        if "rate_abs_min_mdeg_s" in rule and not int(rule["rate_abs_min_mdeg_s"]) <= abs(spec.yaw_rate_mdeg_s) <= int(rule["rate_abs_max_mdeg_s"]):
            raise ValueError("heading rate is out of range")
        expected = identity.active_revision if spec.expected_active_revision is None else int(spec.expected_active_revision)
        return replace(spec, kind=kind, expected_active_revision=expected)

    def emergency_stop(self) -> None:
        """Queue a raw ``S`` ahead of every not-yet-started normal write."""
        self._queue_write(b"S", priority=0, timeout=min(0.5, self.timeout))

    def read_event(self, timeout: float | None = None) -> object | None:
        try:
            item = self._events.get(timeout=timeout)
        except queue.Empty:
            return None
        if isinstance(item, IntegrityError):
            raise item
        return item

    @contextmanager
    def event_tap(self):
        """Create an independent event stream without consuming GUI events."""
        tap: queue.Queue[object] = queue.Queue()
        with self._state_lock:
            self._event_taps.add(tap)
        try:
            yield tap
        finally:
            with self._state_lock:
                self._event_taps.discard(tap)

    def start_recording(
        self,
        output_base: Any,
        *,
        duration_s: float | None = None,
        period_ms: int = 20,
        test_mode: str = "wheel",
        requested_speed: float = 0.0,
        parameters: Mapping[str, Any] | None = None,
    ) -> object:
        """Start a non-competing recording tap; use :meth:`stop_recording` to finish."""
        if self._recording_thread is not None and self._recording_thread.is_alive():
            raise RuntimeError("a Session recording is already active")
        from .recording import Recorder

        stop = threading.Event()
        self._recording_stop = stop
        self._recording_result = None
        self._recording_error = None

        def run() -> None:
            try:
                self._recording_result = Recorder(self).record(
                    output_base,
                    duration_s=duration_s,
                    period_ms=period_ms,
                    test_mode=test_mode,
                    requested_speed=requested_speed,
                    identity=self.identity,
                    parameters=parameters,
                    stop_event=stop,
                )
            except BaseException as exc:
                self._recording_error = exc

        thread = threading.Thread(target=run, name=f"m6link-recorder:{self.port}", daemon=True)
        self._recording_thread = thread
        thread.start()
        return thread

    def stop_recording(self, *, timeout: float = 5.0) -> object | None:
        thread = self._recording_thread
        if thread is None:
            return None
        assert self._recording_stop is not None
        self._recording_stop.set()
        thread.join(timeout=timeout)
        if thread.is_alive():
            raise TimeoutError("recording did not stop before timeout")
        self._recording_thread = None
        if self._recording_error is not None:
            error = self._recording_error
            self._recording_error = None
            raise error
        result = self._recording_result
        self._recording_result = None
        return result

    def read_telemetry(self, timeout: float | None = None) -> TelemetrySample | None:
        try:
            return self._telemetry.get(timeout=timeout)
        except queue.Empty:
            return None

    def events(self, *, timeout: float = 0.1):
        while self.connected:
            event = self.read_event(timeout)
            if event is not None:
                yield event

    def read_legacy_line(self, timeout: float | None = None) -> str:
        try:
            return self._line_events.get(timeout=timeout) if timeout is not None else self._line_events.get_nowait()
        except queue.Empty:
            return ""

    def legacy_command(self, command: str) -> None:
        if self._route_mode != "legacy":
            raise IncompatibleFirmware("legacy command bytes are disabled in V2 mode")
        if len(command) != 1 or command in ("S", " "):
            raise ValueError("legacy command must be one non-stop character")
        self._queue_write(command.encode("ascii"), priority=10)

    def close(self, *, clean: bool = True) -> None:
        if self._recording_thread is not None:
            try:
                self.stop_recording()
            except Exception:
                pass
        if clean and self.mode == "v2" and self._outcome_unknown is None:
            try:
                self._rpc(RpcOpcode.SESSION_CLOSE, timeout=min(0.5, self.timeout))
            except Exception:
                pass
        self._shutdown_transport()

    def _shutdown_transport(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None
        self._transport = None
        self.mode = "disconnected"
        self._route_mode = "legacy"
        self._session_id = 0

    def _require_identity(self) -> Identity:
        if self.identity is None:
            raise Disconnected("device identity is unavailable")
        return self.identity

    # Public short aliases used by existing CLI/controller code.
    get_reset = get_reset_info
    get_tasks = get_task_stats
    get_uart = get_uart_stats
    get_link = get_link_stats
    stage = stage_parameter
    get = get_parameter
    apply = apply_parameters
    revert = revert_parameters
    load = load_parameters
    defaults = default_parameters
    save = save_parameters


def _parameter_id(parameter: str | int) -> int:
    if isinstance(parameter, str):
        try:
            return int(PARAMETER_BY_KEY[parameter]["id"])
        except KeyError as exc:
            raise ValueError(f"unknown parameter key: {parameter}") from exc
    parameter_id = int(parameter)
    if parameter_id not in PARAMETER_BY_ID:
        raise ValueError(f"unknown parameter id: {parameter_id}")
    return parameter_id


def _numeric_value(text: str) -> float | int | str:
    try:
        value = float(text)
    except ValueError:
        return text
    if value != value or value in (float("inf"), float("-inf")):
        return text
    if value.is_integer() and "." not in text:
        return int(value)
    return value


def _enum_or_int(enum_type: type[Any], value: Any) -> Any:
    try:
        return enum_type(int(value))
    except ValueError:
        return int(value)
