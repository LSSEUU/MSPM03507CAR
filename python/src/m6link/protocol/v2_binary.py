"""Payload packing and decoding for registry revision 1."""

from __future__ import annotations

import math
import struct
from typing import Any, Mapping

from ..errors import IntegrityError
from ..registry import LAYOUTS, PARAMETER_BY_ID, ParameterType, REGISTRY, TELEMETRY


_SCALAR_FORMATS = {
    "u8": "<B",
    "i8": "<b",
    "u16": "<H",
    "i16": "<h",
    "u32": "<I",
    "i32": "<i",
    "u64": "<Q",
    "i64": "<q",
    "f32": "<f",
}


def _require_bytes(data: bytes, offset: int, size: int, name: str) -> bytes:
    if offset < 0 or size < 0 or offset + size > len(data):
        raise IntegrityError(f"payload is too short for {name}")
    return data[offset : offset + size]


def decode_layout(name: str, data: bytes, *, exact: bool = True) -> dict[str, Any]:
    """Decode a named registry layout and reject non-zero reserved fields."""

    try:
        layout = LAYOUTS[name]
    except KeyError as exc:
        raise IntegrityError(f"unknown wire layout: {name}") from exc
    declared_size = int(layout["size"])
    if exact and len(data) != declared_size:
        raise IntegrityError(f"{name} has {len(data)} bytes; expected {declared_size}")
    if len(data) > declared_size:
        raise IntegrityError(f"{name} exceeds {declared_size} bytes")
    result: dict[str, Any] = {}
    for field in layout["fields"]:
        field_name = str(field["name"])
        offset = int(field["offset"])
        size = int(field["size"])
        kind = str(field["kind"])
        if offset >= len(data):
            if kind == "layout_array":
                count = int(result[str(field["count_field"])])
                if count == 0:
                    result[field_name] = ()
                    continue
                raise IntegrityError(f"{name}.{field_name} is truncated")
            continue
        raw = _require_bytes(data, offset, min(size, len(data) - offset), f"{name}.{field_name}")
        if len(raw) != size and kind not in ("bytes", "layout_array", "layout_union"):
            raise IntegrityError(f"payload is truncated at {name}.{field_name}")
        if field.get("reserved") and any(raw):
            raise IntegrityError(f"non-zero reserved field {name}.{field_name}")
        if kind in _SCALAR_FORMATS:
            count = int(field.get("count", 1))
            scalar_size = struct.calcsize(_SCALAR_FORMATS[kind])
            if count == 1:
                result[field_name] = struct.unpack_from(_SCALAR_FORMATS[kind], raw)[0]
            else:
                if scalar_size * count != size:
                    raise IntegrityError(f"bad registry scalar array {name}.{field_name}")
                result[field_name] = tuple(
                    struct.unpack_from(_SCALAR_FORMATS[kind], raw, index * scalar_size)[0]
                    for index in range(count)
                )
        elif kind == "bytes":
            result[field_name] = raw
        elif kind == "layout_ref":
            result[field_name] = decode_layout(str(field["layout_ref"]), raw)
        elif kind == "layout_array":
            count = int(result[str(field["count_field"])])
            maximum = int(field["max_count"])
            if count > maximum:
                raise IntegrityError(f"{name}.{field_name} count exceeds {maximum}")
            item_layout = LAYOUTS[str(field["layout_ref"])]
            item_size = int(item_layout["size"])
            required = count * item_size
            if len(raw) < required:
                raise IntegrityError(f"{name}.{field_name} is truncated")
            if any(raw[required:]):
                raise IntegrityError(f"{name}.{field_name} unused bytes are non-zero")
            result[field_name] = tuple(
                decode_layout(str(field["layout_ref"]), raw[index * item_size : (index + 1) * item_size])
                for index in range(count)
            )
        elif kind == "layout_union":
            selector = str(result[str(field["selector_field"])])
            length = int(result[str(field["length_field"])])
            selected = field["layout_by_selector"].get(selector)
            if selected is None:
                raise IntegrityError(f"{name}.{field_name} has unknown selector {selector}")
            if length > len(raw) or any(raw[length:]):
                raise IntegrityError(f"{name}.{field_name} has invalid union length")
            result[field_name] = decode_layout(str(selected), raw[:length])
        else:
            raise IntegrityError(f"unsupported layout kind {kind} in {name}.{field_name}")
    return result


def encode_hello(challenge: int, required: int, optional: int) -> bytes:
    if challenge == 0 or not 0 <= challenge <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("HELLO challenge must be a nonzero u64")
    if required & optional:
        raise ValueError("required and optional capabilities overlap")
    return struct.pack(
        "<IHHQQQHHB3x",
        int(REGISTRY["semantics"]["handshake"]["hello_magic_u32"]),
        1,
        1,
        challenge,
        required,
        optional,
        int(REGISTRY["framing"]["max_payload"]),
        int(REGISTRY["framing"]["max_rpc_request_payload"]),
        int(REGISTRY["framing"]["control_window"]),
    )


def decode_ready(data: bytes) -> dict[str, Any]:
    return decode_layout("ready_v1", data)


def encode_rpc(opcode: int, body: bytes = b"") -> bytes:
    payload = struct.pack("<H", int(opcode)) + bytes(body)
    if len(payload) > int(REGISTRY["framing"]["max_rpc_request_payload"]):
        raise ValueError("RPC request payload exceeds the frozen 32-byte maximum")
    return payload


def decode_rpc_response(payload: bytes) -> tuple[int, int, int, bytes]:
    if len(payload) < 8:
        raise IntegrityError("RPC response is shorter than its 8-byte prefix")
    opcode, result, detail = struct.unpack_from("<HHI", payload)
    return opcode, result, detail, payload[8:]


def value_to_bits(parameter_type: ParameterType | int, value: Any) -> int:
    parameter_type = ParameterType(int(parameter_type))
    if parameter_type == ParameterType.F32:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("parameter value must be finite")
        return struct.unpack("<I", struct.pack("<f", number))[0]
    if parameter_type == ParameterType.BOOL:
        if value not in (False, True, 0, 1):
            raise ValueError("BOOL parameter accepts only false/true or 0/1")
        return int(bool(value))
    if parameter_type in (ParameterType.U32, ParameterType.ENUM_U8):
        integer = int(value)
        maximum = 0xFF if parameter_type == ParameterType.ENUM_U8 else 0xFFFFFFFF
        if not 0 <= integer <= maximum:
            raise ValueError(f"parameter value must fit {parameter_type.name}")
        return integer
    raise ValueError(f"invalid parameter type: {parameter_type}")


def bits_to_value(parameter_type: ParameterType | int, bits: int) -> float | int | bool:
    parameter_type = ParameterType(int(parameter_type))
    if parameter_type == ParameterType.F32:
        value = struct.unpack("<f", struct.pack("<I", bits & 0xFFFFFFFF))[0]
        if not math.isfinite(value):
            raise IntegrityError("device returned a non-finite F32 parameter")
        return value
    if parameter_type == ParameterType.BOOL:
        if bits not in (0, 1):
            raise IntegrityError("device returned a non-boolean BOOL parameter")
        return bool(bits)
    if parameter_type == ParameterType.ENUM_U8:
        if bits > 0xFF:
            raise IntegrityError("device returned an oversized ENUM_U8 parameter")
        return bits
    if parameter_type == ParameterType.U32:
        return bits
    raise IntegrityError(f"device returned invalid parameter type {parameter_type}")


def parameter_type_for_id(parameter_id: int) -> ParameterType:
    try:
        name = str(PARAMETER_BY_ID[int(parameter_id)]["type"])
    except KeyError as exc:
        raise ValueError(f"unknown parameter id: {parameter_id}") from exc
    return ParameterType[name]


def decode_telemetry(payload: bytes) -> tuple[int, int, dict[str, Any]]:
    if len(payload) < int(TELEMETRY["common_size"]):
        raise IntegrityError("telemetry payload is shorter than its common prefix")
    stream_id, schema, reserved, sample_id = struct.unpack_from("<BBHI", payload)
    if reserved != 0 or schema != 1:
        raise IntegrityError("unsupported telemetry prefix")
    spec = next((item for item in TELEMETRY["streams"] if int(item["value"]) == stream_id), None)
    if spec is None:
        raise IntegrityError(f"unknown telemetry stream {stream_id}")
    body = payload[8:]
    if len(body) != int(spec["body_size"]):
        raise IntegrityError(f"telemetry stream {stream_id} body length mismatch")
    return stream_id, sample_id, decode_layout(str(spec["body_layout"]), body)


def decode_event(payload: bytes) -> tuple[dict[str, Any], dict[str, Any]]:
    if len(payload) < 12:
        raise IntegrityError("event payload is shorter than its common prefix")
    common = decode_layout("event_common_v1", payload[:12])
    body_length = int(common["body_length"])
    if len(payload) != 12 + body_length:
        raise IntegrityError("event body length mismatch")
    entry = next(
        (
            item
            for item in REGISTRY["registries"]["event_codes"]
            if int(item["value"]) == int(common["event_code"])
        ),
        None,
    )
    if entry is None or int(common["event_schema"]) != int(entry["schema"]):
        raise IntegrityError("unknown event code or schema")
    if body_length != int(entry["body_size"]):
        raise IntegrityError("event body does not match its registry size")
    return common, decode_layout(str(entry["body_layout"]), payload[12:])


def pack_subscription(entries: list[tuple[int, int]]) -> bytes:
    if len(entries) > 4 or len({stream for stream, _ in entries}) != len(entries):
        raise ValueError("subscription must contain zero to four unique streams")
    encoded = bytearray(struct.pack("<BBH", 1, len(entries), 0))
    for stream, period_ms in sorted(entries):
        spec = next((item for item in TELEMETRY["streams"] if int(item["value"]) == int(stream)), None)
        if spec is None:
            raise ValueError(f"unknown telemetry stream {stream}")
        if not int(spec["period_min_ms"]) <= period_ms <= int(spec["period_max_ms"]):
            raise ValueError(f"stream {stream} period is out of range")
        if period_ms % int(spec["period_quantum_ms"]):
            raise ValueError(f"stream {stream} period violates its quantum")
        encoded.extend(struct.pack("<BBH", int(stream), 0, int(period_ms)))
    return bytes(encoded)
