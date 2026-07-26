#!/usr/bin/env python3
"""Validate the frozen M6 V2 registry and its deterministic vectors."""

from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
from typing import Any, Iterable

try:
    import jsonschema
except ImportError as exc:  # pragma: no cover - the repository test image has it
    raise SystemExit("jsonschema is required to check M6 V2 vectors") from exc


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "Tests" / "vectors" / "m6_v2_wire_v1.json"
DEFAULT_SCHEMA = ROOT / "Tests" / "vectors" / "m6_v2_wire.schema.json"
DEFAULT_INC = ROOT / "Tests" / "generated" / "m6_v2_wire_vectors.inc"
DEFAULT_C_WIRE = ROOT / "System" / "m6_wire.h"
FROZEN_REGISTRY_SHA256 = "23fbd056db28c1fedc7987a34d0893594fddb417f34058851667ffc2d5dca0a2"
FROZEN_SCHEMA_SHA256 = "95191cda4ea5ed0691126af1fa027ff37b3dd6f920ede9c099cad0b16fce7ede"


class RegistryError(ValueError):
    """A deterministic registry or vector invariant was violated."""


def _validate_c_wire_registry(registry: dict[str, Any], path: Path = DEFAULT_C_WIRE) -> None:
    """Keep firmware IDs mechanically bound to the frozen JSON registry."""
    text = path.read_text(encoding="utf-8")
    link_text = (ROOT / "System" / "m6_link.h").read_text(encoding="utf-8")
    combined = text + "\n" + link_text
    constants = {
        "M6_WIRE_VERSION": registry["artifact"]["wire_version"],
        "M6_REGISTRY_REVISION": registry["artifact"]["registry_revision"],
        "M6_FRAME_DELIMITER": registry["framing"]["delimiter"],
        "M6_FRAME_ESCAPE": registry["framing"]["escape"],
        "M6_ESCAPE_XOR": registry["framing"]["escape_xor"],
        "M6_FRAME_HEADER_SIZE": registry["framing"]["header_size"],
        "M6_FRAME_CRC_SIZE": registry["framing"]["crc_size"],
        "M6_MAX_PAYLOAD": registry["framing"]["max_payload"],
        "M6_MAX_RPC_REQUEST_PAYLOAD": registry["framing"]["max_rpc_request_payload"],
        "M6_MAX_DECODED_FRAME": registry["framing"]["max_decoded_frame"],
        "M6_MAX_WIRE_FRAME": registry["framing"]["max_wire_frame"],
        "M6_INTERBYTE_TIMEOUT_MS": registry["parser"]["interbyte_timeout_ms"],
        "M6_TOTAL_TIMEOUT_MS": registry["parser"]["total_timeout_ms"],
        "M6_HELLO_TIMEOUT_MS": registry["semantics"]["resync"]["hello_timeout_ms"],
        "M6_V1_DRAIN_TIMEOUT_MS": registry["semantics"]["resync"]["tx_drain_timeout_ms"],
    }
    for name, value in constants.items():
        match = re.search(rf"^#define\s+{name}\s+(0x[0-9A-Fa-f]+|[0-9]+)U?\s*$", text, re.MULTILINE)
        if match is None or int(match.group(1), 0) != value:
            raise RegistryError(f"C wire constant drift: {name}")
    for entry in registry["registries"]["frame_types"]:
        name = f"M6_FRAME_{entry['name']}"
        match = re.search(rf"^\s*{name}\s*=\s*(0x[0-9A-Fa-f]+|[0-9]+)", text, re.MULTILINE)
        if match is None or int(match.group(1), 0) != entry["value"]:
            raise RegistryError(f"C frame registry drift: {name}")
    for entry in registry["registries"]["capabilities"]:
        name = f"M6_CAP_{entry['name']}"
        if name not in text:
            # Deliberately unpublished capabilities need no firmware symbol.
            continue
        match = re.search(rf"^#define\s+{name}\s+\(UINT64_C\(1\)\s*<<\s*([0-9]+)\)", text, re.MULTILINE)
        if match is None or int(match.group(1)) != entry["bit"]:
            raise RegistryError(f"C capability registry drift: {name}")
    masks = {entry["name"]: int(entry["value_hex"], 16)
             for entry in registry["registries"]["capability_masks"]}
    match = re.search(
        r"^#define\s+M6_HANDSHAKE_REQUIRED\s+UINT64_C\(0x([0-9A-Fa-f]+)\)\s*$",
        text,
        re.MULTILINE,
    )
    if match is None or int(match.group(1), 16) != masks["HANDSHAKE_REQUIRED"]:
        raise RegistryError("C capability mask drift: M6_HANDSHAKE_REQUIRED")
    enum_registries = (
        ("rpc_opcodes", "M6_RPC_"),
        ("rpc_results", "M6_RESULT_"),
        ("action_kinds", "M6_ACTION_KIND_"),
        ("telemetry_streams", "M6_STREAM_"),
        ("event_codes", "M6_EVENT_"),
        ("gap_reasons", "M6_GAP_"),
        ("link_modes", "M6_LINK_"),
    )
    for registry_name, prefix in enum_registries:
        for entry in registry["registries"][registry_name]:
            name = prefix + entry["name"]
            if name not in combined:
                continue
            match = re.search(rf"^\s*{name}\s*=\s*(0x[0-9A-Fa-f]+|[0-9]+)", combined, re.MULTILINE)
            if match is None or int(match.group(1), 0) != entry["value"]:
                raise RegistryError(f"C {registry_name} drift: {name}")


def _object_no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise RegistryError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise RegistryError(f"non-finite JSON number is forbidden: {value}")


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise RegistryError(f"non-finite JSON number is forbidden: {value}")
    return parsed


def strict_load(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(
            stream,
            object_pairs_hook=_object_no_duplicates,
            parse_constant=_reject_constant,
            parse_float=_finite_float,
        )
    if not isinstance(value, dict):
        raise RegistryError(f"{path} must contain a JSON object")
    return value


def parse_hex(value: str) -> bytes:
    if not value:
        return b""
    canonical = bytes.fromhex(value)
    if canonical.hex(" ") != value:
        raise RegistryError(f"non-canonical hex bytes: {value!r}")
    return canonical


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def escape_bytes(data: bytes, framing: dict[str, Any]) -> bytes:
    escape = framing["escape"]
    xor = framing["escape_xor"]
    mandatory = set(framing["must_escape"])
    output = bytearray()
    for byte in data:
        if byte in mandatory:
            output.extend((escape, byte ^ xor))
        else:
            output.append(byte)
    return bytes(output)


def session_input(vector: dict[str, Any], session: dict[str, Any]) -> bytes:
    challenge = int(vector["challenge_hex"], 16)
    return session["prefix_ascii"].encode("ascii") + struct.pack(
        "<QIIIH",
        challenge,
        vector["boot_id"],
        vector["raw_reset_cause"],
        vector["session_generation"],
        vector["parameter_schema"],
    )


def frame_type_value(name: str, registry: dict[str, Any]) -> int:
    if name.startswith("UNREGISTERED_"):
        return int(name.removeprefix("UNREGISTERED_"), 16)
    values = {entry["name"]: entry["value"] for entry in registry["registries"]["frame_types"]}
    if name not in values:
        raise RegistryError(f"unknown frame type name: {name}")
    return values[name]


def build_frame(vector: dict[str, Any], registry: dict[str, Any]) -> dict[str, Any]:
    framing = registry["framing"]
    payload = parse_hex(vector["payload_hex"])
    if len(payload) > framing["max_payload"]:
        raise RegistryError(f"{vector['id']}: payload exceeds max_payload")
    frame_type = frame_type_value(vector["type"], registry)
    crc_input = struct.pack(
        "<BBBIIH",
        registry["artifact"]["wire_version"],
        frame_type,
        vector["flags"],
        vector["session"],
        vector["sequence"],
        len(payload),
    ) + payload
    crc = crc16_ccitt_false(crc_input)
    crc_le = struct.pack("<H", crc)
    decoded = crc_input + crc_le
    wire = bytes((framing["delimiter"],)) + escape_bytes(decoded, framing) + bytes((framing["delimiter"],))
    return {
        "crc_input_hex": crc_input.hex(" "),
        "crc16": crc,
        "crc_le_hex": crc_le.hex(" "),
        "decoded_hex": decoded.hex(" "),
        "wire_hex": wire.hex(" "),
        "decoded_length": len(decoded),
        "wire_length": len(wire),
    }


def _assert_unique(entries: list[dict[str, Any]], label: str) -> None:
    names = [entry["name"] for entry in entries]
    values = [entry["value"] for entry in entries]
    if len(names) != len(set(names)):
        raise RegistryError(f"duplicate {label} name")
    if len(values) != len(set(values)):
        raise RegistryError(f"duplicate {label} value")


def _validate_layouts(registry: dict[str, Any]) -> None:
    layouts = registry["layouts"]
    names = [layout["name"] for layout in layouts]
    if len(names) != len(set(names)):
        raise RegistryError("duplicate layout name")
    by_name = {layout["name"]: layout for layout in layouts}
    for layout in layouts:
        cursor = 0
        field_names: set[str] = set()
        for field in layout["fields"]:
            prior_fields = set(field_names)
            if field["name"] in field_names:
                raise RegistryError(f"{layout['name']}: duplicate field {field['name']}")
            field_names.add(field["name"])
            if field["offset"] != cursor:
                raise RegistryError(f"{layout['name']}: gap/overlap at {field['name']}")
            scalar_width = {
                "u8": 1, "i8": 1, "u16": 2, "i16": 2,
                "u32": 4, "i32": 4, "u64": 8,
            }.get(field["kind"])
            expected_size = scalar_width * field.get("count", 1) if scalar_width else field["size"]
            if field["size"] != expected_size:
                raise RegistryError(f"{layout['name']}: size/count mismatch at {field['name']}")
            kind = field["kind"]
            if kind == "layout_ref":
                reference = by_name.get(field.get("layout_ref", ""))
                if reference is None or field["size"] != reference["size"]:
                    raise RegistryError(f"{layout['name']}: invalid layout_ref at {field['name']}")
            elif kind == "layout_array":
                reference = by_name.get(field.get("layout_ref", ""))
                count_field = field.get("count_field")
                max_count = field.get("max_count")
                if reference is None or count_field not in prior_fields or not isinstance(max_count, int):
                    raise RegistryError(f"{layout['name']}: invalid layout_array at {field['name']}")
                if field["size"] != reference["size"] * max_count:
                    raise RegistryError(f"{layout['name']}: layout_array size mismatch at {field['name']}")
            elif kind == "layout_union":
                selector = field.get("selector_field")
                length_field = field.get("length_field")
                mapping = field.get("layout_by_selector", {})
                references = [by_name.get(name) for name in mapping.values()]
                if selector not in prior_fields or length_field not in prior_fields or not references or any(item is None for item in references):
                    raise RegistryError(f"{layout['name']}: invalid layout_union at {field['name']}")
                if field["size"] != max(item["size"] for item in references if item is not None):
                    raise RegistryError(f"{layout['name']}: layout_union size mismatch at {field['name']}")
            elif kind == "bytes" and "length_field" in field:
                if field["length_field"] not in prior_fields or field.get("max_count") != field["size"]:
                    raise RegistryError(f"{layout['name']}: invalid length-delimited bytes at {field['name']}")
                if field.get("encoding") != "ascii":
                    raise RegistryError(f"{layout['name']}: variable bytes must declare ASCII encoding")
            elif any(key in field for key in ("layout_ref", "count_field", "length_field", "selector_field", "layout_by_selector", "max_count", "encoding")):
                raise RegistryError(f"{layout['name']}: unexpected reference metadata at {field['name']}")
            registry_name = field.get("registry")
            if registry_name is not None:
                entries = registry["registries"].get(registry_name)
                if not isinstance(entries, list) or not entries:
                    raise RegistryError(f"{layout['name']}: unknown registry {registry_name} at {field['name']}")
                if kind not in ("u8", "i8", "u16", "u32", "u64"):
                    raise RegistryError(f"{layout['name']}: registry field {field['name']} must be integer")
                if kind == "i8":
                    min_wire_value, max_wire_value = -128, 127
                else:
                    min_wire_value, max_wire_value = 0, (1 << (8 * field["size"])) - 1
                if any(not min_wire_value <= entry["value"] <= max_wire_value for entry in entries):
                    raise RegistryError(f"{layout['name']}: registry {registry_name} exceeds {field['name']} width")
            cursor += field["size"]
        if cursor != layout["size"]:
            raise RegistryError(f"{layout['name']}: fields end at {cursor}, size is {layout['size']}")
    required_sizes = {
        "frame_header_v1": 13,
        "hello_v1": 40,
        "ready_v1": 64,
        "rpc_request_common_v1": 2,
        "rpc_response_common_v1": 8,
        "rpc_empty_body_v1": 0,
        "rpc_ping_request_body_v1": 4,
        "rpc_ping_success_body_v1": 12,
        "rpc_identity_get_success_body_v1": 63,
        "rpc_status_get_success_body_v1": 60,
        "rpc_task_stats_get_success_body_v1": 64,
        "rpc_uart_stats_get_success_body_v1": 32,
        "rpc_link_stats_get_success_body_v1": 76,
        "subscription_entry_v1": 4,
        "rpc_subscription_get_success_body_v1": 24,
        "rpc_snapshot_get_request_body_v1": 4,
        "rpc_snapshot_get_success_body_v1": 68,
        "rpc_param_summary_get_success_body_v1": 24,
        "rpc_parameter_id_request_body_v1": 4,
        "rpc_param_descriptor_get_success_body_v1": 51,
        "rpc_param_value_get_success_body_v1": 28,
        "rpc_trace_chunk_get_request_body_v1": 8,
        "rpc_trace_chunk_get_success_body_v1": 12,
        "rpc_fault_clear_request_body_v1": 8,
        "rpc_fault_clear_success_body_v1": 12,
        "rpc_param_stage_set_request_body_v1": 12,
        "rpc_param_stage_set_success_body_v1": 12,
        "rpc_param_apply_request_body_v1": 12,
        "rpc_param_apply_success_body_v1": 20,
        "rpc_param_revert_request_body_v1": 8,
        "rpc_param_revert_success_body_v1": 12,
        "rpc_param_load_request_body_v1": 8,
        "rpc_param_load_success_body_v1": 12,
        "rpc_param_defaults_request_body_v1": 4,
        "rpc_param_defaults_success_body_v1": 8,
        "rpc_param_save_request_body_v1": 8,
        "rpc_param_save_success_body_v1": 12,
        "rpc_subscription_set_request_body_v1": 20,
        "rpc_action_arm_request_body_v1": 24,
        "rpc_action_arm_success_body_v1": 32,
        "rpc_action_confirm_request_body_v1": 8,
        "rpc_action_confirm_success_body_v1": 16,
        "rpc_action_cancel_request_body_v1": 4,
        "rpc_action_cancel_success_body_v1": 12,
        "rpc_boot_id_request_body_v1": 4,
        "rpc_heading_calibrate_success_body_v1": 8,
        "rpc_grayscale_capture_start_success_body_v1": 8,
        "protocol_error_v1": 8,
        "event_common_v1": 12,
        "event_reset_body_v1": 32,
        "event_safety_fault_body_v1": 36,
        "event_emergency_stop_body_v1": 24,
        "event_action_state_body_v1": 28,
        "event_param_committed_body_v1": 20,
        "event_telemetry_gap_body_v1": 12,
        "event_storage_fault_body_v1": 12,
        "event_link_state_body_v1": 16,
        "event_grayscale_ready_body_v1": 40,
        "telemetry_common_v1": 8,
        "telemetry_core_body_v1": 32,
        "telemetry_wheel_body_v1": 52,
        "telemetry_line_body_v1": 48,
        "telemetry_health_body_v1": 64,
        "bulk_common_v1": 16,
        "line_trace_info_v1": 24,
        "line_trace_record_v1": 30,
        "reset_snapshot_v1": 76,
    }
    for name, size in required_sizes.items():
        if name not in by_name or by_name[name]["size"] != size:
            raise RegistryError(f"layout {name} must be exactly {size} bytes")

    exact_fields = {
        "bulk_common_v1": [
            ("kind", 0, 1, "u8", False),
            ("schema", 1, 1, "u8", False),
            ("reserved", 2, 2, "u16", True),
            ("transfer_id", 4, 4, "u32", False),
            ("object_id", 8, 4, "u32", False),
            ("offset", 12, 2, "u16", False),
            ("count", 14, 1, "u8", False),
            ("record_size", 15, 1, "u8", False),
        ],
        "event_grayscale_ready_body_v1": [
            ("request_id", 0, 4, "u32", False),
            ("scan_id", 4, 4, "u32", False),
            ("adc", 8, 16, "u16", False),
            ("error_count", 24, 4, "u32", False),
            ("last_adc_status", 28, 4, "u32", False),
            ("last_adc_flags", 32, 4, "u32", False),
            ("state_flags", 36, 4, "u32", False),
        ],
        "subscription_entry_v1": [
            ("stream_id", 0, 1, "u8", False),
            ("flags", 1, 1, "u8", True),
            ("period_ms", 2, 2, "u16", False),
        ],
    }
    for name, expected in exact_fields.items():
        actual = [
            (field["name"], field["offset"], field["size"], field["kind"], field.get("reserved", False))
            for field in by_name[name]["fields"]
        ]
        if actual != expected:
            raise RegistryError(f"{name}: frozen field layout changed")


def _validate_rpc_contract(registry: dict[str, Any]) -> None:
    expected_values = {
        "PING": 0x0001,
        "IDENTITY_GET": 0x0002,
        "STATUS_GET": 0x0003,
        "RESET_GET": 0x0004,
        "TASK_STATS_GET": 0x0005,
        "UART_STATS_GET": 0x0006,
        "LINK_STATS_GET": 0x0007,
        "ACTION_GET": 0x0008,
        "SUBSCRIPTION_GET": 0x0009,
        "SNAPSHOT_GET": 0x000A,
        "PARAM_SUMMARY_GET": 0x0100,
        "PARAM_DESCRIPTOR_GET": 0x0101,
        "PARAM_VALUE_GET": 0x0102,
        "TRACE_INFO_GET": 0x0200,
        "TRACE_CHUNK_GET": 0x0201,
        "SESSION_CLOSE": 0x8000,
        "FAULT_CLEAR": 0x8001,
        "PARAM_STAGE_SET": 0x8100,
        "PARAM_APPLY": 0x8101,
        "PARAM_REVERT": 0x8102,
        "PARAM_LOAD": 0x8103,
        "PARAM_DEFAULTS": 0x8104,
        "PARAM_SAVE": 0x8105,
        "SUBSCRIPTION_SET": 0x8200,
        "ACTION_ARM": 0x8300,
        "ACTION_CONFIRM": 0x8301,
        "ACTION_CANCEL": 0x8302,
        "HEADING_CALIBRATE": 0x8303,
        "GRAYSCALE_CAPTURE_START": 0x8304,
    }
    opcodes = registry["registries"]["rpc_opcodes"]
    if [entry["name"] for entry in opcodes] != list(expected_values):
        raise RegistryError("wire-v1 RPC opcode inventory/order changed")
    if {entry["name"]: entry["value"] for entry in opcodes} != expected_values:
        raise RegistryError("wire-v1 RPC opcode values changed")

    framing = registry["framing"]
    layouts = {layout["name"]: layout for layout in registry["layouts"]}
    request_totals: list[int] = []
    success_totals: list[int] = []
    variable_sides: set[tuple[str, str]] = set()
    for opcode in opcodes:
        for side, common_size, limit in (
            ("request", 2, framing["max_rpc_request_payload"]),
            ("success", 8, framing["max_payload"]),
        ):
            minimum = opcode[f"{side}_body_min"]
            maximum = opcode[f"{side}_body_max"]
            layout = layouts.get(opcode[f"{side}_body_layout"])
            if minimum > maximum:
                raise RegistryError(f"{opcode['name']}: {side} body minimum exceeds maximum")
            if layout is None or layout["size"] != maximum:
                raise RegistryError(f"{opcode['name']}: {side} body layout/max mismatch")
            if common_size + maximum > limit:
                raise RegistryError(f"{opcode['name']}: {side} payload exceeds frozen limit")
            if minimum != maximum:
                variable_sides.add((opcode["name"], side))
            (request_totals if side == "request" else success_totals).append(common_size + maximum)

    expected_variable_sides = {
        ("IDENTITY_GET", "success"),
        ("SUBSCRIPTION_GET", "success"),
        ("SNAPSHOT_GET", "success"),
        ("PARAM_DESCRIPTOR_GET", "success"),
        ("SUBSCRIPTION_SET", "request"),
        ("SUBSCRIPTION_SET", "success"),
    }
    if variable_sides != expected_variable_sides:
        raise RegistryError("wire-v1 variable RPC body set changed")
    if max(request_totals) != 26 or max(success_totals) != 84:
        raise RegistryError("wire-v1 largest RPC payload must remain request=26/success=84")
    if registry["semantics"]["rpc_failure_body_bytes"] != 0:
        raise RegistryError("failed RPC responses must not append a body")

    by_opcode = {entry["name"]: entry for entry in opcodes}
    exact_ranges = {
        ("IDENTITY_GET", "success"): (32, 63),
        ("SUBSCRIPTION_GET", "success"): (8, 24),
        ("SNAPSHOT_GET", "success"): (36, 68),
        ("PARAM_DESCRIPTOR_GET", "success"): (20, 51),
        ("SUBSCRIPTION_SET", "request"): (4, 20),
        ("SUBSCRIPTION_SET", "success"): (8, 24),
    }
    for (name, side), expected in exact_ranges.items():
        actual = (by_opcode[name][f"{side}_body_min"], by_opcode[name][f"{side}_body_max"])
        if actual != expected:
            raise RegistryError(f"{name}: variable {side} range changed")

    identity_tail = layouts["rpc_identity_get_success_body_v1"]["fields"][-1]
    descriptor_tail = layouts["rpc_param_descriptor_get_success_body_v1"]["fields"][-1]
    if (identity_tail["offset"], identity_tail["max_count"], identity_tail["length_field"]) != (32, 31, "firmware_id_length"):
        raise RegistryError("IDENTITY_GET firmware ID length equation changed")
    if (descriptor_tail["offset"], descriptor_tail["max_count"], descriptor_tail["length_field"]) != (20, 31, "key_length"):
        raise RegistryError("PARAM_DESCRIPTOR_GET key length equation changed")

    for name, prefix in (
        ("rpc_subscription_get_success_body_v1", 8),
        ("rpc_subscription_set_request_body_v1", 4),
    ):
        entries = layouts[name]["fields"][-1]
        if (entries["offset"], entries["layout_ref"], entries["count_field"], entries["max_count"]) != (prefix, "subscription_entry_v1", "count", 4):
            raise RegistryError(f"{name}: subscription entry equation changed")

    snapshot = layouts["rpc_snapshot_get_success_body_v1"]["fields"][-1]
    expected_snapshot_map = {
        "1": "telemetry_core_body_v1",
        "2": "telemetry_wheel_body_v1",
        "3": "telemetry_line_body_v1",
        "4": "telemetry_health_body_v1",
    }
    if snapshot.get("selector_field") != "stream_id" or snapshot.get("length_field") != "data_length" or snapshot.get("layout_by_selector") != expected_snapshot_map:
        raise RegistryError("SNAPSHOT_GET telemetry selector/layout mapping changed")

    normalized = layouts["rpc_action_arm_success_body_v1"]["fields"][-1]
    if (normalized["offset"], normalized["size"], normalized.get("layout_ref")) != (8, 24, "rpc_action_arm_request_body_v1"):
        raise RegistryError("ACTION_ARM normalized request echo changed")


def _validate_semantic_contract(registry: dict[str, Any]) -> None:
    semantics = registry["semantics"]
    if semantics["version_mismatch"] != {
        "validation_order": "after-crc-before-flags-session-type",
        "v2_armed_effect": "clear-candidate-and-v1-resync",
        "v2_established_effect": "silent-drop-count-frame-error-preserve-session",
        "protocol_error_eligible": False,
    }:
        raise RegistryError("wire-version mismatch semantics changed")
    if semantics["rx_overflow"] != {
        "v2_armed_effect": "clear-candidate-and-v1-resync",
        "v2_established_effect": "clear-parser-only",
        "cancels_admitted_apply": False,
    }:
        raise RegistryError("RX-overflow state semantics changed")
    if semantics["emergency"] != {
        "isr_safety_precedes_transport": True,
        "clear_queues_before_next_arbitration": ["P2", "P3"],
        "cancel_pending_apply": True,
        "preserve_fault": True,
        "v2_established_effect": "preserve-session",
    }:
        raise RegistryError("emergency transport semantics changed")
    if semantics["safety_fault"] != {
        "clear_queues_before_next_arbitration": ["P2", "P3"],
        "cancel_pending_apply": True,
        "abort_admitted_p3": True,
        "preserve_fault": True,
    }:
        raise RegistryError("Safety-fault transport semantics changed")

    handshake = semantics["handshake"]
    required_handshake = {
        "mode_attempts": 1,
        "hello_attempts": 1,
        "ready_replay_after_established": False,
        "mode_challenge_single_use": True,
        "session_generation_allocations_per_challenge": 1,
        "mode_request_template": ":{transaction_id},mode,v2,{challenge_hex16}\r\n",
        "v1_ready_template": "ok,{transaction_id},v2-ready,{challenge_hex16},2,1,96,32,1\r\n",
        "challenge_hex_width": 16,
        "challenge_hex_case": "lower",
        "challenge_nonzero": True,
        "hello_magic_u32": 0x3256364D,
        "hello_magic_wire_hex": "4d 36 56 32",
        "ready_magic_u32": 0x59443252,
        "ready_magic_wire_hex": "52 32 44 59",
        "header_session": 0,
        "header_sequence": 0,
        "registry_revision": 1,
        "registry_acceptance": "registry_min<=1<=registry_max",
        "host_min_payload": 96,
        "host_min_rpc_request_payload": 32,
        "host_rx_window": 1,
        "success_selected_registry": 1,
        "success_selected_max_payload": 96,
        "success_selected_max_rpc_request_payload": 32,
        "success_device_rx_window": 1,
        "success_session_nonzero": True,
        "host_required_must_include_mask": "000000000000600f",
        "required_optional_overlap": "invalid-argument-lowest-overlap-bit",
        "failure_ready_zero_fields": [
            "selected_registry", "assigned_session", "enabled_capabilities",
            "selected_max_payload", "selected_max_rpc_request", "device_rx_window",
        ],
        "failure_ready_echo_challenge": True,
        "failure_ready_diagnostic_fields_may_populate": [
            "boot_id", "raw_reset_cause", "session_generation", "parameter_schema", "device_capabilities",
        ],
        "failure_ready_after": "physical-tx-complete-then-v1-resync",
        "host_payload_below_ready_size": "silent-handshake-error-then-v1-resync",
        "host_payload_below_ready_size_threshold": 64,
    }
    for key, expected in required_handshake.items():
        if handshake.get(key) != expected:
            raise RegistryError(f"handshake semantic changed: {key}")
    expected_failure_rules = [
        {"condition": "bad-magic", "result": "MALFORMED", "detail": "constant", "detail_value": 0},
        {"condition": "zero-or-mode-mismatched-challenge", "result": "INVALID_ARGUMENT", "detail": "field-byte-offset", "detail_value": 8},
        {"condition": "registry-min-greater-than-max", "result": "INVALID_ARGUMENT", "detail": "field-byte-offset", "detail_value": 4},
        {"condition": "first-nonzero-reserved", "result": "MALFORMED", "detail": "first-byte-offset-37-to-39"},
        {"condition": "host-window-not-one", "result": "BAD_LENGTH", "detail": "expected-hi16-actual-lo16", "expected": 1},
        {"condition": "registry-no-intersection", "result": "INCOMPATIBLE_REGISTRY", "detail": "device-registry-revision"},
        {"condition": "host-required-missing-handshake-required", "result": "UNSUPPORTED_CAPABILITY", "detail": "lowest-missing-handshake-required-bit-index"},
        {"condition": "required-optional-overlap", "result": "INVALID_ARGUMENT", "detail": "lowest-overlap-bit-index"},
        {"condition": "required-capability-missing-or-unknown", "result": "UNSUPPORTED_CAPABILITY", "detail": "lowest-missing-or-unknown-bit-index"},
        {"condition": "host-max-payload-64-to-95", "result": "BAD_LENGTH", "detail": "expected-hi16-actual-lo16", "expected": 96},
        {"condition": "host-max-rpc-request-below-32", "result": "BAD_LENGTH", "detail": "expected-hi16-actual-lo16", "expected": 32},
    ]
    if handshake["failure_detail_rules"] != expected_failure_rules:
        raise RegistryError("HELLO failure detail precedence/encoding changed")

    resync = semantics["resync"]
    expected_resync_scalars = {
        "quiet_ms": 100, "probe_timeout_ms": 500, "probe_attempts": 1,
        "v1_transaction_timeout_ms": 1000, "tx_drain_timeout_ms": 250,
        "v1_ready_timeout_ms": 1000, "hello_timeout_ms": 500, "auto_fallback_ms": 0,
        "transaction_id": 1, "v1_producers_frozen": True,
        "probe_requires_simultaneous_rx_tx_empty": True,
        "restore_v1_after_reply_physical_complete": True,
        "waits": "nonblocking-wrap-safe",
    }
    for key, expected in expected_resync_scalars.items():
        if resync.get(key) != expected:
            raise RegistryError(f"resync semantic changed: {key}")
    if resync["physical_drain_predicates"] != [
        "upper-records-empty", "legacy-fifo-empty", "uart-fifo-empty", "uart-not-busy", "rx-empty",
    ]:
        raise RegistryError("physical V1/V2 drain predicate changed")
    if resync["cancel_not_started"] != ["TELEMETRY", "DIAGNOSTIC", "G6", "HELP"]:
        raise RegistryError("V2 drain cancellation inventory changed")

    sequence = semantics["sequence"]
    expected_sequence = {
        "host_initial": 1, "async_initial": 1, "reserved": 0xFFFFFFFF,
        "host_last_usable": 0xFFFFFFFE, "async_last_normal": 0xFFFFFFFD, "host_window": 1,
        "window_pending_check": "before-sequence", "rpc_entries": 4,
        "rpc_entry_combines_request_replay_and_result": True,
        "rpc_entry_request_max": 32, "rpc_entry_response_max": 96,
        "rpc_entry_max_bytes": 152,
        "entry_states": ["PENDING", "QUEUED", "IN_FLIGHT", "COMPLETED"],
        "eviction_policy": "oldest-completed-only",
        "non_evictable_states": ["PENDING", "QUEUED", "IN_FLIGHT"],
        "pending_duplicate": "associate-original-no-temporary-response",
        "completed_duplicate": "exact-request-bytes-reencode-with-replay",
        "different_bytes_same_sequence": "SEQUENCE_CONFLICT",
        "evicted_sequence": "REPLAY_EXPIRED", "higher_sequence": "SEQUENCE_GAP",
        "reserve_entry_before_advance_or_execute": True, "deterministic_errors_cached": True,
        "busy_latch_independent": True, "busy_borrows_rpc_entry": False,
        "busy_latch_occupied_counter": "control_error_drop",
        "link_stats_replay_used": "entry-count-0-to-4",
        "async_terminal_frame": "LINK_STATE", "async_terminal_reason": "SEQUENCE_EXHAUSTED",
        "async_terminal_sequence": 0xFFFFFFFE,
        "async_terminal_delivery": "direct-to-inflight-not-p1", "async_terminal_droppable": False,
        "async_terminal_priority_after": "CONTROL_ERROR",
        "async_terminal_event_occurrence_count": 1, "async_terminal_event_severity": "WARNING",
        "async_terminal_frame_flags": 0, "async_terminal_payload_mode": "V2_ESTABLISHED",
        "async_terminal_payload_session": "current-session",
        "async_terminal_payload_next_host_sequence": "current-expected",
        "async_terminal_payload_next_async_sequence": 0xFFFFFFFF,
        "async_terminal_activation": "next-async-equals-terminal-before-normal-arbitration",
        "async_terminal_only_async_candidate": True,
        "async_terminal_non_async_predecessors": ["RPC_RESULT", "CONTROL_ERROR"],
        "async_terminal_p0_after_activation": "retain-latches-query-via-rpc",
        "stop_async_after_terminal": True, "preserve_rpc_after_terminal": True, "wrap_allowed": False,
    }
    if sequence != expected_sequence:
        raise RegistryError("sequence/replay/terminal-link contract changed")

    queues = semantics["queues"]
    if queues["arbitration"] != ["P0", "RPC_RESULT", "CONTROL_ERROR", "TERMINAL_LINK", "P1", "P2", "P3"]:
        raise RegistryError("TX priority order changed")
    if queues["p0"] != {
        "policy": "fixed-topic-latch", "topic_key": "event-code", "same_topic_body": "latest", "occurrence_initial": 1,
        "same_topic_occurrence": "saturating-u16-increment", "same_topic_sets_gap": True,
        "occurrence_after_physical_complete": 1,
        "same_topic_overwrite_counter": "p0_drop",
    }:
        raise RegistryError("P0 latch/coalescing contract changed")
    if queues["p1"]["drop_first"] != ["LINK_STATE", "TELEMETRY_GAP"] or queues["p1"]["drop_first_selection"] != "listed-topic-order-then-oldest":
        raise RegistryError("P1 eviction order changed")
    if queues["p1"]["topic_key_default"] != "event-code" or queues["p1"]["topic_key_telemetry_gap"] != "event-code-plus-stream-id":
        raise RegistryError("P1 coalescing key changed")
    if queues["p3"]["token_bytes_per_second"] != 1000 or queues["p3"]["token_bucket_bytes"] != 224:
        raise RegistryError("P3 token budget changed")
    if queues["p3"]["abort_triggers"] != ["EMERGENCY", "SAFETY_FAULT", "SESSION_TEARDOWN"]:
        raise RegistryError("P3 abort trigger inventory changed")

    apply = semantics["parameter_apply"]
    if apply["diagnostic_parameter_ids"] != [1, 2, 3, 4, 13] or apply["diagnostic_boundary_period_ms"] != 5:
        raise RegistryError("diagnostic parameter boundary changed")
    if apply["pending_cancel_triggers"] != ["EMERGENCY", "SAFETY_FAULT"] or apply["cancelled_result_options"] != ["SAFETY_FAULT", "WRONG_STATE"]:
        raise RegistryError("pending parameter-apply cancellation contract changed")
    if apply["change_mask_bit"] != "parameter-id-minus-one" or apply["stage_revision_initial"] != 1:
        raise RegistryError("parameter revision/change-mask semantics changed")
    if apply["stage_revision_increment_after"] != ["PARAM_STAGE_SET", "PARAM_REVERT", "PARAM_LOAD", "PARAM_DEFAULTS", "PARAM_APPLY"] or apply["stage_revision_persisted_in_flash"]:
        raise RegistryError("RAM-only stage revision lifecycle changed")

    protocol = semantics["protocol_error"]
    if protocol != {
        "rate_limit_ms": 100, "header_session": "current-session", "header_sequence": "offending-sequence",
        "consumes_host_sequence": False,
        "silent_conditions": ["CRC_ERROR", "UNTRUSTED_HEADER", "VERSION_MISMATCH", "SESSION_MISMATCH"],
        "wrong_direction_result": "UNSUPPORTED_TYPE",
        "legal_results": ["BAD_FLAGS", "BAD_LENGTH", "UNSUPPORTED_TYPE", "MALFORMED"],
    }:
        raise RegistryError("PROTOCOL_ERROR eligibility/rate contract changed")
    bulk = semantics["bulk_transfer"]
    if bulk != {
        "data_max_bytes": 80, "schema": 1, "count_min": 1, "count_max": 2,
        "record_size": 30, "data_length_equation": "count*record_size",
        "offset_order": "chronological-record-index", "admission_gate": "frozen-matching-capture",
        "p3_abort_retry": "new-sequence-same-capture-and-offset",
    }:
        raise RegistryError("LINE_TRACE bulk contract changed")

    actions = semantics["actions"]
    if actions["deadline_arithmetic"] != "u32-wrap" or actions["flags_required"] != 0:
        raise RegistryError("action common validation contract changed")
    if actions["token_binding"] != ["session", "arm_sequence", "boot_id", "active_revision", "full_action_parameters"]:
        raise RegistryError("action arm-token binding changed")
    if actions["confirm_revalidate"] != ["session", "boot_id", "token", "expiry", "safety", "system_state", "action_state", "sensor_state", "active_revision", "all_limits"]:
        raise RegistryError("action-confirm full revalidation set changed")
    expected_action_caps = {
        "WHEEL_SPEED_TEST": ["WHEEL_DIAG_ACTION"],
        "HEADING_HOLD": ["HEADING_ACTION"],
        "HEADING_TURN": ["HEADING_ACTION"],
        "HEADING_RATE": ["HEADING_ACTION", "HEADING_RATE_ACTION"],
        "LINE_FOLLOW": ["LINE_ACTION"],
        "MOTOR_SELF_TEST": ["MOTOR_SELF_TEST"],
    }
    if {item["name"]: item["required_capabilities"] for item in actions["kinds"]} != expected_action_caps:
        raise RegistryError("action capability mapping changed")
    expected_action_kinds = [
        {
            "name": "WHEEL_SPEED_TEST", "required_capabilities": ["WHEEL_DIAG_ACTION"],
            "wheel_mask_allowed": [1, 2, 3], "unselected_targets_zero": True,
            "target_relation": "selected-abs-range", "target_abs_min_mm_s": 50,
            "target_abs_max_mm_s": 400, "zero_fields": ["angle_mdeg", "yaw_rate_mdeg_s"],
            "duration_min_ms": 100, "duration_max_ms": 10000,
        },
        {
            "name": "HEADING_HOLD", "required_capabilities": ["HEADING_ACTION"],
            "wheel_mask_allowed": [3], "unselected_targets_zero": False,
            "target_relation": "left-equals-right-nonzero", "target_abs_min_mm_s": 50,
            "target_abs_max_mm_s": 400, "zero_fields": ["angle_mdeg", "yaw_rate_mdeg_s"],
            "duration_min_ms": 100, "duration_max_ms": 30000,
        },
        {
            "name": "HEADING_TURN", "required_capabilities": ["HEADING_ACTION"],
            "angle_extension_capability": "CONFIGURABLE_TURN", "wheel_mask_allowed": [3],
            "unselected_targets_zero": False, "target_relation": "all-zero",
            "target_abs_min_mm_s": 0, "target_abs_max_mm_s": 0,
            "zero_fields": ["target_l_mm_s", "target_r_mm_s", "yaw_rate_mdeg_s"],
            "angle_abs_min_mdeg": 1000, "angle_abs_max_mdeg": 360000,
            "without_configurable_turn_angles_mdeg": [-90000, 90000],
            "duration_min_ms": 500, "duration_max_ms": 30000,
        },
        {
            "name": "HEADING_RATE", "required_capabilities": ["HEADING_ACTION", "HEADING_RATE_ACTION"],
            "wheel_mask_allowed": [3], "unselected_targets_zero": False,
            "target_relation": "all-zero", "target_abs_min_mm_s": 0, "target_abs_max_mm_s": 0,
            "zero_fields": ["target_l_mm_s", "target_r_mm_s", "angle_mdeg"],
            "rate_abs_min_mdeg_s": 1000, "rate_abs_max_mdeg_s": 360000,
            "duration_min_ms": 100, "duration_max_ms": 30000,
        },
        {
            "name": "LINE_FOLLOW", "required_capabilities": ["LINE_ACTION"],
            "wheel_mask_allowed": [3], "unselected_targets_zero": False,
            "target_relation": "all-zero", "target_abs_min_mm_s": 0, "target_abs_max_mm_s": 0,
            "zero_fields": ["target_l_mm_s", "target_r_mm_s", "angle_mdeg", "yaw_rate_mdeg_s"],
            "duration_min_ms": 100, "duration_max_ms": 30000,
        },
        {
            "name": "MOTOR_SELF_TEST", "required_capabilities": ["MOTOR_SELF_TEST"],
            "wheel_mask_allowed": [3], "unselected_targets_zero": False,
            "target_relation": "all-zero", "target_abs_min_mm_s": 0, "target_abs_max_mm_s": 0,
            "zero_fields": ["target_l_mm_s", "target_r_mm_s", "angle_mdeg", "yaw_rate_mdeg_s"],
            "duration_min_ms": 2500, "duration_max_ms": 5000,
            "compile_time_sequence_only": True, "safety_path_required": True, "raw_pwm_forbidden": True,
        },
    ]
    if actions["kinds"] != expected_action_kinds:
        raise RegistryError("wire-v1 per-kind action validation profile changed")
    turn = next(item for item in actions["kinds"] if item["name"] == "HEADING_TURN")
    if turn.get("angle_extension_capability") != "CONFIGURABLE_TURN":
        raise RegistryError("HEADING_TURN extended-angle capability binding changed")
    if semantics["session_close"] != {
        "success_gates": ["IDLE", "FAULT_FREE", "ACTION_IDLE", "NO_PENDING_APPLY", "FLASH_IDLE"],
        "response_completion_target": "V1_RESYNC",
        "emergency_after_response_started": "finish-response-then-v1-resync",
        "preserve_fault": True, "preserve_braking": True, "preserve_breadcrumb": True,
    }:
        raise RegistryError("SESSION_CLOSE gate/teardown contract changed")
    expected_detail_rules = [
        {"results": ["BAD_LENGTH"], "encoding": "expected-hi16-actual-lo16"},
        {"results": ["UNSUPPORTED_TYPE", "UNSUPPORTED_OPCODE"], "encoding": "unsupported-value"},
        {"results": ["SEQUENCE_GAP"], "encoding": "expected-sequence"},
        {"results": ["SEQUENCE_CONFLICT", "REPLAY_EXPIRED"], "encoding": "request-sequence"},
        {"results": ["UNKNOWN_PARAMETER", "READ_ONLY", "INVALID_VALUE", "OUT_OF_RANGE", "RELATION"], "encoding": "parameter-id-low8"},
        {"results": ["STALE_ACTIVE_REVISION", "STALE_STAGE_REVISION", "STALE_GENERATION"], "encoding": "device-current-value"},
        {"results": ["WRONG_STATE"], "encoding": "system-low8-action-next8"},
        {"results": ["SAFETY_FAULT"], "encoding": "full-fault-mask"},
        {"results": ["BUSY"], "encoding": "resource-high8-wait-ms-low24"},
    ]
    if semantics["result_detail_rules"] != expected_detail_rules:
        raise RegistryError("RPC result detail encodings changed")


def _validate_registry_invariants(registry: dict[str, Any]) -> None:
    framing = registry["framing"]
    parser = registry["parser"]
    if framing["header_size"] != 13 or framing["crc_size"] != 2:
        raise RegistryError("wire-v1 header/CRC sizes changed")
    expected_decoded = framing["header_size"] + framing["max_payload"] + framing["crc_size"]
    if framing["max_decoded_frame"] != expected_decoded:
        raise RegistryError("max_decoded_frame formula mismatch")
    if framing["max_wire_frame"] != 2 + 2 * expected_decoded:
        raise RegistryError("max_wire_frame formula mismatch")
    if framing["max_rpc_request_payload"] > framing["max_payload"]:
        raise RegistryError("max_rpc_request_payload exceeds max_payload")
    if parser["rx_ring_effective"] != parser["rx_ring_physical"] - 1:
        raise RegistryError("RX ring effective capacity mismatch")
    if parser["rx_ring_effective"] < framing["max_wire_frame"]:
        raise RegistryError("RX ring cannot hold one maximum wire frame")
    if parser["interbyte_timeout_ms"] != 50 or parser["total_timeout_ms"] != 100:
        raise RegistryError("wire-v1 parser timeout changed")
    if sorted(framing["must_escape"]) != [0x20, 0x53, 0x7D, 0x7E]:
        raise RegistryError("mandatory escape set changed")
    followers = {byte ^ framing["escape_xor"] for byte in framing["must_escape"]}
    if followers != {0x00, 0x73, 0x5D, 0x5E}:
        raise RegistryError("canonical escape followers changed")

    registries = registry["registries"]
    for key, entries in registries.items():
        if key == "capability_masks":
            continue
        _assert_unique(entries, key)
    expected_frame_types = [
        {"name": "HELLO", "value": 0x01, "direction": "host-to-device", "allowed_flags": 0},
        {"name": "READY", "value": 0x02, "direction": "device-to-host", "allowed_flags": 0},
        {"name": "RPC_REQUEST", "value": 0x10, "direction": "host-to-device", "allowed_flags": 0},
        {"name": "RPC_RESPONSE", "value": 0x11, "direction": "device-to-host", "allowed_flags": 1},
        {"name": "PROTOCOL_ERROR", "value": 0x12, "direction": "device-to-host", "allowed_flags": 0},
        {"name": "EVENT", "value": 0x20, "direction": "device-to-host", "allowed_flags": 2},
        {"name": "TELEMETRY", "value": 0x30, "direction": "device-to-host", "allowed_flags": 2},
        {"name": "BULK", "value": 0x31, "direction": "device-to-host", "allowed_flags": 2},
    ]
    if registries["frame_types"] != expected_frame_types:
        raise RegistryError("wire-v1 frame type/direction/flag registry changed")
    capabilities = registries["capabilities"]
    if len(capabilities) != 25:
        raise RegistryError("wire-v1 requires capability bits 0..24")
    for expected_bit, entry in enumerate(capabilities):
        if entry.get("bit") != expected_bit or entry["value"] != 1 << expected_bit:
            raise RegistryError("capability bits must be contiguous 0..24")
    mask_entries = registries["capability_masks"]
    mask_names = [entry["name"] for entry in mask_entries]
    mask_hex = [entry["value_hex"] for entry in mask_entries]
    if len(mask_names) != len(set(mask_names)) or len(mask_hex) != len(set(mask_hex)):
        raise RegistryError("duplicate capability aggregate mask")
    masks = {entry["name"]: int(entry["value_hex"], 16) for entry in mask_entries}
    all_defined = 0
    for entry in capabilities:
        all_defined |= entry["value"]
    if masks != {
        "HANDSHAKE_REQUIRED": 0x000000000000600F,
        "HELLO_OPTIONAL": 0x0000000001FF9FF0,
        "ALL_DEFINED": 0x0000000001FFFFFF,
    }:
        raise RegistryError("wire-v1 capability aggregate masks changed")
    if masks["ALL_DEFINED"] != all_defined:
        raise RegistryError("ALL_DEFINED capability mask mismatch")
    if masks["HANDSHAKE_REQUIRED"] & masks["HELLO_OPTIONAL"]:
        raise RegistryError("required and optional capability masks overlap")
    for opcode in registries["rpc_opcodes"]:
        if opcode["mutating"] != bool(opcode["value"] & 0x8000):
            raise RegistryError(f"{opcode['name']}: mutating bit/opcode mismatch")
    opcode_values = {item["name"]: item["value"] for item in registries["rpc_opcodes"]}
    if opcode_values.get("GRAYSCALE_CAPTURE_START") != 0x8304:
        raise RegistryError("GRAYSCALE_CAPTURE_START must remain 0x8304")
    results = {item["name"]: item["value"] for item in registries["rpc_results"]}
    expected_results = {
        "OK": 0, "MALFORMED": 1, "BAD_FLAGS": 2, "BAD_LENGTH": 3,
        "UNSUPPORTED_TYPE": 4, "UNSUPPORTED_OPCODE": 5, "INCOMPATIBLE_REGISTRY": 6,
        "UNSUPPORTED_CAPABILITY": 7, "SESSION_REQUIRED": 8, "SESSION_MISMATCH": 9,
        "SEQUENCE_ZERO": 10, "SEQUENCE_GAP": 11, "SEQUENCE_CONFLICT": 12,
        "REPLAY_EXPIRED": 13, "SEQUENCE_EXHAUSTED": 14, "BUSY": 15,
        "SESSION_CLOSING": 16, "INVALID_ARGUMENT": 0x100, "UNKNOWN_PARAMETER": 0x101,
        "READ_ONLY": 0x102, "INVALID_VALUE": 0x103, "OUT_OF_RANGE": 0x104,
        "RELATION": 0x105, "STALE_ACTIVE_REVISION": 0x106,
        "STALE_STAGE_REVISION": 0x107, "STALE_GENERATION": 0x108,
        "WRONG_STATE": 0x109, "SAFETY_FAULT": 0x10A, "ACTION_ACTIVE": 0x10B,
        "NOT_ARMED": 0x10C, "ARM_EXPIRED": 0x10D, "SENSOR_NOT_READY": 0x10E,
        "UNSUPPORTED_RATE": 0x10F, "BANDWIDTH_EXCEEDED": 0x110, "NOT_FOUND": 0x111,
        "STORAGE_IO": 0x200, "STORAGE_VERIFY": 0x201, "STORAGE_SCHEMA": 0x202,
        "INTERNAL": 0x7FFF,
    }
    if results != expected_results:
        raise RegistryError("wire-v1 RPC result registry changed")
    for error in registries["protocol_errors"]:
        if results.get(error["name"]) != error["value"]:
            raise RegistryError(f"protocol error {error['name']} diverges from result registry")
    if [item["name"] for item in registries["protocol_errors"]] != [
        "MALFORMED", "BAD_FLAGS", "BAD_LENGTH", "UNSUPPORTED_TYPE"
    ]:
        raise RegistryError("PROTOCOL_ERROR legal result domain changed")
    if {item["name"]: item["value"] for item in registries["parameter_units"]} != {
        "NONE": 0,
        "TUNING_GAIN": 1,
        "RATIO": 2,
        "MM_PER_S": 3,
        "MM_PER_S2": 4,
        "MM_PER_S3": 5,
        "PWM": 6,
        "PWM_PER_MM_PER_S": 7,
        "DEGREE": 8,
        "BOOL": 9,
    }:
        raise RegistryError("wire-v1 parameter unit registry changed")
    if {item["name"]: item["value"] for item in registries["relation_groups"]} != {
        "NONE": 0,
        "LEFT_FF": 1,
        "RIGHT_FF": 2,
        "LINE_SPEED": 3,
        "REMOTE_SPEED": 4,
    }:
        raise RegistryError("wire-v1 parameter relation-group registry changed")
    if {item["name"]: item["value"] for item in registries["status_flags"]} != {
        "WHEEL_CONTROL_ENABLED": 1,
        "STAGE_DIRTY": 2,
        "SAFETY_ACTION_ACTIVE": 4,
        "HEADING_VALID": 8,
        "HEADING_CALIBRATED": 16,
        "HEADING_CALIBRATING": 32,
        "GRAYSCALE_PENDING": 64,
    }:
        raise RegistryError("wire-v1 STATUS flags changed")
    if {item["name"]: item["value"] for item in registries["link_flags"]} != {
        "SESSION_CLOSING": 1,
        "RX_OVERFLOW": 2,
    }:
        raise RegistryError("wire-v1 LINK_STATS flags changed")
    exact_named_registries = {
        "boolean_values": {"FALSE": 0, "TRUE": 1},
        "image_flags": {"TEST_ONLY": 1, "FAULT_INJECTION": 2},
        "param_summary_flags": {"DIRTY": 1, "PENDING_APPLY": 2, "SLOT_A_VALID": 4, "SLOT_B_VALID": 8},
        "reset_flags": {"COARSE": 1, "DETAIL": 2, "HARDFAULT": 4},
        "parameter_boundaries": {"IMMEDIATE": 0, "WHEEL_5MS": 1},
        "busy_resources": {
            "RPC_ENTRY": 1, "CONTROL_ERROR": 2, "BULK": 3, "PARAM_BOUNDARY": 4,
            "FLASH": 5, "ACTION": 6, "TX_DRAIN": 7, "ASYNC_EVENT": 8,
        },
        "action_sources": {"NONE": 0, "V1_UART": 1, "V2_RPC": 2, "PB11_IR": 3, "BOOT_SELF_TEST": 4, "INTERNAL": 5},
        "action_kinds": {"NONE": 0, "WHEEL_SPEED_TEST": 1, "HEADING_HOLD": 2, "HEADING_TURN": 3, "HEADING_RATE": 4, "LINE_FOLLOW": 5, "MOTOR_SELF_TEST": 6},
        "action_states": {"IDLE": 0, "ARMED": 1, "RUNNING": 2, "RAMP_DOWN": 3, "COMPLETE": 4, "REJECTED": 5, "FAULT": 6},
        "emergency_sources": {"UNKNOWN": 0, "UART_S": 1, "UART_SPACE": 2, "INTERNAL": 3},
        "gap_reasons": {"QUEUE_OVERWRITE": 1, "TOKEN_BUDGET": 2, "PRODUCER_DROP": 3, "EMERGENCY_FLUSH": 4, "SESSION_TEARDOWN": 5},
        "link_modes": {"V1": 0, "V2_DRAIN": 1, "V2_ARMED": 2, "V2_ESTABLISHED": 3, "V1_RESYNC": 4},
        "link_reasons": {"NONE": 0, "MODE_REQUEST": 1, "READY_SENT": 2, "HELLO_OK": 3, "PRE_ESTABLISHED_FAILURE": 4, "SESSION_CLOSE": 5, "EMERGENCY": 6, "SEQUENCE_EXHAUSTED": 7},
        "telemetry_streams": {"CORE": 1, "WHEEL": 2, "LINE": 3, "HEALTH": 4},
        "search_directions": {"NEGATIVE_SIDE": -1, "NONE": 0, "POSITIVE_SIDE": 1},
        "safety_sensor_bits": {"GYRO": 1, "GRAYSCALE": 2},
        "safety_fault_bits": {
            "EMERGENCY_STOP": 1, "GYRO_TIMEOUT": 2, "GRAYSCALE_TIMEOUT": 4,
            "ACTION_TIMEOUT": 8, "SCHEDULER_DEADLINE": 16, "WATCHDOG_RESET": 32,
            "ENCODER_INVALID": 64, "WHEEL_STALL": 128, "LINE_LOST": 256,
        },
        "scheduler_task_bits": {"TASK_5MS": 1, "TASK_10MS": 2, "TASK_20MS": 4, "TASK_100MS": 8},
        "adc_interrupt_flags": {"OVERFLOW": 1, "TRIG_OVF": 2, "UNDERFLOW": 0x40, "MEM0_LOADED": 0x100},
        "grayscale_channel_bits": {
            "CHANNEL_0": 1, "CHANNEL_1": 2, "CHANNEL_2": 4, "CHANNEL_3": 8,
            "CHANNEL_4": 16, "CHANNEL_5": 32, "CHANNEL_6": 64, "CHANNEL_7": 128,
        },
        "grayscale_state_flags": {"LAST_SCAN_OK": 1, "ADC_POWERED": 2, "ADC_ENABLED": 4, "ADC_STARTED": 8, "SCAN_PENDING": 16},
        "trace_state_flags": {"ACTIVE": 1, "TRIGGERED": 2, "FROZEN": 4},
        "trace_record_flags": {"BRANCH_LATCHED": 1},
    }
    for registry_name, expected in exact_named_registries.items():
        actual = {item["name"]: item["value"] for item in registries[registry_name]}
        if actual != expected:
            raise RegistryError(f"wire-v1 {registry_name} registry changed")
    for registry_name, entries in registries.items():
        if registry_name == "capability_masks":
            continue
        bit_entries = [entry for entry in entries if "bit" in entry]
        if bit_entries:
            if any(entry["value"] < 0 for entry in bit_entries):
                raise RegistryError(f"{registry_name}: bitmask values must be nonnegative")
            if len({entry["bit"] for entry in bit_entries}) != len(bit_entries):
                raise RegistryError(f"duplicate bit index in {registry_name}")
            if any(entry["value"] != 1 << entry["bit"] for entry in bit_entries):
                raise RegistryError(f"{registry_name}: bit/value mismatch")

    semantics = registry["semantics"]
    if semantics["pre_established_failure_target"] != "V1_RESYNC":
        raise RegistryError("pre-established failures must enter V1_RESYNC")
    if semantics["busy_consumes_sequence"] or semantics["busy_retry_sequence"] != "same-next":
        raise RegistryError("non-consuming BUSY sequence rule changed")
    if semantics["async_sequence_allocation"] != "tx-arbitration":
        raise RegistryError("async sequence allocation point changed")
    if semantics["active_v2_ascii_takeover"]:
        raise RegistryError("active V2 must never use ASCII takeover")
    if semantics["rpc_opcode_registry_binding"] != "decode-known-preserve-unknown-for-unsupported-opcode":
        raise RegistryError("RPC opcode registry binding/unknown echo semantics changed")

    _validate_semantic_contract(registry)
    _validate_layouts(registry)
    _validate_rpc_contract(registry)
    layouts_by_name = {layout["name"]: layout for layout in registry["layouts"]}
    fields_by_layout = {
        name: {field["name"]: field for field in layout["fields"]}
        for name, layout in layouts_by_name.items()
    }
    expected_field_registries = {
        ("rpc_request_common_v1", "opcode"): "rpc_opcodes",
        ("rpc_response_common_v1", "opcode"): "rpc_opcodes",
        ("rpc_status_get_success_body_v1", "required_sensor_mask"): "safety_sensor_bits",
        ("rpc_task_stats_get_success_body_v1", "scheduler_healthy"): "boolean_values",
        ("rpc_task_stats_get_success_body_v1", "pending_mask"): "scheduler_task_bits",
        ("rpc_task_stats_get_success_body_v1", "missed_mask"): "scheduler_task_bits",
        ("rpc_task_stats_get_success_body_v1", "health_mask"): "scheduler_task_bits",
        ("rpc_fault_clear_success_body_v1", "cleared_fault_bits"): "safety_fault_bits",
        ("rpc_param_stage_set_success_body_v1", "dirty"): "boolean_values",
        ("rpc_param_revert_success_body_v1", "dirty"): "boolean_values",
        ("rpc_param_load_success_body_v1", "dirty"): "boolean_values",
        ("rpc_param_defaults_success_body_v1", "dirty"): "boolean_values",
        ("rpc_subscription_set_request_body_v1", "replace"): "boolean_values",
        ("event_reset_body_v1", "previous_fault_bits"): "safety_fault_bits",
        ("event_reset_body_v1", "previous_health_mask"): "scheduler_task_bits",
        ("event_safety_fault_body_v1", "required_sensor_mask"): "safety_sensor_bits",
        ("event_grayscale_ready_body_v1", "last_adc_flags"): "adc_interrupt_flags",
        ("telemetry_line_body_v1", "active_mask"): "grayscale_channel_bits",
        ("telemetry_line_body_v1", "search_direction"): "search_directions",
        ("line_trace_record_v1", "active_mask"): "grayscale_channel_bits",
        ("line_trace_record_v1", "search_direction"): "search_directions",
        ("reset_snapshot_v1", "previous_fault_bits"): "safety_fault_bits",
        ("reset_snapshot_v1", "previous_pending_mask"): "scheduler_task_bits",
        ("reset_snapshot_v1", "previous_missed_mask"): "scheduler_task_bits",
        ("reset_snapshot_v1", "previous_health_mask"): "scheduler_task_bits",
    }
    for (layout_name, field_name), expected_registry in expected_field_registries.items():
        if fields_by_layout[layout_name][field_name].get("registry") != expected_registry:
            raise RegistryError(f"{layout_name}.{field_name}: registry binding changed")
    if "control_drop" in fields_by_layout["rpc_link_stats_get_success_body_v1"]:
        raise RegistryError("LINK_STATS must expose control_error_drop, not control_drop")
    replay_used = fields_by_layout["rpc_link_stats_get_success_body_v1"].get("replay_used")
    if replay_used is None or replay_used["kind"] != "u8":
        raise RegistryError("LINK_STATS replay_used entry-count field changed")
    value_flags = fields_by_layout["rpc_param_value_get_success_body_v1"].get("reserved_flags")
    if value_flags is None or not value_flags.get("reserved"):
        raise RegistryError("PARAM_VALUE_GET byte 24 must remain reserved_flags=0")
    for layout_name in ("rpc_param_apply_success_body_v1", "event_param_committed_body_v1"):
        change_mask = fields_by_layout[layout_name]["change_mask"]
        if (change_mask["offset"], change_mask["size"], change_mask["kind"]) != (8, 8, "u64"):
            raise RegistryError(f"{layout_name}: 35-parameter change_mask must remain u64")
    wheel_field_names = [field["name"] for field in layouts_by_name["telemetry_wheel_body_v1"]["fields"]]
    for name in ("p_pwm_l", "p_pwm_r", "i_pwm_l", "i_pwm_r", "ff_pwm_l", "ff_pwm_r"):
        if name not in wheel_field_names:
            raise RegistryError("WHEEL PI/feedforward PWM field names changed")
    trace_fields = fields_by_layout["line_trace_record_v1"]
    if "position_x1000" not in trace_fields or "correction_mm_s" not in trace_fields:
        raise RegistryError("line-trace position/correction units are no longer self-describing")
    for event in registries["event_codes"]:
        layout = layouts_by_name.get(event.get("body_layout"))
        if layout is None or layout["size"] != event.get("body_size"):
            raise RegistryError(f"{event['name']}: event body layout/size mismatch")
        if event["priority"] not in ("P0", "P1"):
            raise RegistryError(f"{event['name']}: frozen event must be P0/P1")
        if event["schema"] != 1:
            raise RegistryError(f"{event['name']}: event schema must remain 1")
    grayscale = next(item for item in registries["event_codes"] if item["name"] == "GRAYSCALE_READY")
    if grayscale["priority"] != "P1":
        raise RegistryError("GRAYSCALE_READY must reserve a P1 event slot")

    parameters = registry["parameters"]
    if [entry["id"] for entry in parameters] != list(range(1, 36)):
        raise RegistryError("parameter IDs must be exactly 1..35")
    keys = [entry["key"] for entry in parameters]
    if len(keys) != len(set(keys)):
        raise RegistryError("duplicate parameter key")
    for parameter in parameters:
        default = int(parameter["default"]) if isinstance(parameter["default"], bool) else parameter["default"]
        if not parameter["minimum"] <= default <= parameter["maximum"]:
            raise RegistryError(f"{parameter['key']}: default outside range")
        has_relation = bool(parameter["relations"])
        if ("RELATION" in parameter["flags"]) != has_relation:
            raise RegistryError(f"{parameter['key']}: relation flag/list mismatch")
        if parameter["type"] == "BOOL" and parameter["domain_rule"] != "BOOL":
            raise RegistryError(f"{parameter['key']}: BOOL parameter needs BOOL domain")
    expected_units = [
        "TUNING_GAIN", "TUNING_GAIN", "TUNING_GAIN", "TUNING_GAIN", "RATIO",
        "MM_PER_S2", "MM_PER_S2", "MM_PER_S3", "PWM", "PWM_PER_MM_PER_S",
        "PWM", "PWM_PER_MM_PER_S", "MM_PER_S", "TUNING_GAIN", "TUNING_GAIN",
        "TUNING_GAIN", "TUNING_GAIN", "RATIO", "MM_PER_S", "MM_PER_S", "DEGREE",
        "MM_PER_S", "MM_PER_S", "RATIO", "RATIO", "TUNING_GAIN", "TUNING_GAIN",
        "MM_PER_S", "MM_PER_S2", "RATIO", "MM_PER_S", "MM_PER_S", "MM_PER_S",
        "MM_PER_S", "BOOL",
    ]
    expected_relation_groups = ["NONE"] * 35
    for parameter_id in (9, 10):
        expected_relation_groups[parameter_id - 1] = "LEFT_FF"
    for parameter_id in (11, 12):
        expected_relation_groups[parameter_id - 1] = "RIGHT_FF"
    for parameter_id in (22, 23):
        expected_relation_groups[parameter_id - 1] = "LINE_SPEED"
    for parameter_id in (32, 33, 34):
        expected_relation_groups[parameter_id - 1] = "REMOTE_SPEED"
    if [parameter["unit"] for parameter in parameters] != expected_units:
        raise RegistryError("wire-v1 parameter unit assignments changed")
    if [parameter["relation_group"] for parameter in parameters] != expected_relation_groups:
        raise RegistryError("wire-v1 parameter relation-group assignments changed")
    for parameter in parameters:
        if (parameter["relation_group"] != "NONE") != bool(parameter["relations"]):
            raise RegistryError(f"{parameter['key']}: relation group/list mismatch")
    diagnostic_ids = {entry["id"] for entry in parameters if "DIAGNOSTIC_BOUNDARY" in entry["flags"]}
    if diagnostic_ids != {1, 2, 3, 4, 13}:
        raise RegistryError("diagnostic-boundary parameter set changed")

    telemetry = registry["telemetry"]
    _assert_unique(telemetry["streams"], "telemetry stream")
    expected_streams = [
        {"name": "CORE", "value": 1, "schema": 1, "body_layout": "telemetry_core_body_v1", "body_size": 32, "payload_size": 40, "period_min_ms": 20, "period_max_ms": 1000, "period_quantum_ms": 20},
        {"name": "WHEEL", "value": 2, "schema": 1, "body_layout": "telemetry_wheel_body_v1", "body_size": 52, "payload_size": 60, "period_min_ms": 20, "period_max_ms": 1000, "period_quantum_ms": 20},
        {"name": "LINE", "value": 3, "schema": 1, "body_layout": "telemetry_line_body_v1", "body_size": 48, "payload_size": 56, "period_min_ms": 40, "period_max_ms": 1000, "period_quantum_ms": 20},
        {"name": "HEALTH", "value": 4, "schema": 1, "body_layout": "telemetry_health_body_v1", "body_size": 64, "payload_size": 72, "period_min_ms": 200, "period_max_ms": 5000, "period_quantum_ms": 100},
    ]
    if telemetry["streams"] != expected_streams:
        raise RegistryError("wire-v1 telemetry stream profiles changed")
    stream_registry = {item["name"]: item["value"] for item in registries["telemetry_streams"]}
    if {item["name"]: item["value"] for item in telemetry["streams"]} != stream_registry:
        raise RegistryError("telemetry stream registry/contract diverged")
    for stream in telemetry["streams"]:
        body_layout = layouts_by_name.get(stream["body_layout"])
        if body_layout is None or body_layout["size"] != stream["body_size"]:
            raise RegistryError(f"{stream['name']}: telemetry body layout/size mismatch")
        if stream["payload_size"] != telemetry["common_size"] + stream["body_size"]:
            raise RegistryError(f"{stream['name']}: telemetry payload size mismatch")
        if stream["payload_size"] > framing["max_payload"]:
            raise RegistryError(f"{stream['name']}: telemetry exceeds max payload")
        if stream["period_min_ms"] % stream["period_quantum_ms"]:
            raise RegistryError(f"{stream['name']}: minimum period is off quantum")
        if stream["period_max_ms"] % stream["period_quantum_ms"]:
            raise RegistryError(f"{stream['name']}: maximum period is off quantum")
    estimator = telemetry["admission_estimator"]
    def estimated_bps(stream: dict[str, Any], period_ms: int) -> int:
        return math.ceil(1000 / period_ms) * (
            estimator["delimiter_bytes"] + estimator["header_bytes"]
            + stream["payload_size"] + estimator["crc_bytes"]
        )
    stream_by_name = {item["name"]: item for item in telemetry["streams"]}
    if estimated_bps(stream_by_name["CORE"], 20) + estimated_bps(stream_by_name["WHEEL"], 20) != 6700:
        raise RegistryError("CORE+WHEEL 20 ms baseline must remain 6700 B/s")
    if estimator["max_bytes_per_second"] != 7000 or estimator["token_bytes_per_second"] != 7000 or estimator["token_bucket_bytes"] != 448:
        raise RegistryError("telemetry admission/token limits changed")
    subscription_semantics = {
        "sample_id_initial": 1,
        "sample_id_scope": "session-per-stream",
        "sample_id_increment": "actual-subscribed-sample",
        "reset_on_disable_enable": False,
        "disabled_streams_sample": False,
        "snapshot_affects_sample_id": False,
        "subscription_change_clears_affected_p2_before_response": True,
        "subscription_change_preserves_next_sample_id": True,
        "subscription_replace_required": True,
        "subscription_count_min": 0,
        "subscription_count_max": 4,
        "subscription_unknown_stream_result": "INVALID_ARGUMENT",
        "subscription_duplicate_stream_result": "INVALID_ARGUMENT",
        "subscription_success_order": "stream-id-ascending",
        "subscription_entry_flags_required": 0,
    }
    for key, expected in subscription_semantics.items():
        if telemetry.get(key) != expected:
            raise RegistryError(f"subscription/sample-id semantic changed: {key}")
    bulk_layout = layouts_by_name["bulk_common_v1"]
    trace_layout = layouts_by_name["line_trace_record_v1"]
    bulk_semantics = semantics["bulk_transfer"]
    if framing["max_payload"] - bulk_layout["size"] != bulk_semantics["data_max_bytes"]:
        raise RegistryError("BULK data capacity equation changed")
    if trace_layout["size"] != bulk_semantics["record_size"] or bulk_semantics["count_max"] * trace_layout["size"] > bulk_semantics["data_max_bytes"]:
        raise RegistryError("LINE_TRACE BULK record/count equation changed")

    resources = registry["resources"]
    exact_resources = {
        "new_static_ram_limit": 2560,
        "p0_latches": 216,
        "rpc_replay_result_entries": 608,
        "p1_async": 288,
        "control_error": 32,
        "telemetry_slots": 320,
        "bulk_slots": 224,
        "wire_and_decoded": 336,
        "other_limit": 536,
    }
    if resources != exact_resources:
        raise RegistryError("wire-v1 static RAM resource table changed")
    parts = [value for key, value in resources.items() if key != "new_static_ram_limit"]
    if sum(parts) > resources["new_static_ram_limit"]:
        raise RegistryError("resource budget exceeds hard limit")
    if resources["wire_and_decoded"] != framing["max_wire_frame"] + 112:
        raise RegistryError("wire/decoded resource budget mismatch")
    if resources["rpc_replay_result_entries"] != semantics["sequence"]["rpc_entries"] * semantics["sequence"]["rpc_entry_max_bytes"]:
        raise RegistryError("combined RPC entry RAM equation changed")
    if registry["artifact"] != {
        "name": "m6-v2-wire-v1", "wire_version": 2, "registry_revision": 1, "status": "frozen",
    }:
        raise RegistryError("wire-v1 frozen artifact identity changed")
    digest = canonical_digest(registry)
    if digest != FROZEN_REGISTRY_SHA256:
        raise RegistryError(
            f"frozen wire-v1 canonical SHA256 changed: {digest}; expected {FROZEN_REGISTRY_SHA256}"
        )


def _validate_crc_escape_session_vectors(registry: dict[str, Any]) -> None:
    vectors = registry["vectors"]
    for vector in vectors["crc16"]:
        actual = crc16_ccitt_false(parse_hex(vector["input_hex"]))
        if actual != vector["expected"]:
            raise RegistryError(f"{vector['id']}: CRC16 expected {vector['expected']:#06x}, got {actual:#06x}")
    for vector in vectors["escape"]:
        actual = escape_bytes(parse_hex(vector["input_hex"]), registry["framing"])
        if actual != parse_hex(vector["expected_hex"]):
            raise RegistryError(f"{vector['id']}: escape bytes mismatch")
    for vector in vectors["session"]:
        actual_input = session_input(vector, registry["session"])
        if actual_input != parse_hex(vector["input_hex"]):
            raise RegistryError(f"{vector['id']}: session input mismatch")
        raw_crc = binascii.crc32(actual_input) & 0xFFFFFFFF
        if raw_crc != vector["raw_crc32"]:
            raise RegistryError(f"{vector['id']}: session CRC32 mismatch")
        mapped = raw_crc or registry["session"]["zero_maps_to"]
        if mapped != vector["expected_session"]:
            raise RegistryError(f"{vector['id']}: mapped session mismatch")
    handshake = registry["semantics"]["handshake"]
    legacy = vectors["legacy_ascii"]
    if len(legacy) != 1:
        raise RegistryError("wire-v1 requires exactly one legacy MODE/READY sample")
    sample = legacy[0]
    challenge = sample["challenge_hex"]
    if len(challenge) != handshake["challenge_hex_width"] or challenge != challenge.lower() or int(challenge, 16) == 0:
        raise RegistryError("legacy MODE challenge encoding changed")
    expected_mode = handshake["mode_request_template"].format(
        transaction_id=sample["transaction_id"], challenge_hex16=challenge,
    )
    expected_ready = handshake["v1_ready_template"].format(
        transaction_id=sample["transaction_id"], challenge_hex16=challenge,
    )
    if sample["transaction_id"] != 42 or sample["mode_ascii"] != expected_mode or sample["ready_ascii"] != expected_ready:
        raise RegistryError("legacy MODE/READY sample does not match frozen templates")
    if sample["mode_ascii"].encode("ascii") != parse_hex(sample["mode_hex"]):
        raise RegistryError("legacy MODE ASCII/hex mismatch")
    if sample["ready_ascii"].encode("ascii") != parse_hex(sample["ready_hex"]):
        raise RegistryError("legacy READY ASCII/hex mismatch")
    resync = registry["semantics"]["resync"]
    samples = vectors["resync"]
    if len(samples) != 1:
        raise RegistryError("wire-v1 requires exactly one resync sample")
    sample = samples[0]
    all_defined = next(
        item["value_hex"] for item in registry["registries"]["capability_masks"]
        if item["name"] == "ALL_DEFINED"
    )
    expected_reply = resync["reply_ascii_template"].format(capabilities_hex16=all_defined)
    if sample["transaction_id"] != resync["transaction_id"] or sample["probe_ascii"] != resync["probe_ascii"]:
        raise RegistryError("resync probe transaction changed")
    if sample["reply_ascii"] != expected_reply:
        raise RegistryError("resync capability reply mask changed")
    if sample["probe_ascii"].encode("ascii") != parse_hex(sample["probe_hex"]):
        raise RegistryError("resync probe ASCII/hex mismatch")
    if sample["reply_ascii"].encode("ascii") != parse_hex(sample["reply_hex"]):
        raise RegistryError("resync reply ASCII/hex mismatch")


def _validate_frame_vectors(registry: dict[str, Any]) -> dict[str, bytes]:
    frame_types = {entry["name"]: entry for entry in registry["registries"]["frame_types"]}
    known_flags = 0
    for flag in registry["registries"]["flags"]:
        known_flags |= flag["value"]
    decoded_by_id: dict[str, bytes] = {}
    wire_by_id: dict[str, bytes] = {}
    vectors_by_id: dict[str, dict[str, Any]] = {}
    for vector in registry["vectors"]["frames"]:
        if vector["id"] in decoded_by_id:
            raise RegistryError(f"duplicate frame vector id: {vector['id']}")
        actual = build_frame(vector, registry)
        if actual != vector["expected"]:
            for key in actual:
                if actual[key] != vector["expected"].get(key):
                    raise RegistryError(f"{vector['id']}: expected {key} drift")
            raise RegistryError(f"{vector['id']}: frame drift")
        if vector["flags"] & ~known_flags:
            raise RegistryError(f"{vector['id']}: unknown flags")
        if vector["type"] in frame_types:
            allowed = frame_types[vector["type"]]["allowed_flags"]
            if vector["flags"] & ~allowed:
                raise RegistryError(f"{vector['id']}: flag illegal for {vector['type']}")
        elif vector["flags"]:
            raise RegistryError(f"{vector['id']}: unregistered type must use flags=0")
        decoded = parse_hex(actual["decoded_hex"])
        wire = parse_hex(actual["wire_hex"])
        if len(decoded) > registry["framing"]["max_decoded_frame"]:
            raise RegistryError(f"{vector['id']}: decoded frame too large")
        if len(wire) > registry["framing"]["max_wire_frame"]:
            raise RegistryError(f"{vector['id']}: wire frame too large")
        interior = wire[1:-1]
        if 0x20 in interior or 0x53 in interior or 0x7E in interior:
            raise RegistryError(f"{vector['id']}: naked emergency/delimiter byte inside wire frame")
        index = 0
        allowed_followers = {byte ^ registry["framing"]["escape_xor"] for byte in registry["framing"]["must_escape"]}
        while index < len(interior):
            if interior[index] == registry["framing"]["escape"]:
                if index + 1 >= len(interior) or interior[index + 1] not in allowed_followers:
                    raise RegistryError(f"{vector['id']}: non-canonical escape")
                index += 2
            else:
                index += 1
        decoded_by_id[vector["id"]] = decoded
        wire_by_id[vector["id"]] = wire
        vectors_by_id[vector["id"]] = vector

    required = {
        "hello_request", "ready", "ready_bad_host_payload_64", "ping_request", "ping_response",
        "ping_response_replay", "unknown_type", "unsupported_type_error",
        "core_telemetry_gap", "max_payload_bulk_gap",
    }
    if set(decoded_by_id) != required:
        raise RegistryError("golden frame vector set is incomplete or unexpected")
    framing_only = registry["vectors"]["framing_only_frame_ids"]
    if framing_only != ["max_payload_bulk_gap"]:
        raise RegistryError("framing-only vector registry changed")
    if len(parse_hex(vectors_by_id["max_payload_bulk_gap"]["payload_hex"])) != registry["framing"]["max_payload"]:
        raise RegistryError("maximum framing-only payload is not 96 bytes")

    masks = {
        entry["name"]: int(entry["value_hex"], 16)
        for entry in registry["registries"]["capability_masks"]
    }
    nominal_session = next(item for item in registry["vectors"]["session"] if item["id"] == "nominal")
    hello = vectors_by_id["hello_request"]
    hello_fields = struct.unpack("<4sHHQQQHHB3s", parse_hex(hello["payload_hex"]))
    if hello["session"] != 0 or hello["sequence"] != 0 or hello_fields != (
        b"M6V2", 1, 1, int(nominal_session["challenge_hex"], 16),
        masks["HANDSHAKE_REQUIRED"], masks["HELLO_OPTIONAL"],
        registry["framing"]["max_payload"], registry["framing"]["max_rpc_request_payload"],
        registry["framing"]["control_window"], b"\0\0\0",
    ):
        raise RegistryError("HELLO semantic vector mismatch")

    ready = vectors_by_id["ready"]
    ready_fields = struct.unpack("<4sHHIQIIIIH2sQQHHB3s", parse_hex(ready["payload_hex"]))
    expected_ready = (
        b"R2DY", 1, 0, 0, int(nominal_session["challenge_hex"], 16),
        nominal_session["expected_session"], nominal_session["boot_id"],
        nominal_session["raw_reset_cause"], nominal_session["session_generation"],
        nominal_session["parameter_schema"], b"\0\0", masks["ALL_DEFINED"],
        masks["ALL_DEFINED"], registry["framing"]["max_payload"],
        registry["framing"]["max_rpc_request_payload"],
        registry["framing"]["control_window"], b"\0\0\0",
    )
    if ready["session"] != 0 or ready["sequence"] != 0 or ready_fields != expected_ready:
        raise RegistryError("READY semantic vector mismatch")
    failed_ready = vectors_by_id["ready_bad_host_payload_64"]
    failed_fields = struct.unpack("<4sHHIQIIIIH2sQQHHB3s", parse_hex(failed_ready["payload_hex"]))
    expected_failed = (
        b"R2DY", 0, 3, (96 << 16) | 64, int(nominal_session["challenge_hex"], 16),
        0, nominal_session["boot_id"], nominal_session["raw_reset_cause"],
        nominal_session["session_generation"], nominal_session["parameter_schema"], b"\0\0",
        masks["ALL_DEFINED"], 0, 0, 0, 0, b"\0\0\0",
    )
    if failed_ready["session"] != 0 or failed_ready["sequence"] != 0 or failed_fields != expected_failed:
        raise RegistryError("failure READY zero/diagnostic field vector mismatch")

    opcodes = {item["name"]: item["value"] for item in registry["registries"]["rpc_opcodes"]}
    ping_request = vectors_by_id["ping_request"]
    ping_opcode, ping_cookie = struct.unpack("<HI", parse_hex(ping_request["payload_hex"]))
    if ping_opcode != opcodes["PING"] or ping_cookie != 0x7D53207E:
        raise RegistryError("PING request semantic vector mismatch")
    ping_response = vectors_by_id["ping_response"]
    response_fields = struct.unpack("<HHIIII", parse_hex(ping_response["payload_hex"]))
    if response_fields != (opcodes["PING"], 0, 0, ping_cookie, 0x20537D7E, nominal_session["boot_id"]):
        raise RegistryError("PING response semantic vector mismatch")
    replay = vectors_by_id["ping_response_replay"]
    if replay["payload_hex"] != ping_response["payload_hex"] or replay["flags"] != 1:
        raise RegistryError("PING replay must preserve payload and set REPLAY only")
    protocol_fields = struct.unpack("<HBBI", parse_hex(vectors_by_id["unsupported_type_error"]["payload_hex"]))
    if protocol_fields != (4, 0x7F, 0, 0x7F):
        raise RegistryError("UNSUPPORTED_TYPE protocol-error vector mismatch")
    telemetry = parse_hex(vectors_by_id["core_telemetry_gap"]["payload_hex"])
    if len(telemetry) != 40 or struct.unpack_from("<BBHI", telemetry) != (1, 1, 0, 1):
        raise RegistryError("generic CORE telemetry prefix mismatch")
    return wire_by_id


COUNTER_NAMES = (
    "crc_error", "short_frame", "length_error", "escape_error",
    "decoded_overflow", "wire_overflow", "interbyte_timeout",
    "total_timeout", "epoch_abort",
)


class ReferenceParser:
    def __init__(self, registry: dict[str, Any], decoded_names: dict[bytes, str]):
        self.registry = registry
        self.framing = registry["framing"]
        self.config = registry["parser"]
        self.decoded_names = decoded_names
        self.counters = {name: 0 for name in COUNTER_NAMES}
        self.frames: list[str] = []
        self.state = "HUNT"
        self.decoded = bytearray()
        self.wire_count = 0
        self.started = False
        self.start_ms = 0
        self.last_ms = 0

    @staticmethod
    def elapsed(now: int, then: int) -> int:
        return (now - then) & 0xFFFFFFFF

    def _empty_collect(self) -> None:
        self.state = "COLLECT"
        self.decoded.clear()
        self.wire_count = 1
        self.started = False

    def _hunt(self) -> None:
        self.state = "HUNT"
        self.decoded.clear()
        self.wire_count = 0
        self.started = False

    def _expire(self, now: int) -> None:
        if self.state not in ("COLLECT", "ESCAPED", "DISCARD") or not self.started:
            return
        if self.elapsed(now, self.start_ms) >= self.config["total_timeout_ms"]:
            self.counters["total_timeout"] += 1
            self._hunt()
        elif self.elapsed(now, self.last_ms) >= self.config["interbyte_timeout_ms"]:
            self.counters["interbyte_timeout"] += 1
            self._hunt()

    def tick(self, now: int) -> None:
        self._expire(now)

    def epoch(self) -> None:
        if self.state != "HUNT" and (self.started or self.state in ("ESCAPED", "DISCARD")):
            self.counters["epoch_abort"] += 1
        self._hunt()

    def _append_decoded(self, byte: int) -> None:
        if len(self.decoded) >= self.framing["max_decoded_frame"]:
            self.counters["decoded_overflow"] += 1
            self.state = "DISCARD"
            return
        self.decoded.append(byte)
        self.state = "COLLECT"

    def _finalize(self) -> None:
        decoded = bytes(self.decoded)
        minimum = self.framing["header_size"] + self.framing["crc_size"]
        if len(decoded) < minimum:
            self.counters["short_frame"] += 1
            return
        payload_length = struct.unpack_from("<H", decoded, 11)[0]
        expected_length = self.framing["header_size"] + payload_length + self.framing["crc_size"]
        if payload_length > self.framing["max_payload"] or len(decoded) != expected_length:
            self.counters["length_error"] += 1
            return
        expected_crc = crc16_ccitt_false(decoded[:-2])
        received_crc = struct.unpack_from("<H", decoded, len(decoded) - 2)[0]
        if expected_crc != received_crc:
            self.counters["crc_error"] += 1
            return
        name = self.decoded_names.get(decoded)
        if name is None:
            raise RegistryError("stream emitted a valid frame not named by a golden vector")
        self.frames.append(name)

    def feed_byte(self, byte: int, now: int) -> None:
        self._expire(now)
        delimiter = self.framing["delimiter"]
        if byte == delimiter:
            if self.state == "COLLECT" and self.started:
                self._finalize()
            elif self.state == "ESCAPED":
                self.counters["escape_error"] += 1
            self._empty_collect()
            return
        if self.state == "HUNT":
            return
        if self.state == "DISCARD":
            self.last_ms = now
            return

        # Count both delimiters in max_wire_frame. Before accepting an interior
        # byte there must still be room for the closing delimiter.
        if self.wire_count + 2 > self.framing["max_wire_frame"]:
            self.counters["wire_overflow"] += 1
            self.state = "DISCARD"
            self.last_ms = now
            return
        if not self.started:
            self.started = True
            self.start_ms = now
        self.last_ms = now
        self.wire_count += 1

        if self.state == "COLLECT":
            if byte == self.framing["escape"]:
                self.state = "ESCAPED"
            else:
                self._append_decoded(byte)
            return

        allowed = {value ^ self.framing["escape_xor"] for value in self.framing["must_escape"]}
        if byte not in allowed:
            self.counters["escape_error"] += 1
            self.state = "DISCARD"
            return
        self._append_decoded(byte ^ self.framing["escape_xor"])

    def feed(self, data: bytes, now: int) -> None:
        for byte in data:
            self.feed_byte(byte, now)


def _validate_stream_vectors(registry: dict[str, Any], wires: dict[str, bytes]) -> None:
    decoded_names = {
        parse_hex(vector["expected"]["decoded_hex"]): vector["id"]
        for vector in registry["vectors"]["frames"]
    }
    stream_ids: set[str] = set()
    for vector in registry["vectors"]["streams"]:
        if vector["id"] in stream_ids:
            raise RegistryError(f"duplicate stream id: {vector['id']}")
        stream_ids.add(vector["id"])
        parser = ReferenceParser(registry, decoded_names)
        for step in vector["steps"]:
            operation = step["op"]
            now = step["at_ms"]
            if operation == "tick":
                parser.tick(now)
            elif operation in ("overflow_epoch", "emergency_epoch"):
                parser.epoch()
            elif operation == "feed_repeat":
                data = parse_hex(step["repeat_hex"]) * step["repeat"]
                parser.feed(data, now)
            elif operation == "feed":
                has_hex = "hex" in step
                has_frame = "frame" in step
                if has_hex == has_frame:
                    raise RegistryError(f"{vector['id']}: feed needs exactly one of hex/frame")
                data = parse_hex(step["hex"]) if has_hex else wires[step["frame"]]
                start = step.get("start", 0)
                end = step.get("end", len(data))
                if start > end or end > len(data):
                    raise RegistryError(f"{vector['id']}: invalid frame slice")
                parser.feed(data[start:end], now)
            else:  # pragma: no cover - schema prevents this
                raise RegistryError(f"unknown stream operation: {operation}")
        expected = vector["expected"]
        if parser.frames != expected["frames"]:
            raise RegistryError(f"{vector['id']}: frames {parser.frames}, expected {expected['frames']}")
        if parser.state != expected["state"]:
            raise RegistryError(f"{vector['id']}: state {parser.state}, expected {expected['state']}")
        if parser.counters != expected["counters"]:
            raise RegistryError(f"{vector['id']}: counters {parser.counters}, expected {expected['counters']}")


def canonical_digest(registry: dict[str, Any]) -> str:
    payload = json.dumps(registry, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def _validate_frozen_schema(schema: dict[str, Any]) -> None:
    digest = canonical_digest(schema)
    if digest != FROZEN_SCHEMA_SHA256:
        raise RegistryError(
            f"frozen wire-v1 schema canonical SHA256 changed: {digest}; expected {FROZEN_SCHEMA_SHA256}"
        )


def _c_array(name: str, data: bytes) -> list[str]:
    lines = [f"static const unsigned char {name}[] = {{"]
    if data:
        for index in range(0, len(data), 12):
            chunk = ", ".join(f"0x{byte:02X}" for byte in data[index:index + 12])
            lines.append(f"    {chunk},")
    else:
        lines.append("    0x00, /* portable storage sentinel; logical length is zero */")
    lines.append("};")
    return lines


def render_inc(registry: dict[str, Any]) -> str:
    lines = [
        "/* Generated by scripts/check_m6_v2_vectors.py; do not edit. */",
        f"/* registry-sha256: {canonical_digest(registry)} */",
        "#ifndef M6_V2_WIRE_VECTORS_INC",
        "#define M6_V2_WIRE_VECTORS_INC",
        "",
    ]
    for vector in registry["vectors"]["crc16"]:
        prefix = f"m6_v2_crc_{vector['id']}"
        data = parse_hex(vector["input_hex"])
        lines.extend(_c_array(prefix + "_input", data))
        lines.append(f"#define {prefix.upper()}_INPUT_LENGTH {len(data)}U")
        lines.append(f"#define {prefix.upper()}_EXPECTED 0x{vector['expected']:04X}U")
        lines.append("")
    for vector in registry["vectors"]["escape"]:
        prefix = f"m6_v2_escape_{vector['id']}"
        input_bytes = parse_hex(vector["input_hex"])
        expected = parse_hex(vector["expected_hex"])
        lines.extend(_c_array(prefix + "_input", input_bytes))
        lines.append(f"#define {prefix.upper()}_INPUT_LENGTH {len(input_bytes)}U")
        lines.append("")
        lines.extend(_c_array(prefix + "_expected", expected))
        lines.append(f"#define {prefix.upper()}_EXPECTED_LENGTH {len(expected)}U")
        lines.append("")
    for vector in registry["vectors"]["session"]:
        prefix = f"m6_v2_session_{vector['id']}"
        input_bytes = parse_hex(vector["input_hex"])
        lines.extend(_c_array(prefix + "_input", input_bytes))
        lines.append(f"#define {prefix.upper()}_INPUT_LENGTH {len(input_bytes)}U")
        lines.append(f"#define {prefix.upper()}_RAW_CRC32 0x{vector['raw_crc32']:08X}UL")
        lines.append(f"#define {prefix.upper()}_EXPECTED_SESSION 0x{vector['expected_session']:08X}UL")
        lines.append("")
    for vector in registry["vectors"]["legacy_ascii"]:
        prefix = f"m6_v2_legacy_{vector['id']}"
        mode = parse_hex(vector["mode_hex"])
        ready = parse_hex(vector["ready_hex"])
        lines.extend(_c_array(prefix + "_mode", mode))
        lines.append(f"#define {prefix.upper()}_MODE_LENGTH {len(mode)}U")
        lines.append("")
        lines.extend(_c_array(prefix + "_ready", ready))
        lines.append(f"#define {prefix.upper()}_READY_LENGTH {len(ready)}U")
        lines.append("")
    for vector in registry["vectors"]["resync"]:
        prefix = f"m6_v2_resync_{vector['id']}"
        probe = parse_hex(vector["probe_hex"])
        reply = parse_hex(vector["reply_hex"])
        lines.extend(_c_array(prefix + "_probe", probe))
        lines.append(f"#define {prefix.upper()}_PROBE_LENGTH {len(probe)}U")
        lines.append("")
        lines.extend(_c_array(prefix + "_reply", reply))
        lines.append(f"#define {prefix.upper()}_REPLY_LENGTH {len(reply)}U")
        lines.append("")
    for vector in registry["vectors"]["frames"]:
        prefix = f"m6_v2_{vector['id']}"
        decoded = parse_hex(vector["expected"]["decoded_hex"])
        wire = parse_hex(vector["expected"]["wire_hex"])
        lines.extend(_c_array(prefix + "_decoded", decoded))
        lines.append(f"#define {prefix.upper()}_DECODED_LENGTH {len(decoded)}U")
        lines.append("")
        lines.extend(_c_array(prefix + "_wire", wire))
        lines.append(f"#define {prefix.upper()}_WIRE_LENGTH {len(wire)}U")
        lines.append("")
    lines.extend(["#endif", ""])
    return "\n".join(lines)


def validate_repository_files(
    registry_path: Path = DEFAULT_REGISTRY,
    schema_path: Path = DEFAULT_SCHEMA,
    inc_path: Path = DEFAULT_INC,
    check_inc: bool = True,
) -> dict[str, int]:
    schema = strict_load(schema_path)
    registry = strict_load(registry_path)
    _validate_frozen_schema(schema)
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(registry)
    _validate_registry_invariants(registry)
    _validate_crc_escape_session_vectors(registry)
    wires = _validate_frame_vectors(registry)
    _validate_stream_vectors(registry, wires)
    _validate_c_wire_registry(registry)
    if check_inc:
        expected_inc = render_inc(registry)
        try:
            actual_inc = inc_path.read_text(encoding="utf-8")
        except FileNotFoundError as exc:
            raise RegistryError(f"generated include missing: {inc_path}") from exc
        if actual_inc != expected_inc:
            raise RegistryError(f"generated include is stale: {inc_path}")
    return {
        "crc_vectors": len(registry["vectors"]["crc16"]),
        "escape_vectors": len(registry["vectors"]["escape"]),
        "session_vectors": len(registry["vectors"]["session"]),
        "legacy_ascii_vectors": len(registry["vectors"]["legacy_ascii"]),
        "resync_vectors": len(registry["vectors"]["resync"]),
        "frame_vectors": len(registry["vectors"]["frames"]),
        "stream_vectors": len(registry["vectors"]["streams"]),
    }


def _parse_args(argv: Iterable[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--inc", type=Path, default=DEFAULT_INC)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--check", action="store_true", help="validate and reject a stale generated include")
    action.add_argument("--print-inc", action="store_true", help="print the deterministic include; do not write files")
    action.add_argument("--write-inc", action="store_true", help="explicitly update the generated include")
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        schema = strict_load(args.schema)
        registry = strict_load(args.registry)
        _validate_frozen_schema(schema)
        jsonschema.Draft202012Validator.check_schema(schema)
        jsonschema.Draft202012Validator(schema).validate(registry)
        _validate_registry_invariants(registry)
        _validate_crc_escape_session_vectors(registry)
        wires = _validate_frame_vectors(registry)
        _validate_stream_vectors(registry, wires)
        _validate_c_wire_registry(registry)
        rendered = render_inc(registry)
        if args.print_inc:
            sys.stdout.write(rendered)
        elif args.write_inc:
            args.inc.parent.mkdir(parents=True, exist_ok=True)
            args.inc.write_text(rendered, encoding="utf-8", newline="\n")
            print(f"updated {args.inc}")
        else:
            if not args.inc.exists() or args.inc.read_text(encoding="utf-8") != rendered:
                raise RegistryError(f"generated include is missing or stale: {args.inc}")
            counts = validate_repository_files(args.registry, args.schema, args.inc, check_inc=False)
            print(
                "M6 V2 registry OK: "
                f"{counts['crc_vectors']} CRC, {counts['escape_vectors']} escape, "
                f"{counts['session_vectors']} session, {counts['legacy_ascii_vectors']} legacy ASCII, "
                f"{counts['resync_vectors']} resync, "
                f"{counts['frame_vectors']} frame, {counts['stream_vectors']} stream vectors"
            )
    except (RegistryError, jsonschema.ValidationError, jsonschema.SchemaError) as exc:
        print(f"M6 V2 registry check failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
