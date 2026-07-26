"""Read-only access to the frozen ``m6-v2-wire-v1`` JSON registry.

The JSON under ``Tests/vectors`` remains the only numeric wire source.  This
module deliberately derives enums and layouts from it at import time so the
Python implementation cannot silently acquire a second handwritten registry.
"""

from __future__ import annotations

import hashlib
from enum import IntEnum, IntFlag
from importlib import resources
import json
import math
import os
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable

from .errors import IntegrityError


REGISTRY_SHA256 = "23fbd056db28c1fedc7987a34d0893594fddb417f34058851667ffc2d5dca0a2"
SCHEMA_SHA256 = "95191cda4ea5ed0691126af1fa027ff37b3dd6f920ede9c099cad0b16fce7ede"


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise IntegrityError(f"duplicate registry key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise IntegrityError(f"non-finite registry value: {value}")


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise IntegrityError(f"non-finite registry value: {value}")
    return parsed


def _find_repository_file(relative: str) -> Path | None:
    override = os.environ.get("M6LINK_REGISTRY")
    if override and relative.endswith("m6_v2_wire_v1.json"):
        return Path(override).expanduser().resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / relative
        if candidate.is_file():
            return candidate
    return None


def _load(path: Path | None, resource_name: str) -> dict[str, Any]:
    try:
        if path is None:
            stream = resources.files("m6link").joinpath("data", resource_name).open("r", encoding="utf-8")
        else:
            stream = path.open("r", encoding="utf-8")
        with stream:
            value = json.load(
                stream,
                object_pairs_hook=_no_duplicates,
                parse_constant=_reject_constant,
                parse_float=_finite_float,
            )
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrityError(f"cannot read frozen registry {path or resource_name}: {exc}") from exc
    if not isinstance(value, dict):
        raise IntegrityError(f"frozen registry {path or resource_name} is not an object")
    return value


def canonical_digest(value: object) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


REGISTRY_PATH = _find_repository_file("Tests/vectors/m6_v2_wire_v1.json")
SCHEMA_PATH = _find_repository_file("Tests/vectors/m6_v2_wire.schema.json")
REGISTRY = _load(REGISTRY_PATH, "m6_v2_wire_v1.json")
SCHEMA = _load(SCHEMA_PATH, "m6_v2_wire.schema.json")
if canonical_digest(REGISTRY) != REGISTRY_SHA256:
    raise IntegrityError("frozen m6-v2-wire-v1 registry digest changed")
if canonical_digest(SCHEMA) != SCHEMA_SHA256:
    raise IntegrityError("frozen m6-v2-wire-v1 schema digest changed")


def _registry_values(name: str) -> dict[str, int]:
    entries = REGISTRY["registries"][name]
    values: dict[str, int] = {}
    for entry in entries:
        raw = entry.get("value", entry.get("bit"))
        if "value_hex" in entry:
            raw = int(entry["value_hex"], 16)
        elif "bit" in entry and "value" not in entry:
            raw = 1 << int(entry["bit"])
        values[str(entry["name"])] = int(raw)
    return values


def enum(name: str, registry_name: str, *, flag: bool = False) -> type[IntEnum]:
    base = IntFlag if flag else IntEnum
    return base(name, _registry_values(registry_name), module=__name__)  # type: ignore[return-value]


FrameType = enum("FrameType", "frame_types")
FrameFlag = enum("FrameFlag", "flags", flag=True)
Capability = enum("Capability", "capabilities", flag=True)
RpcOpcode = enum("RpcOpcode", "rpc_opcodes")
RpcResult = enum("RpcResult", "rpc_results")
ParameterType = enum("ParameterType", "parameter_types")
ParameterSource = enum("ParameterSource", "parameter_sources")
ActionKind = enum("ActionKind", "action_kinds")
ActionState = enum("ActionState", "action_states")
ActionReason = enum("ActionReason", "action_reasons")
EventCode = enum("EventCode", "event_codes")
Severity = enum("Severity", "severities")
TelemetryStream = enum("TelemetryStream", "telemetry_streams")

FRAMING = MappingProxyType(dict(REGISTRY["framing"]))
PARSER = MappingProxyType(dict(REGISTRY["parser"]))
SESSION = MappingProxyType(dict(REGISTRY["session"]))
SEMANTICS = MappingProxyType(dict(REGISTRY["semantics"]))
LAYOUTS = MappingProxyType({item["name"]: item for item in REGISTRY["layouts"]})
PARAMETERS = tuple(REGISTRY["parameters"])
TELEMETRY = MappingProxyType(dict(REGISTRY["telemetry"]))
RPC_OPCODE_INFO = MappingProxyType(
    {int(item["value"]): item for item in REGISTRY["registries"]["rpc_opcodes"]}
)
RPC_RESULT_NAMES = MappingProxyType(
    {int(item["value"]): str(item["name"]) for item in REGISTRY["registries"]["rpc_results"]}
)
CAPABILITY_MASKS = MappingProxyType(
    {str(item["name"]): int(item["value_hex"], 16) for item in REGISTRY["registries"]["capability_masks"]}
)
PARAMETER_BY_ID = MappingProxyType({int(item["id"]): item for item in PARAMETERS})
PARAMETER_BY_KEY = MappingProxyType({str(item["key"]): item for item in PARAMETERS})


def registry_value(registry_name: str, item_name: str) -> int:
    try:
        return _registry_values(registry_name)[item_name]
    except KeyError as exc:
        raise IntegrityError(f"unknown {registry_name} entry: {item_name}") from exc


def assert_layout(name: str, size: int, fields: Iterable[tuple[str, int, int]]) -> None:
    """Validate a handwritten struct format against the frozen layout."""

    layout = LAYOUTS.get(name)
    if layout is None or int(layout["size"]) != size:
        raise IntegrityError(f"layout {name} size mismatch")
    actual = {field["name"]: (int(field["offset"]), int(field["size"])) for field in layout["fields"]}
    for field_name, offset, field_size in fields:
        if actual.get(field_name) != (offset, field_size):
            raise IntegrityError(f"layout {name}.{field_name} mismatch")
