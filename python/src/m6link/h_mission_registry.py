"""Immutable ``m6-h-mission-v1`` constants and layout checks."""

from __future__ import annotations

from enum import IntEnum
from importlib import resources
import json
from pathlib import Path
from types import MappingProxyType

from .errors import IntegrityError
from .registry import canonical_digest


EXTENSION_SHA256 = "973cb83992eaa81bd4a021391e8c98498e78ddf261c7c13949c163338bab9b67"


def _find() -> Path | None:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "Tests/vectors/m6_h_mission_ext_v1.json"
        if candidate.is_file():
            return candidate
    return None


def _load() -> dict[str, object]:
    path = _find()
    try:
        if path is None:
            source = resources.files("m6link").joinpath(
                "data", "m6_h_mission_ext_v1.json"
            ).open("r", encoding="utf-8")
        else:
            source = path.open("r", encoding="utf-8")
        with source:
            value = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrityError(f"cannot read H mission extension registry: {exc}") from exc
    if not isinstance(value, dict) or canonical_digest(value) != EXTENSION_SHA256:
        raise IntegrityError("m6-h-mission-v1 registry digest changed")
    return value


EXTENSION = _load()
CAPABILITY_BIT = int(EXTENSION["capability"]["bit"])  # type: ignore[index]
CAPABILITY_MASK = 1 << CAPABILITY_BIT
RPC = MappingProxyType(dict(EXTENSION["rpc_opcodes"]))  # type: ignore[arg-type]
LAYOUTS = MappingProxyType(
    {item["name"]: item for item in EXTENSION["layouts"]}  # type: ignore[index]
)
HMissionCommand = IntEnum("HMissionCommand", EXTENSION["commands"])  # type: ignore[arg-type]
HMissionMode = IntEnum("HMissionMode", EXTENSION["modes"])  # type: ignore[arg-type]


def assert_layout(name: str, size: int) -> None:
    layout = LAYOUTS.get(name)
    if layout is None or int(layout["size"]) != size:
        raise IntegrityError(f"H mission extension layout {name} mismatch")


assert_layout("h_mission_command_request", 32)
assert_layout("h_mission_snapshot", 64)
