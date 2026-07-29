"""Immutable ``m6-zdt-stepper-ext-v1`` constants and layout checks."""

from __future__ import annotations

from enum import IntEnum, IntFlag
from importlib import resources
import json
from pathlib import Path
from types import MappingProxyType

from .errors import IntegrityError
from .registry import canonical_digest


EXTENSION_SHA256 = "94400935573dd65d58b28147e7be3995239fa40b87223f1910fed246143edaa6"


def _find() -> Path | None:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "Tests/vectors/m6_zdt_stepper_ext_v1.json"
        if candidate.is_file():
            return candidate
    return None


def _load() -> dict[str, object]:
    path = _find()
    try:
        if path is None:
            source = resources.files("m6link").joinpath(
                "data", "m6_zdt_stepper_ext_v1.json"
            ).open("r", encoding="utf-8")
        else:
            source = path.open("r", encoding="utf-8")
        with source:
            value = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrityError(f"cannot read ZDT extension registry: {exc}") from exc
    if not isinstance(value, dict) or canonical_digest(value) != EXTENSION_SHA256:
        raise IntegrityError("m6-zdt-stepper-ext-v1 registry digest changed")
    return value


EXTENSION = _load()
CAPABILITY_BIT = int(EXTENSION["capability"]["bit"])  # type: ignore[index]
CAPABILITY_MASK = 1 << CAPABILITY_BIT
RPC = MappingProxyType(dict(EXTENSION["rpc_opcodes"]))  # type: ignore[arg-type]
BUSY = MappingProxyType(dict(EXTENSION["busy"]))  # type: ignore[arg-type]
LAYOUTS = MappingProxyType(
    {item["name"]: item for item in EXTENSION["layouts"]}  # type: ignore[index]
)

StepperCommand = IntEnum("StepperCommand", EXTENSION["commands"])  # type: ignore[arg-type]
StepperRefresh = IntFlag(
    "StepperRefresh",
    {name: 1 << int(bit) for name, bit in EXTENSION["refresh_bits"].items()},  # type: ignore[union-attr]
)
ZDT_STEPPER_CONTROL = CAPABILITY_MASK
ZDT_STEPPER_STATUS_GET = int(RPC["ZDT_STEPPER_STATUS_GET"])
ZDT_STEPPER_COMMAND = int(RPC["ZDT_STEPPER_COMMAND"])


def assert_layout(name: str, size: int) -> None:
    layout = LAYOUTS.get(name)
    if layout is None or int(layout["size"]) != size:
        raise IntegrityError(f"ZDT extension layout {name} mismatch")


assert_layout("zdt_stepper_status_request", 4)
assert_layout("zdt_stepper_command_request", 24)
assert_layout("zdt_stepper_snapshot", 88)
