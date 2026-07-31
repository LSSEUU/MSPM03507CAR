"""Immutable ``m6-lever-calibration-ext-v1`` constants."""

from __future__ import annotations

from enum import IntEnum
from importlib import resources
import json
from pathlib import Path
from types import MappingProxyType

from .errors import IntegrityError
from .registry import canonical_digest


EXTENSION_SHA256 = "f119387c22d0263541b44047c5877d28327060777e5de5f89c7fe062ab160275"


def _find() -> Path | None:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "Tests/vectors/m6_lever_calibration_ext_v1.json"
        if candidate.is_file():
            return candidate
    return None


def _load() -> dict[str, object]:
    path = _find()
    try:
        if path is None:
            source = resources.files("m6link").joinpath(
                "data", "m6_lever_calibration_ext_v1.json"
            ).open("r", encoding="utf-8")
        else:
            source = path.open("r", encoding="utf-8")
        with source:
            value = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrityError(f"cannot read lever extension registry: {exc}") from exc
    if not isinstance(value, dict) or canonical_digest(value) != EXTENSION_SHA256:
        raise IntegrityError("m6-lever-calibration-ext-v1 registry digest changed")
    return value


EXTENSION = _load()
CAPABILITY_BIT = int(EXTENSION["capability"]["bit"])  # type: ignore[index]
CAPABILITY_MASK = 1 << CAPABILITY_BIT
RPC = MappingProxyType(dict(EXTENSION["rpc_opcodes"]))  # type: ignore[arg-type]
LAYOUTS = MappingProxyType(
    {item["name"]: item for item in EXTENSION["layouts"]}  # type: ignore[index]
)
LeverCalibrationCommand = IntEnum(
    "LeverCalibrationCommand", EXTENSION["commands"]  # type: ignore[arg-type]
)
LeverCalibrationDirection = IntEnum(
    "LeverCalibrationDirection", EXTENSION["directions"]  # type: ignore[arg-type]
)
LeverCalibrationPage = IntEnum(
    "LeverCalibrationPage", EXTENSION["status_pages"]  # type: ignore[arg-type]
)
LEVER_CALIBRATION = CAPABILITY_MASK
LEVER_CALIBRATION_STATUS_GET = int(RPC["LEVER_CALIBRATION_STATUS_GET"])
LEVER_CALIBRATION_COMMAND = int(RPC["LEVER_CALIBRATION_COMMAND"])


def assert_layout(name: str, size: int) -> None:
    layout = LAYOUTS.get(name)
    if layout is None or int(layout["size"]) != size:
        raise IntegrityError(f"lever extension layout {name} mismatch")


assert_layout("lever_calibration_status_request", 4)
assert_layout("lever_calibration_command_request", 24)
assert_layout("lever_calibration_snapshot", 88)
