"""Immutable ``m6-h3-maneuver-v1`` constants and layout checks."""

from __future__ import annotations

from importlib import resources
import json
from pathlib import Path
from types import MappingProxyType

from .errors import IntegrityError
from .registry import canonical_digest


EXTENSION_SHA256 = "2a176de8d996d6444b2318f20b80506ecde9ea2fe0fd8dcf2848131e6526223b"


def _find() -> Path | None:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "Tests/vectors/m6_h3_maneuver_ext_v1.json"
        if candidate.is_file():
            return candidate
    return None


def _load() -> dict[str, object]:
    path = _find()
    try:
        source = (path.open("r", encoding="utf-8") if path is not None else
                  resources.files("m6link").joinpath(
                      "data", "m6_h3_maneuver_ext_v1.json"
                  ).open("r", encoding="utf-8"))
        with source:
            value = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrityError(f"cannot read H3 maneuver registry: {exc}") from exc
    if not isinstance(value, dict) or canonical_digest(value) != EXTENSION_SHA256:
        raise IntegrityError("m6-h3-maneuver-v1 registry digest changed")
    return value


EXTENSION = _load()
CAPABILITY_BIT = int(EXTENSION["capability"]["bit"])  # type: ignore[index]
CAPABILITY_MASK = 1 << CAPABILITY_BIT
RPC = MappingProxyType(dict(EXTENSION["rpc_opcodes"]))  # type: ignore[arg-type]
PARAMETER_IDS = MappingProxyType(dict(EXTENSION["parameters"]))  # type: ignore[arg-type]
