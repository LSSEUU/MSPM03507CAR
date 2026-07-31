"""Immutable ``m6-h3-ki-v1`` constants and layout checks."""

from __future__ import annotations

from importlib import resources
import json
from pathlib import Path
from types import MappingProxyType

from .errors import IntegrityError
from .registry import canonical_digest


EXTENSION_SHA256 = "d2153777ac77a28ed7c134b0a534242a05c77dd9893388b58cd48c38121bfc25"


def _find() -> Path | None:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "Tests/vectors/m6_h3_ki_ext_v1.json"
        if candidate.is_file():
            return candidate
    return None


def _load() -> dict[str, object]:
    path = _find()
    try:
        source = (path.open("r", encoding="utf-8") if path is not None else
                  resources.files("m6link").joinpath(
                      "data", "m6_h3_ki_ext_v1.json"
                  ).open("r", encoding="utf-8"))
        with source:
            value = json.load(source)
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrityError(f"cannot read H3 Ki extension registry: {exc}") from exc
    if not isinstance(value, dict) or canonical_digest(value) != EXTENSION_SHA256:
        raise IntegrityError("m6-h3-ki-v1 registry digest changed")
    return value


EXTENSION = _load()
CAPABILITY_BIT = int(EXTENSION["capability"]["bit"])  # type: ignore[index]
CAPABILITY_MASK = 1 << CAPABILITY_BIT
RPC = MappingProxyType(dict(EXTENSION["rpc_opcodes"]))  # type: ignore[arg-type]
PARAMETER_ID = int(EXTENSION["parameters"]["ki_mdeg_per_mm_s"])  # type: ignore[index]
