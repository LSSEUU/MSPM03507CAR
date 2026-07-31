"""Check the independently frozen M6 lever calibration extension bindings."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
EXPECTED = "f119387c22d0263541b44047c5877d28327060777e5de5f89c7fe062ab160275"


def digest(value: object) -> str:
    data = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    registry = json.loads(
        (ROOT / "Tests/vectors/m6_lever_calibration_ext_v1.json").read_text(
            encoding="utf-8"
        )
    )
    h_mission = json.loads(
        (ROOT / "Tests/vectors/m6_h_mission_ext_v1.json").read_text(
            encoding="utf-8"
        )
    )
    if digest(registry) != EXPECTED:
        raise SystemExit("lever extension registry digest changed")
    if registry["capability"]["bit"] == h_mission["capability"]["bit"]:
        raise SystemExit("lever extension capability collides with H mission")
    if set(registry["rpc_opcodes"].values()) & set(
        h_mission["rpc_opcodes"].values()
    ):
        raise SystemExit("lever extension RPC opcode collides with H mission")
    layouts = {item["name"]: item for item in registry["layouts"]}
    for name, expected_size in (
        ("lever_calibration_status_request", 4),
        ("lever_calibration_command_request", 24),
        ("lever_calibration_snapshot", 88),
    ):
        layout = layouts[name]
        if layout["size"] != expected_size:
            raise SystemExit(f"{name} size mismatch")
        occupied: set[int] = set()
        for field in layout["fields"]:
            slots = set(range(field["offset"], field["offset"] + field["size"]))
            if occupied & slots or max(slots) >= expected_size:
                raise SystemExit(f"{name}.{field['name']} overlaps or exceeds layout")
            occupied |= slots
        if occupied != set(range(expected_size)):
            raise SystemExit(f"{name} has uncovered bytes")
    header = (ROOT / "System/m6_lever_calibration_ext.h").read_text(encoding="utf-8")
    required = {
        "M6_RPC_LEVER_CALIBRATION_STATUS_GET": "0x0010U",
        "M6_RPC_LEVER_CALIBRATION_COMMAND": "0x830AU",
        "M6_LEVER_CAL_STATUS_REQUEST_SIZE": "4U",
        "M6_LEVER_CAL_COMMAND_REQUEST_SIZE": "24U",
        "M6_LEVER_CAL_SNAPSHOT_SIZE": "88U",
    }
    for name, value in required.items():
        if not re.search(rf"#define\s+{name}\s+{re.escape(value)}\b", header):
            raise SystemExit(f"C binding mismatch: {name}")
    python_binding = (
        ROOT / "python/src/m6link/lever_calibration_registry.py"
    ).read_text(encoding="utf-8")
    if EXPECTED not in python_binding:
        raise SystemExit("Python lever extension digest binding mismatch")
    print("m6-lever-calibration-ext-v1 contract OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
