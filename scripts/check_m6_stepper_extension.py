"""Check the independently frozen M6 ZDT stepper extension bindings."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
EXPECTED = "94400935573dd65d58b28147e7be3995239fa40b87223f1910fed246143edaa6"
EXPECTED_SCHEMA = "a6e7237a59a447acdc3d7c0d8f3bfde0403a7e4695225e8a476ccf68fd4ec83f"


def digest(value: object) -> str:
    data = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    registry = json.loads((ROOT / "Tests/vectors/m6_zdt_stepper_ext_v1.json").read_text(encoding="utf-8"))
    schema = json.loads((ROOT / "Tests/vectors/m6_zdt_stepper_ext.schema.json").read_text(encoding="utf-8"))
    if digest(registry) != EXPECTED or digest(schema) != EXPECTED_SCHEMA:
        raise SystemExit("ZDT extension registry or schema digest changed")
    if registry["name"] != schema["properties"]["name"]["const"]:
        raise SystemExit("ZDT extension schema/name mismatch")
    layouts = {item["name"]: item for item in registry["layouts"]}
    for name, expected_size in (("zdt_stepper_status_request", 4),
                                ("zdt_stepper_command_request", 24),
                                ("zdt_stepper_snapshot", 88)):
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
    header = (ROOT / "System/m6_zdt_stepper_ext.h").read_text(encoding="utf-8")
    required = {
        "M6_RPC_ZDT_STEPPER_STATUS_GET": "0x000EU",
        "M6_RPC_ZDT_STEPPER_COMMAND": "0x8308U",
        "M6_ZDT_STEPPER_RESOURCE": "9U",
        "M6_ZDT_STEPPER_STATUS_REQUEST_SIZE": "4U",
        "M6_ZDT_STEPPER_COMMAND_REQUEST_SIZE": "24U",
        "M6_ZDT_STEPPER_SNAPSHOT_SIZE": "88U",
    }
    for name, value in required.items():
        if not re.search(rf"#define\s+{name}\s+{re.escape(value)}\b", header):
            raise SystemExit(f"C binding mismatch: {name}")
    python_binding = (ROOT / "python/src/m6link/stepper_registry.py").read_text(encoding="utf-8")
    if EXPECTED not in python_binding:
        raise SystemExit("Python extension digest binding mismatch")
    print("m6-zdt-stepper-ext-v1 contract OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
