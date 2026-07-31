#!/usr/bin/env python3
"""Read or change one H3 RAM parameter without rebuilding or flashing."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link import H3TuneParameter, Session  # noqa: E402
from m6link.h3_ki_registry import PARAMETER_ID as KI_PARAMETER_ID  # noqa: E402
from m6link.h3_maneuver_registry import (  # noqa: E402
    CAPABILITY_MASK as MANEUVER_CAPABILITY,
    PARAMETER_IDS as MANEUVER_PARAMETER_IDS,
)


FIRMWARE_ID = "M6-runtime-20260801-r158-stop"

FIELDS = {
    "plus_max_deg": (
        "plus_max_mdeg", int(H3TuneParameter.plus_max_mdeg), 1000,
    ),
    "kp": ("kp_mdeg_per_mm", int(H3TuneParameter.kp_mdeg_per_mm), 1),
    "kd": ("kd_mdeg_per_mm_s", int(H3TuneParameter.kd_mdeg_per_mm_s), 1),
    "ki": ("ki_mdeg_per_mm_s", KI_PARAMETER_ID, 1),
    "plus_topup_deg": (
        "plus_topup_mdeg", int(MANEUVER_PARAMETER_IDS["plus_topup_mdeg"]), 1000,
    ),
    "plus_brake_deg": (
        "plus_brake_mdeg", int(MANEUVER_PARAMETER_IDS["plus_brake_mdeg"]), 1000,
    ),
    "plus_brake_trigger_mm": (
        "plus_brake_trigger_x10",
        int(MANEUVER_PARAMETER_IDS["plus_brake_trigger_x10"]), 10,
    ),
    "plus_topup_brake_mm": (
        "plus_topup_brake_trigger_x10",
        int(MANEUVER_PARAMETER_IDS["plus_topup_brake_trigger_x10"]), 10,
    ),
    "plus_topup_max_speed": (
        "plus_topup_max_speed_mm_s",
        int(MANEUVER_PARAMETER_IDS["plus_topup_max_speed_mm_s"]), 1,
    ),
    "minus_brake_trigger_mm": (
        "minus_brake_trigger_x10",
        int(MANEUVER_PARAMETER_IDS["minus_brake_trigger_x10"]), 10,
    ),
    "minus_brake_deg": (
        "minus_brake_mdeg", int(MANEUVER_PARAMETER_IDS["minus_brake_mdeg"]), 1000,
    ),
    "minus_return_predict_ms": (
        "minus_brake_return_predict_ms",
        int(MANEUVER_PARAMETER_IDS["minus_brake_return_predict_ms"]), 1,
    ),
    "minus_return_rpm": (
        "minus_return_rpm",
        int(MANEUVER_PARAMETER_IDS["minus_return_rpm"]), 1,
    ),
    "tracking_rpm": ("tracking_rpm", int(H3TuneParameter.tracking_rpm), 1),
    "plus_launch_rpm": (
        "plus_launch_rpm", int(H3TuneParameter.plus_launch_rpm), 1,
    ),
    "minus_launch_rpm": (
        "minus_launch_rpm", int(H3TuneParameter.minus_launch_rpm), 1,
    ),
}


def display(snapshot) -> None:
    print(json.dumps({
        "active_revision": snapshot.active_revision,
        "stage_revision": snapshot.stage_revision,
        "ram_only": True,
        "parameters": asdict(snapshot.active),
    }, ensure_ascii=False, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read or change one H3 RAM parameter; never writes Flash."
    )
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--firmware-id", default=FIRMWARE_ID)
    for option in FIELDS:
        parser.add_argument("--" + option.replace("_", "-"), type=float)
    args = parser.parse_args()
    changes = [(name, getattr(args, name)) for name in FIELDS
               if getattr(args, name) is not None]
    if len(changes) > 1:
        parser.error("change exactly one parameter per command")

    session = Session(port=args.port, timeout=0.8,
                      firmware_id=args.firmware_id)
    try:
        identity = session.connect()
        if (int(identity.enabled_capabilities) & MANEUVER_CAPABILITY) == 0:
            raise SystemExit("firmware lacks RAM trajectory controls; flash r158 once")
        snapshot = session.get_h3_tune()
        if not changes:
            display(snapshot)
            return 0
        if snapshot.authorized or snapshot.host_run_active:
            raise SystemExit("H3 is authorized or running; stop the batch first")
        if snapshot.dirty:
            snapshot = session.revert_h3_tune(snapshot=snapshot)
        option, requested = changes[0]
        field, parameter_id, scale = FIELDS[option]
        value = int(round(requested * scale))
        old_value = int(getattr(snapshot.active, field))
        staged = session.stage_h3_tune_field(
            parameter_id, value, snapshot=snapshot,
        )
        applied = session.apply_h3_tune(snapshot=staged)
        actual = int(getattr(applied.active, field))
        if actual != value:
            raise SystemExit(f"Apply verification failed: requested {value}, got {actual}")
        print(json.dumps({
            "ok": True,
            "ram_only": True,
            "parameter": field,
            "old": old_value,
            "new": actual,
            "active_revision": applied.active_revision,
            "note": "reset or power cycle restores compiled defaults",
        }, ensure_ascii=False, indent=2))
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
