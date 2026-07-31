"""RAM-only H3 automatic tuning loop for ``m6-h3-tune-v1``.

Only ``run`` and ``resume`` open the serial port.  The other commands exchange
atomic files with that single owner, so feedback and aborts can never create a
second serial writer.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time
from typing import Any, Iterable, Mapping

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link import FrameType, H3TuneParameter, Session  # noqa: E402
from m6link.errors import DeviceRejected  # noqa: E402
from m6link.h3_tune_registry import CAPABILITY_MASK as H3_TUNE_CAPABILITY  # noqa: E402
from m6link.h3_ki_registry import (  # noqa: E402
    CAPABILITY_MASK as H3_KI_CAPABILITY, PARAMETER_ID as H3_KI_PARAMETER_ID,
)
from m6link.h3_maneuver_registry import (  # noqa: E402
    CAPABILITY_MASK as H3_MANEUVER_CAPABILITY,
    PARAMETER_IDS as H3_MANEUVER_PARAMETER_IDS,
)


FIRMWARE_ID = "M6-runtime-20260801-r158-stop"
H3_RECORD = struct.Struct("<BBBBIII7hiiIHBBbBh")
H3_FIELDS = (
    "kind", "schema", "state", "flags", "time_ms", "run_id",
    "profile_revision", "x_mm_x10", "vx_mm_s", "target_mm_x10",
    "error_mm_x10", "raw_pd_mdeg", "forced_tilt_mdeg", "tilt_mdeg",
    "actuator_counts", "actuator_target_counts", "fault",
    "vision_age_ms", "vision_seq", "actuator_state",
    "actuator_direction", "actuator_rpm",
    "integral_mdeg",
)
PROFILE_FIELDS = (
    "plus_start_mdeg", "plus_max_mdeg", "minus_start_mdeg",
    "minus_max_mdeg", "kp_mdeg_per_mm", "kd_mdeg_per_mm_s",
    "normal_slew_mdeg_per_20ms", "breakaway_slew_mdeg_per_20ms",
    "brake_slew_mdeg_per_20ms", "tracking_rpm", "plus_launch_rpm",
    "minus_launch_rpm",
    "ki_mdeg_per_mm_s",
    "plus_topup_mdeg", "plus_brake_mdeg",
    "minus_brake_trigger_x10", "minus_brake_mdeg",
    "plus_brake_trigger_x10",
    "plus_topup_brake_trigger_x10",
    "minus_brake_return_predict_ms", "minus_return_rpm",
    "plus_topup_max_speed_mm_s",
)
PARAMETER_IDS = {name: H3TuneParameter[name] for name in (
    "plus_start_mdeg", "plus_max_mdeg", "minus_start_mdeg",
    "minus_max_mdeg", "kp_mdeg_per_mm", "kd_mdeg_per_mm_s",
    "normal_slew_mdeg_per_20ms", "breakaway_slew_mdeg_per_20ms",
    "brake_slew_mdeg_per_20ms", "tracking_rpm", "plus_launch_rpm",
    "minus_launch_rpm",
)}
PARAMETER_IDS["ki_mdeg_per_mm_s"] = H3_KI_PARAMETER_ID
PARAMETER_IDS.update(H3_MANEUVER_PARAMETER_IDS)
FEEDBACK_RESULTS = (
    "no_roll", "rolled", "overshoot", "oscillation", "normal", "unsafe"
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
    # Windows readers do not always share DELETE access.  A simultaneous
    # ``status`` read can therefore deny the atomic replace for a few
    # milliseconds even though neither side is modifying the same handle.
    for attempt in range(30):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 29:
                raise
            time.sleep(0.01)


def read_json(path: Path, default: Any = None) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def latest_session(root: Path) -> Path:
    sessions = sorted(path for path in root.glob("*") if path.is_dir())
    if not sessions:
        raise SystemExit(f"no H3 sessions below {root}")
    return sessions[-1]


def resolve_session(args: argparse.Namespace) -> Path:
    return args.session.resolve() if args.session else latest_session(args.root.resolve())


def decode_h3(payload: bytes) -> dict[str, int]:
    if len(payload) != H3_RECORD.size:
        raise ValueError(f"H3 telemetry length {len(payload)} != {H3_RECORD.size}")
    row = dict(zip(H3_FIELDS, H3_RECORD.unpack(payload), strict=True))
    if row["kind"] != 3 or row["schema"] != 3:
        raise ValueError("not H3 schema 3 telemetry")
    return row


def h3_row(event: object) -> dict[str, int] | None:
    payload = getattr(event, "payload", b"")
    if (getattr(event, "type", -1) != int(FrameType.BULK) or
            len(payload) != H3_RECORD.size or payload[:2] != b"\x03\x03"):
        return None
    return decode_h3(payload)


def run_frame_state(
    row: Mapping[str, int], expected_run_id: int, started_observed: bool,
) -> tuple[bool, bool]:
    """Return updated start observation and terminal status for one run.

    Pre-start H3 frames can still be queued when START returns.  They must not
    be interpreted as a terminal frame for the newly allocated run id.
    """
    same_run = int(row["run_id"]) == int(expected_run_id)
    started = bool(
        started_observed or (same_run and (int(row["flags"]) & 128))
    )
    terminal = bool(
        same_run and started and not (int(row["flags"]) & 128) and
        int(row["state"]) in (0, 6)
    )
    return started, terminal


def host_soft_stop_required(row: Mapping[str, int]) -> bool:
    """Stop before an outward-moving ball outruns a lagging lever.

    The firmware's 58 mm predicted-edge guard remains the final authority.
    This earlier host guard is deliberately conservative: it only intervenes
    after breakaway, while velocity is outward, and while encoder feedback is
    more than two degrees behind the requested actuator position.
    """
    if int(row["state"]) not in (1, 3) or int(row["fault"]) != 0:
        return False
    # RUN_MINUS begins with a 1.5 s target ramp from the positive pass point,
    # so target_mm_x10 remains positive during the critical brake handoff.
    # Direction belongs to the stage, not to that transient ramp value.
    target_sign = 1 if int(row["state"]) == 1 else -1
    outward_speed = target_sign * int(row["vx_mm_s"])
    directed_position = target_sign * int(row["x_mm_x10"])
    # Only a target that is farther outward than encoder feedback is lag.
    # An absolute difference also fires while the lever is already ahead of
    # the requested angle and withdrawing; that prematurely ended the r102
    # pass-through test at +33.2 mm (5355 current, 4619 target).
    actuator_lag = target_sign * (
        int(row["actuator_target_counts"]) - int(row["actuator_counts"])
    )
    actuator_recovering = int(row["actuator_direction"]) == target_sign
    return bool(
        # Position is encoded in 0.1 mm.  The old value 50 meant only 5 mm
        # and stopped r109 at +35.6 mm while its lever was already recovering.
        directed_position >= 500 and outward_speed >= 10 and
        actuator_lag > 720 and
        target_sign * int(row["tilt_mdeg"]) > 500 and
        not actuator_recovering
    )


def host_launch_pulse_required(
    row: Mapping[str, int], horizontal_counts: int | None, pulse_mdeg: int | None,
) -> bool:
    """Withdraw a launch pulse before waiting for visible ball motion.

    This is an opt-in diagnostic guard.  The lever encoder, rather than ball
    displacement, ends the pulse so mechanical return can begin before the
    delayed breakaway acceleration becomes large.  It never changes firmware
    parameters and the normal firmware edge/fault guards remain authoritative.
    """
    if (horizontal_counts is None or pulse_mdeg is None or pulse_mdeg <= 0 or
            int(row["state"]) not in (1, 3) or int(row["fault"]) != 0):
        return False
    target_sign = 1 if int(row["target_mm_x10"]) > 0 else -1
    forced_tilt = int(row["forced_tilt_mdeg"])
    if target_sign * forced_tilt <= 0:
        return False
    # The actuator uses 65536 encoder counts per revolution.  Round the pulse
    # threshold upward so the host cannot stop one count prematurely.
    threshold_counts = (int(pulse_mdeg) * 65536 + 359999) // 360000
    directed_counts = target_sign * (
        int(row["actuator_counts"]) - int(horizontal_counts)
    )
    return directed_counts >= threshold_counts


class H3HeartbeatPump:
    """Keep the H3 authorization alive without violating the RPC RX window.

    The device advertises a one-request receive window.  Every heartbeat must
    therefore receive its response before the next heartbeat is transmitted.
    A dedicated thread keeps that confirmed exchange independent of telemetry
    capture and bounds a lost response below the firmware's 300 ms watchdog.
    The actuator can defer a reply beyond 180 ms while stepping, so leave a
    small watchdog margin but do not mistake that normal latency for a lost
    heartbeat.
    """

    def __init__(self, session: Session, token: int, *,
                 interval: float = 0.08, timeout: float = 0.25):
        if not 0.0 < timeout < 0.30:
            raise ValueError("H3 heartbeat timeout must stay below 300 ms")
        self.session = session
        self.token = token
        self.interval = interval
        self.timeout = timeout
        self._stop = threading.Event()
        self._error: Exception | None = None
        self._thread = threading.Thread(
            target=self._run, name="h3-heartbeat", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self.session.heartbeat_h3_tune(
                    self.token, timeout=self.timeout
                )
            except DeviceRejected as exc:
                if exc.name == "NOT_ARMED":
                    # A real firmware fault revokes authorization before its
                    # terminal telemetry necessarily reaches the host.  Stop
                    # pumping and let capture_run persist that fault record.
                    self._stop.set()
                    return
                self._error = exc
                self._stop.set()
                return
            except Exception as exc:
                self._error = exc
                self._stop.set()
                return
            remaining = self.interval - (time.monotonic() - started)
            if self._stop.wait(max(0.0, remaining)):
                return

    def raise_if_failed(self) -> None:
        if self._error is not None:
            raise RuntimeError(
                f"confirmed H3 heartbeat failed: {self._error}"
            ) from self._error

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(self.timeout + 0.1)



def load_feedback(session_dir: Path, run_number: int) -> list[dict[str, Any]]:
    result = []
    for path in sorted((session_dir / "feedback").glob(f"run_{run_number:03d}_*.json")):
        value = read_json(path)
        if isinstance(value, dict):
            result.append(value)
    return result


def _direction_metrics(rows: list[dict[str, int]], state: int,
                       target_x10: int, flag: int = 32) -> dict[str, Any]:
    selected = [row for row in rows if row["state"] == state]
    if not selected:
        return {
            "observed": False, "rolled": False, "actuator_followed": False,
            "roll_tilt_mdeg": None, "completion_ms": None,
        }
    start_x = selected[0]["x_mm_x10"]
    sign = 1 if target_x10 > 0 else -1
    roll = next((row for row in selected if row["flags"] & flag or
                 sign * (row["x_mm_x10"] - start_x) >= 10), None)
    elapsed = selected[-1]["time_ms"] - selected[0]["time_ms"]
    followed_rows = [row for row in selected if
                     row["time_ms"] - selected[0]["time_ms"] >= 1000 and
                     abs(row["forced_tilt_mdeg"]) > 0 and
                     abs(row["actuator_target_counts"] - row["actuator_counts"]) <= 128]
    return {
        "observed": True,
        "rolled": roll is not None,
        "roll_tilt_mdeg": abs(roll["tilt_mdeg"]) if roll else None,
        "roll_time_ms": (roll["time_ms"] - selected[0]["time_ms"]) if roll else None,
        "actuator_followed": bool(followed_rows),
        "target_command_reached_2500ms": any(
            abs(row["forced_tilt_mdeg"]) > 0 and
            abs(row["actuator_target_counts"] - row["actuator_counts"]) <= 128
            for row in selected if row["time_ms"] - selected[0]["time_ms"] <= 2500
        ),
        "completion_ms": elapsed,
    }


def _persisted_lag(rows: list[dict[str, int]]) -> bool:
    started: int | None = None
    for row in rows:
        braking = row["raw_pd_mdeg"] * row["vx_mm_s"] < 0
        lagged = abs(row["actuator_target_counts"] - row["actuator_counts"]) > 360
        if braking and lagged:
            started = row["time_ms"] if started is None else started
            if row["time_ms"] - started >= 300:
                return True
        else:
            started = None
    return False


def _slow_forced_recovery(rows: list[dict[str, int]]) -> bool:
    for before, after in zip(rows, rows[1:]):
        if before["forced_tilt_mdeg"] and not after["forced_tilt_mdeg"]:
            sign = 1 if before["tilt_mdeg"] > 0 else -1
            deadline = after["time_ms"] + 300
            window = [row for row in rows if after["time_ms"] <= row["time_ms"] <= deadline]
            if window and all(sign * row["tilt_mdeg"] >
                              sign * row["raw_pd_mdeg"] + 500 for row in window):
                return True
    return False


def _plus_brake_timing(rows: list[dict[str, int]]) -> dict[str, Any]:
    active = [row for row in sorted(rows, key=lambda item: item["time_ms"])
              if row["state"] in (1, 3, 4)]
    if not active:
        return {
            "run_start_ms": None, "pd_negative": None,
            "predictive_brake_request": None,
            "entered_plus_40": None, "actuator_reverse": None,
            "brake_request_to_actuator_ms": None,
            "pd_to_actuator_ms": None, "plus_40_to_actuator_ms": None,
        }
    run_start_ms = active[0]["time_ms"]

    def event(row: Mapping[str, int] | None) -> dict[str, int] | None:
        if row is None:
            return None
        return {
            "time_ms": int(row["time_ms"]),
            "elapsed_ms": int(row["time_ms"]) - run_start_ms,
            "x_mm_x10": int(row["x_mm_x10"]),
            "vx_mm_s": int(row["vx_mm_s"]),
            "tilt_mdeg": int(row["tilt_mdeg"]),
            "actuator_counts": int(row["actuator_counts"]),
        }

    pd_row = next((row for row in active
                   if row["state"] == 1 and row["raw_pd_mdeg"] < 0), None)
    predictive_row = next((row for row in active
                           if row["state"] == 1 and
                           row["forced_tilt_mdeg"] < 0), None)
    plus_40_row = next((row for row in active
                        if row["x_mm_x10"] >= 400), None)
    reverse_row = None
    brake_rows = [row for row in (predictive_row, pd_row) if row is not None]
    brake_request_row = min(brake_rows, key=lambda row: row["time_ms"]) \
        if brake_rows else None
    if brake_request_row is not None:
        for before, after in zip(active, active[1:]):
            if (after["time_ms"] >= brake_request_row["time_ms"] and
                    after["actuator_direction"] < 0 and
                    after["actuator_counts"] <= before["actuator_counts"] - 8):
                reverse_row = after
                break
    return {
        "run_start_ms": run_start_ms,
        "pd_negative": event(pd_row),
        "predictive_brake_request": event(predictive_row),
        "entered_plus_40": event(plus_40_row),
        "actuator_reverse": event(reverse_row),
        "brake_request_to_actuator_ms": (
            reverse_row["time_ms"] - brake_request_row["time_ms"]
            if reverse_row is not None and brake_request_row is not None else None
        ),
        "pd_to_actuator_ms": (
            reverse_row["time_ms"] - pd_row["time_ms"]
            if reverse_row is not None and pd_row is not None else None
        ),
        "plus_40_to_actuator_ms": (
            reverse_row["time_ms"] - plus_40_row["time_ms"]
            if reverse_row is not None and plus_40_row is not None else None
        ),
    }


def analyze_rows(rows: list[dict[str, int]],
                 feedback: Iterable[Mapping[str, Any]] = ()) -> dict[str, Any]:
    rows = sorted(rows, key=lambda row: row["time_ms"])
    feedback_values = [str(item.get("result", "")) for item in feedback]
    plus = _direction_metrics(rows, 1, 500)
    minus = _direction_metrics(rows, 3, -500)
    # Visual feedback describes physical motion, including motion that begins
    # after an early host STOP has moved H3 into RETURNING.  Direction metrics
    # deliberately remain limited to their RUN states for candidate tuning,
    # but the contradiction guard must use the complete captured trajectory.
    positions = [int(row["x_mm_x10"]) for row in rows]
    telemetry_rolled = bool(
        (positions and max(positions) - min(positions) >= 10) or
        any(int(row["flags"]) & 32 for row in rows)
    )
    contradiction = (("no_roll" in feedback_values and telemetry_rolled) or
                     ("rolled" in feedback_values and not telemetry_rolled))
    gap_ms = max((later["time_ms"] - earlier["time_ms"]
                  for earlier, later in zip(rows, rows[1:])), default=0)
    max_abs_x10 = max((abs(row["x_mm_x10"]) for row in rows), default=0)
    observed_faults = sorted({int(row["fault"]) for row in rows
                              if int(row["fault"]) != 0})
    # APP_H3_FAULT_USER_STOP (8) is the normal host-controlled end of an
    # unsuccessful/successful capture.  Every other nonzero value is hard.
    hard_faults = [value for value in observed_faults if value != 8]
    fault = hard_faults[0] if hard_faults else 0
    hard_reasons = []
    if fault:
        hard_reasons.append(f"fault:0x{fault:08X}")
    if gap_ms > 250:
        hard_reasons.append(f"telemetry_gap:{gap_ms}ms")
    if max_abs_x10 >= 580:
        hard_reasons.append("ball_edge")
    if "unsafe" in feedback_values:
        hard_reasons.append("visual_unsafe")
    if contradiction:
        hard_reasons.append("vision_telemetry_contradiction")

    plus_overshoot = max(0, max((row["x_mm_x10"] - 500 for row in rows), default=0))
    minus_overshoot = max(0, max((-500 - row["x_mm_x10"] for row in rows), default=0))
    overshoot_x10 = max(plus_overshoot, minus_overshoot)
    oscillations = 0
    previous_sign = 0
    for row in rows:
        error = row["target_mm_x10"] - row["x_mm_x10"]
        sign = 1 if error > 10 else (-1 if error < -10 else 0)
        if sign and previous_sign and sign != previous_sign:
            oscillations += 1
        if sign:
            previous_sign = sign
    plus_passed = any(row["x_mm_x10"] >= 400 for row in rows)
    hold_rows = [row for row in rows if row["state"] == 4]
    complete = plus_passed and minus["rolled"] and bool(hold_rows)
    # A host STOP intentionally returns the platform to level after success,
    # so post-run coast is useful safety evidence but is not the scored final
    # -50 mm hold.  Prefer the firmware's already-dwelled HOLD_MINUS sample.
    tail = hold_rows[-5:] if hold_rows else rows[-5:]
    steady_error_x10 = (sum(abs(row["error_mm_x10"]) for row in tail) /
                        len(tail)) if tail else 9999.0
    active_rows = [row for row in rows if row["state"] in (1, 3, 4)]
    completion_ms = (
        hold_rows[0]["time_ms"] - active_rows[0]["time_ms"]
        if hold_rows and active_rows else
        ((plus.get("completion_ms") or 0) +
         (minus.get("completion_ms") or 0))
    )
    edge_margin_x10 = 580 - max_abs_x10
    score = None
    if complete:
        score = (completion_ms + overshoot_x10 * 100 + steady_error_x10 * 50 +
                 oscillations * 200 - edge_margin_x10 * 5)
    final_speed_ok = bool(hold_rows) and abs(hold_rows[-1]["vx_mm_s"]) <= 20
    success = bool(
        complete and completion_ms <= 5000 and overshoot_x10 <= 100 and
        steady_error_x10 <= 100 and final_speed_ok and fault == 0
    )
    return {
        "hard_failure": bool(hard_reasons), "hard_reasons": hard_reasons,
        "safe_unsuccessful": not complete and not hard_reasons,
        "complete": complete, "success": success, "plus_passed": plus_passed,
        "final_speed_ok": final_speed_ok, "plus": plus, "minus": minus,
        "telemetry_rolled": telemetry_rolled,
        "fault": fault, "max_gap_ms": gap_ms,
        "overshoot_mm": overshoot_x10 / 10.0,
        "steady_error_mm": steady_error_x10 / 10.0,
        "oscillations": oscillations, "edge_margin_mm": edge_margin_x10 / 10.0,
        "completion_ms": completion_ms, "score": score,
        "observed_faults": observed_faults,
        "actuator_reverse_lag": _persisted_lag(rows),
        "control_recovery_slow": _slow_forced_recovery(rows),
        "plus_brake_timing": _plus_brake_timing(rows),
        "feedback": feedback_values,
    }


def profile_dict(profile: object) -> dict[str, int]:
    return {name: int(getattr(profile, name)) for name in PROFILE_FIELDS}


def candidate(profile: Mapping[str, int], metrics: Mapping[str, Any]) -> dict[str, Any] | None:
    def change(name: str, value: int, reason: str) -> dict[str, Any] | None:
        old = int(profile[name])
        if value == old:
            return None
        return {"parameter": name, "parameter_id": int(PARAMETER_IDS[name]),
                "old": old, "new": value, "reason": reason}

    # Release from the breakaway angle must be physically tracked before gain
    # changes are meaningful.  Otherwise more Kp/Ki only accelerates a ball
    # while the lever is still lagging behind an already smaller command.
    if metrics["control_recovery_slow"] or metrics["actuator_reverse_lag"]:
        if int(profile["tracking_rpm"]) < 3:
            return change(
                "tracking_rpm", int(profile["tracking_rpm"]) + 1,
                "launch-angle recovery lagged the requested lever position",
            )
        return change(
            "brake_slew_mdeg_per_20ms",
            min(2500, int(profile["brake_slew_mdeg_per_20ms"]) + 250),
            "tracking RPM saturated while launch-angle recovery remained slow",
        )

    # The positive stage rolled but never reached +50, so the negative stage
    # was never entered.  This is a tracking-gain result, not evidence about
    # negative breakaway or the measured launch threshold.
    if (metrics["plus"]["rolled"] and not metrics["minus"]["observed"] and
            float(metrics["overshoot_mm"]) <= 5.0):
        if int(profile.get("ki_mdeg_per_mm_s", 0)) < 60:
            return change(
                "ki_mdeg_per_mm_s",
                min(60, int(profile.get("ki_mdeg_per_mm_s", 0)) + 10),
                "ball stopped before +50 mm; increase bounded static-friction integral",
            )
        return change("kp_mdeg_per_mm",
                      min(300, int(math.ceil(profile["kp_mdeg_per_mm"] * 1.1))),
                      "positive stage rolled but stopped before +50 mm")

    for direction in ("plus", "minus"):
        item = metrics[direction]
        if item["rolled"]:
            measured = item.get("roll_tilt_mdeg")
            name = f"{direction}_start_mdeg"
            if measured is not None:
                proposed = max(1000, int(measured) - 500)
                proposed = min(proposed, int(profile[f"{direction}_max_mdeg"]))
                if abs(proposed - int(profile[name])) >= 250:
                    return change(name, proposed, f"measured {direction} breakaway minus 0.5 degree")
            continue
        if direction == "minus" and not metrics["plus"]["rolled"]:
            break
        max_name = f"{direction}_max_mdeg"
        max_limit = 30000 if direction == "plus" else 19000
        if item["actuator_followed"]:
            result = change(max_name, min(max_limit, int(profile[max_name]) + 4000),
                            f"{direction} actuator followed but ball did not roll")
            if result:
                return result
        rpm_name = f"{direction}_launch_rpm"
        if int(profile[rpm_name]) < 8:
            return change(rpm_name, int(profile[rpm_name]) + 1,
                          f"{direction} target tilt not reached within 2.5 seconds")
        return change("breakaway_slew_mdeg_per_20ms",
                      min(1500, int(profile["breakaway_slew_mdeg_per_20ms"]) + 250),
                      f"{direction} launch RPM saturated")

    if not (metrics["plus"]["rolled"] and metrics["minus"]["rolled"]):
        return None
    if float(metrics["overshoot_mm"]) > 5.0:
        return change("kd_mdeg_per_mm_s",
                      min(120, int(math.ceil(profile["kd_mdeg_per_mm_s"] * 1.2))),
                      "overshoot exceeded 5 mm")
    if metrics["actuator_reverse_lag"]:
        return change("tracking_rpm", min(3, int(profile["tracking_rpm"]) + 1),
                      "reverse tilt actuator lag exceeded 2 degrees for 300 ms")
    if metrics["control_recovery_slow"]:
        return change("brake_slew_mdeg_per_20ms",
                      min(2500, int(profile["brake_slew_mdeg_per_20ms"]) + 250),
                      "controller recovered launch tilt too slowly")
    if not metrics["complete"]:
        return change("kp_mdeg_per_mm",
                      min(300, int(math.ceil(profile["kp_mdeg_per_mm"] * 1.1))),
                      "target was not reached without overshoot")
    return None


def candidate_is_single_bounded(profile: Mapping[str, int], decision: Mapping[str, Any]) -> bool:
    changed = dict(profile)
    name = str(decision["parameter"])
    changed[name] = int(decision["new"])
    bounds = {
        "plus_max_mdeg": (3000, 30000), "minus_max_mdeg": (3000, 19000),
        "plus_launch_rpm": (1, 8), "minus_launch_rpm": (1, 8),
        "tracking_rpm": (1, 3), "kp_mdeg_per_mm": (20, 300),
        "kd_mdeg_per_mm_s": (0, 120),
        "breakaway_slew_mdeg_per_20ms": (200, 1500),
        "brake_slew_mdeg_per_20ms": (200, 2500),
        "normal_slew_mdeg_per_20ms": (100, 500),
        "ki_mdeg_per_mm_s": (0, 100),
    }
    if name in bounds and not bounds[name][0] <= changed[name] <= bounds[name][1]:
        return False
    return (changed["plus_start_mdeg"] <= changed["plus_max_mdeg"] and
            changed["minus_start_mdeg"] <= changed["minus_max_mdeg"] and
            changed["tracking_rpm"] <= changed["plus_launch_rpm"] <= 8 and
            changed["tracking_rpm"] <= changed["minus_launch_rpm"] <= 8 and
            changed["normal_slew_mdeg_per_20ms"] <=
            changed["breakaway_slew_mdeg_per_20ms"] <=
            changed["brake_slew_mdeg_per_20ms"])


class Runner:
    def __init__(self, args: argparse.Namespace, session_dir: Path, resume: bool = False):
        self.args = args
        self.directory = session_dir
        self.resume = resume
        self.state_path = session_dir / "session.json"
        self.control_path = session_dir / "control.json"
        self.owner_path = session_dir / "owner.lock"
        self.session: Session | None = None
        self.token = 0
        self.prior_monitor = False
        self.events: queue.Queue[object] | None = None
        self.event_stream: Any | None = None
        self.state = read_json(self.state_path, {}) if resume else {}
        self.horizontal_counts: int | None = self.state.get("horizontal_counts")

    def save_state(self, **updates: Any) -> None:
        self.state.update(updates)
        self.state["updated_at"] = utc_now()
        atomic_json(self.state_path, self.state)

    def aborted(self) -> bool:
        control = read_json(self.control_path, {})
        return bool(control.get("abort"))

    def emergency(self, reason: str) -> None:
        assert self.session is not None
        self.save_state(phase="aborting", wait_reason=reason)
        must_emergency_stop = True
        try:
            before = self.session.get_h3_tune()
            must_emergency_stop = bool(before.host_run_active or before.h3_active)
        except Exception:
            # If the link outcome itself is unknown, the raw emergency byte is
            # still the only safe choice.
            pass
        if must_emergency_stop:
            self.session.emergency_stop()
            time.sleep(0.05)
        try:
            snapshot = self.session.revoke_h3_tune(self.token)
            atomic_json(self.directory / "rollback.json", {
                "time": utc_now(), "reason": reason,
                "active": profile_dict(snapshot.active),
                "active_revision": snapshot.active_revision,
                "stop_reason": snapshot.stop_reason,
            })
        except Exception as exc:
            atomic_json(self.directory / "rollback.json", {
                "time": utc_now(), "reason": reason, "error": repr(exc),
                "expected": self.state.get("golden_profile"),
            })
        self.save_state(phase="aborted", wait_reason=reason)

    def next_row(self, timeout: float) -> dict[str, int] | None:
        assert self.events is not None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                event = self.events.get(timeout=max(0.0, deadline - time.monotonic()))
            except queue.Empty:
                return None
            row = h3_row(event)
            if row is not None:
                return row
            if self.event_stream is not None:
                value = {
                    "host_time": utc_now(), "type": getattr(event, "type", None),
                    "sequence": getattr(event, "sequence", None),
                    "class": type(event).__name__, "repr": repr(event)[:500],
                }
                self.event_stream.write(json.dumps(value, ensure_ascii=False) + "\n")
                self.event_stream.flush()
        return None

    def wait_center(self, writer: csv.DictWriter, stream: Any, run_number: int) -> None:
        stable_since: float | None = None
        countdown_started: float | None = None
        last_row_at = time.monotonic()
        while True:
            if self.aborted():
                raise InterruptedError("operator_abort")
            row = self.next_row(0.1)
            now = time.monotonic()
            if row is not None:
                writer.writerow(row); stream.flush()
                last_row_at = now
                if row["fault"] not in (0, 8):
                    raise RuntimeError(f"telemetry_fault:0x{row['fault']:08X}")
                centered = (abs(row["x_mm_x10"]) <= 30 and
                            abs(row["vx_mm_s"]) < 5 and row["vision_age_ms"] <= 100 and
                            bool(row["flags"] & 1) and bool(row["flags"] & 2))
                if centered:
                    self.horizontal_counts = int(row["actuator_target_counts"])
                    self.state["horizontal_counts"] = self.horizontal_counts
                    stable_since = now if stable_since is None else stable_since
                else:
                    stable_since = None
                    countdown_started = None
            if stable_since is not None and now - stable_since >= 1.0:
                countdown_started = now if countdown_started is None else countdown_started
                remaining = max(0, 3 - int(now - countdown_started))
                self.save_state(phase="countdown", current_run=run_number,
                                wait_reason=f"center stable; start in {remaining}s")
                if now - countdown_started >= 3.0:
                    return
            elif now - last_row_at > 1.0:
                self.save_state(phase="waiting_center", current_run=run_number,
                                wait_reason="no H3 schema 3 telemetry")
            else:
                self.save_state(phase="waiting_center", current_run=run_number,
                                wait_reason="place ball at 0±3 mm, |v|<5 mm/s; wait H3 ready")

    def capture_run(self, writer: csv.DictWriter, stream: Any,
                    run_number: int, snapshot: Any) -> tuple[list[dict[str, int]], Any]:
        assert self.session is not None
        rows: list[dict[str, int]] = []
        snapshot = self.session.start_h3_tune(self.token, snapshot=snapshot)
        expected_run_id = int(snapshot.run_id)
        self.save_state(phase="running", current_run=run_number,
                        profile_revision=snapshot.active_revision, wait_reason="")
        heartbeat = H3HeartbeatPump(self.session, self.token)
        heartbeat.start()
        # Reserve at least 3 seconds of the firmware's 12 second run lease for
        # the comparatively slow mechanical return-to-horizontal sequence.
        soft_stop_at = time.monotonic() + 8.5
        run_deadline = time.monotonic() + 13.0
        last_row_at = time.monotonic()
        stop_sent = False
        started_observed = False
        try:
            while True:
                heartbeat.raise_if_failed()
                if self.aborted():
                    raise InterruptedError("operator_abort")
                row = self.next_row(0.02)
                if row is not None:
                    writer.writerow(row); stream.flush()
                    last_row_at = time.monotonic()
                    started_observed, terminal = run_frame_state(
                        row, expected_run_id, started_observed
                    )
                    if int(row["run_id"]) == expected_run_id:
                        rows.append(row)
                        if row["fault"] not in (0, 8):
                            # APP_H3 reports EDGE/VISION/etc. while it is
                            # already returning (state 5), then publishes the
                            # terminal fault state (6).  Keep the confirmed
                            # heartbeat alive so that safety return can finish.
                            if row["state"] not in (5, 6):
                                raise RuntimeError(
                                    f"telemetry_fault:0x{row['fault']:08X}"
                                )
                        if (not stop_sent and host_launch_pulse_required(
                                row, self.horizontal_counts,
                                getattr(self.args, "host_launch_pulse_mdeg", None))):
                            self.save_state(
                                phase="launch_pulse_return", current_run=run_number,
                                wait_reason=(
                                    "launch pulse encoder threshold reached; "
                                    "returning horizontal before breakaway acceleration"
                                ),
                            )
                            self.session.stop_h3_tune(
                                self.token, timeout=0.18
                            )
                            stop_sent = True
                        if (not stop_sent and row["fault"] == 0 and
                                host_soft_stop_required(row)):
                            self.save_state(
                                phase="host_soft_stop", current_run=run_number,
                                wait_reason=(
                                    "outward ball speed plus actuator lag; "
                                    "returning horizontal"
                                ),
                            )
                            self.session.stop_h3_tune(
                                self.token, timeout=0.18
                            )
                            stop_sent = True
                        if row["state"] == 4 and not stop_sent:
                            # Keep the heartbeat pump running while STOP returns
                            # the actuator to level; the firmware still watches
                            # the lease throughout that motion.
                            self.session.stop_h3_tune(
                                self.token, timeout=0.18
                            )
                            stop_sent = True
                        if terminal:
                            # Returning the platform to level does not mean the
                            # ball has stopped.  Preserve a bounded post-run
                            # tail so open-loop coast and final visual position
                            # are represented in metrics instead of ending on
                            # a still-fast terminal sample.
                            self.save_state(
                                phase="post_run_coast", current_run=run_number,
                                wait_reason="platform level; recording ball coast",
                            )
                            tail_deadline = time.monotonic() + 2.0
                            tail_stable_since: float | None = None
                            while time.monotonic() < tail_deadline:
                                heartbeat.raise_if_failed()
                                tail = self.next_row(0.02)
                                now_tail = time.monotonic()
                                if tail is None:
                                    if now_tail - last_row_at > 0.3:
                                        raise RuntimeError(
                                            "telemetry_gap: post-run coast stalled >300ms"
                                        )
                                    continue
                                writer.writerow(tail); stream.flush()
                                last_row_at = now_tail
                                if int(tail["run_id"]) != expected_run_id:
                                    continue
                                rows.append(tail)
                                # A terminal firmware fault is the evidence we
                                # are here to preserve.  Keep recording its
                                # bounded coast; analyze_rows will classify it
                                # as hard only after the partial files have
                                # been atomically promoted.
                                stable = (
                                    abs(int(tail["vx_mm_s"])) < 5 and
                                    int(tail["vision_age_ms"]) <= 100
                                )
                                if stable:
                                    tail_stable_since = (
                                        now_tail if tail_stable_since is None
                                        else tail_stable_since
                                    )
                                    if now_tail - tail_stable_since >= 0.3:
                                        break
                                else:
                                    tail_stable_since = None
                            heartbeat.stop()
                            heartbeat.raise_if_failed()
                            snapshot = self.session.get_h3_tune()
                            return rows, snapshot
                now = time.monotonic()
                if now >= soft_stop_at and not stop_sent:
                    # End a non-converging trial before the firmware's 12 s
                    # authorization watchdog.  This produces USER_STOP (8),
                    # returns the lever to horizontal, and remains a normal
                    # safe-unsuccessful sample for candidate generation.
                    self.session.stop_h3_tune(self.token, timeout=0.18)
                    stop_sent = True
                if now - last_row_at > 0.3:
                    raise RuntimeError(
                        "telemetry_gap: H3 schema 3 callback stalled >300ms"
                    )
                if now >= run_deadline:
                    raise RuntimeError(
                        "run_timeout: host did not observe terminal H3 state"
                    )
        finally:
            heartbeat.stop()

    def apply_decision(self, snapshot: Any, decision: Mapping[str, Any]) -> Any:
        assert self.session is not None
        current = profile_dict(snapshot.active)
        if not candidate_is_single_bounded(current, decision):
            raise RuntimeError(f"candidate violates bounds/relations: {decision}")
        staged = self.session.stage_h3_tune_field(
            int(decision["parameter_id"]), int(decision["new"]), snapshot=snapshot
        )
        applied = self.session.apply_h3_tune(snapshot=staged)
        changed = [name for name in PROFILE_FIELDS
                   if int(getattr(applied.active, name)) != current[name]]
        if changed != [decision["parameter"]]:
            raise RuntimeError(f"firmware changed unexpected fields: {changed}")
        return applied

    def audit(self, identity: Any, snapshot: Any) -> None:
        if identity.firmware_id != self.args.firmware_id:
            raise RuntimeError(f"firmware {identity.firmware_id!r} != {self.args.firmware_id!r}")
        if int(identity.enabled_capabilities) & H3_TUNE_CAPABILITY != H3_TUNE_CAPABILITY:
            raise RuntimeError("m6-h3-tune-v1 capability was not negotiated")
        if int(identity.enabled_capabilities) & H3_KI_CAPABILITY != H3_KI_CAPABILITY:
            raise RuntimeError("m6-h3-ki-v1 capability was not negotiated")
        if int(identity.enabled_capabilities) & H3_MANEUVER_CAPABILITY != H3_MANEUVER_CAPABILITY:
            raise RuntimeError("m6-h3-maneuver-v1 capability was not negotiated")
        status = self.session.get_status()  # type: ignore[union-attr]
        if status.fault_bits:
            raise RuntimeError(f"preflight safety fault 0x{status.fault_bits:08X}; not clearing")
        if status.system_state != 2:  # frozen wire IDLE state
            raise RuntimeError(f"system state {status.system_state} is not IDLE")
        if snapshot.h3_active or snapshot.host_run_active:
            raise RuntimeError("H3 is already active")

    def execute(self) -> int:
        self.directory.mkdir(parents=True, exist_ok=True)
        self.owner_path.parent.mkdir(parents=True, exist_ok=True)
        if self.resume and self.owner_path.exists():
            owner_info = read_json(self.owner_path, {})
            pid = int(owner_info.get("pid", 0)) if isinstance(owner_info, dict) else 0
            alive = False
            if pid > 0:
                try:
                    os.kill(pid, 0)
                    alive = True
                except OSError:
                    pass
            if not alive:
                self.owner_path.unlink()
        try:
            owner = self.owner_path.open("x", encoding="utf-8")
        except FileExistsError as exc:
            raise RuntimeError(f"session already has a serial owner: {self.owner_path}") from exc
        owner.write(json.dumps({"pid": os.getpid(), "started_at": utc_now()})); owner.flush()
        try:
            self.session = Session(port=self.args.port, timeout=0.8,
                                   firmware_id=self.args.firmware_id)
            identity = self.session.connect()
            snapshot = self.session.get_h3_tune()
            self.audit(identity, snapshot)
            if self.resume:
                golden = self.state.get("golden_profile")
                if golden and profile_dict(snapshot.active) != golden:
                    raise RuntimeError("resume audit: active RAM profile is not recorded golden")
                if self.state.get("firmware_id") != identity.firmware_id:
                    raise RuntimeError("resume audit: firmware identity changed")
            else:
                if snapshot.dirty:
                    snapshot = self.session.revert_h3_tune(snapshot=snapshot)
                boot_profile = profile_dict(snapshot.active)
                seed_decisions: list[dict[str, Any]] = []
                initial_cap = self.args.initial_breakaway_cap_mdeg
                if initial_cap is not None:
                    for direction in ("plus", "minus"):
                        start_name = f"{direction}_start_mdeg"
                        max_name = f"{direction}_max_mdeg"
                        current = profile_dict(snapshot.active)
                        if initial_cap < current[start_name]:
                            raise RuntimeError(
                                f"initial breakaway cap {initial_cap} is below "
                                f"{start_name}={current[start_name]}"
                            )
                        capped = min(current[max_name], initial_cap)
                        if capped == current[max_name]:
                            continue
                        decision = {
                            "parameter": max_name,
                            "parameter_id": int(PARAMETER_IDS[max_name]),
                            "old": current[max_name], "new": capped,
                            "reason": "operator-observed edge event; safe pre-batch cap",
                        }
                        snapshot = self.apply_decision(snapshot, decision)
                        seed_decisions.append(decision)
                initial_kp = self.args.initial_kp
                if initial_kp is not None:
                    current = profile_dict(snapshot.active)
                    if initial_kp != current["kp_mdeg_per_mm"]:
                        decision = {
                            "parameter": "kp_mdeg_per_mm",
                            "parameter_id": int(PARAMETER_IDS["kp_mdeg_per_mm"]),
                            "old": current["kp_mdeg_per_mm"], "new": initial_kp,
                            "reason": "carry forward prior safe-unsuccessful run",
                        }
                        snapshot = self.apply_decision(snapshot, decision)
                        seed_decisions.append(decision)
                initial_ki = self.args.initial_ki
                if initial_ki is not None:
                    current = profile_dict(snapshot.active)
                    if initial_ki != current["ki_mdeg_per_mm_s"]:
                        decision = {
                            "parameter": "ki_mdeg_per_mm_s",
                            "parameter_id": H3_KI_PARAMETER_ID,
                            "old": current["ki_mdeg_per_mm_s"], "new": initial_ki,
                            "reason": "operator-requested static-friction integral",
                        }
                        snapshot = self.apply_decision(snapshot, decision)
                        seed_decisions.append(decision)
                golden = profile_dict(snapshot.active)
                self.state = {
                    "schema": "h3-autotune-session-v1", "created_at": utc_now(),
                    "port": self.args.port, "firmware_id": identity.firmware_id,
                    "boot_id": identity.boot_id, "round_limit": self.args.rounds,
                    "completed_runs": 0, "golden_profile": golden,
                    "boot_profile": boot_profile, "best_profile": golden,
                    "safety_seed": seed_decisions, "success_streak": 0,
                }
                atomic_json(self.directory / "golden_profile.json", golden)
                if seed_decisions:
                    atomic_json(self.directory / "safety_seed.json", {
                        "time": utc_now(), "boot_profile": boot_profile,
                        "active_profile": golden, "decisions": seed_decisions,
                    })
            completed = int(self.state.get("completed_runs", 0))
            remaining = min(8, self.args.rounds) - completed
            if remaining <= 0:
                raise RuntimeError("session has no remaining run quota")
            snapshot = self.session.authorize_h3_tune(
                remaining, ttl_ms=1_800_000, run_timeout_ms=12_000,
                snapshot=snapshot,
            )
            self.token = snapshot.auth_token
            self.prior_monitor = self.session.get_vision_uart_monitor().receive_enabled
            if self.prior_monitor:
                self.session.set_vision_uart_monitor(False)
            self.save_state(phase="preflight_complete", authorization_token=self.token,
                            authorization_remaining=remaining,
                            active_profile=profile_dict(snapshot.active))

            with self.session.event_tap() as events:
                self.events = events
                last_metrics: dict[str, Any] | None = None
                last_profile: dict[str, int] | None = None
                best_metrics = self.state.get("best_metrics")
                best_profile = dict(self.state["best_profile"])
                success_streak = int(self.state.get("success_streak", 0))
                for run_number in range(completed + 1, min(8, self.args.rounds) + 1):
                    run_dir = self.directory / "runs" / f"run_{run_number:03d}"
                    run_dir.mkdir(parents=True, exist_ok=True)
                    partial = run_dir / "telemetry.partial.csv"
                    events_partial = run_dir / "events.partial.jsonl"
                    with (partial.open("w", newline="", encoding="utf-8") as stream,
                          events_partial.open("w", encoding="utf-8") as event_stream):
                        self.event_stream = event_stream
                        writer = csv.DictWriter(stream, fieldnames=H3_FIELDS)
                        writer.writeheader(); stream.flush()
                        self.wait_center(writer, stream, run_number)

                        decision = None
                        if last_metrics is not None and last_profile is not None:
                            feedback = load_feedback(self.directory, run_number - 1)
                            previous_rows = load_rows(run_dir.parent / f"run_{run_number - 1:03d}")
                            last_metrics = analyze_rows(previous_rows, feedback)
                            atomic_json(run_dir.parent / f"run_{run_number - 1:03d}" /
                                        "metrics.json", last_metrics)
                            if last_metrics["hard_failure"]:
                                raise InterruptedError(";".join(last_metrics["hard_reasons"]))
                            accepted = True
                            if (last_profile != best_profile and
                                    last_metrics["complete"] and best_metrics):
                                accepted = bool(
                                    last_metrics["score"] <= best_metrics["score"] * 0.95 and
                                    last_metrics["overshoot_mm"] <= best_metrics["overshoot_mm"] and
                                    last_metrics["edge_margin_mm"] >= best_metrics["edge_margin_mm"]
                                )
                            if accepted:
                                best_profile = dict(last_profile)
                                if last_metrics["complete"]:
                                    best_metrics = last_metrics
                                decision = candidate(best_profile, last_metrics)
                            else:
                                changed = [name for name in PROFILE_FIELDS
                                           if last_profile[name] != best_profile[name]]
                                if len(changed) != 1:
                                    raise RuntimeError("rejected profile is not one field from best")
                                name = changed[0]
                                decision = {"parameter": name,
                                    "parameter_id": int(PARAMETER_IDS[name]),
                                    "old": last_profile[name], "new": best_profile[name],
                                    "reason": "candidate improved score by less than 5% or degraded guards",
                                    "rollback": True}
                            atomic_json(run_dir.parent / f"run_{run_number - 1:03d}" /
                                        "decision.json", {"accepted": accepted,
                                        "next_candidate": decision})
                        if decision is not None:
                            snapshot = self.apply_decision(snapshot, decision)
                        current_profile = profile_dict(snapshot.active)
                        atomic_json(run_dir / "profile.json", current_profile)
                        self.save_state(active_profile=current_profile,
                                        profile_revision=snapshot.active_revision,
                                        pending_decision=decision)
                        rows, snapshot = self.capture_run(writer, stream, run_number, snapshot)
                        stream.flush(); os.fsync(stream.fileno())
                        event_stream.flush(); os.fsync(event_stream.fileno())
                        self.event_stream = None
                    os.replace(partial, run_dir / "telemetry.csv")
                    os.replace(events_partial, run_dir / "events.jsonl")
                    feedback = load_feedback(self.directory, run_number)
                    metrics = analyze_rows(rows, feedback)
                    atomic_json(run_dir / "metrics.json", metrics)
                    if metrics["hard_failure"]:
                        raise InterruptedError(";".join(metrics["hard_reasons"]))
                    if metrics["success"] and current_profile == last_profile:
                        success_streak += 1
                    elif metrics["success"]:
                        success_streak = 1
                    else:
                        success_streak = 0
                    last_metrics = metrics
                    last_profile = current_profile
                    completed = run_number
                    self.save_state(completed_runs=completed,
                                    success_streak=success_streak,
                                    best_profile=best_profile, best_metrics=best_metrics,
                                    phase="run_complete",
                                    wait_reason="place ball back at center")
                    if success_streak >= 3:
                        break

            if last_metrics is not None and last_profile is not None:
                final_feedback = load_feedback(self.directory, completed)
                final_metrics = analyze_rows(
                    load_rows(self.directory / "runs" / f"run_{completed:03d}"),
                    final_feedback,
                )
                if final_metrics["hard_failure"]:
                    raise InterruptedError(";".join(final_metrics["hard_reasons"]))
                final_accepted = True
                if (last_profile != best_profile and final_metrics["complete"] and
                        best_metrics):
                    final_accepted = bool(
                        final_metrics["score"] <= best_metrics["score"] * 0.95 and
                        final_metrics["overshoot_mm"] <= best_metrics["overshoot_mm"] and
                        final_metrics["edge_margin_mm"] >= best_metrics["edge_margin_mm"]
                    )
                if final_accepted:
                    best_profile = dict(last_profile)
                    if final_metrics["complete"]:
                        best_metrics = final_metrics
                else:
                    changed = [name for name in PROFILE_FIELDS
                               if last_profile[name] != best_profile[name]]
                    if len(changed) != 1:
                        raise RuntimeError("final rejected profile is not one field from best")
                    name = changed[0]
                    rollback = {"parameter": name,
                        "parameter_id": int(PARAMETER_IDS[name]),
                        "old": last_profile[name], "new": best_profile[name],
                        "reason": "final candidate failed 5%/guard acceptance",
                        "rollback": True}
                    snapshot = self.apply_decision(snapshot, rollback)
                atomic_json(self.directory / "runs" / f"run_{completed:03d}" /
                            "decision.json", {"accepted": final_accepted,
                            "next_candidate": None})
            snapshot = self.session.revoke_h3_tune(self.token)
            final = {
                "completed_at": utc_now(), "completed_runs": completed,
                "success_streak": success_streak,
                "validated": success_streak >= 3,
                "active_profile": profile_dict(snapshot.active),
                "active_revision": snapshot.active_revision,
                "firmware_id": identity.firmware_id,
                "firmware_unchanged": identity.firmware_id == self.state["firmware_id"],
                "ram_only": True,
            }
            atomic_json(self.directory / "active_profile.json", final["active_profile"])
            atomic_json(self.directory / "final_report.json", final)
            self.save_state(phase="complete", wait_reason="", **final)
            return 0 if final["validated"] else 2
        except InterruptedError as exc:
            if self.session is not None:
                self.emergency(str(exc))
            return 3
        except Exception as exc:
            if self.session is not None and self.token:
                try:
                    self.emergency(f"exception:{type(exc).__name__}:{exc}")
                except Exception:
                    self.save_state(phase="failed", wait_reason=repr(exc))
            else:
                self.save_state(phase="failed", wait_reason=repr(exc))
            return 4
        finally:
            if self.session is not None:
                try:
                    self.session.set_vision_uart_monitor(self.prior_monitor)
                except Exception:
                    pass
                self.session.close()
            owner.close()
            try:
                self.owner_path.unlink()
            except OSError:
                pass


def load_rows(run_dir: Path) -> list[dict[str, int]]:
    path = run_dir / "telemetry.csv"
    if not path.exists():
        path = run_dir / "telemetry.partial.csv"
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            return [{key: int(value) for key, value in row.items()}
                    for row in csv.DictReader(stream)]
    except OSError:
        return []


def offline_analyze(session_dir: Path) -> dict[str, Any]:
    reports = []
    for run_dir in sorted((session_dir / "runs").glob("run_*")):
        try:
            run_number = int(run_dir.name.split("_")[-1])
        except ValueError:
            continue
        metrics = analyze_rows(load_rows(run_dir), load_feedback(session_dir, run_number))
        atomic_json(run_dir / "metrics.json", metrics)
        reports.append({"run": run_number, "metrics": metrics})
    report = {"analyzed_at": utc_now(), "runs": reports,
              "hard_failures": [item["run"] for item in reports
                                if item["metrics"]["hard_failure"]]}
    atomic_json(session_dir / "analysis_report.json", report)
    return report


def cmd_feedback(args: argparse.Namespace) -> int:
    session_dir = resolve_session(args)
    feedback_dir = session_dir / "feedback"
    value = {"time": utc_now(), "run": args.run, "result": args.result,
             "note": args.note or ""}
    stamp = time.time_ns()
    atomic_json(feedback_dir / f"run_{args.run:03d}_{stamp}.json", value)
    if args.result == "unsafe":
        atomic_json(session_dir / "control.json", {
            "abort": True, "time": utc_now(), "reason": "visual_unsafe"
        })
    print(json.dumps(value, ensure_ascii=False))
    return 0


def cmd_abort(args: argparse.Namespace) -> int:
    session_dir = resolve_session(args)
    value = {"abort": True, "time": utc_now(), "reason": args.reason}
    atomic_json(session_dir / "control.json", value)
    print(json.dumps(value, ensure_ascii=False))
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    session_dir = resolve_session(args)
    value = read_json(session_dir / "session.json", {})
    value["session"] = str(session_dir)
    value["owner_active"] = (session_dir / "owner.lock").exists()
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT / "captures" / "h3_autotune")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run")
    run.add_argument("--port", default="COM10")
    run.add_argument("--firmware-id", default=FIRMWARE_ID)
    run.add_argument("--rounds", type=int, default=8, choices=range(1, 9))
    run.add_argument("--session", type=Path)
    run.add_argument(
        "--initial-breakaway-cap-mdeg", type=int,
        help="RAM-only pre-batch cap for both direction maxima (for example 8000)",
    )
    run.add_argument(
        "--initial-kp", type=int,
        help="RAM-only pre-batch Kp carried forward from an earlier safe run",
    )
    run.add_argument(
        "--initial-ki", type=int,
        help="RAM-only pre-batch Ki for bounded static-friction compensation",
    )
    run.add_argument(
        "--host-launch-pulse-mdeg", type=int,
        help=("diagnostic only: command return-to-horizontal when encoder "
              "reaches this launch angle, before waiting for ball motion"),
    )
    resume = sub.add_parser("resume")
    resume.add_argument("--port", default="COM10")
    resume.add_argument("--firmware-id", default=FIRMWARE_ID)
    resume.add_argument("--rounds", type=int, default=8, choices=range(1, 9))
    resume.add_argument("--session", type=Path, required=True)
    resume.add_argument("--host-launch-pulse-mdeg", type=int)
    for name in ("status", "analyze", "abort", "feedback"):
        item = sub.add_parser(name)
        item.add_argument("--session", type=Path)
    sub.choices["abort"].add_argument("--reason", default="operator_abort")
    feedback = sub.choices["feedback"]
    feedback.add_argument("--run", type=int, required=True)
    feedback.add_argument("--result", choices=FEEDBACK_RESULTS, required=True)
    feedback.add_argument("--note")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.root = args.root.resolve()
    if args.command == "status":
        return cmd_status(args)
    if args.command == "feedback":
        return cmd_feedback(args)
    if args.command == "abort":
        return cmd_abort(args)
    if args.command == "analyze":
        print(json.dumps(offline_analyze(resolve_session(args)), ensure_ascii=False,
                         indent=2, sort_keys=True))
        return 0
    if args.command == "run":
        session_dir = (args.session.resolve() if args.session else
                       args.root / datetime.now().strftime("%Y%m%d-%H%M%S"))
        print(f"session={session_dir}", flush=True)
        return Runner(args, session_dir).execute()
    return Runner(args, args.session.resolve(), resume=True).execute()


if __name__ == "__main__":
    raise SystemExit(main())
