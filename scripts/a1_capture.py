"""Read-only A1 boot-return capture for the active acceptance firmware.

The script waits for the already-enumerated debug COM port, negotiates M6 V2,
and reads only cached stepper/lever/status snapshots.  It never requests a
UART3 refresh and exposes no motion command path.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from datetime import datetime
from enum import Enum
import json
from pathlib import Path
import time
from typing import Any

from m6link import Session


ABSOLUTE_BOTTOM = 38857
HORIZONTAL = 4396
ACCEPTANCE_MIN = HORIZONTAL - 64
ACCEPTANCE_MAX = HORIZONTAL + 64
MOTOR_ENABLED_AND_IN_POSITION = 0x03
UART_FIELDS = (
    "hardware_errors",
    "invalid_frames",
    "checksum_errors",
    "rx_overflow",
)


def plain(value: Any) -> Any:
    if isinstance(value, Enum):
        return int(value.value)
    if is_dataclass(value):
        return {key: plain(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def absolute_from_relative(relative: int) -> int:
    return (ABSOLUTE_BOTTOM + int(relative)) & 0xFFFF


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(plain(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def uart_counts(stepper: dict[str, Any]) -> dict[str, int]:
    return {field: int(stepper[field]) for field in UART_FIELDS}


def direction(value: int) -> int:
    return 1 if value > 0 else -1 if value < 0 else 0


def summarize(identity: Any, samples: list[dict[str, Any]], errors: list[str]) -> dict[str, Any]:
    complete = [sample for sample in samples if sample.get("lever") and sample.get("stepper")]
    result: dict[str, Any] = {
        "firmware": identity.firmware_id,
        "boot_id": identity.boot_id,
        "samples": len(samples),
        "complete_samples": len(complete),
        "capture_errors": errors,
        "pass": False,
        "failure_reasons": [],
    }
    failures: list[str] = result["failure_reasons"]
    if not complete:
        failures.append("NO_COMPLETE_SAMPLES")
        return result

    first = complete[0]
    last = complete[-1]
    initial_relative = int(first["lever"]["current_counts"])
    initial_motor = int(first["stepper"]["position_raw"])
    final_relative = int(last["lever"]["current_counts"])
    final_motor = int(last["stepper"]["position_raw"])
    start_uart = uart_counts(first["stepper"])
    end_uart = uart_counts(last["stepper"])
    result.update({
        "initial_relative": initial_relative,
        "initial_absolute_0x31": absolute_from_relative(initial_relative),
        "initial_motor_0x36": initial_motor,
        "final_relative": final_relative,
        "final_absolute_0x31": absolute_from_relative(final_relative),
        "final_motor_0x36": final_motor,
        "relative_displacement": final_relative - initial_relative,
        "motor_displacement": final_motor - initial_motor,
        "displacement_difference":
            (final_relative - initial_relative) - (final_motor - initial_motor),
        "uart3_start": start_uart,
        "uart3_end": end_uart,
        "final_speed_rpm": int(last["stepper"]["speed_rpm"]),
        "final_motor_status": int(last["stepper"]["motor_status"]),
        "final_actuator_fault": int(last["lever"]["actuator_fault"]),
    })

    reached = [sample for sample in complete if
               ACCEPTANCE_MIN <= int(sample["lever"]["current_counts"]) <= ACCEPTANCE_MAX and
               int(sample["stepper"]["speed_rpm"]) == 0 and
               (int(sample["stepper"]["motor_status"]) &
                MOTOR_ENABLED_AND_IN_POSITION) == MOTOR_ENABLED_AND_IN_POSITION]
    if not reached:
        failures.append("NEVER_REACHED_HORIZONTAL")
    else:
        reached_at = float(reached[0]["elapsed_s"])
        hold = [sample for sample in complete if
                reached_at <= float(sample["elapsed_s"]) <= reached_at + 10.0]
        hold_positions = [int(sample["lever"]["current_counts"]) for sample in hold]
        hold_duration = (float(hold[-1]["elapsed_s"]) - float(hold[0]["elapsed_s"])) if hold else 0.0
        result.update({
            "reached_horizontal_s": reached_at,
            "hold_duration_s": hold_duration,
            "hold_relative_min": min(hold_positions) if hold_positions else None,
            "hold_relative_max": max(hold_positions) if hold_positions else None,
        })
        if reached_at > 5.0:
            failures.append("HORIZONTAL_LATER_THAN_5S")
        if hold_duration < 9.5:
            failures.append("HOLD_SHORTER_THAN_10S")
        if (not hold_positions or min(hold_positions) < ACCEPTANCE_MIN or
                max(hold_positions) > ACCEPTANCE_MAX):
            failures.append("HOLD_OUTSIDE_ACCEPTANCE_WINDOW")

    if any(start_uart[field] != 0 for field in UART_FIELDS):
        failures.append("UART3_NOT_ZERO_AT_START")
    if any(end_uart[field] != 0 for field in UART_FIELDS):
        failures.append("UART3_NOT_ZERO_AT_END")
    if any(end_uart[field] != start_uart[field] for field in UART_FIELDS):
        failures.append("UART3_COUNTER_GROWTH")
    if int(last["stepper"]["speed_rpm"]) != 0:
        failures.append("FINAL_SPEED_NONZERO")
    if (int(last["stepper"]["motor_status"]) & MOTOR_ENABLED_AND_IN_POSITION) != MOTOR_ENABLED_AND_IN_POSITION:
        failures.append("FINAL_MOTOR_NOT_ENABLED_AND_IN_POSITION")
    if int(last["lever"]["actuator_fault"]) != 0:
        failures.append("ACTUATOR_FAULT")
    if last.get("status"):
        status = last["status"]
        result.update({
            "final_system_state": int(status["system_state"]),
            "final_action_state": int(status["action_state"]),
            "final_safety_fault": int(status["fault_bits"]),
            "final_wheel_target": status["target_mm_s"],
            "final_wheel_measured": status["measured_mm_s"],
        })
        if int(status["fault_bits"]) != 0:
            failures.append("SAFETY_FAULT")
        if any(int(value) != 0 for value in status["target_mm_s"] + status["measured_mm_s"]):
            failures.append("WHEELS_NONZERO")

    expected = (0 if ACCEPTANCE_MIN <= initial_relative <= ACCEPTANCE_MAX else
                direction(HORIZONTAL - initial_relative))
    relative_direction = direction(final_relative - initial_relative)
    motor_direction = direction(final_motor - initial_motor)
    result.update({
        "expected_direction": expected,
        "relative_direction": relative_direction,
        "motor_direction": motor_direction,
        "finite_position_response_seen": any(
            int(sample["stepper"]["last_function"]) == 0xFD for sample in complete
        ),
    })
    if expected and relative_direction != expected:
        failures.append("RELATIVE_WRONG_DIRECTION")
    if expected and motor_direction != expected:
        failures.append("MOTOR_WRONG_DIRECTION")
    if abs((final_relative - initial_relative) - (final_motor - initial_motor)) > 128:
        failures.append("FEEDBACK_DISAGREEMENT")
    if not (ACCEPTANCE_MIN <= final_relative <= ACCEPTANCE_MAX):
        failures.append("FINAL_OUTSIDE_ACCEPTANCE_WINDOW")
    if errors:
        failures.append("CAPTURE_ERRORS")
    result["pass"] = not failures
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--firmware-id", default="M6-runtime-20260731-r75-dev4")
    parser.add_argument("--wait-seconds", type=float, default=300.0)
    parser.add_argument("--capture-seconds", type=float, default=20.0)
    parser.add_argument("--period-seconds", type=float, default=0.20)
    parser.add_argument("--wait-for-reset", action="store_true",
                        help="arm on the current boot, then capture only after boot_id changes")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=False)
    status_path = args.output / "capture_status.json"
    samples_path = args.output / "telemetry.jsonl"
    write_json(status_path, {
        "state": "waiting_for_power",
        "port": args.port,
        "firmware_id": args.firmware_id,
        "started": datetime.now().astimezone().isoformat(timespec="milliseconds"),
    })
    print(f"READY_TO_POWER output={args.output}", flush=True)

    deadline = time.monotonic() + args.wait_seconds
    session: Session | None = None
    identity = None
    last_wait_notice = 0.0
    while time.monotonic() < deadline:
        candidate = Session(
            port=args.port,
            timeout=0.20,
            firmware_id=args.firmware_id,
        )
        try:
            identity = candidate.connect()
            session = candidate
            session.timeout = 0.80
            break
        except Exception as exc:
            candidate.close()
            now = time.monotonic()
            if now - last_wait_notice >= 2.0:
                print(f"WAITING {type(exc).__name__}: {exc}", flush=True)
                last_wait_notice = now
            time.sleep(0.05)

    if session is None or identity is None:
        write_json(status_path, {"state": "timeout_waiting_for_power"})
        return 2

    previous_boot_id: int | None = None
    if args.wait_for_reset:
        previous_boot_id = int(identity.boot_id)
        session.close()
        session = None
        identity = None
        print(f"ARMED_FOR_RESET previous_boot_id={previous_boot_id}", flush=True)
        write_json(status_path, {
            "state": "armed_waiting_for_reset",
            "port": args.port,
            "firmware_id": args.firmware_id,
            "previous_boot_id": previous_boot_id,
        })
        last_wait_notice = 0.0
        while time.monotonic() < deadline:
            candidate = Session(
                port=args.port,
                timeout=0.20,
                firmware_id=args.firmware_id,
            )
            try:
                candidate_identity = candidate.connect()
                if int(candidate_identity.boot_id) == previous_boot_id:
                    candidate.close()
                    time.sleep(0.05)
                    continue
                session = candidate
                identity = candidate_identity
                session.timeout = 0.80
                break
            except Exception as exc:
                candidate.close()
                now = time.monotonic()
                if now - last_wait_notice >= 2.0:
                    print(f"WAITING_FOR_RESET {type(exc).__name__}: {exc}",
                          flush=True)
                    last_wait_notice = now
                time.sleep(0.05)

        if session is None or identity is None:
            write_json(status_path, {
                "state": "timeout_waiting_for_reset",
                "previous_boot_id": previous_boot_id,
            })
            return 2

    connected_host = datetime.now().astimezone().isoformat(timespec="milliseconds")
    print(f"CONNECTED firmware={identity.firmware_id} boot_id={identity.boot_id}", flush=True)
    write_json(status_path, {
        "state": "capturing",
        "connected": connected_host,
        "previous_boot_id": previous_boot_id,
        "identity": plain(identity),
    })

    samples: list[dict[str, Any]] = []
    errors: list[str] = []
    capture_start = time.monotonic()
    next_sample = capture_start
    next_status = capture_start
    last_status: dict[str, Any] | None = None
    try:
        with samples_path.open("w", encoding="utf-8", buffering=1) as stream:
            while time.monotonic() - capture_start < args.capture_seconds:
                now = time.monotonic()
                if now < next_sample:
                    time.sleep(min(0.01, next_sample - now))
                    continue
                sample: dict[str, Any] = {
                    "host_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    "elapsed_s": round(now - capture_start, 3),
                }
                try:
                    stepper = plain(session.get_stepper(0))
                    lever = plain(session.get_lever_calibration())
                    sample["stepper"] = stepper
                    sample["lever"] = lever
                    sample["absolute_0x31"] = absolute_from_relative(lever["current_counts"])
                    if now >= next_status:
                        last_status = plain(session.get_status())
                        next_status = now + 1.0
                    if last_status is not None:
                        sample["status"] = last_status
                except Exception as exc:
                    message = f"{sample['elapsed_s']}: {type(exc).__name__}: {exc}"
                    sample["capture_error"] = message
                    errors.append(message)
                samples.append(sample)
                stream.write(json.dumps(sample, ensure_ascii=False) + "\n")
                next_sample += args.period_seconds
    finally:
        session.close()

    summary = summarize(identity, samples, errors)
    summary["path"] = str(args.output.resolve())
    summary["connected_host_time"] = connected_host
    summary["completed_host_time"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
    write_json(args.output / "summary.json", summary)
    write_json(status_path, {"state": "complete", "pass": summary["pass"]})
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
