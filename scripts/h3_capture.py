"""Verify the independent H3 schema-3 serial callback and capture it to CSV.

This probe never enables the raw UART1 mirror and never issues a motion
command.  Gently moving the ball is enough to verify the complete path from
firmware publication through the Python callback to the CSV writer.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import queue
import struct
import time

from m6link import FrameType, Session


H3_RECORD = struct.Struct("<BBBBIII7hiiIHBBbBh")
FIELDS = (
    "received_s", "time_ms", "run_id", "profile_revision", "state", "flags",
    "x_mm_x10", "vx_mm_s", "target_mm_x10", "error_mm_x10",
    "raw_pd_mdeg", "forced_tilt_mdeg", "tilt_mdeg", "actuator_counts",
    "actuator_target_counts", "fault", "vision_age_ms", "vision_seq",
    "actuator_state", "actuator_direction", "actuator_rpm",
    "integral_mdeg",
)


def decode(payload: bytes) -> dict[str, int]:
    if len(payload) != H3_RECORD.size:
        raise ValueError(f"unexpected H3 record length {len(payload)}")
    (
        kind, schema, state, flags, time_ms, run_id, profile_revision,
        x_mm_x10, vx_mm_s, target_mm_x10, error_mm_x10, raw_pd_mdeg,
        forced_tilt_mdeg, tilt_mdeg, actuator_counts, actuator_target_counts,
        fault, vision_age_ms, vision_seq, actuator_state,
        actuator_direction, actuator_rpm, integral_mdeg,
    ) = H3_RECORD.unpack(payload)
    if kind != 3 or schema != 3:
        raise ValueError(f"unexpected H3 prefix kind={kind} schema={schema}")
    return {
        "time_ms": time_ms, "run_id": run_id,
        "profile_revision": profile_revision,
        "state": state,
        "flags": flags,
        "x_mm_x10": x_mm_x10,
        "vx_mm_s": vx_mm_s,
        "target_mm_x10": target_mm_x10, "error_mm_x10": error_mm_x10,
        "raw_pd_mdeg": raw_pd_mdeg, "forced_tilt_mdeg": forced_tilt_mdeg,
        "tilt_mdeg": tilt_mdeg,
        "actuator_counts": actuator_counts,
        "actuator_target_counts": actuator_target_counts,
        "fault": fault,
        "vision_age_ms": vision_age_ms,
        "vision_seq": vision_seq,
        "actuator_state": actuator_state,
        "actuator_direction": actuator_direction,
        "actuator_rpm": actuator_rpm,
        "integral_mdeg": integral_mdeg,
    }


def callback_metrics(times: list[float], positions_x10: list[int]) -> dict[str, float]:
    duration = times[-1] - times[0] if len(times) >= 2 else 0.0
    rate_hz = (len(times) - 1) / duration if duration > 0.0 else 0.0
    gaps = [1000.0 * (right - left) for left, right in zip(times, times[1:])]
    movement_mm = (
        (max(positions_x10) - min(positions_x10)) / 10.0
        if positions_x10 else 0.0
    )
    return {
        "rate_hz": rate_hz,
        "max_gap_ms": max(gaps, default=0.0),
        "movement_mm": movement_mm,
        "valid_positions": float(len(positions_x10)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM10")
    parser.add_argument(
        "--firmware-id", default="M6-runtime-20260801-r158-stop"
    )
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--min-hz", type=float, default=15.0)
    parser.add_argument("--max-gap-ms", type=float, default=200.0)
    parser.add_argument("--require-motion-mm", type=float, default=2.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.seconds <= 0 or args.min_hz <= 0 or args.max_gap_ms <= 0:
        parser.error("seconds, min-hz and max-gap-ms must be positive")
    if args.require_motion_mm < 0 or args.print_hz < 0:
        parser.error("require-motion-mm and print-hz cannot be negative")
    output = args.output or (
        Path("captures") / "h3_callback" /
        f"{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    )

    session = Session(
        port=args.port, timeout=0.8, firmware_id=args.firmware_id
    )
    identity = session.connect()
    rows = 0
    callback_times: list[float] = []
    valid_positions: list[int] = []
    output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    next_print = started
    try:
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            deadline = started + args.seconds
            while time.monotonic() < deadline:
                try:
                    event = session.read_event(
                        timeout=min(0.25, max(0.0, deadline - time.monotonic()))
                    )
                except queue.Empty:
                    continue
                payload = getattr(event, "payload", b"")
                if (
                    getattr(event, "type", -1) != int(FrameType.BULK)
                    or len(payload) != H3_RECORD.size
                    or payload[:2] != b"\x03\x03"
                ):
                    continue
                received = time.monotonic()
                row = decode(payload)
                writer.writerow({
                    "received_s": f"{received - started:.6f}", **row,
                })
                stream.flush()
                callback_times.append(received)
                if row["flags"] & 1:
                    valid_positions.append(row["x_mm_x10"])
                rows += 1
                if args.print_hz and received >= next_print:
                    valid = "valid" if row["flags"] & 1 else "invalid"
                    print(
                        f"callback={rows:04d} x={row['x_mm_x10'] / 10:+6.1f} mm "
                        f"vx={row['vx_mm_s']:+4d} mm/s vision={valid} "
                        f"age={row['vision_age_ms']} ms",
                        flush=True,
                    )
                    next_print = received + 1.0 / args.print_hz
    finally:
        session.close()
    metrics = callback_metrics(callback_times, valid_positions)
    passed = (
        rows >= 2 and metrics["rate_hz"] >= args.min_hz and
        metrics["max_gap_ms"] <= args.max_gap_ms and
        metrics["movement_mm"] >= args.require_motion_mm
    )
    print(
        f"{'PASS' if passed else 'FAIL'} callbacks={rows} "
        f"rate={metrics['rate_hz']:.1f}Hz max_gap={metrics['max_gap_ms']:.1f}ms "
        f"movement={metrics['movement_mm']:.1f}mm firmware={identity.firmware_id} "
        f"output={output}",
        flush=True,
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
