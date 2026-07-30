"""Replay editable H3 ball measurements over a serial port."""

import argparse
import csv
import time

from ball_wire import encode_frame


REQUIRED_COLUMNS = ("delay_ms", "valid", "x_mm", "vx_mm_s")


def round_half_away_from_zero(value):
    return int(value + 0.5) if value >= 0.0 else int(value - 0.5)


def parse_valid(value):
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "valid"):
        return True
    if normalized in ("0", "false", "no", "invalid"):
        return False
    raise ValueError("valid must be one of 1/0, true/false, yes/no")


def load_scenario(path):
    rows = []
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("scenario CSV is missing a header")
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ValueError("scenario CSV is missing: {}".format(", ".join(missing)))

        for line_number, row in enumerate(reader, start=2):
            try:
                delay_ms = float(row["delay_ms"])
                valid = parse_valid(row["valid"])
                x_mm = float(row["x_mm"] or 0.0)
                vx_mm_s = float(row["vx_mm_s"] or 0.0)
            except (TypeError, ValueError) as error:
                raise ValueError("line {}: {}".format(line_number, error)) from error
            if delay_ms < 0.0:
                raise ValueError("line {}: delay_ms must be non-negative".format(line_number))
            rows.append((delay_ms, valid, x_mm, vx_mm_s))

    if not rows:
        raise ValueError("scenario CSV contains no measurements")
    return rows


def encode_scenario(rows, initial_sequence=0):
    sequence = initial_sequence
    for delay_ms, valid, x_mm, vx_mm_s in rows:
        x_mm_x10 = round_half_away_from_zero(x_mm * 10.0)
        velocity = round_half_away_from_zero(vx_mm_s)
        yield delay_ms, encode_frame(sequence, valid, x_mm_x10, velocity)
        sequence = (sequence + 1) & 0xFF


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Replay H3 ball frames from delay_ms,valid,x_mm,vx_mm_s CSV"
    )
    parser.add_argument("scenario", help="CSV scenario to replay")
    parser.add_argument("--port", help="serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seq", type=int, default=0, help="initial u8 sequence")
    parser.add_argument("--loop", action="store_true", help="repeat until interrupted")
    parser.add_argument(
        "--dry-run", action="store_true", help="print frames without opening a port"
    )
    return parser


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if not 0 <= args.seq <= 255:
        parser.error("--seq must fit u8")
    if args.baud <= 0:
        parser.error("--baud must be positive")
    if not args.dry_run and not args.port:
        parser.error("--port is required unless --dry-run is used")
    if args.dry_run and args.loop:
        parser.error("--loop cannot be combined with --dry-run")

    try:
        rows = load_scenario(args.scenario)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    if args.dry_run:
        for _, frame in encode_scenario(rows, args.seq):
            print(" ".join("{:02X}".format(byte) for byte in frame))
        return 0

    try:
        import serial
    except ImportError as error:
        parser.error("pyserial is required for live replay: {}".format(error))

    sequence = args.seq
    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0,
        write_timeout=1,
    ) as port:
        while True:
            for delay_ms, frame in encode_scenario(rows, sequence):
                if delay_ms > 0.0:
                    time.sleep(delay_ms * 0.001)
                written = port.write(frame)
                if written != len(frame):
                    raise IOError("serial write accepted only {} of 9 bytes".format(written))
                sequence = (sequence + 1) & 0xFF
            if not args.loop:
                break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
