#!/usr/bin/env python3
"""Collect two-stage compile-time calibration for the analog line sensor."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import re
import sys
import time
from typing import Iterable, Sequence


CHANNEL_COUNT = 8
SAMPLES_PER_STAGE = 64
ADC_MAX = 4095
MIN_SPAN_ADC = 512
MIN_SPAN_TO_NOISE_RATIO = 8
MAX_CONSECUTIVE_TIMEOUTS = 2

GRAY_PATTERN = re.compile(
    r"GRAY adc=\[([^]]+)\] scans=(\d+) errors=(\d+)"
)


@dataclass(frozen=True)
class GraySample:
    adc: tuple[int, ...]
    scan_count: int
    error_count: int


@dataclass(frozen=True)
class StageStats:
    count: int
    mean: tuple[int, ...]
    minimum: tuple[int, ...]
    maximum: tuple[int, ...]
    peak_to_peak: tuple[int, ...]
    rail_hit: tuple[bool, ...]


def parse_gray_line(line: str) -> GraySample | None:
    match = GRAY_PATTERN.search(line)
    if match is None:
        return None
    values = tuple(int(value.strip()) for value in match.group(1).split(","))
    if len(values) != CHANNEL_COUNT:
        raise ValueError(f"expected {CHANNEL_COUNT} ADC values, got {len(values)}")
    if any(value < 0 or value > ADC_MAX for value in values):
        raise ValueError("ADC value outside the 12-bit range")
    return GraySample(values, int(match.group(2)), int(match.group(3)))


def summarize_stage(samples: Sequence[GraySample]) -> StageStats:
    if not samples:
        raise ValueError("no grayscale samples collected")
    columns = tuple(
        tuple(sample.adc[channel] for sample in samples)
        for channel in range(CHANNEL_COUNT)
    )
    minimum = tuple(min(column) for column in columns)
    maximum = tuple(max(column) for column in columns)
    mean = tuple(
        (sum(column) + len(column) // 2) // len(column) for column in columns
    )
    peak_to_peak = tuple(
        maximum[channel] - minimum[channel]
        for channel in range(CHANNEL_COUNT)
    )
    rail_hit = tuple(
        minimum[channel] == 0 or maximum[channel] == ADC_MAX
        for channel in range(CHANNEL_COUNT)
    )
    return StageStats(
        len(samples), mean, minimum, maximum, peak_to_peak, rail_hit
    )


def validate_calibration(white: StageStats, black: StageStats) -> list[str]:
    problems: list[str] = []
    if white.count < SAMPLES_PER_STAGE:
        problems.append(
            f"white stage has {white.count} samples; need {SAMPLES_PER_STAGE}"
        )
    if black.count < SAMPLES_PER_STAGE:
        problems.append(
            f"black stage has {black.count} samples; need {SAMPLES_PER_STAGE}"
        )
    for channel in range(CHANNEL_COUNT):
        label = f"CH{channel + 1}"
        if white.rail_hit[channel] or black.rail_hit[channel]:
            problems.append(f"{label} touched an ADC rail")
        if white.mean[channel] <= black.mean[channel]:
            problems.append(
                f"{label} white={white.mean[channel]} is not above "
                f"black={black.mean[channel]}"
            )
            continue
        span = white.mean[channel] - black.mean[channel]
        if span < MIN_SPAN_ADC:
            problems.append(
                f"{label} span={span} is below {MIN_SPAN_ADC} ADC counts"
            )
        noise = max(
            white.peak_to_peak[channel], black.peak_to_peak[channel], 1
        )
        if span < MIN_SPAN_TO_NOISE_RATIO * noise:
            problems.append(
                f"{label} span={span} is less than "
                f"{MIN_SPAN_TO_NOISE_RATIO}x peak-to-peak noise={noise}"
            )
    return problems


def format_values(values: Iterable[int]) -> str:
    return "{ " + ", ".join(f"{value}U" for value in values) + " }"


def format_config(white: StageStats, black: StageStats) -> str:
    return "\n".join(
        (
            "#define GRAYSCALE_CALIBRATION_VERIFIED          1U",
            "#define GRAYSCALE_WHITE_ADC_VALUES \\",
            f"    {format_values(white.mean)}",
            "#define GRAYSCALE_BLACK_ADC_VALUES \\",
            f"    {format_values(black.mean)}",
        )
    )


def _read_sample(serial_port, timeout_s: float) -> GraySample:
    deadline = time.monotonic() + timeout_s
    sample = None
    serial_port.write(b"q")
    serial_port.flush()
    while time.monotonic() < deadline:
        raw = serial_port.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        parsed = parse_gray_line(line)
        if parsed is not None:
            if parsed.error_count != 0:
                raise RuntimeError(
                    f"firmware reports grayscale errors={parsed.error_count}"
                )
            sample = parsed
        elif sample is not None and line.startswith("GRAY ADC "):
            return sample
    raise TimeoutError("timed out waiting for a GRAY adc response")


def capture_stage(
    serial_port, label: str, timeout_s: float, prompt: bool = True
) -> StageStats:
    if prompt:
        input(
            f"Place all eight probes over the installed {label} surface, "
            "keep the car still, then press Enter..."
        )
    samples: list[GraySample] = []
    last_scan = None
    consecutive_timeouts = 0
    while len(samples) < SAMPLES_PER_STAGE:
        try:
            sample = _read_sample(serial_port, timeout_s)
        except TimeoutError as exc:
            consecutive_timeouts += 1
            if consecutive_timeouts > MAX_CONSECUTIVE_TIMEOUTS:
                raise TimeoutError(
                    f"{label} report timed out "
                    f"{consecutive_timeouts} consecutive times"
                ) from exc
            print(
                f"\n{label}: transient timeout; retrying "
                f"({consecutive_timeouts}/{MAX_CONSECUTIVE_TIMEOUTS})",
                flush=True,
            )
            continue
        consecutive_timeouts = 0
        if sample.scan_count == last_scan:
            continue
        last_scan = sample.scan_count
        samples.append(sample)
        print(
            f"\r{label}: {len(samples):02d}/{SAMPLES_PER_STAGE}",
            end="",
            flush=True,
        )
    print()
    return summarize_stage(samples)


def print_stats(label: str, stats: StageStats) -> None:
    print(f"{label} mean: {list(stats.mean)}")
    print(f"{label} p-p : {list(stats.peak_to_peak)}")


def format_stage_json(label: str, stats: StageStats) -> str:
    payload = {"label": label, **asdict(stats)}
    return json.dumps(payload, separators=(",", ":"))


def _load_session_type():
    source = Path(__file__).resolve().parents[1] / "python" / "src"
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    from m6link import Session

    return Session


class LegacySessionPort:
    """Expose the small line-oriented surface used by the legacy collector."""

    def __init__(self, session) -> None:
        self.session = session

    def write(self, command: bytes) -> int:
        text = command.decode("ascii")
        self.session.legacy_command(text)
        return len(command)

    def flush(self) -> None:
        # Session.legacy_command waits until its single owner thread has
        # completed the physical write.
        return None

    def readline(self) -> bytes:
        line = self.session.read_legacy_line(timeout=0.1)
        return (line + "\r\n").encode("utf-8") if line else b""


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Collect 64 white and 64 black ADC scans over UART0."
    )
    parser.add_argument("--port", help="UART0 serial port, for example COM10")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--list-ports", action="store_true", help="list serial ports and exit"
    )
    parser.add_argument(
        "--stage",
        choices=("white", "black"),
        help="collect one already-positioned stage without an interactive prompt",
    )
    args = parser.parse_args(argv)

    try:
        Session = _load_session_type()
        if args.list_ports:
            for port in Session.discover_ports():
                print(port)
            return 0
        if not args.port:
            parser.error("--port is required unless --list-ports is used")

        session = Session(
            port=args.port,
            baudrate=args.baud,
            legacy=True,
            timeout=max(0.1, args.timeout),
        )
        try:
            session.connect()
            serial_port = LegacySessionPort(session)
            if args.stage:
                label = args.stage.upper()
                stats = capture_stage(
                    serial_port, label, args.timeout, prompt=False
                )
                print_stats(args.stage, stats)
                print(f"STAGE_JSON {format_stage_json(args.stage, stats)}")
                return 0
            white = capture_stage(serial_port, "WHITE", args.timeout)
            black = capture_stage(serial_port, "BLACK LINE", args.timeout)
        finally:
            session.close(clean=False)

        print_stats("white", white)
        print_stats("black", black)
        problems = validate_calibration(white, black)
        if problems:
            print("Calibration rejected:", file=sys.stderr)
            for problem in problems:
                print(f"- {problem}", file=sys.stderr)
            return 2
        print("Calibration accepted. Replace the placeholders with:")
        print(format_config(white, black))
        return 0
    except (RuntimeError, TimeoutError, ValueError, OSError) as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
