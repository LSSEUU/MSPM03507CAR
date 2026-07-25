#!/usr/bin/env python3
"""Collect two-stage compile-time calibration for the analog line sensor."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import re
import sys
import time
from typing import Iterable, Sequence


CHANNEL_COUNT = 8
SAMPLES_PER_STAGE = 64
ADC_MAX = 4095
MIN_SPAN_ADC = 512
MIN_SPAN_TO_NOISE_RATIO = 8

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
    while len(samples) < SAMPLES_PER_STAGE:
        sample = _read_sample(serial_port, timeout_s)
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


def _load_pyserial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required; install it with: python -m pip install pyserial"
        ) from exc
    return serial, list_ports


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
        serial, list_ports = _load_pyserial()
        if args.list_ports:
            for port in list_ports.comports():
                print(f"{port.device}\t{port.description}")
            return 0
        if not args.port:
            parser.error("--port is required unless --list-ports is used")

        with serial.Serial(args.port, args.baud, timeout=0.1) as serial_port:
            serial_port.reset_input_buffer()
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
