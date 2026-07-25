#!/usr/bin/env python3
"""Verify that a production image does not occupy M6 parameter sectors."""

from __future__ import annotations

import argparse
from pathlib import Path
import re


PARAMETER_SECTOR_A = 0x0001F800
PARAMETER_SECTOR_B = 0x0001FC00
FLASH_END = 0x00020000


def occupied_hex_ranges(path: Path) -> list[tuple[int, int]]:
    """Return half-open address ranges for Intel HEX data records."""
    upper = 0
    ranges: list[tuple[int, int]] = []
    for number, raw_line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"line {number}: missing Intel HEX prefix")
        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"line {number}: invalid record length")
        if sum(record) & 0xFF:
            raise ValueError(f"line {number}: checksum mismatch")
        count = record[0]
        offset = (record[1] << 8) | record[2]
        kind = record[3]
        data = record[4 : 4 + count]
        if kind == 0x00 and count:
            start = upper + offset
            ranges.append((start, start + count))
        elif kind == 0x02:
            if count != 2:
                raise ValueError(f"line {number}: invalid segment address")
            upper = int.from_bytes(data, "big") << 4
        elif kind == 0x04:
            if count != 2:
                raise ValueError(f"line {number}: invalid linear address")
            upper = int.from_bytes(data, "big") << 16
        elif kind in (0x01, 0x03, 0x05):
            continue
        else:
            raise ValueError(f"line {number}: unsupported record type {kind:#x}")
    return ranges


def verify_hex(path: Path) -> int:
    ranges = occupied_hex_ranges(path)
    violations = [item for item in ranges if item[0] < FLASH_END and item[1] > PARAMETER_SECTOR_A]
    if violations:
        detail = ", ".join(f"0x{start:08X}-0x{end - 1:08X}" for start, end in violations)
        raise ValueError(f"ordinary HEX data overlaps reserved sectors: {detail}")
    main_ends = [end for start, end in ranges if start < PARAMETER_SECTOR_A]
    return max(main_ends, default=0)


def verify_map(path: Path) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    if "0x0001f800" not in text.lower():
        raise ValueError("MAP does not contain the 0x0001F800 application limit")
    ramfunc_lines = [line for line in text.splitlines() if "ramfunc" in line.lower()]
    if not ramfunc_lines:
        raise ValueError("MAP does not report a .ramfunc execution section")
    addresses: list[int] = []
    for line in ramfunc_lines:
        addresses.extend(int(value, 16) for value in re.findall(r"0x([0-9a-fA-F]{8})", line))
    if not any(0x20200000 <= address < 0x20208000 for address in addresses):
        raise ValueError("MAP does not place .ramfunc execution in SRAM")
    writer = re.search(
        r"^\s*write_sector_from_ram\s+0x([0-9a-fA-F]{8})\b",
        text,
        flags=re.MULTILINE,
    )
    if writer is None:
        raise ValueError("MAP does not report write_sector_from_ram")
    writer_address = int(writer.group(1), 16) & ~1
    if not 0x20200000 <= writer_address < 0x20208000:
        raise ValueError("write_sector_from_ram does not execute from SRAM")
    if re.search(r"parameter_store\.o\(\.ramfunc\) refers to", text):
        raise ValueError("RAM Flash writer has a non-inlined code dependency")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hex", required=True, type=Path)
    parser.add_argument("--map", dest="map_path", type=Path)
    args = parser.parse_args()
    highest = verify_hex(args.hex)
    if args.map_path is not None:
        verify_map(args.map_path)
    print(
        f"M6 image boundary OK: highest ordinary byte "
        f"0x{max(highest - 1, 0):08X}; reserved "
        f"0x{PARAMETER_SECTOR_A:08X}-0x{FLASH_END - 1:08X}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
