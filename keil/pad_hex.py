import sys
import os
import subprocess

def to_intel_hex(data: bytearray, base_addr: int = 0) -> str:
    lines = []
    offset = 0
    while offset < len(data):
        chunk = data[offset:offset+16]
        addr = base_addr + offset
        if offset == 0 or (addr & 0xFFFF) == 0:
            upper = (addr >> 16) & 0xFFFF
            rec = [0x00, 0x00, 0x04,
                   (upper >> 8) & 0xFF, upper & 0xFF]
            checksum = (-(sum(rec) + 2)) & 0xFF
            lines.append(f":02000004{upper:04X}{checksum:02X}")
        byte_count = len(chunk)
        addr16 = addr & 0xFFFF
        rec_bytes = [byte_count,
                     (addr16 >> 8) & 0xFF, addr16 & 0xFF,
                     0x00] + list(chunk)
        checksum = (-sum(rec_bytes)) & 0xFF
        data_str = ''.join(f'{b:02X}' for b in chunk)
        lines.append(f":{byte_count:02X}{addr16:04X}00{data_str}{checksum:02X}")
        offset += 16
    lines.append(":00000001FF")
    return '\n'.join(lines) + '\n'

def pad_and_hex(axf_path, fromelf_path):
    axf_path    = os.path.abspath(axf_path)
    fromelf_path = os.path.abspath(fromelf_path)
    base     = os.path.splitext(axf_path)[0]
    bin_path = base + "_pad.bin"
    hex_path = base + ".hex"

    print(f"AXF:     {axf_path}")
    print(f"fromelf: {fromelf_path}")

    cmd = [fromelf_path, "--bin", f"--output={bin_path}", axf_path]
    print("Running:", cmd)
    ret = subprocess.run(cmd)
    if ret.returncode != 0:
        print("ERROR: fromelf bin export failed")
        sys.exit(1)

    with open(bin_path, "rb") as f:
        data = bytearray(f.read())
    os.remove(bin_path)

    remainder = len(data) % 8
    if remainder != 0:
        pad_len = 8 - remainder
        data += bytearray([0xFF] * pad_len)
        print(f"Padded {pad_len} bytes -> total = {len(data)} bytes")
    else:
        print(f"Already aligned -> total = {len(data)} bytes")

    hex_str = to_intel_hex(data, base_addr=0x00000000)
    with open(hex_path, "w") as f:
        f.write(hex_str)

    print(f"Done: {hex_path}")

if __name__ == "__main__":
    pad_and_hex(sys.argv[1], sys.argv[2])
