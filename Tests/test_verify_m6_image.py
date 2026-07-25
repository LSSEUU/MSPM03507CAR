import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "verify_m6_image.py"
SPEC = importlib.util.spec_from_file_location("verify_m6_image", SCRIPT)
verify = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify)


def hex_record(address: int, kind: int, data: bytes) -> str:
    body = bytes([len(data), address >> 8, address & 0xFF, kind]) + data
    checksum = (-sum(body)) & 0xFF
    return ":" + (body + bytes([checksum])).hex().upper()


class M6ImageBoundaryTests(unittest.TestCase):
    def write_hex(self, records: list[str]) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "firmware.hex"
        path.write_text("\n".join(records + [hex_record(0, 1, b"")]), encoding="ascii")
        return path

    def test_accepts_last_application_byte(self):
        path = self.write_hex(
            [hex_record(0, 4, b"\x00\x01"), hex_record(0xF7FF, 0, b"\xAA")]
        )
        self.assertEqual(verify.verify_hex(path), 0x1F800)

    def test_rejects_parameter_sector_data(self):
        path = self.write_hex(
            [hex_record(0, 4, b"\x00\x01"), hex_record(0xF800, 0, b"\xAA")]
        )
        with self.assertRaisesRegex(ValueError, "reserved sectors"):
            verify.verify_hex(path)

    def test_extended_linear_address_is_honored(self):
        path = self.write_hex(
            [hex_record(0, 4, b"\x00\x02"), hex_record(0x0000, 0, b"\xAA")]
        )
        self.assertEqual(verify.verify_hex(path), 0)

    def write_map(self, text: str) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "firmware.map"
        path.write_text(text, encoding="utf-8")
        return path

    def test_map_requires_ram_writer_without_flash_dependencies(self):
        valid = self.write_map(
            "Load Region LR_IROM1 0x00000000 0x0001f800\n"
            "write_sector_from_ram 0x20200001 Thumb Code 128 "
            "parameter_store.o(.ramfunc)\n"
            "0x20200000 0x0000B000 0x00000080 Code RO .ramfunc\n"
        )
        verify.verify_map(valid)

        flash_writer = self.write_map(
            "0x0001f800 .ramfunc 0x20200000\n"
            "write_sector_from_ram 0x00001001 Thumb Code 128\n"
        )
        with self.assertRaisesRegex(ValueError, "does not execute from SRAM"):
            verify.verify_map(flash_writer)

        dependency = self.write_map(
            "0x0001f800 .ramfunc 0x20200000\n"
            "write_sector_from_ram 0x20200001 Thumb Code 128\n"
            "parameter_store.o(.ramfunc) refers to helper.o(.text) for helper\n"
        )
        with self.assertRaisesRegex(ValueError, "code dependency"):
            verify.verify_map(dependency)


if __name__ == "__main__":
    unittest.main()
