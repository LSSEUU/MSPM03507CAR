import importlib.util
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "keil" / "pad_hex.py"
SPEC = importlib.util.spec_from_file_location("pad_hex", SCRIPT_PATH)
pad_hex = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pad_hex)


class ResolveFromelfTests(unittest.TestCase):
    def test_finds_fromelf_in_local_keil_install(self):
        resolver = getattr(pad_hex, "resolve_fromelf", None)
        self.assertIsNotNone(resolver, "resolve_fromelf() must be implemented")

        with tempfile.TemporaryDirectory() as temp_dir:
            expected = (
                Path(temp_dir)
                / "Keil_v5"
                / "ARM"
                / "ARMCLANG"
                / "Bin"
                / "fromelf.exe"
            )
            expected.parent.mkdir(parents=True)
            expected.touch()

            with mock.patch.dict(
                os.environ,
                {"LOCALAPPDATA": temp_dir, "FROMELF": ""},
                clear=False,
            ), mock.patch.object(shutil, "which", return_value=None):
                actual = resolver()

        self.assertEqual(Path(actual), expected.resolve())


if __name__ == "__main__":
    unittest.main()
