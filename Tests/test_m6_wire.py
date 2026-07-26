from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M6WireTests(unittest.TestCase):
    def test_frozen_wire_codec(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "m6_wire_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "System"),
                "-I",
                str(ROOT / "Tests"),
                str(ROOT / "Tests" / "m6_wire_test.c"),
                str(ROOT / "System" / "m6_wire.c"),
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
