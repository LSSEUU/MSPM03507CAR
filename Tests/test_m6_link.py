from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M6LinkTests(unittest.TestCase):
    def test_handshake_replay_and_priority(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "m6_link_test.exe"
            command = [
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-DM6_LINK_HOST_TEST",
                "-I", str(ROOT / "System"), "-I", str(ROOT / "Tests"),
                str(ROOT / "Tests" / "m6_link_test.c"),
                str(ROOT / "System" / "m6_wire.c"),
                str(ROOT / "System" / "m6_link.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
