from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M6ActionValidationTests(unittest.TestCase):
    def test_negotiated_capabilities_and_turn_extension(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "m6_action_validate_test.exe"
            command = [
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "System"),
                str(ROOT / "Tests" / "m6_action_validate_test.c"),
                str(ROOT / "System" / "m6_action_validate.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
