from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class BallVisionTests(unittest.TestCase):
    def test_stream_parser(self) -> None:
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "ball_vision_test.exe"
            command = [
                compiler,
                "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "System"),
                "-I", str(ROOT / "Hardware"),
                str(ROOT / "Tests" / "ball_vision_test.c"),
                str(ROOT / "System" / "ball_vision.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
