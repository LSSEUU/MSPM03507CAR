import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class RuntimeGrayscaleCalibrationTests(unittest.TestCase):
    def test_two_stage_state_machine_and_atomic_save(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "grayscale_calibration.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I", str(ROOT / "App"),
                "-I", str(ROOT / "Config"),
                "-I", str(ROOT / "Control"),
                "-I", str(ROOT / "Hardware"),
                "-I", str(ROOT / "System"),
                str(ROOT / "Tests" / "grayscale_calibration_state_test.c"),
                str(ROOT / "System" / "grayscale_calibration.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
