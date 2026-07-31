from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class LeverCalibrationTests(unittest.TestCase):
    def test_state_machine_and_strict_validation(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "lever_calibration_test.exe"
            command = [
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "System"), "-I", str(ROOT / "Config"),
                "-I", str(ROOT / "Hardware"),
                str(ROOT / "Tests" / "lever_calibration_test.c"),
                str(ROOT / "System" / "lever_calibration.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_calibration_source_has_no_motor_commands(self):
        source = (ROOT / "System" / "lever_calibration.c").read_text(encoding="utf-8")
        forbidden = (
            "ZdtStepper_Enable", "ZdtStepper_RunVelocity",
            "ZdtStepper_MovePosition", "ZdtStepper_StartHoming",
            "ZdtStepper_ZeroCurrentPosition", "ZdtStepper_ClearProtection",
        )
        for name in forbidden:
            self.assertNotIn(name, source)


if __name__ == "__main__":
    unittest.main()
