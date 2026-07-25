import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class M6ControlParamsTests(unittest.TestCase):
    def test_parameter_model(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "m6_control_params_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "System"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "m6_control_params_test.c"),
                str(ROOT / "System" / "control_params.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_diagnostic_target_change_resets_both_wheel_controllers(self):
        runtime = (ROOT / "System" / "control_runtime.c").read_text(
            encoding="utf-8"
        )
        diagnostic = runtime[
            runtime.index("commit->diagnostic_boundary") :
            runtime.index("WheelSpeedConfig_t wheel")
            if runtime.index("WheelSpeedConfig_t wheel") >
            runtime.index("commit->diagnostic_boundary")
            else len(runtime)
        ]
        self.assertGreaterEqual(
            diagnostic.count("CONTROL_PARAMS_CHANGE_WHEEL_DIAG_TARGET"), 2
        )
        self.assertIn("WheelSpeed_SetGains(WHEEL_SPEED_LEFT", diagnostic)
        self.assertIn("WheelSpeed_SetGains(WHEEL_SPEED_RIGHT", diagnostic)


if __name__ == "__main__":
    unittest.main()
