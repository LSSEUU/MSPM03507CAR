import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class M3ControlTests(unittest.TestCase):
    def test_control_model(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "m3_control_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "Control"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "m3_control_test.c"),
                str(ROOT / "Control" / "pid.c"),
                str(ROOT / "Control" / "filter.c"),
                str(ROOT / "Control" / "wheel_speed.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_control_layer_has_no_hardware_or_safety_dependency(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "Control").glob("*.[ch]")
        )
        self.assertNotIn("ti_msp_dl_config", sources)
        self.assertNotIn("Motor_SetPWM", sources)
        self.assertNotIn("Safety_", sources)

    def test_runtime_routes_wheel_control_through_safety(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        safety = (ROOT / "System" / "safety.h").read_text(encoding="utf-8")
        five_ms = main.split(
            "if (Scheduler_Take(SCHEDULER_TASK_5MS))", 1
        )[1].split("if (Scheduler_Take(SCHEDULER_TASK_10MS))", 1)[0]

        self.assertIn("Encoder_GetSnapshot", five_ms)
        self.assertIn("WheelSpeed_Update", five_ms)
        self.assertIn("Safety_RequestMotor", five_ms)
        self.assertLess(
            five_ms.index("WheelSpeed_Update"),
            five_ms.index("Safety_RequestMotor"),
        )
        self.assertIn("SAFETY_FAULT_ENCODER_INVALID", safety)
        self.assertIn("SAFETY_FAULT_WHEEL_STALL", safety)
        self.assertIn("j=left k=right b=both speed test", diagnostics)
        self.assertIn("CSV: M3,time_ms", diagnostics)


if __name__ == "__main__":
    unittest.main()
