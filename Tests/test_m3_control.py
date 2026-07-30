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
        self.assertIn(
            "WheelSpeed_UpdateLogical(encoder.count_b, encoder.count_a",
            five_ms,
        )
        self.assertIn("Safety_RequestMotor", main)
        self.assertLess(
            five_ms.index("WheelSpeed_UpdateLogical"),
            five_ms.index("Main_RequestLogicalWheelPWM"),
        )
        self.assertIn(
            "right_pwm * CHASSIS_MOTOR_A_FORWARD_SIGN", main
        )
        self.assertIn(
            "left_pwm * CHASSIS_MOTOR_B_FORWARD_SIGN", main
        )
        vehicle_config = (ROOT / "Config" / "vehicle_config.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("CHASSIS_MOTOR_A_FORWARD_SIGN        (-1)", vehicle_config)
        self.assertIn("CHASSIS_MOTOR_B_FORWARD_SIGN        (-1)", vehicle_config)
        self.assertIn("SAFETY_FAULT_ENCODER_INVALID", safety)
        self.assertIn("SAFETY_FAULT_WHEEL_STALL", safety)
        self.assertIn("j=left k=right b=both speed test", diagnostics)
        self.assertIn("CSV: M3,time_ms", diagnostics)

    def test_measured_wheel_calibration_is_recorded(self):
        config = (ROOT / "Config" / "control_config.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("WHEEL_DIAMETER_MM                    65.0f", config)
        self.assertIn(
            "ENCODER_LEFT_COUNTS_PER_WHEEL_REV   736.4f", config
        )
        self.assertIn(
            "ENCODER_RIGHT_COUNTS_PER_WHEEL_REV  732.6f", config
        )
        self.assertIn("ENCODER_LEFT_FORWARD_SIGN                1", config)
        self.assertIn("ENCODER_RIGHT_FORWARD_SIGN              -1", config)


if __name__ == "__main__":
    unittest.main()
