import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class M5LineFollowTests(unittest.TestCase):
    def test_line_follow_model(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "m5_line_follow_test.exe"
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
                str(ROOT / "Tests" / "m5_line_follow_test.c"),
                str(ROOT / "Control" / "line_follow.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_runtime_routes_line_app_through_control_and_safety(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        app = (ROOT / "App" / "app_line.c").read_text(encoding="utf-8")
        safety = (ROOT / "System" / "safety.h").read_text(encoding="utf-8")
        self.assertIn("AppLine_ProcessGrayscale(&gray)", main)
        self.assertIn("AppLine_Task20ms(started)", main)
        self.assertIn("Grayscale_Task(SystemTime_Now())", main)
        self.assertIn("WheelSpeed_SetTarget", app)
        self.assertIn("SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE)", app)
        self.assertIn("SAFETY_FAULT_LINE_LOST", app)
        self.assertIn("SAFETY_FAULT_LINE_LOST", safety)
        self.assertNotIn("Motor_SetPWM", app)

    def test_project_and_diagnostics_expose_guarded_line_application(self):
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("../Control/line_follow.c", project)
        self.assertIn("../App/app_line.c", project)
        self.assertIn("1=qualify 2=active-level 3=arm line A=start 0=stop", diagnostics)
        self.assertIn("AppLine_Arm", diagnostics)
        self.assertIn("AppLine_ConfirmStart", diagnostics)
        self.assertIn("CSV: M5,time_ms", diagnostics)

    def test_runtime_status_queries_are_bounded_during_line_action(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("Diagnostics_QueueSafetyReport", diagnostics)
        self.assertIn("Diagnostics_ServiceSafetyReport", diagnostics)
        self.assertIn("Diagnostics_ActionTelemetryActive", diagnostics)
        self.assertIn("M5 running: use CSV telemetry; stop before r.", diagnostics)
        self.assertNotIn(
            "case 'v': case 'V': Diagnostics_PrintSafety()", diagnostics
        )

    def test_first_landing_tuning_is_frozen(self):
        config = (ROOT / "Config" / "control_config.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_FOLLOW_BASE_SPEED_MM_S             160.0f", config)
        self.assertIn("LINE_FOLLOW_KP_MM_S                     150.0f", config)
        self.assertIn("LINE_FOLLOW_KD_MM_S_PER_UNIT_S            1.0f", config)
        self.assertIn("LINE_FOLLOW_MAX_CORRECTION_MM_S         150.0f", config)


if __name__ == "__main__":
    unittest.main()
