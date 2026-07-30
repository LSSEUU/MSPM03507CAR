import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AppH2PanelTests(unittest.TestCase):
    def test_key_preflight_debounce_and_oled_timer(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "app_h2_panel_test.exe"
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
                str(ROOT / "Tests" / "app_h2_panel_test.c"),
                str(ROOT / "App" / "app_h2_panel.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_production_wiring_uses_panel_instead_of_raw_buzzer(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        app_line = (ROOT / "App" / "app_line.c").read_text(encoding="utf-8")
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        self.assertIn("AppH2Panel_Init(SystemTime_Now())", main)
        self.assertIn("AppH2Panel_Task20ms(started)", main)
        self.assertIn("AppH2Panel_Task100ms(started)", main)
        self.assertNotIn("if (Key1_IsPressed()) Buzzer_On();", main)
        self.assertIn("../App/app_h2_panel.c", project)
        self.assertIn("bool AppLine_StartH2", app_line)
        self.assertIn("SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO)", app_line)
        self.assertIn("SAFETY_FAULT_GYRO_TIMEOUT", app_line)
        self.assertIn("app_line.elapsed_ms", app_line)


if __name__ == "__main__":
    unittest.main()
