from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M2SystemTests(unittest.TestCase):
    def test_scheduler_state_and_safety_models(self):
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler, "gcc is required for host C regression tests")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "m2_system_test.exe"
            sources = [
                ROOT / "System" / "scheduler.c",
                ROOT / "System" / "system_state.c",
                ROOT / "System" / "system_time.c",
                ROOT / "System" / "safety.c",
                ROOT / "Tests" / "m2_system_test.c",
            ]
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-DSCHEDULER_HOST_TEST",
                "-DSAFETY_HOST_TEST",
                "-DSYSTEM_STATE_HOST_TEST",
                "-I", str(ROOT / "System"),
                "-I", str(ROOT / "Hardware"),
                "-I", str(ROOT / "Config"),
                *map(str, sources),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_runtime_has_single_safety_motor_owner_and_nonblocking_main(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        safety = (ROOT / "System" / "safety.c").read_text(encoding="utf-8")
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(encoding="utf-8")
        uart = (ROOT / "Hardware" / "uart_debug.c").read_text(encoding="utf-8")

        self.assertIn("DL_SYSTICK_init(SYSTICK_PERIOD)", main)
        self.assertIn("DL_SYSTICK_enableInterrupt()", main)
        self.assertNotIn("delay_ms", main)
        self.assertNotIn("Motor_SetPWM", diagnostics)
        self.assertIn("Motor_SetPWM", safety)
        self.assertNotIn("while (DL_UART_isBusy", uart)
        self.assertIn("emergency_callback", uart)

    def test_runtime_oled_refresh_is_split_into_bounded_chunks(self):
        oled = (ROOT / "Hardware" / "oled.c").read_text(encoding="utf-8")
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        task = oled.split("void OLED_Service(void)", 1)[1].split(
            "void OLED_SetPos", 1)[0]

        self.assertIn("OLED_REFRESH_SET_PAGE", task)
        self.assertIn("OLED_REFRESH_WRITE_DATA", task)
        self.assertIn("OLED_Service();", main)
        self.assertIn("oled_refresh_column], 1U);", task)
        self.assertNotIn(", OLED_WIDTH);", task)


if __name__ == "__main__":
    unittest.main()
