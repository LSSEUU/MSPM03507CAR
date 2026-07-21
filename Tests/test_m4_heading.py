import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class M4HeadingTests(unittest.TestCase):
    def test_heading_model(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "m4_heading_test.exe"
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
                str(ROOT / "Tests" / "m4_heading_test.c"),
                str(ROOT / "Control" / "heading.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_runtime_updates_heading_nonblocking(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        twenty_ms = main.split(
            "if (Scheduler_Take(SCHEDULER_TASK_20MS))", 1
        )[1].split("if (Scheduler_Take(SCHEDULER_TASK_100MS))", 1)[0]
        self.assertIn("Heading_Update", twenty_ms)
        self.assertIn("WheelSpeed_SetTarget", twenty_ms)
        self.assertNotIn("delay", twenty_ms.lower())

    def test_project_contains_heading_source(self):
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        self.assertIn("../Control/heading.c", project)

    def test_diagnostics_exposes_guarded_heading_actions(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("g=heading z=calibrate u=+90 i=-90 w=straight", diagnostics)
        self.assertIn("Diagnostics_ArmHeadingTest", diagnostics)
        self.assertIn("Diagnostics_StartHeadingTest", diagnostics)
        self.assertIn("SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO)", diagnostics)
        self.assertIn("CSV: M4,time_ms", diagnostics)
        self.assertIn("WheelSpeed_GetSnapshot(&wheel_snapshot)", diagnostics)


if __name__ == "__main__":
    unittest.main()
