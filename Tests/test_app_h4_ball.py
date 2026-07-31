from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class AppH4BallTests(unittest.TestCase):
    def test_preflight_cascade_guards_and_finish(self) -> None:
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "app_h4_ball_test.exe"
            command = [
                compiler,
                "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "App"),
                "-I", str(ROOT / "Config"),
                "-I", str(ROOT / "Control"),
                "-I", str(ROOT / "System"),
                str(ROOT / "Tests" / "app_h4_ball_test.c"),
                str(ROOT / "App" / "app_h4_ball.c"),
                str(ROOT / "Control" / "ball_cascade.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run(
                [str(executable)], capture_output=True, text=True
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_h4_runtime_wiring(self) -> None:
        app_line = (ROOT / "App" / "app_line.c").read_text(encoding="utf-8")
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        self.assertIn("AppH4Ball_CheckReady(now_ms)", app_line)
        self.assertIn("AppH4Ball_Start(now_ms)", app_line)
        self.assertIn("AppH4Ball_Finish()", app_line)
        self.assertIn("AppH4Ball_Task20ms(started)", main)
        self.assertIn("AppH4Ball_EmergencyStop()", main)
        self.assertIn("../Control/ball_cascade.c", project)
        self.assertIn("../App/app_h4_ball.c", project)


if __name__ == "__main__":
    unittest.main()
