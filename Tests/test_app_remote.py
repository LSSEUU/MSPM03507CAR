import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AppRemoteTests(unittest.TestCase):
    def test_remote_application(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            for verified in (0, 1):
                executable = pathlib.Path(temporary) / f"app_remote_{verified}.exe"
                command = [
                    compiler,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-DIR_REMOTE_KEYMAP_VERIFIED={verified}",
                    "-I", str(ROOT / "App"),
                    "-I", str(ROOT / "Hardware"),
                    "-I", str(ROOT / "System"),
                    "-I", str(ROOT / "Control"),
                    "-I", str(ROOT / "Config"),
                    str(ROOT / "Tests" / "app_remote_test.c"),
                    str(ROOT / "App" / "app_remote.c"),
                    str(ROOT / "App" / "app_manual_drive.c"),
                    "-o", str(executable),
                ]
                compiled = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(compiled.returncode, 0, compiled.stderr)
                run = subprocess.run([str(executable)], capture_output=True, text=True)
                self.assertEqual(run.returncode, 0, run.stderr)

    def test_remote_never_writes_pwm_directly(self):
        app = (ROOT / "App" / "app_remote.c").read_text(encoding="utf-8")
        self.assertIn("AppManualDrive_Submit", app)
        self.assertNotIn("WheelSpeed_SetTarget", app)
        self.assertNotIn("Safety_BeginAction", app)
        self.assertNotIn("Motor_SetPWM", app)


if __name__ == "__main__":
    unittest.main()
