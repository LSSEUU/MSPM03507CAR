from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class LeverActuatorTests(unittest.TestCase):
    def test_unsafe_states_do_not_emit_motion(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "lever_actuator_test.exe"
            command = [
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-DLEVER_BOOT_PROFILE_ENABLED=1",
                "-I", str(ROOT / "System"), "-I", str(ROOT / "Config"),
                "-I", str(ROOT / "Hardware"),
                str(ROOT / "Tests" / "lever_actuator_test.c"),
                str(ROOT / "System" / "lever_actuator.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_raw_rpc_motion_is_gated(self):
        source = (ROOT / "System" / "zdt_stepper_test.c").read_text(encoding="utf-8")
        gate = source.index("Raw host motion is permanently closed")
        dispatch = source.index("if (request->command == ZDT_TEST_COMMAND_STOP")
        self.assertLess(gate, dispatch)
        dashboard = (ROOT / "python" / "src" / "m6link" / "dashboard.py").read_text(encoding="utf-8")
        for route in ("/api/stepper/enable", "/api/stepper/velocity", "/api/stepper/position"):
            self.assertNotIn(f'"{route}":', dashboard)


if __name__ == "__main__":
    unittest.main()
