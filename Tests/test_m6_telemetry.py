import ast
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M6TelemetryTests(unittest.TestCase):
    def test_nonblocking_ring_and_integer_csv(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "m6_telemetry_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "System"),
                "-I",
                str(ROOT / "Hardware"),
                "-I",
                str(ROOT / "Control"),
                "-I",
                str(ROOT / "App"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "m6_telemetry_test.c"),
                str(ROOT / "System" / "control_params.c"),
                str(ROOT / "System" / "m6_telemetry.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_firmware_and_host_field_inventories_match(self):
        host_tree = ast.parse((ROOT / "scripts" / "m6_tune.py").read_text())
        host_fields = None
        for node in host_tree.body:
            if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == "TELEMETRY_FIELDS"
                for target in node.targets
            ):
                host_fields = ast.literal_eval(node.value)
        self.assertIsNotNone(host_fields)
        firmware = (ROOT / "System" / "m6_telemetry.c").read_text(
            encoding="utf-8"
        )
        for field in host_fields:
            self.assertIn(field, firmware)

    def test_capture_path_does_not_format_or_print(self):
        source = (ROOT / "System" / "m6_telemetry.c").read_text(
            encoding="utf-8"
        )
        capture = source.split("void M6Telemetry_Capture20ms", 1)[1].split(
            "void M6Telemetry_Service", 1
        )[0]
        self.assertNotIn("snprintf", capture)
        self.assertNotIn("debug_try_write", capture)


if __name__ == "__main__":
    unittest.main()
