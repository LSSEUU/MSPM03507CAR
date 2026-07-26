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
        # m6_tune.py is intentionally a thin package entry point.  The
        # backward-compatible CSV inventory now lives with the shared host
        # implementation rather than being duplicated in the launcher.
        host_tree = ast.parse(
            (ROOT / "python" / "src" / "m6link" / "compat.py").read_text(
                encoding="utf-8"
            )
        )
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

    def test_m6_logging_suppresses_duplicate_legacy_csv_and_runs_after_tasks(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        task = diagnostics.split("void Diagnostics_Task100ms(void)", 1)[1].split(
            "static void Diagnostics_Service100ms", 1
        )[0]
        legacy_service = diagnostics.split(
            "static void Diagnostics_Service100ms", 1
        )[1].split("const DiagReport_t *Diagnostics_GetReports", 1)[0]
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")

        self.assertNotIn("debug_", task)
        self.assertNotIn("printf", task)
        self.assertIn(
            "event->legacy_csv_enabled = !M6Telemetry_IsLogging();",
            task,
        )
        self.assertIn(
            "bool legacy_csv_enabled = event->legacy_csv_enabled;",
            legacy_service,
        )
        self.assertGreaterEqual(legacy_service.count("legacy_csv_enabled"), 4)
        self.assertLess(
            legacy_service.index('debug_printf("M3,'),
            legacy_service.index("if (infrared_report_pending)"),
        )
        self.assertLess(
            legacy_service.index('debug_printf("M4,'),
            legacy_service.index("if (infrared_report_pending)"),
        )
        self.assertLess(
            legacy_service.index('debug_printf("M5,'),
            legacy_service.index("if (infrared_report_pending)"),
        )
        self.assertLess(
            legacy_service.index("if (Diagnostics_ActionTelemetryActive()) return;"),
            legacy_service.index("if (infrared_report_pending)"),
        )
        self.assertLess(
            main.index("if (Scheduler_Take(SCHEDULER_TASK_100MS))"),
            main.index("M6Telemetry_Service();"),
        )
        service = (ROOT / "System" / "m6_telemetry.c").read_text(
            encoding="utf-8"
        ).split("void M6Telemetry_Service(void)", 1)[1].split(
            "void M6Telemetry_GetStats", 1
        )[0]
        self.assertNotIn("snprintf", service)
        self.assertIn("if (!s_formatted_pending)", service)
        self.assertIn(
            "debug_try_write(s_formatted_line, s_formatted_length)", service
        )


if __name__ == "__main__":
    unittest.main()
