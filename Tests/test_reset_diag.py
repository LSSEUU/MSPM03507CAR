from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class ResetDiagnosticsTests(unittest.TestCase):
    def test_retained_reset_model(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "reset_diag_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-DRESET_DIAG_HOST_TEST",
                "-I",
                str(ROOT / "System"),
                str(ROOT / "Tests" / "reset_diag_test.c"),
                str(ROOT / "System" / "reset_diag.c"),
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_reset_cause_is_read_once_before_platform_init(self):
        source_roots = [ROOT / name for name in ("Source", "System", "Hardware")]
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for source_root in source_roots
            for path in source_root.glob("*.c")
        )
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        self.assertEqual(sources.count("DL_SYSCTL_getResetCause()"), 1)
        read = main.index("DL_SYSCTL_getResetCause()")
        early = main.index("ResetDiag_EarlyInit(reset_cause)")
        platform = main.index("SYSCFG_DL_init()")
        self.assertLess(read, early)
        self.assertLess(early, platform)

    def test_keil_owns_a_dedicated_uninitialized_region(self):
        scatter = (ROOT / "keil" / "mspm0g3507.sct").read_text(
            encoding="utf-8"
        )
        self.assertIn("RW_IRAM2 0x20200000 0x00007F00", scatter)
        self.assertIn("RW_NOINIT 0x20207F00 UNINIT 0x00000100", scatter)
        self.assertIn("reset_diag.o(.bss.noinit.reset_diag)", scatter)

        project_path = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        )
        root = ET.fromstring(project_path.read_text(encoding="utf-8"))
        paths = [node.text for node in root.findall(".//FilePath")]
        self.assertIn("../System/reset_diag.c", paths)

    def test_reset_report_and_current_identity_are_exposed(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        protocol = (ROOT / "System" / "control_protocol.h").read_text(
            encoding="utf-8"
        )
        flash = (ROOT / "scripts" / "flash.ps1").read_text(encoding="utf-8")
        self.assertIn('"RESET raw=0x%02lX', diagnostics)
        self.assertIn('"RESETCTX ', diagnostics)
        self.assertIn('"RESETIO ', diagnostics)
        task100 = diagnostics[diagnostics.index("void Diagnostics_Task100ms") :]
        self.assertNotIn("Diagnostics_ServiceSafetyReport();", task100)
        poll = diagnostics[
            diagnostics.index("void Diagnostics_Poll") :
            diagnostics.index("void Diagnostics_ServiceBackground")
        ]
        self.assertNotIn("Diagnostics_ServiceSafetyReport();", poll)
        background = diagnostics[
            diagnostics.index("void Diagnostics_ServiceBackground") :
            diagnostics.index("void Diagnostics_Task100ms")
        ]
        self.assertIn("Diagnostics_ServiceSafetyReport();", background)
        service = main.index("Watchdog_Service();")
        record = main.index("Main_RecordHealthyWindow(SystemTime_Now()")
        self.assertLess(service, record)
        self.assertLess(
            main.index("if (Scheduler_Take(SCHEDULER_TASK_100MS))"),
            main.index("Diagnostics_ServiceBackground();"),
        )
        self.assertIn('"M6-runtime-20260801-r158-stop"', protocol)
        self.assertEqual(flash.count("M6-runtime-20260801-r158-stop"), 2)

        for prefix in ("RESET ", "RESETCTX ", "RESETIO "):
            source_line = next(
                line for line in diagnostics.splitlines()
                if f'debug_printf("{prefix}' in line
            )
            format_text = source_line.split('debug_printf("', 1)[1].rsplit(
                '",', 1
            )[0]
            self.assertTrue(format_text.endswith(r"\r\n"))
            worst_case = re.sub(r"%0?\d*l?X", "FFFFFFFF", format_text)
            worst_case = re.sub(r"%l?[ud]", "4294967295", worst_case)
            worst_case = worst_case.replace("%s", "wwdt_service")
            self.assertLess(len(worst_case.encode("ascii")), 256)

    def test_emergency_safety_watchdog_and_flash_boundaries_remain(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        uart = (ROOT / "Hardware" / "uart_debug.c").read_text(
            encoding="utf-8"
        )
        store = (ROOT / "System" / "parameter_store.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("M6Telemetry_StopFromISR();", main)
        self.assertIn("Safety_EmergencyStop(SystemTime_Now());", main)
        self.assertIn("(byte == (uint8_t)'S')", uart)
        self.assertIn("(byte == (uint8_t)' ')", uart)
        self.assertIn("Scheduler_ConsumeHealthyWindow()", main)
        self.assertIn("Watchdog_Service();", main)
        self.assertIn('section(".ramfunc")', store)
        self.assertIn("WWDT0->WWDTCNTRST", store)


if __name__ == "__main__":
    unittest.main()
