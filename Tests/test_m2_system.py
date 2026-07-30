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
        self.assertIn("#define OLED_REFRESH_CHUNK_BYTES    4U", oled)
        self.assertIn("oled_refresh_column], OLED_REFRESH_CHUNK_BYTES);", task)
        self.assertNotIn(", OLED_WIDTH);", task)

    def test_status_report_is_emitted_incrementally(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8")
        queue = diagnostics.split("void Diagnostics_PrintReport(void)", 1)[1].split(
            "static void Diagnostics_ServiceReport", 1)[0]
        service = diagnostics.split(
            "static void Diagnostics_ServiceReport(void)", 1)[1].split(
            "void Diagnostics_Poll", 1)[0]
        poll = diagnostics.split("void Diagnostics_Poll(void)", 1)[1].split(
            "void Diagnostics_Task100ms", 1)[0]

        self.assertNotIn("for (", queue)
        self.assertIn("report_pending = true", queue)
        self.assertIn("report_line", service)
        self.assertIn("Diagnostics_ServiceReport();", diagnostics)
        self.assertIn("Diagnostics_QueueInfraredReport();", poll)
        self.assertNotIn("Diagnostics_PrintInfrared();", poll)
        self.assertIn("Diagnostics_ServiceInfraredReport();", diagnostics)

    def test_periodic_tasks_defer_serial_work_and_rx_backlog_skips_sleep(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8")
        remote = (ROOT / "App" / "app_remote.c").read_text(
            encoding="utf-8")
        uart = (ROOT / "Hardware" / "uart_debug.c").read_text(
            encoding="utf-8")
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")

        diagnostics_task = diagnostics.split(
            "void Diagnostics_Task100ms(void)", 1)[1].split(
                "static void Diagnostics_Service100ms", 1)[0]
        remote_task = remote.split(
            "void AppRemote_Task20ms(uint32_t now_ms)", 1)[1].split(
                "bool AppRemote_StartCapture", 1)[0]
        for task in (diagnostics_task, remote_task):
            self.assertNotIn("debug_", task)
            self.assertNotIn("printf", task)
            self.assertNotIn("snprintf", task)

        self.assertIn("Diagnostics100msEvent_t", diagnostics)
        self.assertIn("WheelSpeed_GetSnapshot", diagnostics_task)
        self.assertIn("Heading_GetSnapshot", diagnostics_task)
        self.assertIn("AppLine_GetSnapshot", diagnostics_task)
        self.assertIn("Safety_GetSnapshot", diagnostics_task)
        self.assertIn("diagnostics_100ms_write = next", diagnostics_task)
        self.assertIn("diag100_drop=%lu", diagnostics)
        self.assertIn("Diagnostics_Service100ms", diagnostics)
        self.assertIn("remote_notice_flags |=", remote_task)
        self.assertIn("AppRemote_ServiceNotice();", remote)
        self.assertIn("debug_try_write(k_notice_", remote)
        self.assertIn("#define DEBUG_RX_BUFFER_SIZE   256U", uart)
        self.assertIn("size_t debug_read", uart)
        self.assertIn("#define DIAGNOSTICS_RX_BATCH_BYTES 32U", diagnostics)
        self.assertIn("char command = debug_getc();", diagnostics)
        self.assertIn("only bytes inside a transaction are batch-drained", diagnostics)
        self.assertIn(
            "bool frame_was_active = ControlProtocol_IsFrameActive();",
            diagnostics,
        )
        self.assertIn(
            "if (frame_was_active && !ControlProtocol_IsFrameActive()) break;",
            diagnostics,
        )
        self.assertIn("one completed frame per main pass", diagnostics)
        self.assertIn("if (!debug_kbhit()) __WFI();", main)

    def test_fault_injection_is_excluded_from_production_target(self):
        project = (ROOT / "keil" /
            "empty_LP_MSPM0G3507_nortos_keil.uvprojx").read_text(
                encoding="utf-8")
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8")
        protocol = (ROOT / "System" / "control_protocol.h").read_text(
            encoding="utf-8")
        script = (ROOT / "scripts" / "build_fault_injection.ps1").read_text(
            encoding="utf-8")

        self.assertNotIn("M2_FAULT_INJECTION_TEST", project)
        self.assertIn("#ifdef M2_FAULT_INJECTION_TEST", diagnostics)
        self.assertIn("M2_FAULT_INJECTION_TEST", script)
        self.assertIn("ObjectsFaultInjection", script)
        self.assertIn("fault_injection.generated.uvoptx", script)
        self.assertIn("M6_FLASH_LIBRARY_ONLY", script)
        self.assertIn("Assert-KeilDebugConfiguration", script)
        self.assertNotIn("[switch]$Flash", script)
        self.assertNotIn("fault_injection_flash.log", script)
        self.assertNotIn("'-f'", script)
        self.assertIn("#ifdef M2_FAULT_INJECTION_TEST", protocol)
        self.assertIn('"M6-runtime-20260731-r50-fi"', protocol)

    def test_fault_injection_reset_is_zero_pwm_and_test_only(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8")
        gate = diagnostics.split(
            "static bool Diagnostics_FaultInjectionResetReady", 1)[1].split(
                "#endif", 1)[0]
        commands = diagnostics.split("case 'W':", 1)[1].split(
            "#endif", 1)[0]

        self.assertIn("TEST ONLY:", diagnostics)
        self.assertIn("Z=SYSRST breadcrumb", diagnostics)
        self.assertIn("SystemState_Get() != SYSTEM_STATE_IDLE", gate)
        self.assertIn("Safety_HasFault()", gate)
        self.assertIn("!Diagnostics_ActionIsIdle()", gate)
        self.assertIn("fault_injection_stop_watchdog", gate)
        self.assertIn("Safety_StopMotor();", gate)
        self.assertEqual(commands.count(
            "if (!Diagnostics_FaultInjectionResetReady()) break;"), 2)
        self.assertIn("ResetDiag_HardFaultReset();", commands)
        self.assertIn("expect raw 0x0E; post-boot fault 0x20", commands)
        watchdog = commands.split("case 'Z':", 1)[0]
        self.assertIn("for (;;)", watchdog)
        self.assertIn("Safety_StopMotor();", watchdog)
        self.assertIn("__WFI();", watchdog)


if __name__ == "__main__":
    unittest.main()
