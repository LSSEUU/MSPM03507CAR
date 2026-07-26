from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M6ControlProtocolTests(unittest.TestCase):
    def test_protocol_model(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "m6_control_protocol_test.exe"
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
                str(ROOT / "Tests" / "m6_control_protocol_test.c"),
                str(ROOT / "System" / "control_params.c"),
                str(ROOT / "System" / "control_protocol.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_uart_isr_preempts_every_parser_state(self):
        uart = (ROOT / "Hardware" / "uart_debug.c").read_text(encoding="utf-8")
        protocol = (ROOT / "System" / "control_protocol.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("(byte == (uint8_t)'S')", uart)
        self.assertIn("(byte == (uint8_t)' ')", uart)
        self.assertIn("emergency_epoch++", uart)
        self.assertIn("emergency_source = byte == (uint8_t)'S' ? 1U : 2U", uart)
        self.assertIn("debug_stats.rx_dropped++", uart)
        self.assertIn("DEBUG_V2_BUFFER_SIZE   224U", uart)
        self.assertIn("v2_tx_buffer[DEBUG_V2_BUFFER_SIZE]", uart)
        v2_submit = uart.split("int debug_try_write_v2", 1)[1].split(
            "void debug_begin_v2_drain", 1
        )[0]
        self.assertIn("v2_tx_buffer[i] = data[i]", v2_submit)
        self.assertNotIn("Debug_TryWriteMode", v2_submit)
        self.assertIn("debug_get_emergency_epoch", protocol)
        self.assertIn("ControlParams_CancelPendingApply", protocol)
        self.assertIn("M6Rpc_CancelPendingApply", protocol)
        self.assertIn("Diagnostics_SetLegacyTextEnabled(false)", protocol)
        self.assertIn("AppRemote_SetLegacyTextEnabled(false)", protocol)
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        remote = (ROOT / "App" / "app_remote.c").read_text(encoding="utf-8")
        self.assertIn("diagnostics_100ms_read = diagnostics_100ms_write", diagnostics)
        self.assertIn("Diagnostics_Get100msDropped", diagnostics)
        self.assertIn("line_trace_report_pending = false", diagnostics)
        self.assertIn("remote_notice_flags = 0U", remote)


if __name__ == "__main__":
    unittest.main()
