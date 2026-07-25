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
        self.assertIn("debug_stats.rx_dropped++", uart)
        self.assertIn("debug_get_emergency_epoch", protocol)
        self.assertIn("ControlParams_CancelPendingApply", protocol)


if __name__ == "__main__":
    unittest.main()
