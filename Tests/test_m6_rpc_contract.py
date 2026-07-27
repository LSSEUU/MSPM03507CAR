import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class M6RpcContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "System" / "m6_rpc.c").read_text(encoding="utf-8")

    def _opcode_block(self, opcode: str, next_opcode: str) -> str:
        start = self.source.index(f"if (opcode == M6_RPC_{opcode})")
        end = self.source.index(f"if (opcode == M6_RPC_{next_opcode})", start)
        return self.source[start:end]

    def test_session_close_length_precedes_state(self):
        block = self._opcode_block("SESSION_CLOSE", "FAULT_CLEAR")
        self.assertLess(block.index("length != 0U"), block.index("action_base_safe()"))
        self.assertIn("M6_RESULT_BAD_LENGTH", block)

    def test_param_defaults_distinguishes_stale_stage(self):
        block = self._opcode_block("PARAM_DEFAULTS", "PARAM_LOAD")
        self.assertIn("if (length != 4U) return M6_RESULT_BAD_LENGTH", block)
        self.assertIn("*detail = ControlParams_StageRevision()", block)
        self.assertIn("M6_RESULT_STALE_STAGE_REVISION", block)

    def test_heading_calibrate_length_precedes_boot_validation(self):
        start = self.source.index("if (opcode == M6_RPC_HEADING_CALIBRATE)")
        block = self.source[start:self.source.index("return M6_RESULT_UNSUPPORTED_OPCODE", start)]
        self.assertLess(block.index("length != 4U"), block.index("u32(body) != s_boot_id"))
        self.assertIn("M6_RESULT_BAD_LENGTH", block)

    def test_heading_rate_action_preserves_and_starts_rate_target(self):
        arm = self._opcode_block("ACTION_ARM", "ACTION_CONFIRM")
        confirm = self._opcode_block("ACTION_CONFIRM", "ACTION_CANCEL")
        self.assertIn("s_action.yaw_rate_mdeg_s = i32(&body[12])", arm)
        self.assertIn("M6_ACTION_KIND_HEADING_RATE", confirm)
        self.assertIn("Heading_StartRate", confirm)

    def test_line_action_uses_app_lifecycle_and_timed_stop(self):
        confirm = self._opcode_block("ACTION_CONFIRM", "ACTION_CANCEL")
        service = self.source[self.source.index("void M6Rpc_Service"):]
        stop = self.source[self.source.index("static void stop_action"):
                           self.source.index("void M6Rpc_Init")]
        self.assertIn("AppLine_StartTimed(now, s_action.duration_ms)", confirm)
        self.assertIn("M6_ACTION_KIND_LINE_FOLLOW", stop)
        self.assertIn("AppLine_Stop(now_ms)", stop)
        self.assertIn("line.state != APP_LINE_RUNNING", service)
        self.assertIn("s_action.duration_ms != M6_LINE_ACTION_CONTINUOUS_DURATION_MS", service)

    def test_parameter_reads_distinguish_length_from_reserved_bytes(self):
        for opcode, next_opcode in (
            ("PARAM_VALUE_GET", "PARAM_DESCRIPTOR_GET"),
            ("PARAM_DESCRIPTOR_GET", "SESSION_CLOSE"),
        ):
            block = self._opcode_block(opcode, next_opcode)
            length = block.index("length != 4U")
            reserved = block.index("body[1] | body[2] | body[3]")
            self.assertLess(length, reserved)
            self.assertIn("M6_RESULT_BAD_LENGTH", block[:reserved])
            self.assertIn("M6_RESULT_MALFORMED", block[reserved:])

    def test_snapshot_uses_shared_body_without_consuming_sample_id(self):
        block = self._opcode_block("SNAPSHOT_GET", "IDENTITY_GET")
        self.assertIn("fill_telemetry_payload", block)
        self.assertIn("telemetry_length - 8U", block)
        self.assertNotIn("s_next_sample_id", block)
        telemetry = self.source[self.source.index("static bool fill_telemetry_payload"):
                                self.source.index("void M6Rpc_Telemetry20ms")]
        for length in (40, 60, 56, 72):
            self.assertIn(f"*payload_length = {length}U", telemetry)

    def test_stale_results_report_device_current_value(self):
        for result in (
            "M6_RESULT_STALE_ACTIVE_REVISION",
            "M6_RESULT_STALE_STAGE_REVISION",
            "M6_RESULT_STALE_GENERATION",
        ):
            cursor = 0
            while True:
                index = self.source.find(f"return {result};", cursor)
                if index < 0:
                    break
                prefix = self.source[max(0, index - 120):index]
                self.assertIn("*detail =", prefix, f"missing detail before {result}")
                cursor = index + 1


if __name__ == "__main__":
    unittest.main()
