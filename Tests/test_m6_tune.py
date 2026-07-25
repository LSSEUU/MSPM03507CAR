import importlib.util
import json
import math
from pathlib import Path
import re
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "m6_tune.py"
SPEC = importlib.util.spec_from_file_location("m6_tune", SCRIPT)
m6 = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = m6
SPEC.loader.exec_module(m6)


class FakeTransport:
    def __init__(self, responses=None, *, port="COM42"):
        self.port = port
        self.responses = list(responses or [])
        self.writes = []
        self.closed = False

    def write(self, value):
        self.writes.append(value)
        return len(value)

    def readline(self):
        return self.responses.pop(0) if self.responses else b""

    def close(self):
        self.closed = True

    def reset_input_buffer(self):
        pass


def sample(time_ms, **updates):
    values = {key: 0 for key in m6.TELEMETRY_FIELDS}
    values.update(
        {
            "time_ms": time_ms,
            "revision": 3,
            "request_l": 160,
            "request_r": 160,
            "ramp_l": 160,
            "ramp_r": 160,
            "filter_l": 160,
            "filter_r": 160,
            "gray_mask": 0x18,
            "line_info": 0x11,
        }
    )
    values.update(updates)
    return values


def line_for(values):
    return ",".join(
        [m6.TELEMETRY_PREFIX] + [str(values[key]) for key in m6.TELEMETRY_FIELDS]
    )


class ProtocolTests(unittest.TestCase):
    def test_host_parameter_inventory_matches_public_schema(self):
        self.assertEqual(len(m6.PARAMETER_KEYS), 35)
        self.assertIn("wheel.left.kp", m6.PARAMETER_KEYS)
        self.assertIn("wheel.diag_target", m6.PARAMETER_KEYS)
        self.assertIn("heading.straight.kp", m6.PARAMETER_KEYS)
        self.assertIn("line.position_alpha", m6.PARAMETER_KEYS)
        self.assertIn("line.lost_search_speed", m6.PARAMETER_KEYS)
        self.assertIn("remote.speed_medium", m6.PARAMETER_KEYS)
        self.assertNotIn("wheel.ff_enable", m6.PARAMETER_KEYS)

        descriptor_source = (ROOT / "System" / "control_params.c").read_text(
            encoding="utf-8"
        )
        for key in m6.PARAMETER_KEYS:
            self.assertIn(f'"{key}"', descriptor_source)

    def test_fragmented_response_is_reassembled_and_request_is_written_once(self):
        transport = FakeTransport(
            [b"", b"ok,1,meta.", b"firmware,M6-compat\r\n"]
        )
        client = m6.M6Client(transport, response_timeout=0.2, read_attempts=3)

        self.assertEqual(client.get_one("meta.firmware"), "M6-compat")
        self.assertEqual(transport.writes, [b":1,get,meta.firmware\r\n"])

    def test_interleaved_telemetry_and_other_id_do_not_satisfy_transaction(self):
        transport = FakeTransport(
            [
                (line_for(sample(0)) + "\r\n").encode(),
                b"ok,77,meta.schema,99\r\n",
                b"ok,1,meta.schema,1\r\n",
            ]
        )
        client = m6.M6Client(transport, response_timeout=0.2)

        self.assertEqual(client.get_one("meta.schema"), 1)
        self.assertEqual(len(client.unsolicited), 2)
        self.assertEqual(len(transport.writes), 1)

    def test_device_error_is_typed_and_emergency_stop_is_single_byte(self):
        transport = FakeTransport([b"err,1,state\r\n"])
        client = m6.M6Client(transport, response_timeout=0.1)

        with self.assertRaises(m6.DeviceError) as raised:
            client.apply()
        self.assertEqual(raised.exception.code, "state")
        client.emergency_stop()
        self.assertEqual(transport.writes[-1], b"S")

    def test_plain_decimal_rejects_nan_inf_and_exponents(self):
        for invalid in ("nan", "inf", "-inf", "1e3", math.nan, math.inf):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                m6.decimal_text(invalid)
        self.assertEqual(m6.decimal_text(0.7), "0.7")

    def test_probe_uses_public_meta_keys(self):
        transport = FakeTransport(
            [
                b"ok,1,meta.firmware,M6-test\r\n",
                b"ok,2,meta.schema,1\r\n",
                b"ok,3,meta.revision,7\r\n",
                b"ok,4,meta.source,defaults\r\n",
                b"ok,5,meta.generation,0\r\n",
            ]
        )
        identity = m6.M6Client(transport, response_timeout=0.2).probe("M6", 1)

        self.assertEqual(identity["revision"], 7)
        frames = b"".join(transport.writes)
        for key in (
            b"meta.firmware",
            b"meta.schema",
            b"meta.revision",
            b"meta.source",
        ):
            self.assertIn(key, frames)

    def test_probe_accepts_optional_generation_key_absence(self):
        transport = FakeTransport(
            [
                b"ok,1,meta.firmware,M6-test\r\n",
                b"ok,2,meta.schema,1\r\n",
                b"ok,3,meta.revision,7\r\n",
                b"ok,4,meta.source,defaults\r\n",
                b"err,5,key\r\n",
            ]
        )
        identity = m6.M6Client(transport, response_timeout=0.2).probe("M6", 1)

        self.assertNotIn("generation", identity)

    def test_all_transaction_helpers_emit_lowercase_bounded_frames(self):
        responses = [
            b"ok,1,staged,wheel.left.kp,0.8\r\n",
            b"ok,2,applied,2\r\n",
            b"ok,3,reverted,2\r\n",
            b"ok,4,staged,flash-a\r\n",
            b"ok,5,staged,defaults\r\n",
            b"ok,6,saved,8\r\n",
            b"ok,7,log,m6,20\r\n",
            b"ok,8,log,off,0\r\n",
        ]
        transport = FakeTransport(responses)
        client = m6.M6Client(transport, response_timeout=0.2)

        client.set("wheel.left.kp", 0.8)
        client.apply()
        client.revert()
        client.load()
        client.defaults()
        client.save()
        client.log("m6", 20)
        client.log("off", 0)

        self.assertEqual(len(transport.writes), 8)
        for frame in transport.writes:
            self.assertLessEqual(len(frame), 60)
            self.assertTrue(frame.startswith(b":"))
            self.assertTrue(frame.endswith(b"\r\n"))

    def test_auto_discovery_closes_incompatible_port_and_selects_m6(self):
        created = []

        class Port:
            def __init__(self, device):
                self.device = device

        def factory(port, **kwargs):
            firmware = "legacy" if port == "COM1" else "M6-ready"
            transport = FakeTransport(
                [
                    f"ok,1,meta.firmware,{firmware}\r\n".encode(),
                    b"ok,2,meta.schema,1\r\n",
                    b"ok,3,meta.revision,1\r\n",
                    b"ok,4,meta.source,defaults\r\n",
                    b"ok,5,meta.generation,0\r\n",
                ],
                port=port,
            )
            created.append(transport)
            return transport

        client, identity = m6.discover_client(
            serial_factory=factory,
            port_provider=lambda: [Port("COM1"), Port("COM9")],
            timeout=0.2,
        )

        self.assertEqual(client.port, "COM9")
        self.assertEqual(identity["firmware"], "M6-ready")
        self.assertTrue(created[0].closed)
        self.assertFalse(created[1].closed)


class TelemetryAndScoringTests(unittest.TestCase):
    def test_host_telemetry_schema_matches_firmware_header(self):
        source = (ROOT / "System" / "m6_telemetry.c").read_text(encoding="utf-8")
        match = re.search(r"static const char k_header\[\]\s*=\s*(.*?);", source, re.S)
        self.assertIsNotNone(match)
        fragments = re.findall(r'"([^"\\]*(?:\\.[^"\\]*)*)"', match.group(1))
        firmware_header = bytes("".join(fragments), "utf-8").decode("unicode_escape")
        self.assertEqual(
            tuple(firmware_header.strip("\r\n").split(",")), m6.TELEMETRY_HEADER
        )

    def test_integer_telemetry_parser_rejects_shape_and_float_fields(self):
        values = sample(20)
        self.assertEqual(m6.parse_telemetry_line(line_for(values)), values)
        self.assertIsNone(m6.parse_telemetry_line(",".join(m6.TELEMETRY_HEADER)))
        with self.assertRaises(ValueError):
            m6.parse_telemetry_line("M6,1,2")
        bad = line_for(values).replace(",160,", ",160.0,", 1)
        with self.assertRaises(ValueError):
            m6.parse_telemetry_line(bad)

    def test_scoring_contains_all_required_metrics(self):
        samples = []
        for time_ms in range(0, 2020, 20):
            measured = min(160, int(time_ms * 0.8))
            samples.append(
                sample(
                    time_ms,
                    filter_l=measured,
                    filter_r=measured,
                    pwm_l=300,
                    pwm_r=305,
                    yaw_x100=(time_ms // 20) % 3,
                    pos_filter_x1000=(time_ms // 20) % 5,
                    correction=(time_ms // 20) % 4,
                )
            )
        metrics = m6.score_samples(samples, requested_speed=160, mode="line")

        for key in (
            "rise_time_ms",
            "overshoot_pct",
            "steady_error_pct",
            "error_zero_crossings",
            "saturation_ratio",
            "yaw_rms_deg",
            "line_position_rms",
            "correction_jerk_rms",
            "line_loss_events",
            "missed_task_delta",
            "uart_drop_delta",
            "telemetry_drop_delta",
        ):
            self.assertIn(key, metrics)
        self.assertFalse(metrics["hard_failures"])

    def test_rise_and_steady_error_ignore_pre_arm_and_post_stop_idle_rows(self):
        samples = [sample(0, request_l=0, request_r=0, filter_l=0, filter_r=0)]
        samples += [sample(t) for t in range(20, 1020, 20)]
        samples += [
            sample(t, request_l=0, request_r=0, filter_l=0, filter_r=0)
            for t in range(1020, 2020, 20)
        ]

        metrics = m6.score_samples(samples, requested_speed=160, mode="wheel")

        self.assertEqual(metrics["rise_time_ms"], 0)
        self.assertEqual(metrics["steady_error_pct"], 0)

    def test_fault_drop_line_loss_and_sustained_saturation_are_hard_failures(self):
        cases = {
            "fault": [sample(0), sample(20, fault_bits=4)],
            "drop": [sample(0), sample(20, tx_dropped=1)],
            "line": [sample(0), sample(20, line_info=0)],
            "saturation": [sample(t, saturation_bits=1) for t in range(0, 340, 20)],
        }
        for name, samples in cases.items():
            with self.subTest(name=name):
                mode = "line" if name == "line" else "wheel"
                metrics = m6.score_samples(samples, requested_speed=160, mode=mode)
                self.assertTrue(metrics["hard_failures"])

        ground = [sample(0, line_info=0), sample(20, line_info=0)]
        metrics = m6.score_samples(ground, requested_speed=100, mode="ground")
        self.assertNotIn("line_loss", metrics["hard_failures"])

    def test_acceptance_is_five_percent_and_feedforward_is_ten_percent(self):
        baseline = {
            "score": 100.0,
            "hard_failures": [],
            "overshoot_pct": 5.0,
            "saturation_ratio": 0.05,
            "sustained_saturation_ms": 0,
            "validation_speeds": [100, 160, 220, 300],
        }
        candidate = dict(baseline, score=94.9)
        self.assertTrue(m6.candidate_accepted(baseline, candidate)[0])
        self.assertFalse(m6.candidate_accepted(baseline, candidate, feedforward=True)[0])
        ff_candidate = dict(baseline, score=89.9)
        self.assertTrue(m6.candidate_accepted(baseline, ff_candidate, feedforward=True)[0])
        regressed = dict(ff_candidate, overshoot_pct=5.1)
        self.assertFalse(m6.candidate_accepted(baseline, regressed, feedforward=True)[0])
        single_speed = dict(ff_candidate)
        single_speed.pop("validation_speeds")
        self.assertFalse(m6.candidate_accepted(baseline, single_speed, feedforward=True)[0])

    def test_feedforward_evidence_requires_all_four_speed_points(self):
        def metric(score):
            return {
                "score": score,
                "hard_failures": [],
                "overshoot_pct": 5.0,
                "saturation_ratio": 0.02,
                "sustained_saturation_ms": 0,
            }

        base = {speed: metric(100.0) for speed in (100, 160, 220, 300)}
        good = {speed: metric(89.0) for speed in (100, 160, 220, 300)}
        self.assertTrue(m6.feedforward_evidence_passed(base, good)[0])
        del good[300]
        self.assertFalse(m6.feedforward_evidence_passed(base, good)[0])

    def test_record_writes_csv_and_json_metadata(self):
        values = [sample(t) for t in (0, 20, 40)]
        responses = [b"ok,1,log,m6,20\r\n"]
        responses += [
            (line_for(values[0]) + "\r\n").encode(),
            b"IR manual heartbeat expired; normal brake to IDLE.\r\n",
            (line_for(values[1]) + "\r\n").encode(),
            (line_for(values[2]) + "\r\n").encode(),
        ]

        class RecordTransport(FakeTransport):
            def write(self, value):
                written = super().write(value)
                if value == b":2,log,off,0\r\n":
                    self.responses.append(b"ok,2,log,off,0\r\n")
                return written

        client = m6.M6Client(RecordTransport(responses), response_timeout=0.1)
        with tempfile.TemporaryDirectory() as directory:
            result = m6.TelemetryRecorder(client).record(
                Path(directory) / "run",
                duration_s=0.01,
                period_ms=20,
                test_mode="wheel",
                requested_speed=160,
                identity={
                    "firmware": "M6-test",
                    "schema": 1,
                    "revision": 3,
                    "source": "defaults",
                },
                parameters={"wheel.left.kp": 0.7},
            )
            summary = json.loads(result.json_path.read_text(encoding="utf-8"))
            csv_text = result.csv_path.read_text(encoding="utf-8")

        self.assertIn("M6,time_ms,revision", csv_text)
        self.assertEqual(summary["firmware"], "M6-test")
        self.assertEqual(summary["parameters"]["wheel.left.kp"], 0.7)
        self.assertEqual(len(summary["events"]), 1)
        self.assertEqual(summary["events"][0]["firmware_time_ms"], 0)
        self.assertIn("heartbeat expired", summary["events"][0]["line"])


class FakeSearchClient:
    def __init__(self, fail_rollback=False):
        self.actions = []
        self.stopped = False
        self.fail_rollback = fail_rollback

    def set(self, key, value):
        self.actions.append(("set", key, value))

    def apply(self):
        self.actions.append(("apply",))
        if self.stopped and self.fail_rollback:
            raise m6.DeviceError(99, "state")

    def emergency_stop(self):
        self.stopped = True
        self.actions.append(("stop",))


class CoordinateSearchTests(unittest.TestCase):
    def test_search_never_runs_a_fourth_round_and_each_candidate_changes_one_key(self):
        client = FakeSearchClient()
        observed = []

        def evaluator(candidate, label):
            observed.append(dict(candidate))
            return {"score": 100.0, "hard_failures": []}

        result = m6.CoordinateSearch(client, evaluator).search(
            {"wheel.left.kp": 1.0, "wheel.left.ki": 2.0},
            {"score": 100.0, "hard_failures": []},
            keys=("wheel.left.kp", "wheel.left.ki"),
            max_rounds=3,
        )

        # No improvement ends after round one, which is stricter than the cap.
        self.assertEqual(result.rounds, 1)
        initial = {"wheel.left.kp": 1.0, "wheel.left.ki": 2.0}
        for candidate in observed:
            changed = [key for key in initial if candidate[key] != initial[key]]
            self.assertEqual(len(changed), 1)
        with self.assertRaises(ValueError):
            m6.CoordinateSearch(client, evaluator).search(
                initial,
                {"score": 100.0, "hard_failures": []},
                keys=("wheel.left.kp",),
                max_rounds=4,
            )

    def test_hard_failure_sends_stop_and_reports_pending_rollback(self):
        client = FakeSearchClient(fail_rollback=True)

        def evaluator(candidate, label):
            raise m6.SafetyAbort("firmware_fault")

        result = m6.CoordinateSearch(client, evaluator).search(
            {"wheel.left.kp": 1.0},
            {"score": 10.0, "hard_failures": []},
            keys=("wheel.left.kp",),
        )

        self.assertTrue(result.aborted)
        self.assertTrue(result.pending_rollback)
        self.assertIn(("stop",), client.actions)
        self.assertEqual(result.abort_reason, "firmware_fault")


if __name__ == "__main__":
    unittest.main()
