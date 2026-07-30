from __future__ import annotations

import io
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link.cli import _create_parser  # noqa: E402
from m6link.models import Identity, TelemetrySample  # noqa: E402
from m6link.monitor import (  # noqa: E402
    MONITOR_SUBSCRIPTIONS,
    TelemetryAggregator,
    TrendHistory,
    render_sparkline,
    render_terminal,
    run_monitor,
)
from m6link.registry import (  # noqa: E402
    REGISTRY,
    Capability,
    ParameterSource,
    TelemetryStream,
)


def identity() -> Identity:
    return Identity(
        firmware_id="M6-runtime-test",
        wire_version=2,
        registry_revision=1,
        parameter_schema=1,
        active_revision=3,
        stage_revision=3,
        generation=7,
        boot_id=11,
        source=ParameterSource.DEFAULTS,
        capabilities=Capability(0),
        port="COM5",
    )


def sample(
    stream: TelemetryStream,
    sample_id: int,
    values: dict[str, object],
    *,
    gap: bool = False,
) -> TelemetrySample:
    return TelemetrySample(
        stream=stream,
        sample_id=sample_id,
        time_ms=sample_id * 20,
        values=values,
        gap_before=gap,
    )


class TelemetryAggregatorTests(unittest.TestCase):
    def test_live_profile_keeps_rpc_and_escape_headroom(self):
        self.assertEqual(MONITOR_SUBSCRIPTIONS[TelemetryStream.CORE], 40)
        self.assertEqual(MONITOR_SUBSCRIPTIONS[TelemetryStream.WHEEL], 60)
        self.assertEqual(MONITOR_SUBSCRIPTIONS[TelemetryStream.LINE], 100)
        rules = {item["value"]: item for item in REGISTRY["telemetry"]["streams"]}
        estimated = sum(
            ((1000 + period - 1) // period)
            * (17 + int(rules[int(stream)]["payload_size"]))
            for stream, period in MONITOR_SUBSCRIPTIONS.items()
        )
        self.assertEqual(estimated, 3553)
        self.assertLessEqual(estimated, 3600)

    def test_aggregator_keeps_latest_sample_and_marks_stale_streams(self):
        aggregator = TelemetryAggregator(clock=lambda: 1.0)
        aggregator.ingest(
            sample(TelemetryStream.CORE, 1, {"fault_bits": 0}),
            received_at=0.5,
        )
        aggregator.ingest(
            sample(TelemetryStream.CORE, 2, {"fault_bits": 4}, gap=True),
            received_at=0.95,
        )
        snapshot = aggregator.snapshot(now=1.2)
        core = snapshot.streams[TelemetryStream.CORE]

        self.assertEqual(core.sample_id, 2)
        self.assertEqual(core.values["fault_bits"], 4)
        self.assertEqual(core.gaps, 1)
        self.assertTrue(core.stale)
        self.assertEqual(snapshot.samples_received, 2)
        self.assertEqual(snapshot.gaps_total, 1)

    def test_terminal_renderer_contains_motor_line_and_health_values(self):
        aggregator = TelemetryAggregator(clock=lambda: 1.0)
        aggregator.ingest(
            sample(
                TelemetryStream.CORE,
                1,
                {
                    "request_l_mm_s": 160,
                    "request_r_mm_s": 155,
                    "pwm_l": 280,
                    "pwm_r": 270,
                    "yaw_cdeg": -8608,
                    "yaw_rate_cdeg_s": 123,
                    "rate_source": 2,
                    "fault_bits": 0,
                },
            ),
            received_at=1.0,
        )
        aggregator.ingest(
            sample(
                TelemetryStream.LINE,
                1,
                {
                    "adc": (100, 200, 300, 400, 500, 600, 700, 800),
                    "active_mask": 0x01,
                    "active_count": 1,
                    "cluster_count": 1,
                    "line_state": 2,
                },
            ),
            received_at=1.0,
        )
        text = render_terminal(aggregator.snapshot(now=1.0), identity())

        self.assertIn("M6 LIVE MONITOR", text)
        self.assertIn("160", text)
        self.assertIn("active[0..7]=10000000 mask=0x01", text)
        self.assertIn("yaw=-86.08 deg", text)
        self.assertIn("faults=0x00000000", text)

    def test_trends_sample_at_render_cadence_and_render_newest_on_right(self):
        aggregator = TelemetryAggregator(clock=lambda: 1.0)
        trends = TrendHistory(4)
        for index, speed in enumerate((0, 50, 100, 150), start=1):
            aggregator.ingest(
                sample(
                    TelemetryStream.WHEEL,
                    index,
                    {"filtered_l_mm_s": speed, "filtered_r_mm_s": -speed},
                ),
                received_at=1.0,
            )
            trends.ingest(aggregator.snapshot(now=1.0))

        self.assertEqual(trends.series("speed_l"), (0.0, 50.0, 100.0, 150.0))
        self.assertEqual(render_sparkline(trends.series("speed_l")), "▁▃▆█")


class FakeMonitorSession:
    def __init__(self, samples: list[TelemetrySample]) -> None:
        self.samples = list(samples)
        self.connected = True
        self.subscriptions: list[dict[TelemetryStream, int]] = []

    def subscribe(self, entries):
        self.subscriptions.append(dict(entries))
        return (), 0

    def read_telemetry(self, timeout=None):
        if self.samples:
            return self.samples.pop(0)
        return None


class MonitorRunnerTests(unittest.TestCase):
    def test_parser_exposes_monitor_options(self):
        args = _create_parser().parse_args(
            [
                "--port", "COM5", "monitor", "--refresh-hz", "5",
                "--trend-seconds", "8", "--duration", "2",
            ]
        )
        self.assertEqual(args.command, "monitor")
        self.assertEqual(args.refresh_hz, 5.0)
        self.assertEqual(args.trend_seconds, 8.0)
        self.assertEqual(args.duration, 2.0)

    def test_runner_subscribes_prints_jsonl_and_unsubscribes(self):
        session = FakeMonitorSession(
            [sample(TelemetryStream.CORE, 1, {"fault_bits": 0})]
        )
        output = io.StringIO()
        ticks = iter(index / 100.0 for index in range(0, 50, 2))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "monitor.jsonl"
            result = run_monitor(
                session,  # type: ignore[arg-type]
                identity(),
                refresh_hz=10.0,
                duration_s=0.105,
                jsonl=True,
                output=path,
                stdout=output,
                clock=lambda: next(ticks),
            )
            saved = path.read_text(encoding="utf-8")

        self.assertEqual(result, 0)
        self.assertEqual(session.subscriptions[0], dict(MONITOR_SUBSCRIPTIONS))
        self.assertEqual(session.subscriptions[-1], {})
        self.assertIn('"samples_received":1', output.getvalue())
        self.assertEqual(saved, output.getvalue())


if __name__ == "__main__":
    unittest.main()
