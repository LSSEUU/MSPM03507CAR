from __future__ import annotations

from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import zipfile

import pytest

from m6link.dashboard_recording import DashboardRecordingManager, FORMAT_VERSION
from m6link.models import LinkEvent, TelemetrySample, VisionUartChunk
from m6link.registry import ActionReason, ActionState, EventCode, TelemetryStream


class FakeClock:
    def __init__(self) -> None:
        self.value = 100.0
        self.base = datetime(2026, 7, 27, 1, 2, 3, tzinfo=timezone.utc)

    def monotonic(self) -> float:
        return self.value

    def utc(self) -> datetime:
        return self.base + timedelta(seconds=self.value - 100.0)

    def advance(self, seconds: float) -> None:
        self.value += seconds


class FakeSession:
    connected = True

    def __init__(self) -> None:
        self.abort_calls: list[bool] = []

    def abort_h_mission(self, *, telemetry_gap: bool = False):
        self.abort_calls.append(telemetry_gap)
        return {"stopped": True, "telemetry_gap": telemetry_gap}


def sample(sample_id: int, time_ms: int, *, gap: bool = False, value: float = 10.0) -> TelemetrySample:
    return TelemetrySample(
        TelemetryStream.CORE,
        sample_id,
        time_ms,
        {"speed_l_mm_s": value, "speed_r_mm_s": value + 1, "yaw_cdeg": sample_id},
        gap_before=gap,
    )


def manager(tmp_path: Path, clock: FakeClock) -> DashboardRecordingManager:
    return DashboardRecordingManager(
        FakeSession(),
        {"firmware_id": "M6-test", "boot_id": 42},
        tmp_path / "recordings",
        clock=clock.monotonic,
        utc_clock=clock.utc,
        minimum_free_bytes=0,
        sampling_periods={"core": 20, "wheel": 40, "line": 40, "health": 1000},
    )


def session_directory(root: Path) -> Path:
    matches = list(root.glob("*/*"))
    assert len(matches) == 1
    return matches[0]


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_manual_recording_writes_ai_friendly_session(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)

    status = recording.start_manual("直线测试", "检查左右轮一致性")
    recording.ingest(sample(1, 20, value=12.25))
    clock.advance(0.02)
    recording.ingest(sample(2, 40, gap=True, value=float("nan")))
    for stream, values in (
        (TelemetryStream.WHEEL, {"filtered_l_mm_s": 11.125, "filtered_r_mm_s": 12.375}),
        (TelemetryStream.LINE, {"position_x1000": -123, "confidence_x1000": 987}),
        (TelemetryStream.HEALTH, {"task_missed": 0, "uart_rx_drop": 0}),
    ):
        recording.ingest(TelemetrySample(stream, 1, 40, values))
    camera_bytes = b"target,12\n\x00\x7d\x7e\xff"
    recording.ingest(VisionUartChunk(
        time_ms=41,
        data=camera_bytes,
        status_flags=0x06,
        rx_bytes=120,
        tx_bytes=0,
        rx_overflow=1,
        tx_overflow=0,
        hardware_errors=2,
        bridge_dropped=5,
        gap_before=True,
        sequence=90,
    ))
    recording.ingest(LinkEvent(
        int(EventCode.TELEMETRY_GAP), 1, 40, 1,
        {"stream_id": int(TelemetryStream.LINE), "reason": 1, "drop_delta": 4, "latest_sample_id": 8},
    ))
    recording.ingest(LinkEvent(
        int(EventCode.SAFETY_FAULT), 2, 40, 1,
        {"fault_bits": 0x100, "new_fault_bits": 0x100},
    ))
    recording.record_event("parameter_apply", {"revision_before": 7, "revision_after": 8})
    recording.add_marker("进入直道")
    stopped = recording.stop()

    assert status["recording"] is True
    assert stopped["recording"] is False
    directory = session_directory(tmp_path / "recordings")
    assert {path.name for path in directory.iterdir()} == {
        "README.md", "manifest.json", "schema.json", "summary.json", "timeline.index.json", "timeline.jsonl"
    }
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    schema = json.loads((directory / "schema.json").read_text(encoding="utf-8"))
    records = read_jsonl(directory / "timeline.jsonl")
    assert manifest["format_version"] == FORMAT_VERSION
    assert manifest["name"] == "直线测试"
    assert manifest["sampling"]["core"]["period_ms"] == 20
    assert manifest["complete"] is True
    assert manifest["gaps"]["total"] == 6
    assert set(("core", "wheel", "line", "health")) <= set(manifest["counts"])
    assert summary["faults"][0]["event"] == "SAFETY_FAULT"
    assert summary["markers"][0]["label"] == "进入直道"
    assert schema["streams"]["core"]["stream_id"] == 1
    assert schema["uart_streams"]["vision_uart_rx"]["baud"] == 115200
    uart_record = next(record for record in records if record["record_type"] == "uart_data")
    assert uart_record["stream"] == "vision_uart_rx"
    assert bytes.fromhex(uart_record["values"]["data_hex"]) == camera_bytes
    assert uart_record["values"]["bridge_dropped"] == 5
    assert uart_record["gap"] is True
    assert [record["record_index"] for record in records] == list(range(1, len(records) + 1))
    assert records[2]["values"]["speed_l_mm_s"] is None
    assert "NaN" not in (directory / "timeline.jsonl").read_text(encoding="utf-8")


def test_action_recording_includes_five_second_preroll_and_two_second_postroll(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)
    for index in range(7):
        recording.ingest(sample(index + 1, (index + 1) * 1000))
        if index < 6:
            clock.advance(1.0)

    recording.start_action(name="循迹", action={"action_id": 7, "kind": {"name": "LINE_FOLLOW", "value": 5}})
    clock.advance(1.0)
    recording.ingest(
        LinkEvent(
            int(EventCode.ACTION_STATE), 0, 7000, 1,
            {"action_id": 7, "state": int(ActionState.COMPLETE), "reason": 2},
        )
    )
    assert recording.status()["post_roll"] is True
    clock.advance(1.9)
    recording.tick()
    assert recording.status()["recording"] is True
    clock.advance(0.2)
    recording.tick()

    directory = session_directory(tmp_path / "recordings")
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    records = read_jsonl(directory / "timeline.jsonl")
    telemetry = [record for record in records if record["record_type"] == "telemetry"]
    assert recording.status()["recording"] is False
    assert manifest["pre_roll_ms"] == 5000
    assert manifest["stop_reason"] == "action_complete"
    assert [record["sample_id"] for record in telemetry] == [2, 3, 4, 5, 6, 7]
    assert telemetry[0]["elapsed_ms"] == 0


def test_action_recording_infers_terminal_edge_from_core_targets(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)
    recording.start_action(
        name="wheel-fallback",
        action={"action_id": 9, "state": {"value": 2, "name": "RUNNING"}},
    )
    recording.ingest(TelemetrySample(
        TelemetryStream.CORE,
        1,
        1000,
        {"request_l_mm_s": 100, "request_r_mm_s": 0, "fault_bits": 0},
    ))
    clock.advance(1.0)
    recording.ingest(TelemetrySample(
        TelemetryStream.CORE,
        2,
        2000,
        {"request_l_mm_s": 0, "request_r_mm_s": 0, "fault_bits": 0},
    ))
    assert recording.status()["post_roll"] is True
    clock.advance(2.1)
    recording.tick()

    directory = session_directory(tmp_path / "recordings")
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    records = read_jsonl(directory / "timeline.jsonl")
    assert manifest["stop_reason"] == "action_complete"
    assert summary["action_result"]["state"] == int(ActionState.COMPLETE)
    assert summary["action_result"]["reason"] == int(ActionReason.COMPLETE)
    inferred = next(record for record in records if record["event"] == "action_completion_inferred")
    assert inferred["values"]["source"] == "core_target_zero"


def test_local_k1_line_edge_auto_records_and_gap_aborts(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)
    recording.ingest(sample(1, 1000))
    clock.advance(1.0)
    recording.ingest(TelemetrySample(
        TelemetryStream.LINE, 1, 2000, {"app_state": 2, "active_mask": 0x18}
    ))
    assert recording.status()["recording"] is True
    assert recording.status()["trigger"] == "action"

    clock.advance(0.04)
    recording.ingest(TelemetrySample(
        TelemetryStream.WHEEL, 1, 2040, {"filtered_l_mm_s": 200}, gap_before=True
    ))
    assert recording.session.abort_calls == [True]

    clock.advance(0.04)
    recording.ingest(TelemetrySample(
        TelemetryStream.LINE, 2, 2080, {"app_state": 0, "active_mask": 0x18}
    ))
    assert recording.status()["post_roll"] is True
    clock.advance(2.1)
    recording.tick()
    assert recording.status()["recording"] is False
    directory = session_directory(tmp_path / "recordings")
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    records = read_jsonl(directory / "timeline.jsonl")
    assert manifest["trigger"] == "action"
    assert manifest["pre_roll_ms"] == 1000
    assert any(record["event"] == "h_mission_gap_abort" for record in records)


def test_manual_recording_survives_reconnect_but_action_recording_does_not(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)
    recording.start_manual()
    recording.record_event("connection_lost", {"port": "COM10"})
    assert recording.status()["recording"] is True
    recording.update_identity({"firmware_id": "M6-test", "boot_id": 43})
    assert recording.status()["recording"] is True
    recording.stop()

    clock.advance(1)
    recording.start_action(action={"action_id": 8})
    recording.record_event("connection_lost", {"port": "COM10"})
    assert recording.status()["recording"] is True
    clock.advance(3.1)
    recording.tick()
    assert recording.status()["recording"] is False
    sessions = recording.list_sessions()["recordings"]
    action_session = next(item for item in sessions if item["trigger"] == "action")
    assert action_session["status"] == "incomplete"
    assert action_session["stop_reason"] == "action_connection_lost"


def test_library_timeline_seek_zip_and_path_traversal(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)
    started = recording.start_manual("seek")
    for index in range(5):
        recording.ingest(sample(index + 1, index * 1000))
        clock.advance(1.0)
    recording.stop()
    session_id = started["session_id"]

    listing = recording.list_sessions()["recordings"]
    page = recording.timeline(session_id, from_ms=2000, limit=2)
    assert listing[0]["session_id"] == session_id
    assert len(page["records"]) == 2
    assert page["records"][0]["elapsed_ms"] >= 2000
    assert page["has_more"] is True
    archive = recording.create_download(session_id)
    try:
        with zipfile.ZipFile(archive) as bundle:
            names = set(bundle.namelist())
        assert f"{session_id}/timeline.jsonl" in names
        assert f"{session_id}/manifest.json" in names
    finally:
        archive.unlink(missing_ok=True)
    with pytest.raises(ValueError, match="非法"):
        recording.manifest("../../outside")
    directory = session_directory(tmp_path / "recordings")
    index = json.loads((directory / "timeline.index.json").read_text(encoding="utf-8"))["entries"]
    assert [item["elapsed_ms"] for item in index] == [0, 1000, 2000, 3000, 4000]


def test_stale_recording_is_recovered_as_incomplete(tmp_path: Path) -> None:
    clock = FakeClock()
    first = manager(tmp_path, clock)
    first.start_manual("crash")
    first.ingest(sample(1, 20))

    recovered = manager(tmp_path, clock)
    item = recovered.list_sessions()["recordings"][0]
    assert item["status"] == "incomplete"
    assert item["stop_reason"] == "host_interrupted"
    directory = session_directory(tmp_path / "recordings")
    assert json.loads((directory / "timeline.index.json").read_text(encoding="utf-8"))["entries"]


def test_recovery_truncates_a_partial_final_jsonl_record(tmp_path: Path) -> None:
    clock = FakeClock()
    first = manager(tmp_path, clock)
    first.start_manual("partial")
    first.ingest(sample(1, 20))
    directory = session_directory(tmp_path / "recordings")
    with (directory / "timeline.jsonl").open("ab") as target:
        target.write(b'{"record_index":999')

    recovered = manager(tmp_path, clock)
    manifest = recovered.manifest(directory.name)
    records = read_jsonl(directory / "timeline.jsonl")
    assert manifest["status"] == "incomplete"
    assert records[-1]["record_index"] == 2


def test_disk_write_failure_closes_session_as_incomplete(tmp_path: Path) -> None:
    class FailingTimeline:
        def __init__(self, wrapped) -> None:
            self.wrapped = wrapped

        def tell(self):
            return self.wrapped.tell()

        def write(self, _payload):
            raise OSError("simulated disk failure")

        def flush(self):
            return self.wrapped.flush()

        def close(self):
            return self.wrapped.close()

    clock = FakeClock()
    recording = manager(tmp_path, clock)
    recording.start_manual("disk-failure")
    assert recording._writer is not None
    recording._writer._timeline = FailingTimeline(recording._writer._timeline)
    recording.ingest(sample(1, 20))

    status = recording.status()
    directory = session_directory(tmp_path / "recordings")
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert status["recording"] is False
    assert "写盘失败" in status["last_error"]
    assert manifest["status"] == "incomplete"
    assert manifest["stop_reason"] == "disk_write_failed"


def test_host_shutdown_keeps_manual_session_readable_and_incomplete(tmp_path: Path) -> None:
    clock = FakeClock()
    recording = manager(tmp_path, clock)
    recording.start_manual("shutdown")
    recording.ingest(sample(1, 20))
    recording.close()

    directory = session_directory(tmp_path / "recordings")
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "incomplete"
    assert manifest["stop_reason"] == "host_shutdown"
    assert read_jsonl(directory / "timeline.jsonl")
