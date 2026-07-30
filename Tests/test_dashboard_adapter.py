from __future__ import annotations

import json
from pathlib import Path

import pytest

from m6link.cli import main as cli_main
from m6link.dashboard_adapter import load_dashboard_recording
from m6link.dashboard_recording import FORMAT_VERSION
from m6link.recording import convert_dashboard_recording, load_recording
from m6link.registry import ActionState


def core(time_ms: int, sample_id: int, *, request: int, speed: int) -> dict:
    return {
        "record_type": "telemetry",
        "stream": "core",
        "sample_id": sample_id,
        "device_time_ms": time_ms,
        "gap": False,
        "values": {
            "time_ms": time_ms,
            "parameter_revision": 7,
            "request_l_mm_s": request,
            "request_r_mm_s": request,
            "speed_l_mm_s": speed,
            "speed_r_mm_s": speed,
            "pwm_l": 120 if request else 0,
            "pwm_r": 121 if request else 0,
            "yaw_cdeg": 0,
            "yaw_rate_cdeg_s": 0,
            "saturation_bits": 0,
            "rate_source": 1,
            "fault_bits": 0,
        },
    }


def wheel(time_ms: int, sample_id: int, *, speed: int) -> dict:
    return {
        "record_type": "telemetry",
        "stream": "wheel",
        "sample_id": sample_id,
        "device_time_ms": time_ms,
        "gap": False,
        "values": {
            "time_ms": time_ms,
            "raw_l_mm_s": speed,
            "raw_r_mm_s": speed,
            "filtered_l_mm_s": speed,
            "filtered_r_mm_s": speed,
            "error_l_mm_s": 100 - speed,
            "error_r_mm_s": 100 - speed,
            "p_pwm_l": 10,
            "p_pwm_r": 10,
            "i_pwm_l": 5,
            "i_pwm_r": 5,
            "ff_pwm_l": 105,
            "ff_pwm_r": 106,
            "ramp_l_mm_s": 100,
            "ramp_r_mm_s": 100,
            "pwm_l": 120,
            "pwm_r": 121,
            "sat_ms_l": 0,
            "sat_ms_r": 0,
            "dt_us": 5000,
            "flags": 0,
            "encoder_delta_l": 2,
            "encoder_delta_r": 2,
        },
    }


def line(time_ms: int, sample_id: int) -> dict:
    return {
        "record_type": "telemetry",
        "stream": "line",
        "sample_id": sample_id,
        "device_time_ms": time_ms,
        "gap": False,
        "values": {
            "time_ms": time_ms,
            "active_mask": 0x18,
            "active_count": 2,
            "cluster_count": 1,
            "raw_position_x1000": 0,
            "position_x1000": 0,
            "derivative_x1000": 0,
            "dynamic_base_mm_s": 100,
            "correction_mm_s": 0,
        },
    }


def health(time_ms: int, sample_id: int) -> dict:
    return {
        "record_type": "telemetry",
        "stream": "health",
        "sample_id": sample_id,
        "device_time_ms": time_ms,
        "gap": False,
        "values": {
            "time_ms": time_ms,
            "fault_bits": 0,
            "task_miss": [0, 0, 0, 0],
            "uart_rx_drop": 0,
            "uart_tx_drop": 0,
            "p2_drop": 0,
        },
    }


def action_event(time_ms: int, state: ActionState) -> dict:
    return {
        "record_type": "device_event",
        "stream": None,
        "sample_id": None,
        "device_time_ms": time_ms,
        "gap": False,
        "event": "ACTION_STATE",
        "values": {
            "action_id": 9,
            "state": int(state),
            "reason": 2 if state == ActionState.COMPLETE else 7,
        },
    }


def write_session(
    root: Path,
    *,
    complete: bool = True,
    stop_reason: str = "action_complete",
    extra_records: list[dict] | None = None,
) -> Path:
    directory = root / "2026-07-30" / "session-test"
    directory.mkdir(parents=True)
    spec = {
        "kind": {"name": "WHEEL_SPEED_TEST", "value": 1},
        "wheel_mask": 3,
        "target_l_mm_s": 100,
        "target_r_mm_s": 100,
        "duration_ms": 1000,
    }
    manifest = {
        "format_version": FORMAT_VERSION,
        "session_id": "session-test",
        "trigger": "action",
        "action": {
            "action_id": 9,
            "kind": {"name": "WHEEL_SPEED_TEST", "value": 1},
            "state": {"name": "RUNNING", "value": 2},
            "spec": spec,
        },
        "firmware_identity": {
            "firmware_id": "M6-runtime-test",
            "parameter_schema": 1,
            "active_revision": 7,
            "generation": 3,
            "boot_id": 42,
            "source": {"name": "DEFAULTS", "value": 0},
        },
        "boot_id": 42,
        "parameters": {
            "summary": {"active_revision": 7},
            "active": {"wheel.left.kp": 0.7},
        },
        "sampling": {
            "core": {"period_ms": 20},
            "wheel": {"period_ms": 40},
            "line": {"period_ms": 60},
            "health": {"period_ms": 1000},
        },
        "gaps": {"total": 0, "by_stream_or_event": {}},
        "status": "complete" if complete else "incomplete",
        "complete": complete,
        "stop_reason": stop_reason,
    }
    records = [
        # Capture order is intentionally asynchronous; the adapter must use
        # device time and put non-core updates before CORE at equal timestamps.
        core(0, 1, request=0, speed=0),
        health(0, 1),
        line(0, 1),
        wheel(0, 1, speed=0),
        core(20, 2, request=100, speed=80),
        core(40, 3, request=100, speed=100),
        wheel(40, 2, speed=100),
        line(60, 2),
        core(60, 4, request=100, speed=100),
        health(80, 2),
        wheel(80, 3, speed=100),
        line(80, 3),
        core(80, 5, request=100, speed=100),
        action_event(100, ActionState.COMPLETE),
    ]
    records.extend(extra_records or [])
    for index, record in enumerate(records, start=1):
        record.setdefault("event", None)
        record["record_index"] = index
        record["elapsed_ms"] = index * 20
        record["host_time_utc"] = f"2026-07-30T00:00:{index:02d}.000Z"
    (directory / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False), encoding="utf-8"
    )
    (directory / "summary.json").write_text(
        json.dumps({
            "format_version": FORMAT_VERSION,
            "complete": complete,
            "stop_reason": stop_reason,
            "gaps": manifest["gaps"],
        }),
        encoding="utf-8",
    )
    (directory / "timeline.jsonl").write_text(
        "\n".join(json.dumps(record, ensure_ascii=False) for record in records) + "\n",
        encoding="utf-8",
    )
    return directory


def test_dashboard_adapter_merges_async_streams_and_preserves_line_layout(tmp_path: Path) -> None:
    directory = write_session(tmp_path)

    rows, metadata = load_dashboard_recording(directory)

    assert [row["time_ms"] for row in rows] == [0, 20, 40, 60, 80]
    assert rows[0]["line_info"] == 0x21
    assert rows[2]["raw_l"] == 100
    assert rows[-1]["missed_tasks"] == 0
    assert metadata["test_mode"] == "wheel"
    assert metadata["requested_speed"] == 100
    assert metadata["parameters"]["wheel.left.kp"] == 0.7
    assert metadata["metrics"]["hard_failures"] == []


def test_convert_dashboard_writes_score_compatible_csv_and_json(tmp_path: Path) -> None:
    directory = write_session(tmp_path)
    result = convert_dashboard_recording(directory, tmp_path / "adapted" / "wheel-100")

    rows, metadata = load_recording(result.json_path)

    assert rows == result.samples
    assert metadata["source_format"] == FORMAT_VERSION
    assert metadata["metrics"]["hard_failures"] == []
    assert result.csv_path.read_text(encoding="utf-8").startswith("M6,time_ms,revision")


@pytest.mark.parametrize(
    ("complete", "stop_reason", "extra", "expected"),
    [
        (False, "host_shutdown", [], "recording_incomplete:host_shutdown"),
        (
            True,
            "action_rejected",
            [action_event(1040, ActionState.REJECTED)],
            "action_rejected",
        ),
        (
            False,
            "device_restarted",
            [{
                "record_type": "device_event",
                "stream": None,
                "sample_id": None,
                "device_time_ms": 1040,
                "gap": False,
                "event": "RESET",
                "values": {"boot_id": 43},
            }],
            "device_restarted",
        ),
    ],
)
def test_dashboard_adapter_promotes_terminal_evidence_to_hard_failure(
    tmp_path: Path,
    complete: bool,
    stop_reason: str,
    extra: list[dict],
    expected: str,
) -> None:
    directory = write_session(
        tmp_path, complete=complete, stop_reason=stop_reason, extra_records=extra
    )

    _, metadata = load_dashboard_recording(directory)

    assert expected in metadata["metrics"]["hard_failures"]


def test_dashboard_adapter_promotes_sample_gap_to_hard_failure(tmp_path: Path) -> None:
    directory = write_session(tmp_path)
    timeline = [
        json.loads(line)
        for line in (directory / "timeline.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    next(record for record in timeline if record.get("stream") == "wheel")["gap"] = True
    (directory / "timeline.jsonl").write_text(
        "\n".join(json.dumps(record) for record in timeline) + "\n", encoding="utf-8"
    )

    _, metadata = load_dashboard_recording(directory)

    assert "telemetry_gap" in metadata["metrics"]["hard_failures"]


def test_dashboard_adapter_unwraps_device_time_and_ignores_same_boot_refresh(
    tmp_path: Path,
) -> None:
    directory = write_session(tmp_path)
    timeline = [
        json.loads(line)
        for line in (directory / "timeline.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    offset = 0xFFFFFFF0
    for record in timeline:
        if record.get("record_type") != "telemetry":
            continue
        wrapped = (int(record["device_time_ms"]) + offset) & 0xFFFFFFFF
        record["device_time_ms"] = wrapped
        record["values"]["time_ms"] = wrapped
    timeline.insert(-1, {
        "record_index": 0,
        "elapsed_ms": 90,
        "host_time_utc": "2026-07-30T00:00:15.000Z",
        "device_time_ms": None,
        "record_type": "dashboard_event",
        "stream": None,
        "event": "identity_changed",
        "sample_id": None,
        "gap": False,
        "values": {"previous_boot_id": 42, "boot_id": 42},
    })
    for index, record in enumerate(timeline, start=1):
        record["record_index"] = index
    (directory / "timeline.jsonl").write_text(
        "\n".join(json.dumps(record) for record in timeline) + "\n", encoding="utf-8"
    )

    rows, metadata = load_dashboard_recording(directory)

    assert [row["time_ms"] for row in rows] == [0, 20, 40, 60, 80]
    assert "device_restarted" not in metadata["metrics"]["hard_failures"]


def test_dashboard_adapter_rejects_cancelled_action(tmp_path: Path) -> None:
    cancelled = {
        "record_type": "dashboard_event",
        "stream": None,
        "sample_id": None,
        "device_time_ms": None,
        "gap": False,
        "event": "control_cancel",
        "values": {},
    }
    directory = write_session(tmp_path, extra_records=[cancelled])

    _, metadata = load_dashboard_recording(directory)

    assert "action_cancelled" in metadata["metrics"]["hard_failures"]


def test_cli_scores_and_adapts_dashboard_session_without_opening_serial(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    directory = write_session(tmp_path)

    assert cli_main(["score", str(directory)]) == 0
    score_output = json.loads(capsys.readouterr().out)
    assert score_output["metrics"]["hard_failures"] == []

    output = tmp_path / "cli" / "wheel"
    assert cli_main(["adapt-dashboard", str(directory), str(output)]) == 0
    adapted_output = json.loads(capsys.readouterr().out)
    assert Path(adapted_output["csv"]).exists()
    assert Path(adapted_output["json"]).exists()
