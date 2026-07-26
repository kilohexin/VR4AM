from __future__ import annotations

import json
from pathlib import Path

import pytest

from app.recording.commissioning import (
    CommissioningRecorder,
    RecorderUnavailable,
)


def _read_kinds(path: Path) -> list[str]:
    return [
        json.loads(line)["kind"]
        for line in path.read_text(encoding="utf-8").splitlines()
    ]


@pytest.mark.asyncio
async def test_recorder_creates_jsonl_summary_and_human_template(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={"git_commit": "abc123", "backend": "LEBAI_READONLY"},
        session_id="test-session",
    )
    await recorder.start()
    await recorder.write_critical_event(
        "preflight_result",
        {"ready": False, "reason": "real_robot_readonly"},
        100,
    )
    await recorder.close()

    session = tmp_path / "test-session"
    kinds = _read_kinds(session / "session.jsonl")
    assert kinds == [
        "session_started",
        "preflight_result",
        "session_ended",
    ]
    summary = json.loads(
        (session / "summary.json").read_text(encoding="utf-8")
    )
    assert summary["critical_events"] == 3
    assert summary["dropped_normal_events"] == 0
    assert (session / "commissioning-report.md").exists()


@pytest.mark.asyncio
async def test_full_normal_buffer_drops_new_sample_not_critical_stop(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={},
        session_id="bounded",
        capacity=2,
    )
    await recorder.write_event({"kind": "robot_state_sample", "n": 1}, 1)
    await recorder.write_event({"kind": "robot_state_sample", "n": 2}, 2)
    await recorder.write_event({"kind": "robot_state_sample", "n": 3}, 3)
    await recorder.write_critical_event(
        "stop_requested",
        {"reason": "stale"},
        4,
    )
    await recorder.start()
    await recorder.close()

    kinds = _read_kinds(tmp_path / "bounded" / "session.jsonl")
    assert "stop_requested" in kinds
    assert kinds.count("robot_state_sample") == 2
    assert recorder.dropped_normal_events == 1


@pytest.mark.asyncio
async def test_full_critical_queue_fails_fast_instead_of_waiting(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={},
        session_id="critical-full",
        critical_capacity=1,
    )
    await recorder.write_critical_event("stop_requested", {}, 1)

    with pytest.raises(
        RecorderUnavailable,
        match="^critical_log_queue_full$",
    ):
        await recorder.write_critical_event("robot_fault", {}, 2)

    assert recorder.fatal_error == "critical_log_queue_full"


@pytest.mark.asyncio
async def test_non_finite_event_is_rejected_without_corrupting_jsonl(
    tmp_path: Path,
) -> None:
    recorder = CommissioningRecorder(
        tmp_path,
        metadata={},
        session_id="finite",
    )
    await recorder.start()

    with pytest.raises(
        RecorderUnavailable,
        match="^event_not_json_serializable$",
    ):
        await recorder.write_event({"kind": "bad", "value": float("nan")}, 1)

    await recorder.close()
    for line in (
        tmp_path / "finite" / "session.jsonl"
    ).read_text(encoding="utf-8").splitlines():
        json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError()))
