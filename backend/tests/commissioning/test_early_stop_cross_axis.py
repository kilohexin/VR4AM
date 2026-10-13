from __future__ import annotations

import asyncio
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest

from app.commissioning.actions import SmokeOptions, TranslationAction
from app.commissioning.smoke import run_smoke
from app.config import REAL_ROBOT_CONFIRMATION
from tests.commissioning.test_actions import _control_config
from tests.robots.fake_lebai import FakeLebaiClient


def _events(tmp_path: Path) -> list[dict]:
    return [json.loads(line) for path in (tmp_path / "logs").glob("*/session.jsonl")
            for line in path.read_text(encoding="utf-8").splitlines()]


def _drifting_client(
    axis: str, drift: float, *, stop_fails: bool = False, stop_late: bool = False,
):
    client = FakeLebaiClient.idle()
    initial = dict(client.kin_data["actual_tcp_pose"])
    move = client.move_pvat
    stop = client.stop_move

    async def ik(_pose, joints):
        return list(joints)

    async def drifting_move(p, v, a, t):
        result = await move(p, v, a, t)
        pose = dict(initial)
        pose[axis] = float(initial[axis]) + drift
        client.kin_data["actual_tcp_pose"] = pose
        client.kin_data["actual_joint_speed"] = [0.0] * 6
        return result

    async def failing_stop():
        await stop()
        if stop_late:
            await asyncio.sleep(0.25)
            return
        raise RuntimeError("offline stop transport failure")

    client.kinematics_inverse = ik
    client.move_pvat = drifting_move
    if stop_fails or stop_late:
        client.stop_move = failing_stop
    return client


@pytest.mark.asyncio
@pytest.mark.parametrize("target_axis,drift_axis,drift", [
    ("z", "x", 0.001), ("z", "x", -0.001),
    ("z", "y", 0.001), ("z", "y", -0.001),
    ("x", "y", 0.001), ("y", "z", 0.001),
    ("z", "x", 0.0005429522737700615), ("z", "y", 0.000501),
])
async def test_early_stop_aborts_cross_axis_drift_before_axis_motion_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
    target_axis: str, drift_axis: str, drift: float,
) -> None:
    client = _drifting_client(drift_axis, drift)
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_MOTION_TIMEOUT_S", 0.12)
    with pytest.raises(RuntimeError, match="^smoke_cross_axis_deviation$"):
        await run_smoke(SmokeOptions(_control_config(tmp_path),
                                    TranslationAction(target_axis, 0.003),
                                    REAL_ROBOT_CONFIRMATION, early_stop_on_motion=True),
                        AsyncMock(return_value=client))
    events = _events(tmp_path)
    guard = [e for e in events if e["kind"] == "smoke_cross_axis_deviation"]
    assert len(guard) == 1
    assert guard[0]["axis"] == target_axis
    assert guard[0]["cross_axis_drift_m"] == pytest.approx(abs(drift))
    assert guard[0]["cross_axis_limit_m"] == 0.0005
    assert guard[0]["signed_progress_m"] == 0.0
    assert guard[0]["state_server_mono_ns"] > 0
    assert not any(e["kind"] in {"smoke_result", "smoke_early_stop_triggered",
                                "smoke_early_stop_not_reproduced"} for e in events)
    calls = [c[0] for c in client.write_calls]
    assert calls.count("stop_move") == 1
    assert not any(c in {"stop_sys", "set_claw", "movej"} for c in calls)
    assert "move_pvat" not in calls[calls.index("stop_move") + 1:]


@pytest.mark.asyncio
@pytest.mark.parametrize("axis,drift", [("x", 0.0004), ("y", 0.0005), ("x", 0.0005)])
async def test_cross_axis_guard_keeps_inside_and_exact_boundary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, axis: str, drift: float,
) -> None:
    client = _drifting_client(axis, drift)
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_MOTION_TIMEOUT_S", 0.12)
    with pytest.raises(RuntimeError, match="^smoke_early_stop_not_reproduced$"):
        await run_smoke(SmokeOptions(_control_config(tmp_path), TranslationAction("z", 0.003),
                                    REAL_ROBOT_CONFIRMATION, early_stop_on_motion=True),
                        AsyncMock(return_value=client))
    assert not any(e["kind"] == "smoke_cross_axis_deviation" for e in _events(tmp_path))


@pytest.mark.asyncio
@pytest.mark.parametrize("stop_late", [False, True])
async def test_cross_axis_abort_keeps_failed_stop_latch_and_never_reports_success(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, stop_late: bool,
) -> None:
    client = _drifting_client("x", 0.001, stop_fails=not stop_late, stop_late=stop_late)
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_MOTION_TIMEOUT_S", 0.12)
    with pytest.raises(RuntimeError, match="^smoke_cross_axis_deviation$"):
        await run_smoke(SmokeOptions(_control_config(tmp_path), TranslationAction("z", 0.003),
                                    REAL_ROBOT_CONFIRMATION, early_stop_on_motion=True,
                                    observe_stop_seconds=0.15),
                        AsyncMock(return_value=client))
    events = _events(tmp_path)
    assert any(e["kind"] == "stop_diagnostics" and e["latched_fault"] == "stop_unverified"
               for e in events)
    assert not any(e["kind"] == "smoke_result" for e in events)
    assert [c[0] for c in client.write_calls].count("stop_move") == 1
    assert any(e["kind"] == "stop_observation_summary" for e in events)
    if stop_late:
        assert any(e["kind"] == "stop_rpc_lifecycle" and e["outcome"] == "returned_late"
                   for e in events)


@pytest.mark.asyncio
async def test_cross_axis_event_recording_failure_still_releases_and_stops(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    from app.commissioning import smoke

    client = _drifting_client("x", 0.001)
    record = smoke._SmokeRecorder.write_critical_event

    async def failing_record(self, kind, *args, **kwargs):
        if kind == "smoke_cross_axis_deviation":
            raise RuntimeError("offline_guard_recorder_failed")
        return await record(self, kind, *args, **kwargs)

    monkeypatch.setattr(smoke._SmokeRecorder, "write_critical_event", failing_record)
    monkeypatch.setattr(smoke, "COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    monkeypatch.setattr(smoke, "COMMISSIONING_MOTION_TIMEOUT_S", 0.12)
    with pytest.raises(RuntimeError, match="^offline_guard_recorder_failed$"):
        await run_smoke(SmokeOptions(_control_config(tmp_path), TranslationAction("z", 0.003),
                                    REAL_ROBOT_CONFIRMATION, early_stop_on_motion=True),
                        AsyncMock(return_value=client))
    calls = [c[0] for c in client.write_calls]
    assert calls.count("stop_move") == 1
    assert "move_pvat" not in calls[calls.index("stop_move") + 1:]
    assert not any(c in {"stop_sys", "set_claw", "movej"} for c in calls)
    assert not any(e["kind"] == "smoke_result" for e in _events(tmp_path))


@pytest.mark.asyncio
async def test_cancellation_during_cross_axis_recording_still_stops_once(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    from app.commissioning import smoke

    client = _drifting_client("x", 0.001)
    entered = asyncio.Event()
    record = smoke._SmokeRecorder.write_critical_event

    async def blocked_record(self, kind, *args, **kwargs):
        if kind == "smoke_cross_axis_deviation":
            entered.set()
            await asyncio.Future()
        return await record(self, kind, *args, **kwargs)

    monkeypatch.setattr(smoke._SmokeRecorder, "write_critical_event", blocked_record)
    monkeypatch.setattr(smoke, "COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    monkeypatch.setattr(smoke, "COMMISSIONING_MOTION_TIMEOUT_S", 0.12)
    task = asyncio.create_task(run_smoke(
        SmokeOptions(_control_config(tmp_path), TranslationAction("z", 0.003),
                     REAL_ROBOT_CONFIRMATION, early_stop_on_motion=True),
        AsyncMock(return_value=client),
    ))
    try:
        await asyncio.wait_for(entered.wait(), timeout=1.0)
        # Guard reporting can block. The original stop must already have run
        # before waiting for this diagnostic record, rather than upon cancel.
        assert [c[0] for c in client.write_calls].count("stop_move") == 1
    finally:
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, timeout=1.0)
    calls = [c[0] for c in client.write_calls]
    assert calls.count("stop_move") == 1
    assert "move_pvat" not in calls[calls.index("stop_move") + 1:]
    assert "stop_sys" not in calls
