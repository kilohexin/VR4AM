import asyncio
import json
from unittest.mock import AsyncMock

import pytest

from app.commissioning.actions import PrepareAction, SmokeOptions, StopAction
from app.commissioning.smoke import parse_smoke_args, run_smoke
from app.config import REAL_ROBOT_CONFIRMATION
from app.robots.base import BackendCommandError, StopReason
from tests.commissioning.test_actions import _control_config
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.test_lebai_adapter_control import _connected_control_adapter
from tests.robots.test_lebai_adapter_control import FakeClock
from app.commissioning.stop_observation import observe_after_stop
from app.recording.commissioning import CommissioningRecorder
from app.robots.lebai_adapter import RealLebaiAdapter


def test_observation_option_is_opt_in_and_bounded():
    args = ["stop", "--config", "unused.yaml", "--confirm", REAL_ROBOT_CONFIRMATION]
    assert getattr(parse_smoke_args(args), "observe_stop_seconds", None) == 0
    assert parse_smoke_args(args + ["--observe-stop-seconds", "60"]).observe_stop_seconds == 60
    for value in ("-1", "121", "nan", "inf"):
        with pytest.raises(ValueError, match="smoke_observation_out_of_bounds"):
            parse_smoke_args(args + ["--observe-stop-seconds", value])


@pytest.mark.asyncio
async def test_observation_can_read_latched_fault_without_writes_or_clearing_it():
    adapter, client, _ = await _connected_control_adapter()
    adapter._latch_unverified_stop()
    client.robot_state = "STOPPING"
    try:
        state = await adapter.read_stop_observation()
        assert state["raw_robot_state"] == "STOPPING"
        assert state["latched_fault"] == "stop_unverified"
        assert state["actual_q"] == client.kin_data["actual_joint_pose"]
        assert state["actual_qd"] == [0.] * 6
        assert client.write_calls == []
        assert not (await adapter.preflight()).ready
        assert adapter._latched_fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("failed_stop", [False, True])
async def test_smoke_observes_after_stop_even_when_stop_failed(tmp_path, monkeypatch, capsys, failed_stop):
    client = FakeLebaiClient.idle()
    if failed_stop:
        async def failing_stop():
            client.write_calls.append(("stop_move",))
            raise RuntimeError("transport_failed")
        client.stop_move = failing_stop
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    options = SmokeOptions(_control_config(tmp_path), StopAction(), REAL_ROBOT_CONFIRMATION,
                           observe_stop_seconds=.05)
    if failed_stop:
        with pytest.raises(BackendCommandError, match="sdk_call_failed:stop_move"):
            await run_smoke(options, AsyncMock(return_value=client))
    else:
        result = await run_smoke(options, AsyncMock(return_value=client))
        assert result.stable
    events = [json.loads(line) for file in (tmp_path / "logs").glob("*/session.jsonl")
              for line in file.read_text(encoding="utf-8").splitlines()]
    start = next(e for e in events if e["kind"] == "stop_observation_started")
    samples = [e for e in events if e["kind"] == "stop_observation_sample"]
    summary = next(e for e in events if e["kind"] == "stop_observation_summary")
    assert samples
    assert summary["successful_samples"] == len(samples)
    assert summary["assessment"] == "diagnostic_only"
    assert all(e["server_mono_ns"] >= start["server_mono_ns"] for e in samples)
    assert not any(e["kind"] == "stop_requested" and e["server_mono_ns"] > start["server_mono_ns"]
                   for e in events)
    assert {call[0] for call in client.write_calls} <= {"stop_move", "stop_sys"}
    if failed_stop:
        assert samples[0]["state"]["latched_fault"] == "stop_unverified"
        assert "失败" in capsys.readouterr().out
        assert start["prior_error"] == "sdk_call_failed:stop_move"


@pytest.mark.asyncio
async def test_invalid_observation_duration_rejected_before_connect(tmp_path):
    factory = AsyncMock()
    with pytest.raises(ValueError, match="smoke_observation_out_of_bounds"):
        await run_smoke(SmokeOptions(tmp_path / "missing.yaml", StopAction(),
                                    REAL_ROBOT_CONFIRMATION, observe_stop_seconds=999), factory)
    factory.assert_not_awaited()


@pytest.mark.asyncio
async def test_final_read_is_not_cancelled_merely_because_observation_window_ends(tmp_path):
    recorder = CommissioningRecorder(tmp_path, {})
    await recorder.start()

    class Source:
        async def read_stop_observation(self):
            await asyncio.sleep(.08)
            return {"raw_robot_state": "IDLE"}

    try:
        await observe_after_stop(Source(), recorder, .05)
    finally:
        await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    summary = next(e for e in events if e["kind"] == "stop_observation_summary")
    assert summary["complete"]
    assert summary["read_errors"] == 0


@pytest.mark.asyncio
async def test_read_failure_is_logged_and_observation_continues_without_retrying_motion(tmp_path):
    clock = FakeClock()
    recorder = CommissioningRecorder(tmp_path, {})
    await recorder.start()

    class Source:
        calls = 0

        async def read_stop_observation(self):
            self.calls += 1
            clock.advance_ms(80)
            if self.calls == 1:
                raise RuntimeError("robot_disconnected")
            return {"raw_robot_state": "STOPPING", "actual_q": [0.] * 6}

    try:
        with pytest.raises(RuntimeError, match="^stop_observation_incomplete$"):
            await observe_after_stop(Source(), recorder, 1., clock=clock.now_ns, sleep=clock.sleep)
    finally:
        await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    failures = [e for e in events if e["kind"] == "stop_observation_read_failed"]
    summary = next(e for e in events if e["kind"] == "stop_observation_summary")
    assert len(failures) == 1
    assert failures[0]["error"] == "robot_disconnected"
    assert summary["successful_samples"] >= 1
    assert summary["read_errors"] == 1
    assert not summary["complete"]
    assert summary["max_successful_sample_gap_ms"] >= 280


@pytest.mark.asyncio
async def test_cancelled_observation_records_interruption_and_propagates_cancel(tmp_path):
    recorder = CommissioningRecorder(tmp_path, {})
    await recorder.start()
    reading = asyncio.Event()

    class Source:
        async def read_stop_observation(self):
            reading.set()
            await asyncio.Event().wait()

    task = asyncio.create_task(observe_after_stop(Source(), recorder, 60))
    try:
        await asyncio.wait_for(reading.wait(), .5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    finally:
        await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    summary = next(e for e in events if e["kind"] == "stop_observation_summary")
    assert summary["interrupted"]
    assert not summary["complete"]


@pytest.mark.asyncio
async def test_prepare_tail_rejected_before_its_different_stop_cleanup_path(tmp_path):
    factory = AsyncMock()
    with pytest.raises(ValueError, match="^smoke_observation_not_supported_for_prepare$"):
        await run_smoke(SmokeOptions(tmp_path / "missing.yaml", PrepareAction(),
                                    REAL_ROBOT_CONFIRMATION, observe_stop_seconds=60), factory)
    factory.assert_not_awaited()


@pytest.mark.asyncio
async def test_dropped_observation_samples_cannot_be_reported_complete(tmp_path):
    clock = FakeClock()
    # Hold the consumer until after collection to deterministically exercise
    # the real recorder's bounded queue, rather than mocking its drop counter.
    recorder = CommissioningRecorder(tmp_path, {}, capacity=1)

    class Source:
        async def read_stop_observation(self):
            return {"raw_robot_state": "IDLE"}

    with pytest.raises(RuntimeError, match="^stop_observation_incomplete$"):
        await observe_after_stop(Source(), recorder, .8, clock=clock.now_ns, sleep=clock.sleep)
    await recorder.start()
    await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    summary = next(e for e in events if e["kind"] == "stop_observation_summary")
    assert summary["dropped_events"] >= 1
    assert not summary["complete"]


@pytest.mark.asyncio
async def test_timeout_of_every_read_preserves_empty_gap_in_summary(tmp_path):
    recorder = CommissioningRecorder(tmp_path, {})
    await recorder.start()

    class Source:
        async def read_stop_observation(self):
            await asyncio.Event().wait()

    try:
        with pytest.raises(RuntimeError, match="^stop_observation_incomplete$"):
            await observe_after_stop(Source(), recorder, .05)
    finally:
        await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    failure = next(e for e in events if e["kind"] == "stop_observation_read_failed")
    summary = next(e for e in events if e["kind"] == "stop_observation_summary")
    assert failure["error_type"] == "TimeoutError"
    assert summary["successful_samples"] == 0
    assert summary["read_errors"] == 1
    assert summary["max_successful_sample_gap_ms"] >= 450
    assert not summary["complete"]


@pytest.mark.asyncio
async def test_smoke_cancellation_skips_tail_and_still_disconnects(tmp_path, monkeypatch):
    client = FakeLebaiClient.idle()
    calls = 0

    async def cancelled_stop():
        nonlocal calls
        calls += 1
        client.write_calls.append(("stop_move",))
        if calls == 1:
            raise asyncio.CancelledError()

    client.stop_move = cancelled_stop
    disconnected = []
    original_disconnect = RealLebaiAdapter.disconnect

    async def disconnect(adapter):
        await original_disconnect(adapter)
        disconnected.append(adapter._client is None)

    observer = AsyncMock()
    monkeypatch.setattr(RealLebaiAdapter, "disconnect", disconnect)
    monkeypatch.setattr("app.commissioning.smoke.observe_after_stop", observer)
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    with pytest.raises(asyncio.CancelledError):
        await run_smoke(SmokeOptions(_control_config(tmp_path), StopAction(),
                                    REAL_ROBOT_CONFIRMATION, observe_stop_seconds=60),
                        AsyncMock(return_value=client))
    observer.assert_not_awaited()
    assert disconnected == [True]
