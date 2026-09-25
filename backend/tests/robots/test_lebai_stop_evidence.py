import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_lebai_adapter_control import _connected_control_adapter


@pytest.mark.asyncio
@pytest.mark.parametrize("raw_state", ["STOPPING", 10, "MOVING", "TEACHING"])
async def test_zero_speed_during_nonterminal_state_does_not_confirm_stop(raw_state):
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = raw_state
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("raw_state", ["IDLE", 5, "STOP", 12, "PAUSED", 6])
async def test_terminal_stop_still_requires_full_stable_window(raw_state):
    adapter, client, clock = await _connected_control_adapter()
    client.robot_state = raw_state
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert clock.value >= 300_000_000
        assert client.write_calls == [("stop_move",)]
        if raw_state in ("STOP", 12, "PAUSED", 6):
            assert not (await adapter.preflight()).ready
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stopping_to_idle_requires_new_stability_window():
    adapter, client, clock = await _connected_control_adapter()

    async def state_transition():
        return "STOPPING" if clock.value < 100_000_000 else "IDLE"

    client.get_robot_state = state_transition
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert clock.value >= 400_000_000
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_hung_diagnostic_recorder_cannot_delay_failed_stop():
    adapter, client, _ = await _connected_control_adapter()

    async def failed_stop():
        client.write_calls.append(("stop_move",))
        raise RuntimeError("stop_rpc_failed")

    async def blocked_recorder(event, timestamp):
        if event.get("kind") == "stop_diagnostics":
            assert client.write_calls == [("stop_move",)]
            assert not adapter._sdk_lock.locked()
            await asyncio.Event().wait()

    client.stop_move = failed_stop
    adapter._event_callback = blocked_recorder
    try:
        with pytest.raises(BackendCommandError, match="^sdk_call_failed:stop_move$"):
            await asyncio.wait_for(adapter.stop(StopReason.GRIP_RELEASED), 0.5)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_blocked_kinematics_recorder_cannot_prevent_stop_failure():
    adapter, client, _ = await _connected_control_adapter()
    client.kin_data["actual_joint_speed"] = [0.1] * 6

    async def blocked_recorder(event, timestamp):
        await asyncio.Event().wait()

    adapter._event_callback = blocked_recorder
    task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await asyncio.wait_for(task, 0.5)
        assert client.write_calls == [("stop_move",)]
    finally:
        if not task.done():
            task.cancel()
        try:
            await task
        except (BackendCommandError, asyncio.CancelledError):
            pass
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_cancellation_during_diagnostic_flush_is_not_swallowed():
    adapter, client, _ = await _connected_control_adapter()
    recording = asyncio.Event()

    async def recorder(event, timestamp):
        if event.get("kind") == "stop_diagnostics":
            recording.set()
            await asyncio.Event().wait()

    adapter._event_callback = recorder
    task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        await asyncio.wait_for(recording.wait(), 0.5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("component", ["joint", "tcp"])
async def test_zero_reported_speed_cannot_hide_position_drift(component):
    adapter, client, _ = await _connected_control_adapter()
    original = client.get_kin_data

    async def drifting_kinematics():
        if component == "joint":
            client.kin_data["actual_joint_pose"][1] += 0.0002
        else:
            client.kin_data["actual_tcp_pose"]["x"] += 0.0001
        return await original()

    client.get_kin_data = drifting_kinematics
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_timeout_records_only_motion_stop_without_hiding_primary_failure():
    adapter, client, _ = await _connected_control_adapter()
    events = []

    async def recorder(event, timestamp):
        events.append(event)

    async def timed_out_stop():
        client.write_calls.append(("stop_move",))
        await asyncio.Event().wait()

    client.stop_move = timed_out_stop
    adapter._event_callback = recorder
    try:
        with pytest.raises(BackendCommandError, match="^sdk_timeout:stop_move$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        diagnostics = [e for e in events if e.get("kind") == "stop_diagnostics"]
        assert len(diagnostics) == 1
        event = diagnostics[0]
        assert event["outcome"] == "failed"
        assert event["error"] == "sdk_timeout:stop_move"
        calls = event["rpc_calls"]
        assert [(c["method"], c["outcome"]) for c in calls] == [
            ("stop_move", "timeout"),
        ]
        assert event["stop_policy"] == "stop_move_only"
        assert all(c["completed_ns"] >= c["started_ns"] for c in calls)
        assert (await adapter.get_state()).fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stopping_samples_are_preserved_in_failed_stop_diagnostics():
    adapter, client, _ = await _connected_control_adapter()
    events = []

    async def recorder(event, timestamp):
        events.append(event)

    adapter._event_callback = recorder
    client.robot_state = "STOPPING"
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        event = next(e for e in events if e.get("kind") == "stop_diagnostics")
        assert len(event["samples"]) > 1
        assert all(s["raw_robot_state"] == "STOPPING" for s in event["samples"])
        assert all(not s["stationary"] for s in event["samples"])
        assert "actual_q" in event["samples"][0]
        assert "actual_tcp" in event["samples"][0]
    finally:
        await adapter.disconnect()
