"""Stop confirmation must resolve an ID, not confuse it with physical motion.

Round 26 supplied STOP / ID 111 / unchanged pose / zero speed. Its motion
state was NOT collected: FINISHED and non-finished are both tested hypotheses.
No test below contacts the SDK or treats that field run as a successful stop.
"""

import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_lebai_adapter_control import _connected_control_adapter


@pytest.mark.asyncio
@pytest.mark.parametrize("raw_state", ["STOP", 12, "PAUSED", 6, "IDLE", 5])
async def test_finished_id_confirms_only_after_window_and_preserves_raw_evidence(raw_state):
    adapter, client, clock = await _connected_control_adapter()
    client.robot_state = raw_state
    client.running_motion = 111
    queried_ids = []
    events = []

    async def motion_state(motion_id):
        queried_ids.append(motion_id)
        return "FINISHED"

    async def record(event, timestamp):
        events.append(event)

    client.get_motion_state = motion_state
    adapter._event_callback = record
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert clock.value >= 300_000_000
        assert client.write_calls == [("stop_move",)]
        assert queried_ids and set(queried_ids) == {111}
        samples = next(e for e in events if e["kind"] == "stop_diagnostics")["samples"]
        assert all(s["raw_running_motion"] == 111 for s in samples)
        assert all(s["motion_state"] == "FINISHED" for s in samples)
        assert all(s["running_motion"] is None for s in samples)
        observation = await adapter.read_stop_observation()
        assert observation["raw_running_motion"] == 111
        assert observation["motion_state"] == "FINISHED"
        assert observation["raw_robot_state"] == raw_state
        if raw_state in ("STOP", 12, "PAUSED", 6):
            preflight = await adapter.preflight()
            assert not preflight.ready
            assert preflight.reason == "robot_not_idle"
            with pytest.raises(BackendCommandError, match="preflight_not_ready"):
                await adapter.set_gripper(0.5)
            assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("motion_state", ["WAIT", "RUNNING", "UNKNOWN", None, 0, {}, True])
async def test_unfinished_or_unknown_motion_id_cannot_confirm_stop(motion_state):
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = 111
    client.motion_state = motion_state
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert "get_motion_state" in client.read_calls
        assert client.write_calls == [("stop_move",), ("stop_sys",)]
        assert (await adapter.read_stop_observation())["running_motion"] == 111
        assert (await adapter.get_state()).fault == "stop_incomplete"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("raw_state", ["STOPPING", 10, "MOVING", "TEACHING"])
async def test_finished_id_cannot_override_nonsettled_robot_state(raw_state):
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = raw_state
    client.running_motion = 111
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert "get_motion_state" not in client.read_calls
        assert client.write_calls == [("stop_move",), ("stop_sys",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("component", ["joint", "tcp", "speed"])
async def test_finished_id_cannot_hide_drift_or_nonzero_speed(component):
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = 111
    original = client.get_kin_data

    async def changing_pose():
        if component == "joint":
            client.kin_data["actual_joint_pose"][1] += 0.0002
        elif component == "tcp":
            client.kin_data["actual_tcp_pose"]["x"] -= 0.0001
        else:
            client.kin_data["actual_joint_speed"][1] = 0.03
        return await original()

    client.get_kin_data = changing_pose
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert "get_motion_state" in client.read_calls
        assert client.write_calls == [("stop_move",), ("stop_sys",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("failure", ["error", "timeout", "stale"])
async def test_motion_state_read_failure_never_confirms_stop(failure):
    adapter, client, clock = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = 111

    async def failed_read(motion_id):
        assert motion_id == 111
        if failure == "error":
            raise RuntimeError("read failed")
        if failure == "timeout":
            await asyncio.Event().wait()
        clock.advance_ms(301)
        return "FINISHED"

    client.get_motion_state = failed_read
    expected = {"error": "sdk_call_failed:get_motion_state",
                "timeout": "sdk_timeout:get_motion_state", "stale": "robot_state_stale"}
    try:
        with pytest.raises(BackendCommandError, match=f"^{expected[failure]}$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == [("stop_move",), ("stop_sys",)]
        assert adapter._latched_fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_successful_finished_id_stop_does_not_clear_existing_fault():
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = 111
    adapter._latched_fault = "stop_unverified"
    try:
        await adapter.stop(StopReason.SHUTDOWN)
        assert (await adapter.get_state()).fault == "stop_unverified"
        assert not (await adapter.preflight()).ready
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_finished_state_is_not_cached_across_reads_or_motion_ids():
    adapter, client, clock = await _connected_control_adapter()
    client.robot_state = "STOP"
    queried_ids = []

    async def running_id():
        return 111 if clock.value < 200_000_000 else 112

    async def motion_state(motion_id):
        queried_ids.append(motion_id)
        return "FINISHED" if motion_id == 111 else "RUNNING"

    client.get_running_motion = running_id
    client.get_motion_state = motion_state
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert set(queried_ids) == {111, 112}
        assert client.write_calls == [("stop_move",), ("stop_sys",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_wait_to_finished_starts_a_new_full_stable_window():
    adapter, client, clock = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = 111

    async def motion_state(motion_id):
        return "WAIT" if clock.value < 100_000_000 else "FINISHED"

    client.get_motion_state = motion_state
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert 400_000_000 <= clock.value < 500_000_000
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_finished_id_never_overrides_estop():
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = 111
    client.estop_reason = "HARD_ESTOP"
    try:
        with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == [("stop_move",), ("stop_sys",)]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("motion_id", [None, 0])
async def test_absent_motion_id_does_not_add_a_motion_state_rpc(motion_id):
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = "STOP"
    client.running_motion = motion_id
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        observation = await adapter.read_stop_observation()
        assert observation["raw_running_motion"] == motion_id
        assert observation["motion_state"] is None
        assert "get_motion_state" not in client.read_calls
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()
