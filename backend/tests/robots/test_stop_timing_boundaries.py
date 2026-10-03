"""Local timing boundaries with injected clients; no real SDK or device access."""

import asyncio

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_lebai_adapter_control import (
    _connected_control_adapter, _frame, _target, _wait_until,
)


@pytest.mark.parametrize(("lock_ms", "sdk_ms"), [(15, 16), (31, 609), (250, 16)])
async def test_stop_lock_wait_and_sdk_await_have_separate_budgets(lock_ms, sdk_ms):
    adapter, client, clock = await _connected_control_adapter()
    events = []

    async def record(event, _timestamp):
        events.append(event)

    async def sdk_stop():
        client.write_calls.append(("stop_move",))
        clock.advance_ms(sdk_ms)

    adapter._event_callback = record
    client.stop_move = sdk_stop
    await adapter._sdk_lock.acquire()
    started_ns = clock.now_ns()
    task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        await _wait_until(lambda: adapter._stop_transaction is not None)
        await asyncio.sleep(0)
        clock.advance_ms(lock_ms)
        adapter._sdk_lock.release()
        if sdk_ms > 200:
            with pytest.raises(BackendCommandError, match="^sdk_timeout:stop_move$"):
                await asyncio.wait_for(task, 1)
            # Even an IDLE response after the late return must not clear the latch.
            assert (await adapter.get_state()).fault == "stop_unverified"
            assert not (await adapter.preflight()).ready
            with pytest.raises(BackendCommandError):
                await adapter.command_tcp(_target(), 99)
        else:
            await asyncio.wait_for(task, 1)
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        lifecycle = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
        assert diagnostic["stop_move_lock"]["wait_ns"] == lock_ms * 1_000_000
        assert lifecycle["started_ns"] == started_ns + lock_ms * 1_000_000
        assert lifecycle["sdk_await_started_ns"] == lifecycle["started_ns"]
        assert lifecycle["sdk_await_completed_ns"] - lifecycle["sdk_await_started_ns"] == sdk_ms * 1_000_000
        assert lifecycle["deadline_ns"] - lifecycle["started_ns"] == 200_000_000
        assert lifecycle["outcome"] == ("returned_late" if sdk_ms > 200 else "returned")
        assert lifecycle["wait_failed"] is (sdk_ms > 200)
        assert diagnostic["outcome"] == ("failed" if sdk_ms > 200 else "confirmed")
        assert client.write_calls == [("stop_move",)]
    finally:
        if adapter._sdk_lock.locked():
            adapter._sdk_lock.release()
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


async def test_snapshot_records_lock_read_and_logging_boundaries_separately():
    adapter, client, clock = await _connected_control_adapter()
    events = []
    original_read = client.get_kin_data

    async def delayed_read():
        clock.advance_ms(23)
        return await original_read()

    async def delayed_log(event, timestamp):
        events.append({**event, "server_mono_ns": timestamp})
        clock.advance_ms(11)

    client.get_kin_data = delayed_read
    adapter._event_callback = delayed_log
    await adapter._sdk_lock.acquire()
    requested_ns = clock.now_ns()
    read = asyncio.create_task(adapter.get_state())
    try:
        await asyncio.sleep(0)
        clock.advance_ms(7)
        adapter._sdk_lock.release()
        state = await asyncio.wait_for(read, 1)
        event = next(e for e in events if e["kind"] == "robot_kinematics")
        assert event["snapshot_read_requested_ns"] == requested_ns
        assert event["snapshot_read_started_ns"] == requested_ns + 7_000_000
        assert event["snapshot_read_completed_ns"] == requested_ns + 30_000_000
        assert event["server_mono_ns"] == event["snapshot_read_started_ns"]
        assert event["snapshot_lock_wait_ms"] == 7
        assert event["sdk_latencies_ms"]["get_kin_data"] == 23
        assert state.server_mono_ns == event["snapshot_read_started_ns"]
        assert clock.now_ns() - requested_ns == 41_000_000
        assert not client.write_calls
    finally:
        if adapter._sdk_lock.locked():
            adapter._sdk_lock.release()
        if not read.done():
            read.cancel()
        await asyncio.gather(read, return_exceptions=True)
        await adapter.disconnect()


async def test_identical_payloads_have_new_local_read_times_without_source_freshness_evidence():
    adapter, client, clock = await _connected_control_adapter()
    events = []

    async def record(event, timestamp):
        events.append({**event, "server_mono_ns": timestamp})

    adapter._event_callback = record
    try:
        first = await adapter.get_state()
        clock.advance_ms(100)
        second = await adapter.get_state()
        assert first.actual_tcp == second.actual_tcp
        assert first.actual_q == second.actual_q
        assert first.server_mono_ns != second.server_mono_ns
        samples = [e for e in events if e["kind"] == "robot_kinematics"]
        assert samples[1]["snapshot_read_completed_ns"] - samples[0]["snapshot_read_completed_ns"] == 100_000_000
        # The fixture deliberately supplies no device clock or source sequence.
        assert "device_timestamp" not in client.kin_data
        assert "source_sequence" not in client.kin_data
        assert not client.write_calls
    finally:
        await adapter.disconnect()


async def test_public_sample_age_is_vr_input_age_even_after_a_new_robot_read():
    adapter, client, clock = await _connected_control_adapter()
    latest = LatestVRFrame()
    control = RobotControl(
        backend=adapter, latest=latest, clock=clock, recorder=NoopRecorder(),
    )
    events = []
    original_read = client.get_kin_data

    async def delayed_read():
        clock.advance_ms(23)
        return await original_read()

    async def record(event, timestamp):
        events.append({**event, "server_mono_ns": timestamp})

    client.get_kin_data = delayed_read
    adapter._event_callback = record
    try:
        received_ns = clock.now_ns()
        latest.publish(_frame(1, grip=False), received_ns)
        clock.advance_ms(1000)
        state = await control.state_message()
        assert state.sample_age_ms == 1023
        sample = next(e for e in events if e["kind"] == "robot_kinematics")
        assert state.server_mono_ns - sample["snapshot_read_started_ns"] == 23_000_000
        assert state.actual_tcp.model_dump() == sample["actual_tcp"]
        assert not client.write_calls
    finally:
        await adapter.disconnect()


async def test_stop_verification_and_observation_preserve_local_read_boundaries():
    adapter, client, clock = await _connected_control_adapter()
    events = []
    original_read = client.get_kin_data

    async def delayed_read():
        clock.advance_ms(25)
        return await original_read()

    async def record(event, _timestamp):
        events.append(event)

    client.get_kin_data = delayed_read
    adapter._event_callback = record
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        observation = await adapter.read_stop_observation()
        assert diagnostic["samples"]
        for sample in [*diagnostic["samples"], observation]:
            assert sample["read_requested_ns"] <= sample["captured_ns"]
            assert sample["read_completed_ns"] - sample["captured_ns"] == 25_000_000
            assert sample["read_completed_ns"] <= sample["observed_ns"]
        assert client.write_calls == [("stop_move",)]
    finally:
        await adapter.disconnect()
