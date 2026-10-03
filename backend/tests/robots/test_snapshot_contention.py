"""Deterministic contention probes, using injected clients and clocks only.

No TTL cache, SDK concurrency or runtime policy changes are introduced here.
"""

import asyncio
from contextlib import asynccontextmanager
import json

import pytest

from app.robots.base import StopReason
from tests.robots.test_lebai_adapter_control import (
    _connected_control_adapter, _target, _wait_until,
)


@asynccontextmanager
async def delayed_adapter(*, block_first_read=False, later_read_ms=16):
    adapter, client, clock = await _connected_control_adapter()
    events = []
    tasks = []
    read_entered = asyncio.Event()
    release_read = asyncio.Event()
    originals = {name: getattr(client, name) for name in (
        "get_robot_state", "get_kin_data", "kinematics_inverse", "move_pvat",
    )}
    read_count = 0

    async def record(event, timestamp):
        events.append({**event, "server_mono_ns": timestamp})

    async def state():
        clock.advance_ms(8)
        return await originals["get_robot_state"]()

    async def kin():
        nonlocal read_count
        read_count += 1
        if read_count == 1 and block_first_read:
            read_entered.set()
            await release_read.wait()
        clock.advance_ms(8 if read_count == 1 else later_read_ms - 8)
        return await originals["get_kin_data"]()

    async def ik(*args):
        clock.advance_ms(8)
        return await originals["kinematics_inverse"](*args)

    async def send(*args):
        clock.advance_ms(8)
        return await originals["move_pvat"](*args)

    client.get_robot_state = state
    client.get_kin_data = kin
    client.kinematics_inverse = ik
    client.move_pvat = send
    adapter._event_callback = record
    try:
        yield adapter, client, clock, events, tasks, read_entered, release_read
    finally:
        release_read.set()
        if adapter._sdk_lock.locked():
            # Only cancel tasks here; never release a lock owned by a live read.
            for task in tasks:
                if not task.done():
                    task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await adapter.disconnect()
        assert client.network_calls == client.discovery_calls == 0


def waiter_count(adapter):
    # Test scheduling barrier only: inspect queued lock waiters before releasing
    # the fixture's lock. Avoid timing-dependent asyncio sleeps as ordering proof.
    return len(adapter._sdk_lock._waiters or ())


def pvat(events):
    return next(e for e in events if e["kind"] == "pvat_sent")


@pytest.mark.parametrize("readers", [0, 1, 3])
async def test_display_reads_before_motion_add_snapshot_lock_wait(readers):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        await adapter._sdk_lock.acquire()
        try:
            for _ in range(readers):
                tasks.append(asyncio.create_task(adapter.get_state()))
            await _wait_until(lambda: waiter_count(adapter) == readers)
            await adapter.command_tcp(_target(), 1)
            await _wait_until(lambda: waiter_count(adapter) == readers + 1)
        finally:
            adapter._sdk_lock.release()
        await _wait_until(lambda: any(e["kind"] == "pvat_sent" for e in events))
        await asyncio.gather(*tasks)
        timing = pvat(events)["timing_ms"]
        assert timing["snapshot_lock_wait"] == readers * 16
        assert timing["snapshot_read"] == (readers + 1) * 16
        assert timing["command_lock_wait"] == 0
        assert timing["handler_to_send_complete"] == 32 + readers * 16
        assert len([e for e in events if e["kind"] == "robot_kinematics"]) == readers + 1
        purposes = [e["snapshot_read_purpose"] for e in events if e["kind"] == "robot_kinematics"]
        assert purposes.count("state") == readers
        assert purposes.count("motion") == 1
        print("CONTENTION_RESULT " + json.dumps(dict(order="display_before_motion", readers=readers, timing_ms=timing)))


@pytest.mark.parametrize("readers", [0, 1, 3])
async def test_display_reads_between_motion_snapshot_and_ik_add_command_wait(readers):
    async with delayed_adapter(block_first_read=True) as (adapter, client, clock, events, tasks, entered, release):
        await adapter.command_tcp(_target(), 1)
        await asyncio.wait_for(entered.wait(), 1)
        for _ in range(readers):
            tasks.append(asyncio.create_task(adapter.get_state()))
        await _wait_until(lambda: waiter_count(adapter) == readers)
        release.set()
        await _wait_until(lambda: any(e["kind"] == "pvat_sent" for e in events))
        await asyncio.gather(*tasks)
        timing = pvat(events)["timing_ms"]
        assert timing["snapshot_lock_wait"] == 0
        assert timing["snapshot_read"] == 16
        assert timing["command_lock_wait"] == readers * 16
        assert timing["handler_to_send_complete"] == 32 + readers * 16
        assert timing["snapshot_age_at_send"] == 24 + readers * 16
        print("CONTENTION_RESULT " + json.dumps(dict(order="display_between_read_and_ik", readers=readers, timing_ms=timing)))


@pytest.mark.parametrize("readers", [0, 1, 3])
async def test_display_reads_queued_before_stop_delay_rpc_creation(readers):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        await adapter._sdk_lock.acquire()
        try:
            for _ in range(readers):
                tasks.append(asyncio.create_task(adapter.get_state()))
            await _wait_until(lambda: waiter_count(adapter) == readers)
            stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
            tasks.append(stop)
            await _wait_until(lambda: waiter_count(adapter) == readers + 1)
        finally:
            adapter._sdk_lock.release()
        await asyncio.wait_for(asyncio.gather(*tasks), 1)
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
        assert diagnostic["stop_move_lock"]["wait_ns"] == readers * 16_000_000
        assert rpc["started_ns"] == readers * 16_000_000
        assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
        assert rpc["outcome"] == "returned"
        assert client.write_calls == [("stop_move",)]
        assert diagnostic["samples"]
        assert all(s["read_purpose"] == "stop_verification" for s in diagnostic["samples"])
        print("CONTENTION_RESULT " + json.dumps(dict(order="display_before_stop", readers=readers, stop_lock_wait_ms=readers * 16)))


async def test_newer_display_snapshot_cannot_refresh_an_old_motion_snapshot():
    async with delayed_adapter(block_first_read=True, later_read_ms=120) as (adapter, client, clock, events, tasks, entered, release):
        await adapter.command_tcp(_target(), 1)
        await asyncio.wait_for(entered.wait(), 1)
        for _ in range(3):
            tasks.append(asyncio.create_task(adapter.get_state()))
        await _wait_until(lambda: waiter_count(adapter) == 3)
        release.set()
        await _wait_until(lambda: adapter.pump_fault is not None)
        await asyncio.gather(*tasks)
        assert str(adapter.pump_fault) == "robot_state_stale"
        assert adapter._snapshot.captured_ns == 256_000_000
        assert clock.now_ns() - adapter._snapshot.captured_ns < 340_000_000
        assert not client.write_calls
        assert not any(e["kind"] == "pvat_sent" for e in events)


async def test_purpose_tags_preserve_one_read_per_call_and_observation_semantics():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        assert adapter._snapshot.read_purpose == "connection"
        before = client.read_calls.count("get_kin_data")
        await adapter.preflight()
        assert adapter._snapshot.read_purpose == "preflight"
        observation = await adapter.read_stop_observation()
        assert observation["read_purpose"] == "stop_observation"
        snapshot = await adapter._read_snapshot()
        assert snapshot.read_purpose == "unspecified"
        assert client.read_calls.count("get_kin_data") - before == 3
        assert [e["snapshot_read_purpose"] for e in events if e["kind"] == "robot_kinematics"] == ["preflight", "unspecified"]
        assert not client.write_calls
