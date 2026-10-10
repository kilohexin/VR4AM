"""Gripper dispatch must not bypass newer robot evidence or old-read expiry."""

import asyncio
from contextlib import asynccontextmanager
from dataclasses import replace

import pytest

from app.robots.base import BackendCommandError
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _wait_until


@asynccontextmanager
async def paused_gripper():
    """Pause after a real full read; retain its evidence and SDK side effects."""
    adapter, client, clock = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get("read_purpose") == "gripper":
            entered.set()
            await release.wait()
        return snapshot

    adapter._read_snapshot = read
    task = asyncio.create_task(adapter.set_gripper(0.5))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        yield adapter, client, clock, task, release
    finally:
        release.set()
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()
        assert client.network_calls == client.discovery_calls == 0


async def assert_rejected(adapter, client, task, release, code):
    release.set()
    result = (await asyncio.gather(task, return_exceptions=True))[0]
    assert client.write_calls == []
    assert isinstance(result, BackendCommandError)
    assert str(result) == code
    assert not adapter._preflight_ready


@pytest.mark.parametrize("fault", ["state", "estop", "tcp"])
@pytest.mark.parametrize("expired", [False, True])
async def test_observed_fault_then_health_cannot_authorize_queued_claw(fault, expired):
    adapter, client, clock = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get("read_purpose") == "gripper":
            entered.set()
            await release.wait()
        return snapshot

    adapter._read_snapshot = read
    task = asyncio.create_task(adapter.set_gripper(0.5))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        tcp = dict(client.tcp)
        if fault == "state":
            client.robot_state = "ERROR"
        elif fault == "estop":
            client.estop_reason = "hardestop"
        else:
            client.tcp["z"] = 0.3
        await adapter.get_state()
        client.robot_state, client.estop_reason, client.tcp = "IDLE", 0, tcp
        await adapter.get_state()
        if expired:
            clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000 + 1)
        release.set()
        result = (await asyncio.gather(task, return_exceptions=True))[0]
        assert client.write_calls == []
        assert isinstance(result, BackendCommandError)
        assert str(result) == "preflight_not_ready:" + {
            "state": "robot_state_fault", "estop": "estop:hard_estop", "tcp": "tcp_mismatch",
        }[fault]
        assert not adapter._preflight_ready
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()


@pytest.mark.parametrize("failure", ["sdk_error", "cancelled", "invalid_kin"])
async def test_read_failure_then_health_cannot_authorize_claw(failure):
    async with paused_gripper() as (adapter, client, _, task, release):
        entered = asyncio.Event()
        original_tcp = client.get_tcp
        original_kin = client.kin_data

        async def broken_tcp():
            entered.set()
            if failure == "cancelled":
                await asyncio.Event().wait()
            raise RuntimeError("injected_read_error")

        if failure == "invalid_kin":
            client.kin_data = {**original_kin, "actual_joint_pose": []}
        else:
            client.get_tcp = broken_tcp
        observer = asyncio.create_task(adapter.get_state())
        try:
            if failure == "cancelled":
                await asyncio.wait_for(entered.wait(), 1)
                observer.cancel()
            result = (await asyncio.gather(observer, return_exceptions=True))[0]
            assert isinstance(result, BaseException)
        finally:
            observer.cancel()
            await asyncio.gather(observer, return_exceptions=True)
            client.get_tcp, client.kin_data = original_tcp, original_kin
        await adapter.get_state()
        source = {
            "sdk_error": "sdk_call_failed:get_tcp",
            "cancelled": "robot_state_stale",
            "invalid_kin": "invalid_sdk_joint_vector:actual_joint_pose",
        }[failure]
        await assert_rejected(adapter, client, task, release, f"preflight_not_ready:{source}")


@pytest.mark.parametrize("fault", ["joints", "workspace", "capabilities"])
async def test_latest_complete_read_gets_full_gripper_boundary_check(fault):
    async with paused_gripper() as (adapter, client, _, task, release):
        if fault == "joints":
            client.kin_data["actual_joint_pose"] = [4.0, *client.kin_data["actual_joint_pose"][1:]]
        elif fault == "workspace":
            client.kin_data["actual_tcp_pose"] = {**client.kin_data["actual_tcp_pose"], "x": 0.9}
        else:
            adapter._capabilities = replace(adapter._capabilities, names=())
        await adapter.get_state()
        source = {"joints": "joint_outside_soft_limits", "workspace": "tcp_outside_startup_envelope",
                  "capabilities": "sdk_capability_missing"}[fault]
        await assert_rejected(adapter, client, task, release, f"preflight_not_ready:{source}")


@pytest.mark.parametrize("advance_ms", [0, 1])
async def test_new_complete_healthy_read_preserves_valid_claw(advance_ms):
    async with paused_gripper() as (adapter, client, clock, task, release):
        clock.advance_ms(advance_ms)
        await adapter.get_state()
        release.set()
        await asyncio.wait_for(task, 1)
        assert client.write_calls == [("set_claw", 30, 50)]
        assert adapter._preflight_ready


@pytest.mark.parametrize("change", ["client", "generation", "incomplete", "wrong_client", "wrong_generation"])
async def test_gripper_dispatch_rejects_invalid_authority_or_read_provenance(change):
    async with paused_gripper() as (adapter, client, _, task, release):
        read = adapter._latest_snapshot_read
        new_client = FakeLebaiClient.idle()
        if change == "client":
            adapter._client = new_client
        elif change == "generation":
            adapter._pump.invalidate()
        elif change == "incomplete":
            adapter._latest_snapshot_read = replace(read, snapshot=None)
        elif change == "wrong_client":
            adapter._latest_snapshot_read = replace(read, client=new_client)
        else:
            adapter._latest_snapshot_read = replace(read, generation=read.generation - 1)
        try:
            code = {"client": "robot_disconnected", "generation": "preflight_not_ready"}.get(
                change, "preflight_not_ready:robot_state_stale")
            await assert_rejected(adapter, client, task, release, code)
            assert new_client.write_calls == []
        finally:
            adapter._client = client


async def test_gripper_original_read_can_expire_while_waiting_for_sdk_lock():
    async with paused_gripper() as (adapter, client, clock, task, release):
        await adapter._sdk_lock.acquire()
        try:
            release.set()
            # Wait for the actual final SDK lock acquisition to queue.
            for _ in range(100):
                if adapter._sdk_lock._waiters:
                    break
                await asyncio.sleep(0)
            assert adapter._sdk_lock._waiters
            clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000 + 1)
        finally:
            adapter._sdk_lock.release()
        await assert_rejected(adapter, client, task, release, "robot_state_stale")


async def test_cancelled_gripper_waiter_never_writes_and_releases_queue():
    async with paused_gripper() as (adapter, client, _, task, release):
        await adapter._sdk_lock.acquire()
        try:
            release.set()
            for _ in range(100):
                if adapter._sdk_lock._waiters:
                    break
                await asyncio.sleep(0)
            assert adapter._sdk_lock._waiters
            task.cancel()
            result = (await asyncio.gather(task, return_exceptions=True))[0]
            assert isinstance(result, asyncio.CancelledError)
            assert not adapter._sdk_lock._waiters
            assert client.write_calls == []
        finally:
            adapter._sdk_lock.release()
        # Ordinary observation still acquires the lock after cancellation.
        await asyncio.wait_for(adapter.get_state(), 1)


async def test_valid_claw_adds_no_dispatch_read_or_other_write():
    adapter, client, _ = await _connected_control_adapter()
    client.read_calls.clear()
    try:
        await adapter.set_gripper(0.5)
        assert client.read_calls == ["is_connected", "get_robot_state", "get_estop_reason",
                                     "get_kin_data", "get_tcp", "get_claw", "get_running_motion"]
        assert client.write_calls == [("set_claw", 30, 50)]
        assert client.network_calls == client.discovery_calls == 0
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize("runtime_fault", [False, True])
async def test_failed_snapshot_recording_cannot_be_erased_or_hide_runtime_fault(runtime_fault):
    async with paused_gripper() as (adapter, client, _, task, release):
        async def broken_record(event, _timestamp):
            if event.get("kind") == "robot_kinematics":
                raise RuntimeError("injected_record_error")

        adapter._event_callback = broken_record
        if runtime_fault:
            client.robot_state = "ERROR"
        try:
            with pytest.raises(BackendCommandError, match="^recording_unavailable$"):
                await adapter.get_state()
        finally:
            adapter._event_callback = None
            client.robot_state = "IDLE"
        await adapter.get_state()
        source = "robot_state_fault" if runtime_fault else "recording_unavailable"
        await assert_rejected(adapter, client, task, release, f"preflight_not_ready:{source}")


async def test_real_sdk_fifo_observation_before_queued_claw_blocks_write():
    async with paused_gripper() as (adapter, client, _, task, release):
        observer = None
        await adapter._sdk_lock.acquire()
        try:
            client.robot_state = "ERROR"
            observer = asyncio.create_task(adapter.get_state())
            await _wait_until(lambda: len(adapter._sdk_lock._waiters) == 1)
            release.set()
            await _wait_until(lambda: len(adapter._sdk_lock._waiters) == 2)
        finally:
            adapter._sdk_lock.release()
        try:
            await asyncio.wait_for(observer, 1)
            await assert_rejected(adapter, client, task, release, "preflight_not_ready:robot_state_fault")
        finally:
            client.robot_state = "IDLE"
            if observer is not None:
                observer.cancel()
                await asyncio.gather(observer, return_exceptions=True)


async def test_read_started_before_gripper_cannot_lose_its_failure_to_read_floor():
    adapter, client, _ = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()
    original = client.get_tcp
    gripper = None

    async def broken_tcp_once():
        entered.set()
        await release.wait()
        client.get_tcp = original
        raise RuntimeError("injected_earlier_read_error")

    client.get_tcp = broken_tcp_once
    observer = asyncio.create_task(adapter.get_state())
    try:
        await asyncio.wait_for(entered.wait(), 1)
        earlier_sequence = adapter._snapshot_read_sequence
        gripper = asyncio.create_task(adapter.set_gripper(0.5))
        await _wait_until(lambda: len(adapter._sdk_lock._waiters) == 1)
        release.set()
        result = (await asyncio.gather(observer, return_exceptions=True))[0]
        assert isinstance(result, BackendCommandError)
        assert str(result) == "sdk_call_failed:get_tcp"
        result = (await asyncio.gather(gripper, return_exceptions=True))[0]
        assert isinstance(result, BackendCommandError)
        assert str(result) == "preflight_not_ready:sdk_call_failed:get_tcp"
        assert adapter._latest_snapshot_read.sequence > earlier_sequence
        assert adapter._latest_snapshot_read.snapshot is not None
        assert not adapter._preflight_ready
        assert client.write_calls == []
    finally:
        release.set()
        observer.cancel()
        if gripper is not None:
            gripper.cancel()
            await asyncio.gather(gripper, return_exceptions=True)
        await asyncio.gather(observer, return_exceptions=True)
        client.get_tcp = original
        await adapter.disconnect()


async def test_new_healthy_read_cannot_extend_expired_gripper_request():
    adapter, client, clock = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get("read_purpose") == "gripper":
            entered.set()
            await release.wait()
        return snapshot

    adapter._read_snapshot = read
    task = asyncio.create_task(adapter.set_gripper(0.5))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000 + 1)
        await adapter.get_state()
        release.set()
        result = (await asyncio.gather(task, return_exceptions=True))[0]
        assert client.write_calls == []
        assert isinstance(result, BackendCommandError)
        assert str(result) == "robot_state_stale"
        assert not adapter._preflight_ready
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()
