"""Stop admission ordering with the real adapter and an injected SDK only."""

import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_lebai_adapter_control import _target, _wait_until
from tests.robots.test_snapshot_contention import delayed_adapter, waiter_count


@pytest.mark.parametrize("readers", [1, 3])
async def test_stop_rpc_overtakes_queued_full_reads_without_skipping_them(readers):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        before = client.read_calls.count("get_kin_data")
        await adapter._sdk_lock.acquire()
        try:
            for _ in range(readers):
                tasks.append(asyncio.create_task(adapter.get_state()))
            await _wait_until(lambda: waiter_count(adapter) == readers)
            tasks.append(asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED)))
            await _wait_until(lambda: waiter_count(adapter) == readers + 1)
        finally:
            adapter._sdk_lock.release()
        await asyncio.wait_for(asyncio.gather(*tasks), 1)
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
        assert diagnostic["stop_move_lock"]["wait_ns"] == 0
        assert rpc["started_ns"] == 0
        assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
        assert diagnostic["outcome"] == "confirmed"
        assert client.read_calls.count("get_kin_data") - before == readers + len(diagnostic["samples"])
        assert client.write_calls == [("stop_move",)]


async def test_stop_waits_for_active_full_read_but_overtakes_other_queued_reads():
    async with delayed_adapter(block_first_read=True) as (
        adapter, client, clock, events, tasks, entered, release,
    ):
        tasks.append(asyncio.create_task(adapter.get_state()))
        await asyncio.wait_for(entered.wait(), 1)
        for _ in range(3):
            tasks.append(asyncio.create_task(adapter.get_state()))
        await _wait_until(lambda: waiter_count(adapter) == 3)
        tasks.append(asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED)))
        await _wait_until(lambda: waiter_count(adapter) == 4)
        assert client.write_calls == []
        release.set()
        await asyncio.wait_for(asyncio.gather(*tasks), 1)
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
        # The active read has already consumed 8 ms; its remaining 8 ms
        # still runs, whereas the three pending 16 ms reads cannot delay RPC.
        assert diagnostic["stop_move_lock"]["wait_ns"] == 8_000_000
        assert rpc["started_ns"] == 16_000_000
        assert client.write_calls == [("stop_move",)]


async def test_stop_invalidates_queued_motion_without_a_late_pvat():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        await adapter._sdk_lock.acquire()
        try:
            tasks.append(asyncio.create_task(adapter.get_state()))
            await _wait_until(lambda: waiter_count(adapter) == 1)
            admission = await adapter.submit_tcp(_target(), 7)
            await _wait_until(lambda: waiter_count(adapter) == 2)
            tasks.append(asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED)))
            await _wait_until(lambda: waiter_count(adapter) == 3)
        finally:
            adapter._sdk_lock.release()
        await asyncio.wait_for(asyncio.gather(*tasks), 1)
        await asyncio.wait_for(admission.completion.wait(), 1)
        assert admission.completion.invalidated
        assert client.ik_calls == []
        assert client.write_calls == [("stop_move",)]
        assert not any(e["kind"] == "pvat_sent" for e in events)


@pytest.mark.parametrize("sdk_ms", [16, 609])
async def test_stop_priority_keeps_rpc_deadline_and_late_reply_latch(sdk_ms):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        async def stop_rpc():
            client.write_calls.append(("stop_move",))
            clock.advance_ms(sdk_ms)

        client.stop_move = stop_rpc
        await adapter._sdk_lock.acquire()
        try:
            for _ in range(3):
                tasks.append(asyncio.create_task(adapter.get_state()))
            await _wait_until(lambda: waiter_count(adapter) == 3)
            stopping = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
            tasks.append(stopping)
            await _wait_until(lambda: waiter_count(adapter) == 4)
        finally:
            adapter._sdk_lock.release()
        if sdk_ms > 200:
            with pytest.raises(BackendCommandError, match="^sdk_timeout:stop_move$"):
                await asyncio.wait_for(stopping, 1)
            assert (await adapter.get_state()).fault == "stop_unverified"
            assert not (await adapter.preflight()).ready
            with pytest.raises(BackendCommandError):
                await adapter.submit_tcp(_target(), 100)
        else:
            await asyncio.wait_for(stopping, 1)
        await asyncio.gather(*tasks[:-1])
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
        rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        assert diagnostic["stop_move_lock"]["wait_ns"] == 0
        assert rpc["started_ns"] == 0
        assert rpc["deadline_ns"] == 200_000_000
        assert rpc["sdk_await_completed_ns"] - rpc["sdk_await_started_ns"] == sdk_ms * 1_000_000
        assert rpc["outcome"] == ("returned_late" if sdk_ms > 200 else "returned")
        assert client.write_calls == [("stop_move",)]


async def test_cancel_priority_stop_in_queue_keeps_owner_and_latches_unverified():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        await adapter._sdk_lock.acquire()
        reader = asyncio.create_task(adapter.get_state())
        tasks.append(reader)
        await _wait_until(lambda: waiter_count(adapter) == 1)
        stopping = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
        tasks.append(stopping)
        try:
            await _wait_until(lambda: waiter_count(adapter) == 2)
            stopping.cancel()
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(stopping, 1)
            assert adapter._sdk_lock.locked()
            assert not reader.done()
            assert client.write_calls == []
            assert adapter._latched_fault == "stop_unverified"
        finally:
            adapter._sdk_lock.release()
        await asyncio.wait_for(reader, 1)
        assert not (await adapter.preflight()).ready
        assert client.write_calls == []
