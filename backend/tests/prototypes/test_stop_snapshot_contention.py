"""Characterize current adapter locking, without SDK/network or policy changes."""
import asyncio

import pytest

from app.robots.base import StopReason
from tests.robots.test_lebai_adapter_control import _connected_control_adapter


class ObservedLock(asyncio.Lock):
    def __init__(self):
        super().__init__()
        self.contended = asyncio.Event()

    async def acquire(self):
        if self.locked():
            self.contended.set()
        return await super().acquire()


@pytest.mark.parametrize("cancel_reader", [False, True])
async def test_pending_stop_holds_snapshot_lock_and_reader_cancellation_is_isolated(cancel_reader):
    adapter, client, _ = await _connected_control_adapter()
    lock = ObservedLock()
    adapter._sdk_lock = lock
    entered = asyncio.Event()
    release = asyncio.Event()
    rpc_cancelled = asyncio.Event()
    original = client.stop_move

    async def pending_stop():
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            rpc_cancelled.set()
            raise
        await original()

    client.stop_move = pending_stop
    stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    reader = None
    try:
        await asyncio.wait_for(entered.wait(), 1)
        reader = asyncio.create_task(adapter._read_snapshot(emit_kinematics=False))
        await asyncio.wait_for(lock.contended.wait(), 1)
        # Event-based evidence: reader has attempted the real adapter lock,
        # not merely failed to run yet. No scheduling sleep is the assertion.
        assert not reader.done()
        assert not stop.done()
        if cancel_reader:
            reader.cancel()
            with pytest.raises(asyncio.CancelledError):
                await reader
            assert not rpc_cancelled.is_set()
            assert not stop.done()
        release.set()
        await asyncio.wait_for(stop, 1)
        if not cancel_reader:
            snapshot = await asyncio.wait_for(reader, 1)
            assert snapshot.raw_robot_state == "IDLE"
        assert not rpc_cancelled.is_set()
    finally:
        release.set()
        await asyncio.gather(stop, *([reader] if reader is not None else []), return_exceptions=True)
        await adapter.disconnect()
