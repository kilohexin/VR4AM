"""Disconnect must withdraw authority before yielding to backend stop."""

import asyncio

import pytest

from app.schemas.messages import TeleopMode
from tests.control.test_target_lifecycle import setup


@pytest.mark.parametrize('lifecycle', [False, True])
async def test_disconnect_wait_blocks_tick_and_current_authority(lifecycle):
    control, hooks, backend, *_ = await setup()
    if not lifecycle:
        control.target_lifecycle = None
    started, release = asyncio.Event(), asyncio.Event()
    original = backend.stop
    async def blocked(reason):
        started.set()
        await release.wait()
        await original(reason)
    backend.stop = blocked
    pending = asyncio.create_task(control.on_disconnect())
    try:
        await started.wait()
        assert not control._target_context().authorized
        assert not control._control_generation_is_current(control.control_generation)
        count, collects = len(backend.targets), len(hooks.contexts)
        await control.tick()
        assert len(backend.targets) == count and len(hooks.contexts) == collects
    finally:
        release.set()
        await pending
    assert control.mode is TeleopMode.DISCONNECTED
    assert control._disconnecting == 0
    assert control._fault is None


@pytest.mark.parametrize('outcome', ['cancelled', 'error'])
async def test_disconnect_guard_clears_after_failed_stop_but_fault_keeps_authority_revoked(outcome):
    control, _, backend, *_ = await setup()
    started, release = asyncio.Event(), asyncio.Event()
    async def blocked(reason):
        started.set()
        await release.wait()
        raise RuntimeError('injected_stop_failure')
    backend.stop = blocked
    pending = asyncio.create_task(control.on_disconnect())
    await started.wait()
    if outcome == 'cancelled':
        pending.cancel()
    else:
        release.set()
    with pytest.raises(asyncio.CancelledError if outcome == 'cancelled' else RuntimeError):
        await pending
    assert control._disconnecting == 0
    assert control._fault == 'stop_unverified'
    assert not control._target_context().authorized
