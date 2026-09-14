import asyncio

import pytest

from tests.prototypes.stop_request_owner import StopRequestOwner


class ControlledRequest:
    """Only a local coroutine. No SDK, sockets, config, or robot commands."""

    def __init__(self, error=None):
        self.started = asyncio.Event()
        self.release = asyncio.Event()
        self.calls = 0
        self.cancelled = False
        self.error = error

    async def send(self):
        self.calls += 1
        self.started.set()
        try:
            await self.release.wait()
        except asyncio.CancelledError:
            self.cancelled = True
            raise
        if self.error:
            raise self.error


async def test_timeout_then_shutdown_retains_one_request_and_records_late_reply():
    request = ControlledRequest()
    owner = StopRequestOwner(request.send)
    try:
        assert await owner.wait('grip', .01) == 'UNKNOWN'
        assert await owner.wait('shutdown', .01) == 'UNKNOWN'
        assert request.calls == 1
        assert not request.cancelled
        request.release.set()
        assert await owner.wait('disconnect', 1) == 'RETURNED_LATE'
        assert owner.fault_latched
        assert not owner.stop_confirmed
        assert not owner.motion_allowed
    finally:
        request.release.set()


async def test_cancelled_waiter_cannot_cancel_owned_request_or_trigger_resend():
    request = ControlledRequest()
    owner = StopRequestOwner(request.send)
    waiter = asyncio.create_task(owner.wait('grip', 1))
    try:
        await asyncio.wait_for(request.started.wait(), 1)
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter
        assert not request.cancelled
        request.release.set()
        assert await owner.wait('shutdown', 1) == 'RETURNED_LATE'
        assert request.calls == 1
        assert owner.fault_latched
    finally:
        request.release.set()
        if not waiter.done():
            waiter.cancel()
        await asyncio.gather(waiter, return_exceptions=True)


async def test_concurrent_waiters_share_result_but_do_not_confirm_physical_stop():
    request = ControlledRequest()
    request.release.set()
    owner = StopRequestOwner(request.send)
    results = await asyncio.gather(owner.wait('grip', 1), owner.wait('shutdown', 1))
    assert results == ['RETURNED', 'RETURNED']
    assert request.calls == 1
    assert not owner.stop_confirmed
    assert not owner.motion_allowed


async def test_late_exception_is_preserved_without_resending():
    request = ControlledRequest(RuntimeError('remote_failure'))
    owner = StopRequestOwner(request.send)
    try:
        assert await owner.wait('grip', .01) == 'UNKNOWN'
        request.release.set()
        assert await owner.wait('shutdown', 1) == 'FAILED'
        assert owner.error == 'RuntimeError: remote_failure'
        assert await owner.wait('disconnect', 1) == 'FAILED'
        assert request.calls == 1
        assert owner.fault_latched
    finally:
        request.release.set()


async def test_unresolved_request_remains_unknown_without_cleanup_retry():
    request = ControlledRequest()
    owner = StopRequestOwner(request.send)
    try:
        for reason in ['grip', 'shutdown', 'disconnect']:
            assert await owner.wait(reason, .01) == 'UNKNOWN'
        assert request.calls == 1
        assert not request.cancelled
        assert owner.fault_latched
        assert not owner.stop_confirmed
    finally:
        # Test-only cleanup: release the local coroutine, not a robot request.
        request.release.set()
        await owner.wait('test_teardown', 1)
