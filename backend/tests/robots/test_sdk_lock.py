"""Cancellation, fairness and mutual exclusion of the SDK admission lock."""

import asyncio

import pytest

from app.robots.lebai_sdk_lock import SdkLock
from tests.robots.test_lebai_adapter_control import _wait_until


async def _use(lock, order, name, stop=False):
    context = lock.stop_priority() if stop else lock
    async with context:
        order.append(name)
        await asyncio.sleep(0)


async def test_pending_stops_are_fifo_and_ordinary_operations_keep_their_order():
    lock = SdkLock()
    order = []
    tasks = []
    await lock.acquire()
    for index, (name, stop) in enumerate([
        ("read1", False), ("stop1", True), ("read2", False),
        ("stop2", True), ("read3", False),
    ], 1):
        tasks.append(asyncio.create_task(_use(lock, order, name, stop)))
        await _wait_until(lambda: len(lock._waiters) == index)
    assert order == []
    lock.release()
    await asyncio.wait_for(asyncio.gather(*tasks), 1)
    assert order == ["stop1", "stop2", "read1", "read2", "read3"]
    assert not lock.locked()
    assert not lock._waiters


async def test_new_stop_cannot_steal_a_granted_ordinary_operation():
    lock = SdkLock()
    order = []
    await lock.acquire()
    ordinary = asyncio.create_task(_use(lock, order, "read"))
    await _wait_until(lambda: len(lock._waiters) == 1)
    lock.release()
    # Start a stop in this task before the granted reader has resumed. The
    # reader already owns the reservation and cannot be revoked by priority.
    async with lock.stop_priority():
        order.append("stop")
    await ordinary
    assert order == ["read", "stop"]
    assert not lock.locked()


@pytest.mark.parametrize("stop", [False, True])
async def test_cancel_queued_waiter_does_not_release_the_active_owner(stop):
    lock = SdkLock()
    order = []
    await lock.acquire()
    cancelled = asyncio.create_task(_use(lock, order, "cancelled", stop))
    await _wait_until(lambda: len(lock._waiters) == 1)
    following = asyncio.create_task(_use(lock, order, "following"))
    await _wait_until(lambda: len(lock._waiters) == 2)
    cancelled.cancel()
    with pytest.raises(asyncio.CancelledError):
        await cancelled
    assert lock.locked()
    assert order == []
    lock.release()
    await asyncio.wait_for(following, 1)
    assert order == ["following"]
    assert not lock.locked()
    assert not lock._waiters


@pytest.mark.parametrize("stop", [False, True])
async def test_cancel_granted_waiter_transfers_reservation_to_next_waiter(stop):
    lock = SdkLock()
    order = []
    await lock.acquire()
    cancelled = asyncio.create_task(_use(lock, order, "cancelled", stop))
    await _wait_until(lambda: len(lock._waiters) == 1)
    following = asyncio.create_task(_use(lock, order, "following"))
    await _wait_until(lambda: len(lock._waiters) == 2)
    lock.release()
    cancelled.cancel()  # Grant occurred, but cancelled task has not resumed.
    with pytest.raises(asyncio.CancelledError):
        await cancelled
    await asyncio.wait_for(following, 1)
    assert order == ["following"]
    assert not lock.locked()
    assert not lock._waiters


@pytest.mark.parametrize("stop", [False, True])
async def test_cancel_active_context_releases_once_and_never_overlaps_owners(stop):
    lock = SdkLock()
    entered = asyncio.Event()
    release = asyncio.Event()
    active = 0
    maximum = 0

    async def holder():
        nonlocal active, maximum
        async with (lock.stop_priority() if stop else lock):
            active += 1
            maximum = max(maximum, active)
            entered.set()
            try:
                await release.wait()
            finally:
                active -= 1

    async def follower():
        nonlocal active, maximum
        async with lock:
            active += 1
            maximum = max(maximum, active)
            await asyncio.sleep(0)
            active -= 1

    held = asyncio.create_task(holder())
    await asyncio.wait_for(entered.wait(), 1)
    next_task = asyncio.create_task(follower())
    await _wait_until(lambda: len(lock._waiters) == 1)
    held.cancel()
    with pytest.raises(asyncio.CancelledError):
        await held
    await asyncio.wait_for(next_task, 1)
    assert active == 0
    assert maximum == 1
    assert not lock.locked()
    assert not lock._waiters


@pytest.mark.parametrize("stop", [False, True])
async def test_exception_inside_context_releases_lock_for_the_next_operation(stop):
    lock = SdkLock()
    with pytest.raises(ValueError):
        async with (lock.stop_priority() if stop else lock):
            raise ValueError("SDK failed")
    await asyncio.wait_for(lock.acquire(), 1)
    lock.release()
    with pytest.raises(RuntimeError):
        lock.release()


async def test_cancel_all_waiters_then_reuse_lock_after_owner_finishes():
    lock = SdkLock()
    order = []
    await lock.acquire()
    tasks = [asyncio.create_task(_use(lock, order, str(i), i % 2 == 0)) for i in range(20)]
    await _wait_until(lambda: len(lock._waiters) == 20)
    for task in tasks:
        task.cancel()
    # Cancelled futures may still be in the queue when the owner releases.
    lock.release()
    await asyncio.gather(*tasks, return_exceptions=True)
    await asyncio.wait_for(_use(lock, order, "new"), 1)
    assert order == ["new"]
    assert not lock.locked()
    assert not lock._waiters


async def test_multiple_stops_and_reads_share_one_exclusive_sdk_slot():
    lock = SdkLock()
    active = 0
    maximum = 0
    completed = []

    async def operation(index):
        nonlocal active, maximum
        async with (lock.stop_priority() if index % 3 == 0 else lock):
            active += 1
            maximum = max(maximum, active)
            await asyncio.sleep(0)
            completed.append(index)
            active -= 1

    await asyncio.gather(*(operation(i) for i in range(40)))
    assert maximum == 1
    assert active == 0
    assert sorted(completed) == list(range(40))
    assert not lock.locked()
