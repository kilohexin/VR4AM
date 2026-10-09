import asyncio
import math

import pytest

from app.robots.lebai_observer_window import ObserverWindow


@pytest.mark.parametrize('value', [-1, .021, math.inf, math.nan, True])
def test_observer_window_rejects_invalid_budget(value):
    with pytest.raises(ValueError):
        ObserverWindow(value)


@pytest.mark.asyncio
async def test_disabled_window_never_blocks():
    window = ObserverWindow(0)
    assert window.begin() is None
    await window.wait()


@pytest.mark.asyncio
async def test_wait_finishes_when_motion_finishes():
    window = ObserverWindow(.02)
    lease = window.begin()
    waiting = asyncio.create_task(window.wait())
    await asyncio.sleep(0)
    assert not waiting.done()
    window.finish(lease)
    await asyncio.wait_for(waiting, 1)


@pytest.mark.asyncio
async def test_wait_uses_one_bounded_timeout_without_cancelling_motion(monkeypatch):
    window = ObserverWindow(.02)
    lease = window.begin()
    calls = []

    async def timeout(awaitable, timeout):
        calls.append(timeout)
        awaitable.close()
        raise TimeoutError

    monkeypatch.setattr('app.robots.lebai_observer_window.asyncio.wait_for', timeout)
    await window.wait()
    assert calls == [.02]
    assert window.active
    window.finish(lease)
    assert not window.active


@pytest.mark.asyncio
async def test_cancelling_observer_does_not_finish_motion():
    window = ObserverWindow(.02)
    lease = window.begin()
    waiting = asyncio.create_task(window.wait())
    await asyncio.sleep(0)
    waiting.cancel()
    with pytest.raises(asyncio.CancelledError):
        await waiting
    assert window.active
    window.finish(lease)


@pytest.mark.asyncio
async def test_old_motion_cannot_finish_new_window_after_stop_release():
    window = ObserverWindow(.02)
    old = window.begin()
    waiting = asyncio.create_task(window.wait())
    await asyncio.sleep(0)
    released_event = window._idle
    window.release_all()
    new = window.begin()
    assert released_event.is_set(), 'new motion must not reclose the stop release'
    await asyncio.wait_for(waiting, 1)
    window.finish(old)
    assert window.active
    window.finish(new)
    assert not window.active


@pytest.mark.asyncio
async def test_overlapping_motion_leases_are_independent():
    window = ObserverWindow(.02)
    a, b = window.begin(), window.begin()
    window.finish(a)
    window.finish(a)
    assert window.active
    window.finish(b)
    assert not window.active
