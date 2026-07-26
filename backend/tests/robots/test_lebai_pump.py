from __future__ import annotations

import asyncio

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_pump import PvatPump, PvatRequest
from app.schemas.messages import Pose


def _pose(x: float = 0.0) -> Pose:
    return Pose(p=(x, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))


async def _wait_until(predicate, timeout: float = 0.5) -> None:
    async def wait() -> None:
        while not predicate():
            await asyncio.sleep(0)

    await asyncio.wait_for(wait(), timeout)


@pytest.mark.asyncio
async def test_latest_request_replaces_unsent_request() -> None:
    handled: list[PvatRequest] = []

    async def handler(request: PvatRequest) -> None:
        handled.append(request)

    pump = PvatPump(handler, period_s=0.04)
    await pump.start()
    pump.submit(_pose(), 1)
    pump.submit(_pose(0.001), 2)
    await _wait_until(lambda: len(handled) == 1)
    await pump.stop()

    assert handled[0].command_id == 2


@pytest.mark.asyncio
async def test_invalidate_blocks_delayed_generation() -> None:
    started = asyncio.Event()
    release = asyncio.Event()
    sent: list[int] = []

    async def handler(request: PvatRequest) -> None:
        started.set()
        await release.wait()
        if pump.is_current(request.generation):
            sent.append(request.command_id)

    pump = PvatPump(handler, period_s=0.04)
    await pump.start()
    pump.submit(_pose(), 1)
    await started.wait()

    generation = pump.invalidate()
    release.set()
    await asyncio.sleep(0)
    await pump.stop()

    assert generation == 1
    assert sent == []


@pytest.mark.asyncio
async def test_handler_error_latches_stable_fault_and_stops_pump() -> None:
    async def fail(request: PvatRequest) -> None:
        raise RuntimeError("private details")

    pump = PvatPump(fail, period_s=0.001)
    await pump.start()
    pump.submit(_pose(), 1)
    await _wait_until(lambda: pump.fault is not None)

    assert isinstance(pump.fault, BackendCommandError)
    assert str(pump.fault) == "pvat_pump_failed"
    assert pump.running is False
    assert pump.has_pending is False
    await pump.stop()


@pytest.mark.asyncio
async def test_stop_clears_pending_and_rejects_later_submit() -> None:
    handled: list[int] = []

    async def handler(request: PvatRequest) -> None:
        handled.append(request.command_id)

    pump = PvatPump(handler, period_s=0.04)
    await pump.start()
    await pump.stop()

    with pytest.raises(BackendCommandError, match="^pvat_pump_not_running$"):
        pump.submit(_pose(), 1)
    assert pump.running is False
    assert pump.has_pending is False
    assert handled == []
