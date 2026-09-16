import asyncio

import pytest

from tests.prototypes.diagnostic_sampler import DiagnosticSampler


async def test_parallel_read_records_samples_while_stop_is_pending():
    gate = asyncio.Event()
    stop = asyncio.create_task(gate.wait())

    async def read():
        return {"q": [0.] * 6}

    sampler = DiagnosticSampler(read)
    try:
        await sampler.run()
        assert [e["kind"] for e in sampler.events] == ["sample"] * 3 + ["closed"]
        assert sampler.events[0]["value"] == {"q": [0.] * 6}
        assert all(e["finished"] >= e["started"] for e in sampler.events[:3])
        assert not stop.done()
    finally:
        gate.set()
        await stop


async def test_serial_transport_timeout_does_not_retry_or_cancel_stop():
    lock = asyncio.Lock()
    gate = asyncio.Event()
    entered = asyncio.Event()
    calls = 0

    async def stop_request():
        async with lock:
            entered.set()
            await gate.wait()

    async def read():
        nonlocal calls
        calls += 1
        async with lock:
            return {"q": [0.] * 6}

    stop = asyncio.create_task(stop_request())
    await entered.wait()
    sampler = DiagnosticSampler(read)
    try:
        await asyncio.wait_for(sampler.run(), 1)
        assert [e["kind"] for e in sampler.events] == ["timeout", "closed"]
        assert calls == 1
        assert not stop.done()
        assert not sampler.events[-1]["unresolved"]
    finally:
        gate.set()
        await stop


async def test_read_error_is_preserved_and_not_treated_as_sample():
    async def read():
        raise RuntimeError("transport unavailable")

    sampler = DiagnosticSampler(read)
    await sampler.run()
    assert [e["kind"] for e in sampler.events] == ["error", "closed"]
    assert sampler.events[0]["error"] == "RuntimeError: transport unavailable"


async def test_cancelled_sampler_records_cancellation_and_drains_cooperative_read():
    entered = asyncio.Event()

    async def read():
        entered.set()
        await asyncio.Event().wait()

    sampler = DiagnosticSampler(read, timeout=1)
    runner = asyncio.create_task(sampler.run())
    await asyncio.wait_for(entered.wait(), 1)
    runner.cancel()
    with pytest.raises(asyncio.CancelledError):
        await runner
    assert [e["kind"] for e in sampler.events] == ["cancelled", "closed"]
    assert not sampler.events[-1]["unresolved"]
    assert sampler.pending.done()


async def test_cancellation_resistant_read_is_retained_and_never_retried():
    release = asyncio.Event()

    async def read():
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        raise RuntimeError("late failure")

    sampler = DiagnosticSampler(read)
    try:
        await asyncio.wait_for(sampler.run(), 1)
        assert [e["kind"] for e in sampler.events] == ["timeout", "closed"]
        assert sampler.events[-1]["unresolved"]
        assert not sampler.pending.done()
        with pytest.raises(RuntimeError, match="single use"):
            await sampler.run()
    finally:
        release.set()
        await asyncio.gather(sampler.pending, return_exceptions=True)
    assert not any(e["kind"] == "sample" for e in sampler.events)


@pytest.mark.parametrize("options", [{"samples": 0}, {"samples": 1.5},
    {"timeout": float("nan")}, {"timeout": -1}, {"cleanup": 0}, {"interval": -1}])
def test_invalid_limits_rejected_before_start(options):
    with pytest.raises(ValueError):
        DiagnosticSampler(None, **options)
