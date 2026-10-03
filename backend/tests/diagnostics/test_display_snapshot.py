"""Offline prototype tests; the default state sender is not connected to this view."""

import asyncio

import pytest

from app.diagnostics.display_snapshot import DisplaySnapshotView
from app.robots.base import BackendCommandError
from app.schemas.messages import BackendState
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target


@pytest.fixture
async def prepared():
    adapter, client, clock = await _connected_control_adapter()
    view = DisplaySnapshotView(clock=clock.now_ns, max_local_age_ms=80)
    generation = view.begin_connection()
    state = await adapter.get_state()
    snapshot = adapter._snapshot
    try:
        yield adapter, client, clock, view, generation, state, snapshot
    finally:
        await adapter.disconnect()


def publish(view, generation, state, *, started=0, completed=0, purpose="state"):
    return view.publish(generation, state, read_started_ns=started,
                        read_completed_ns=completed, read_purpose=purpose)


async def test_repeated_display_reads_do_not_call_sdk(prepared):
    adapter, client, clock, view, generation, state, snapshot = prepared
    assert publish(view, generation, state)
    reads = list(client.read_calls)
    writes = list(client.write_calls)
    for _ in range(100):
        result = view.read(fault=None)
        assert result.status == "available"
        assert result.state == state
        assert result.local_read_age_ms == 0
        assert result.source_freshness == "unknown"
    assert client.read_calls == reads
    assert client.write_calls == writes
    assert adapter._snapshot is snapshot


@pytest.mark.parametrize("age_ms,status", [(79, "available"), (80, "stale"), (81, "stale")])
async def test_age_boundary_uses_original_read_start_without_refresh(prepared, age_ms, status):
    _, client, clock, view, generation, state, _ = prepared
    assert publish(view, generation, state)
    reads = list(client.read_calls)
    clock.advance_ms(age_ms)
    result = view.read(fault=None)
    assert result.status == status
    assert result.local_read_age_ms == age_ms
    assert result.state.server_mono_ns == state.server_mono_ns
    assert result.state.sample_age_ms == state.sample_age_ms
    assert view.read(fault=None).local_read_age_ms == age_ms
    assert client.read_calls == reads


async def test_missing_and_disconnected_do_not_start_fallback_reads(prepared):
    _, client, _, view, _, _, _ = prepared
    reads = list(client.read_calls)
    result = view.read(fault=None)
    assert result.status == "missing"
    assert result.state is None and result.local_read_age_ms is None
    view.disconnect()
    result = view.read(fault="stop_unverified")
    assert result.status == "disconnected"
    assert result.state is None and result.fault == "stop_unverified"
    assert client.read_calls == reads


async def test_fault_overlay_does_not_clear_or_mutate_cached_state(prepared):
    _, _, _, view, generation, state, _ = prepared
    assert publish(view, generation, state)
    for fault in ("stop_unverified", "robot_state_stale"):
        result = view.read(fault=fault)
        assert result.fault == fault
        assert result.state.fault == fault
        assert result.state.robot_state is BackendState.FAULT
    assert state.fault is None
    # Clearing a fault is a decision of the authoritative caller, not this cache.
    assert view.read(fault=None).state.fault is None


async def test_sample_fault_is_not_erased_by_absent_overlay(prepared):
    _, _, _, view, generation, state, _ = prepared
    state.fault = "stop_unverified"
    assert publish(view, generation, state)
    assert view.read(fault=None).fault == "stop_unverified"


async def test_publish_and_return_are_defensive_copies(prepared):
    _, _, _, view, generation, state, _ = prepared
    assert publish(view, generation, state)
    state.actual_tcp.p = (9, 9, 9)
    first = view.read(fault=None)
    assert first.state.actual_tcp.p != (9, 9, 9)
    first.state.actual_tcp.p = (8, 8, 8)
    assert view.read(fault=None).state.actual_tcp.p != (8, 8, 8)


async def test_reconnect_rejects_old_inflight_results(prepared):
    _, _, clock, view, generation, state, _ = prepared
    assert publish(view, generation, state)
    view.disconnect()
    assert not publish(view, generation, state)
    current = view.begin_connection()
    clock.advance_ms(10)
    assert current != generation
    assert not publish(view, generation, state, started=10_000_000, completed=10_000_000)
    assert view.read(fault=None).status == "missing"
    assert publish(view, current, state, started=10_000_000, completed=10_000_000)
    assert view.read(fault=None).generation == current


async def test_out_of_order_or_duplicate_completion_cannot_replace_newer_sample(prepared):
    _, _, clock, view, generation, state, _ = prepared
    clock.advance_ms(40)
    assert publish(view, generation, state, started=20_000_000, completed=30_000_000)
    for started, completed in [(0, 10_000_000), (20_000_000, 30_000_000)]:
        assert not publish(view, generation, state, started=started, completed=completed)
    result = view.read(fault=None)
    assert result.local_read_age_ms == 20
    assert result.local_completion_age_ms == 10


@pytest.mark.parametrize("started,completed", [(-1, 0), (1, 0), (0, 1)])
async def test_invalid_or_future_read_boundaries_are_rejected(prepared, started, completed):
    _, _, _, view, generation, state, _ = prepared
    with pytest.raises(ValueError, match="invalid_read_boundaries"):
        publish(view, generation, state, started=started, completed=completed)
    assert view.read(fault=None).status == "missing"


@pytest.mark.parametrize("threshold", [0, -1, float("nan"), float("inf")])
def test_requires_explicit_positive_finite_display_threshold(threshold):
    with pytest.raises(ValueError, match="invalid_display_age_limit"):
        DisplaySnapshotView(clock=lambda: 0, max_local_age_ms=threshold)


async def test_cancelled_display_consumer_does_not_cancel_producer(prepared):
    _, _, _, view, generation, state, _ = prepared
    consumer_entered = asyncio.Event()
    finish_producer = asyncio.Event()

    async def producer():
        await finish_producer.wait()
        return publish(view, generation, state)

    async def consumer():
        assert view.read(fault=None).status == "missing"
        consumer_entered.set()
        await asyncio.Event().wait()

    producing = asyncio.create_task(producer())
    consuming = asyncio.create_task(consumer())
    try:
        await asyncio.wait_for(consumer_entered.wait(), 1)
        consuming.cancel()
        with pytest.raises(asyncio.CancelledError):
            await consuming
        assert not producing.done()
        finish_producer.set()
        assert await asyncio.wait_for(producing, 1)
        assert view.read(fault=None).status == "available"
    finally:
        for task in (consuming, producing):
            if not task.done():
                task.cancel()
        await asyncio.gather(consuming, producing, return_exceptions=True)


async def test_display_view_does_not_wait_for_sdk_lock_or_alter_control_checks(prepared):
    adapter, client, clock, view, generation, state, _ = prepared
    assert publish(view, generation, state)
    reads = list(client.read_calls)
    await adapter._sdk_lock.acquire()
    try:
        clock.advance_ms(100)
        assert view.read(fault="stop_unverified").status == "stale"
        assert client.read_calls == reads
    finally:
        adapter._sdk_lock.release()
    await adapter.get_state()
    assert len(client.read_calls) > len(reads)


async def test_display_reads_do_not_renew_motion_snapshot_age(prepared):
    adapter, client, clock, view, generation, state, snapshot = prepared
    assert publish(view, generation, state)
    clock.advance_ms(341)
    for _ in range(100):
        assert view.read(fault=None).status == "stale"
    with pytest.raises(BackendCommandError, match="^robot_state_stale$"):
        await adapter.command_tcp(_target(), 1)
    assert adapter._snapshot is snapshot
    assert not client.write_calls


async def test_display_fault_overlay_cannot_unlock_backend(prepared):
    adapter, client, _, view, generation, state, _ = prepared
    assert publish(view, generation, state)
    adapter._latch_unverified_stop()
    assert view.read(fault=adapter._latched_fault).fault == "stop_unverified"
    # Even a caller omitting the current fault cannot change the backend latch.
    view.read(fault=None)
    assert adapter._latched_fault == "stop_unverified"
    assert not (await adapter.preflight()).ready
    assert not client.write_calls


async def test_clock_rollback_is_unavailable_not_a_fresh_negative_age(prepared):
    _, _, clock, view, generation, state, _ = prepared
    clock.advance_ms(10)
    assert publish(view, generation, state, started=5_000_000, completed=10_000_000)
    clock.value = 9_000_000
    result = view.read(fault="stop_unverified")
    assert result.status == "invalid_clock"
    assert result.state is None and result.local_read_age_ms is None
    assert result.fault == "stop_unverified"
