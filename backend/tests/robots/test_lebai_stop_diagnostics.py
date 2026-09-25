import asyncio
import time
from unittest.mock import AsyncMock

import pytest

from app.robots.lebai_stop_diagnostics import StopDiagnosticSampler
from tests.robots.fake_lebai import FakeLebaiClient
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.base import BackendCommandError, StopReason
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


async def wait_event(events, outcome):
    async def wait():
        while not any(e.get("outcome") == outcome for e in events):
            await asyncio.sleep(0)
    await asyncio.wait_for(wait(), .5)


async def test_sampler_records_raw_data_and_field_times_without_writes():
    client = FakeLebaiClient.idle()
    events = []
    sampler = StopDiagnosticSampler(client, 7, time.monotonic_ns, events.append)
    sampler.start()
    try:
        await wait_event(events, "sample")
    finally:
        await sampler.close()
    sample = next(e for e in events if e.get("outcome") == "sample")
    assert sample["episode_id"] == 7
    assert sample["raw_robot_state"] == "IDLE"
    assert sample["raw_kinematics"]["actual_joint_pose"] == client.kin_data["actual_joint_pose"]
    assert len(sample["reads"]) == 3
    assert all(r["completed_ns"] >= r["started_ns"] for r in sample["reads"])
    assert client.write_calls == []
    assert not sampler.unresolved


async def test_adapter_samples_before_stop_reply_without_updating_control_snapshot():
    client = FakeLebaiClient.idle()
    clock = FakeClock()
    events = []
    entered, release = asyncio.Event(), asyncio.Event()

    async def record(event, _timestamp):
        events.append(event)

    async def stop_move():
        entered.set()
        await release.wait()

    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, sleep=clock.sleep, event_callback=record, stop_diagnostic_sampling=True)
    await adapter.connect()
    snapshot = adapter._snapshot
    client.stop_move = stop_move
    stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        await asyncio.wait_for(entered.wait(), .5)
        # A short independent watchdog: sample must arrive before replying.
        await asyncio.wait_for(wait_event(events, "sample"), .1)
        assert not stop.done()
        assert adapter._snapshot is snapshot
        assert not adapter._preflight_ready
        release.set()
        await asyncio.wait_for(stop, 1)
    finally:
        release.set()
        await asyncio.gather(stop, return_exceptions=True)
        await adapter.disconnect()


async def test_blocked_read_times_out_once_and_preserves_no_sample():
    client = FakeLebaiClient.idle()
    calls = 0

    async def blocked():
        nonlocal calls
        calls += 1
        await asyncio.Event().wait()

    client.get_robot_state = blocked
    events = []
    sampler = StopDiagnosticSampler(client, 1, time.monotonic_ns, events.append, read_timeout=.01)
    sampler.start()
    try:
        await wait_event(events, "timeout")
    finally:
        await sampler.close()
    assert calls == 1
    assert not any(e.get("outcome") == "sample" for e in events)
    assert not sampler.unresolved
    assert client.write_calls == []


async def test_uncooperative_read_close_is_bounded_and_retains_unknown():
    client = FakeLebaiClient.idle()
    entered = asyncio.Event()
    release = asyncio.Event()

    async def blocked():
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        raise RuntimeError("late read error")

    client.get_robot_state = blocked
    events = []
    sampler = StopDiagnosticSampler(client, 1, time.monotonic_ns, events.append, cleanup_timeout=.01)
    sampler.start()
    try:
        await asyncio.wait_for(entered.wait(), .5)
        await asyncio.wait_for(sampler.close(), .5)
        assert sampler.unresolved
        assert any(e.get("unresolved_read") for e in events)
        assert not any(e.get("outcome") == "sample" for e in events)
    finally:
        release.set()
        if sampler.pending:
            await asyncio.gather(sampler.pending, return_exceptions=True)
        await sampler.close()


async def test_read_error_stops_sampling_without_device_writes():
    client = FakeLebaiClient.idle()

    async def failed():
        raise RuntimeError("read failed")

    client.get_robot_state = failed
    events = []
    sampler = StopDiagnosticSampler(client, 1, time.monotonic_ns, events.append)
    sampler.start()
    try:
        await wait_event(events, "error")
    finally:
        await sampler.close()
    assert next(e for e in events if e.get("outcome") == "error")["error_type"] == "RuntimeError"
    assert client.write_calls == []


async def test_diagnostic_timeout_does_not_change_stop_failure_or_resend():
    client = FakeLebaiClient.idle()
    events = []
    release = asyncio.Event()

    async def record(event, _timestamp):
        events.append(event)

    async def stop_move():
        client.write_calls.append(("stop_move",))
        await release.wait()

    async def stop_sys():
        client.write_calls.append(("stop_sys",))
        await release.wait()

    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
                              event_callback=record, stop_diagnostic_sampling=True)
    await adapter.connect()
    client.stop_move, client.stop_sys = stop_move, stop_sys

    async def blocked_read():
        await asyncio.Event().wait()

    client.get_robot_state = blocked_read
    try:
        with pytest.raises(BackendCommandError, match="sdk_timeout:stop_move"):
            await asyncio.wait_for(adapter.stop(StopReason.GRIP_RELEASED), 1)
        with pytest.raises(BackendCommandError, match="sdk_timeout:stop_move"):
            await asyncio.wait_for(adapter.stop(StopReason.SHUTDOWN), 1)
        assert [w[0] for w in client.write_calls] == ["stop_move"]
        assert adapter._latched_fault == "stop_unverified"
        assert not adapter._preflight_ready
        assert not adapter._stop_transaction.confirmed
        assert sum(e.get("outcome") == "timeout" and e["kind"] == "stop_diagnostic_read" for e in events) == 1
        assert sum(e["kind"] == "stop_diagnostic_reader_closed" for e in events) == 1
        for request in adapter._stop_transaction.requests.values():
            assert request.deadline_ns - request.started_ns == 200_000_000
    finally:
        release.set()
        await adapter.disconnect()


async def test_immediate_close_before_first_turn_is_final_and_idempotent():
    client = FakeLebaiClient.idle()
    events = []
    sampler = StopDiagnosticSampler(client, 1, time.monotonic_ns, events.append)
    sampler.start()
    await asyncio.gather(sampler.close(), sampler.close())
    sampler.start()
    assert not sampler.unresolved
    assert sampler.closed
    assert client.read_calls == []
    assert sum(e["kind"] == "stop_diagnostic_reader_closed" for e in events) == 1


async def test_timeout_cleanup_finishes_even_when_close_arrives_during_cleanup():
    client = FakeLebaiClient.idle()
    cancelled, release = asyncio.Event(), asyncio.Event()

    async def blocked():
        try:
            await release.wait()
        except asyncio.CancelledError:
            cancelled.set()
            await release.wait()
        return "IDLE"

    client.get_robot_state = blocked
    events = []
    sampler = StopDiagnosticSampler(client, 1, time.monotonic_ns, events.append,
                                    read_timeout=.01, cleanup_timeout=.02)
    sampler.start()
    try:
        await asyncio.wait_for(cancelled.wait(), .5)
        await sampler.close()
        assert sampler.closed
        assert sampler.unresolved
        assert events[-1]["unresolved_read"]
    finally:
        release.set()
        await asyncio.gather(sampler.pending, return_exceptions=True)
    # Late completion cannot launch get_kin_data after the reader is closed.
    assert "get_kin_data" not in client.read_calls


@pytest.mark.parametrize("options", [{"read_timeout": 0}, {"interval": -1},
                                    {"duration": float("nan")}, {"cleanup_timeout": float("inf")}])
def test_invalid_sampler_budgets_are_rejected(options):
    with pytest.raises(ValueError):
        StopDiagnosticSampler(FakeLebaiClient.idle(), 1, time.monotonic_ns, lambda _: None, **options)


async def test_recording_failure_does_not_escape_sampler_or_leak_task():
    client = FakeLebaiClient.idle()
    sampled = asyncio.Event()

    def broken_recorder(event):
        sampled.set()
        raise RuntimeError("disk unavailable")

    sampler = StopDiagnosticSampler(client, 1, time.monotonic_ns, broken_recorder)
    sampler.start()
    try:
        await asyncio.wait_for(sampled.wait(), .5)
    finally:
        await sampler.close()
    assert sampler.dropped_events > 0
    assert not sampler.unresolved
    assert client.write_calls == []


async def test_unresolved_diagnostic_read_prevents_reconnect_until_local_completion():
    client = FakeLebaiClient.idle()
    clock = FakeClock()
    entered, release_read, release_stop = asyncio.Event(), asyncio.Event(), asyncio.Event()

    async def record(_event, _timestamp):
        pass

    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, sleep=clock.sleep, event_callback=record, stop_diagnostic_sampling=True)
    await adapter.connect()
    original_read = client.get_robot_state
    calls = 0

    async def one_blocked_read():
        nonlocal calls
        calls += 1
        if calls == 1:
            entered.set()
            try:
                await release_read.wait()
            except asyncio.CancelledError:
                await release_read.wait()
        return await original_read()

    async def stop_move():
        await release_stop.wait()

    client.get_robot_state, client.stop_move = one_blocked_read, stop_move
    stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        await asyncio.wait_for(entered.wait(), .5)
        release_stop.set()
        await asyncio.wait_for(stop, 1)
        assert adapter._stop_diagnostic_sampler.unresolved
        await adapter.disconnect()
        with pytest.raises(BackendCommandError, match="stop_diagnostic_read_pending"):
            await adapter.connect()
        release_read.set()
        await asyncio.gather(adapter._stop_diagnostic_sampler.pending, return_exceptions=True)
        await adapter.connect()
        assert adapter._client is client
    finally:
        release_stop.set()
        release_read.set()
        await asyncio.gather(stop, return_exceptions=True)
        await adapter.disconnect()
