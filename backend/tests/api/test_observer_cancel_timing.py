"""Cancellation-to-stop boundaries with fake time and explicitly gated SDK calls."""

import asyncio

import pytest
from fastapi import WebSocketDisconnect

from app.robots.base import BackendCommandError
from app.schemas.messages import TeleopMode
from tests.api.test_observer_session_lifecycle import session_probe
from tests.robots.test_lebai_adapter_control import _wait_until
from tests.robots.test_snapshot_contention import waiter_count


def capture_boundaries(probe, adapter):
    clock = probe.observer.clock
    times, events = {}, []
    original_run = probe.observer.run
    original_close = probe.observer.request_close
    original_disconnect = probe.control.on_disconnect
    original_event = adapter._event_callback

    async def run():
        try:
            await original_run()
        finally:
            times["observer_exit"] = clock.now_ns()

    def close():
        times.setdefault("cancel_requested", clock.now_ns())
        original_close()

    async def disconnect():
        times["disconnect_started"] = clock.now_ns()
        await original_disconnect()

    async def event(payload, timestamp):
        events.append(payload)
        await original_event(payload, timestamp)

    probe.observer.run = run
    probe.observer.request_close = close
    probe.control.on_disconnect = disconnect
    adapter._event_callback = event
    probe.release_cleanup.set()  # No artificial cleanup pause in timing cases.
    return times, events


class CancelGate:
    """One fake operation; no real timer or device cancellation assumption."""

    def __init__(self, clock, *, behavior, delay_ms):
        self.clock = clock
        self.behavior = behavior
        self.delay_ms = delay_ms
        self.entered = asyncio.Event()
        self.cancelled = asyncio.Event()
        self.release = asyncio.Event()
        self.used = False

    async def wait_once(self):
        if self.used:
            return
        self.used = True
        self.entered.set()
        try:
            await self.release.wait()
        except asyncio.CancelledError:
            self.cancelled.set()
            if self.behavior == "cooperative":
                raise
            await self.release.wait()
            self.clock.advance_ms(self.delay_ms)
            if self.behavior == "delayed_raise":
                raise


def install_gate(probe, client, phase, gate):
    if phase == "record":
        probe.observer.recorder.on_write = gate.wait_once
    else:
        method = "get_robot_state" if phase == "state" else "get_kin_data"
        original = getattr(client, method)

        async def blocked():
            await gate.wait_once()
            return await original()

        setattr(client, method, blocked)


async def stop_events(events):
    await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
    rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
    diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
    return rpc, diagnostic


CASES = [
    ("cooperative", 0),
    ("delayed_raise", 75), ("delayed_raise", 350),
    ("delayed_return", 75), ("delayed_return", 350),
]


@pytest.mark.parametrize("phase", ["state", "kinematics", "record"])
@pytest.mark.parametrize("behavior,delay_ms", CASES)
async def test_cancel_join_delay_precedes_stop_rpc_deadline(monkeypatch, phase, behavior, delay_ms):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        times, events = capture_boundaries(probe, adapter)
        previous_latest = probe.state.latest
        await probe.start()
        clock = probe.observer.clock
        gate = CancelGate(clock, behavior=behavior, delay_ms=delay_ms)
        install_gate(probe, client, phase, gate)
        probe.observer.gate.tokens.put_nowait(None)
        try:
            await asyncio.wait_for(gate.entered.wait(), 1)
            probe.socket.incoming.put_nowait(WebSocketDisconnect())
            await asyncio.wait_for(gate.cancelled.wait(), 1)
            if behavior != "cooperative":
                assert not probe.cleanup_entered.is_set()
                assert not probe.joined.is_set()
                assert not any(e["kind"] == "stop_rpc_lifecycle" for e in events)
                assert client.write_calls == []
                await probe.assert_competing_connection_rejected()
                gate.release.set()

            stale_return = behavior == "delayed_return" and delay_ms == 350 and phase != "record"
            # Receiver-close already cancelled the sender group. Its primary
            # cancellation wins over a later observer error; gather still owns
            # that error, which remains inspectable on the finished child.
            await asyncio.wait_for(probe.session, 1)
            if stale_return:
                error = probe.children[0].exception()
                assert isinstance(error, BackendCommandError)
                assert str(error) == "robot_state_stale"
            rpc, diagnostic = await stop_events(events)

            # A returned state call also finishes the remaining sequential SDK
            # reads. A stale state return is rejected before the kinematics call.
            remainder_ms = 0
            if behavior == "delayed_return":
                if phase == "state":
                    remainder_ms = 8 if stale_return else 16
                elif phase == "kinematics":
                    remainder_ms = 8
            expected_join_ms = delay_ms + remainder_ms
            assert (times["observer_exit"] - times["cancel_requested"]) / 1e6 == expected_join_ms
            assert times["disconnect_started"] == times["observer_exit"]
            assert rpc["started_ns"] == times["disconnect_started"]
            assert diagnostic["stop_move_lock"]["wait_ns"] == 0
            assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
            assert rpc["outcome"] == "returned"
            assert not rpc["wait_failed"]
            assert client.write_calls == [("stop_move",)]
            assert probe.control._fault is None
            assert probe.control.mode is TeleopMode.DISCONNECTED
            expected_records = 2 if phase == "record" or (
                behavior == "delayed_return" and not stale_return
            ) else 1
            assert len(probe.observer.recorder.rows) == expected_records
            probe.assert_finished(previous_latest)
        finally:
            gate.release.set()


@pytest.mark.parametrize("phase", ["state", "kinematics"])
async def test_stale_return_is_primary_when_observer_finishes_before_session_cancellation(monkeypatch, phase):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        times, events = capture_boundaries(probe, adapter)
        await probe.start()
        gate = CancelGate(probe.observer.clock, behavior="delayed_return", delay_ms=350)
        install_gate(probe, client, phase, gate)
        probe.observer.gate.tokens.put_nowait(None)
        try:
            await asyncio.wait_for(gate.entered.wait(), 1)
            # Explicit local observer cancellation: the outer receiver is still
            # alive, so it has not already made group cancellation primary.
            probe.observer.request_close()
            await asyncio.wait_for(gate.cancelled.wait(), 1)
            gate.release.set()
            with pytest.raises(BackendCommandError, match="robot_state_stale"):
                await asyncio.wait_for(probe.session, 1)
            rpc, _ = await stop_events(events)
            assert rpc["started_ns"] - times["cancel_requested"] == 358_000_000
            assert probe.state.teleop_owner is None
            assert probe.control._fault is None
            assert client.write_calls == [("stop_move",)]
        finally:
            gate.release.set()


@pytest.mark.parametrize("lock_ms", [75, 350])
async def test_cancelled_lock_waiter_exits_but_stop_waits_for_the_actual_lock_owner(monkeypatch, lock_ms):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        times, events = capture_boundaries(probe, adapter)
        await probe.start()
        clock = probe.observer.clock
        await adapter._sdk_lock.acquire()
        held_by_test = True
        try:
            probe.observer.gate.tokens.put_nowait(None)
            await _wait_until(lambda: waiter_count(adapter) == 1)
            before = list(client.read_calls)
            probe.socket.incoming.put_nowait(WebSocketDisconnect())
            await asyncio.wait_for(probe.cleanup_entered.wait(), 1)
            await _wait_until(lambda: waiter_count(adapter) == 1)
            assert times["observer_exit"] == times["cancel_requested"]
            assert times["disconnect_started"] == times["cancel_requested"]
            assert client.read_calls == before
            assert client.write_calls == []
            await probe.assert_competing_connection_rejected()
            clock.advance_ms(lock_ms)
            adapter._sdk_lock.release()
            held_by_test = False
            await asyncio.wait_for(probe.session, 1)
            rpc, diagnostic = await stop_events(events)
            assert diagnostic["stop_move_lock"]["wait_ns"] == lock_ms * 1_000_000
            assert rpc["started_ns"] - times["cancel_requested"] == lock_ms * 1_000_000
            assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
            assert rpc["outcome"] == "returned"
            assert probe.control._fault is None
        finally:
            if held_by_test:
                adapter._sdk_lock.release()


@pytest.mark.parametrize("sdk_ms", [16, 609])
async def test_join_wait_and_stop_sdk_lateness_are_distinct_fault_boundaries(monkeypatch, sdk_ms):
    async with session_probe(monkeypatch) as (probe, adapter, client):
        times, events = capture_boundaries(probe, adapter)
        await probe.start()
        clock = probe.observer.clock
        gate = CancelGate(clock, behavior="delayed_raise", delay_ms=75)
        install_gate(probe, client, "record", gate)

        async def stop_move():
            client.write_calls.append(("stop_move",))
            clock.advance_ms(sdk_ms)

        client.stop_move = stop_move
        probe.observer.gate.tokens.put_nowait(None)
        try:
            await asyncio.wait_for(gate.entered.wait(), 1)
            probe.socket.incoming.put_nowait(WebSocketDisconnect())
            await asyncio.wait_for(gate.cancelled.wait(), 1)
            gate.release.set()
            if sdk_ms > 200:
                with pytest.raises(BackendCommandError, match="sdk_timeout:stop_move"):
                    await asyncio.wait_for(probe.session, 1)
                assert probe.control._fault == "stop_unverified"
                assert probe.control.mode is TeleopMode.FAULT
            else:
                await asyncio.wait_for(probe.session, 1)
                assert probe.control._fault is None
            rpc, diagnostic = await stop_events(events)
            assert rpc["started_ns"] - times["cancel_requested"] == 75_000_000
            assert diagnostic["stop_move_lock"]["wait_ns"] == 0
            assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
            assert rpc["sdk_await_completed_ns"] - rpc["sdk_await_started_ns"] == sdk_ms * 1_000_000
            assert rpc["outcome"] == ("returned_late" if sdk_ms > 200 else "returned")
            assert rpc["wait_failed"] is (sdk_ms > 200)
            assert client.write_calls == [("stop_move",)]
            assert probe.state.teleop_owner is None
        finally:
            gate.release.set()
