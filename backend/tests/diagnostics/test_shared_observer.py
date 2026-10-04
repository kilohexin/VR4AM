"""Offline experiment only: no runtime task or sender is installed.

One session owner drives state_message independently of passive consumers.
The manual sleep gate models completion-relative scheduling, not wall time.
"""

import asyncio
from contextlib import suppress

import pytest

from app.api.teleop_ws import state_sender
from app.recording.commissioning import RecorderUnavailable
from app.robots.base import StopReason
from app.schemas.messages import TeleopMode
from tests.control.test_state_observation import StateRecorder
from tests.diagnostics.test_display_integration import DisplayHarness
from tests.robots.test_lebai_adapter_control import _target, _wait_until
from tests.robots.test_snapshot_contention import delayed_adapter, waiter_count


class StepGate:
    def __init__(self, clock):
        self.clock = clock
        self.delays = []
        self.tokens = asyncio.Queue()

    async def sleep(self, seconds):
        self.delays.append(seconds)
        await self.tokens.get()
        self.clock.advance_ms(seconds * 1000)

    async def wait(self, count):
        await _wait_until(lambda: len(self.delays) == count)

    async def step(self):
        count = len(self.delays)
        self.tokens.put_nowait(None)
        await self.wait(count + 1)


class SharedObserverProbe(DisplayHarness):
    """Test-only owner. Reconnect is forbidden until the prior task is joined.

    Closing invalidates the view immediately, then cancels/joins observation.
    Failure escapes to the owner without retry or invented control fault rules.
    """

    def __init__(self, adapter, clock, *, period_s=0.04):
        super().__init__(adapter, clock)
        self.gate = StepGate(clock)
        self.recorder = StateRecorder()
        self.control.recorder = self.recorder
        self.control.machine.connect()  # Synthetic session; adapter is injected.
        self.task = None
        self.closing = False
        self.completed = 0
        self.period_s = period_s  # Explicit experiment parameter, no deployment.

    def start(self):
        if self.task is not None:
            raise RuntimeError("prior_observer_not_joined")
        self.generation = self.view.begin_connection()
        self.closing = False
        self.task = asyncio.create_task(self.run())
        return self.task

    async def run(self):
        try:
            while not self.closing:
                await self.control.state_message()
                self.completed += 1
                if not self.closing:
                    await self.gate.sleep(self.period_s)
        finally:
            self.view.disconnect()

    def request_close(self):
        if not self.closing:
            self.closing = True
            self.view.disconnect()
            if self.task is not None:
                self.task.cancel()

    async def close(self):
        self.request_close()
        try:
            if self.task is not None:
                with suppress(asyncio.CancelledError):
                    await self.task
        finally:
            self.task = None


@pytest.mark.parametrize("consumers", [0, 1, 3])
async def test_idle_observation_and_recording_continue_independent_of_consumer_count(consumers):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        before = client.read_calls.count("get_kin_data")
        tasks.append(probe.start())
        try:
            await probe.gate.wait(1)
            for cycle in range(3):
                for _ in range(consumers):
                    assert probe.compose().status == "available"
                if cycle < 2:
                    await probe.gate.step()
            assert probe.completed == 3
            assert len(probe.recorder.rows) == 3
            assert client.read_calls.count("get_kin_data") - before == 3
            assert [r[1] for r in probe.recorder.rows] == [16_000_000, 72_000_000, 128_000_000]
        finally:
            await probe.close()
        assert probe.compose().status == "disconnected"


async def test_slow_reads_do_not_create_overlapping_reads_or_catchup_ticks():
    async with delayed_adapter(later_read_ms=120) as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        tasks.append(probe.start())
        try:
            await probe.gate.wait(1)
            await probe.gate.step()
            assert probe.compose().status == "stale"
            await probe.gate.step()
            reads = [e for e in events if e["kind"] == "robot_kinematics"]
            assert [e["server_mono_ns"] for e in reads] == [0, 56_000_000, 216_000_000]
            assert probe.gate.delays == [0.04, 0.04, 0.04]
            assert probe.completed == 3
        finally:
            await probe.close()


@pytest.mark.parametrize(
    "period_s,timestamps,last_gap_age,status",
    [
        (0.02, [16_000_000, 52_000_000, 88_000_000], 35, "available"),
        (0.04, [16_000_000, 72_000_000, 128_000_000], 55, "available"),
        (0.08, [16_000_000, 112_000_000, 208_000_000], 95, "stale"),
    ],
)
async def test_cadence_candidates_include_read_time_and_display_age(period_s, timestamps, last_gap_age, status):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock, period_s=period_s)
        tasks.append(probe.start())
        try:
            await probe.gate.wait(1)
            await probe.gate.step()
            await probe.gate.step()
            assert [r[1] for r in probe.recorder.rows] == timestamps
            # Observe just before the next scheduled wake without releasing it.
            clock.advance_ms(period_s * 1000 - 1)
            sample = probe.compose()
            assert sample.local_read_age_ms == last_gap_age
            assert sample.status == status
            assert sample.source_freshness == "unknown"
            assert probe.completed == 3
        finally:
            await probe.close()


async def test_blocked_producer_ages_display_without_consumer_fallback_reads():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        entered = asyncio.Event()

        async def blocked_read():
            entered.set()
            await asyncio.Event().wait()

        tasks.append(probe.start())
        try:
            await probe.gate.wait(1)
            client.get_kin_data = blocked_read
            probe.gate.tokens.put_nowait(None)
            await asyncio.wait_for(entered.wait(), 1)
            before = list(client.read_calls)
            clock.advance_ms(100)
            for _ in range(3):
                assert probe.compose().status == "stale"
            assert client.read_calls == before
            assert probe.completed == 1
            assert len(probe.recorder.rows) == 1
        finally:
            await probe.close()
        assert not adapter._sdk_lock.locked()
        assert probe.compose().status == "disconnected"


async def test_blocked_display_consumer_does_not_delay_fault_observation_or_recording():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        tasks.append(probe.start())
        sending = asyncio.Event()

        async def blocked_consumer():
            probe.compose()
            sending.set()
            await asyncio.Event().wait()

        consumer = None
        try:
            await probe.gate.wait(1)
            consumer = asyncio.create_task(blocked_consumer())
            await sending.wait()
            adapter._latch_unverified_stop()
            await probe.gate.step()
            assert not consumer.done()
            assert probe.control._fault == "stop_unverified"
            assert probe.control.mode is TeleopMode.FAULT
            assert probe.control._pending_stop_completion
            assert probe.recorder.rows[-1][0].fault == "stop_unverified"
            consumer.cancel()
            with suppress(asyncio.CancelledError):
                await consumer
            await probe.gate.step()
            assert probe.completed == 3
            assert not probe.task.done()
        finally:
            if consumer is not None:
                consumer.cancel()
                with suppress(asyncio.CancelledError):
                    await consumer
            await probe.close()


async def test_existing_sender_stops_observing_while_send_is_blocked():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        entered = asyncio.Event()

        class BlockedSocket:
            async def send_json(self, payload):
                entered.set()
                await asyncio.Event().wait()

        task = asyncio.create_task(state_sender(BlockedSocket(), probe.control))
        tasks.append(task)
        try:
            await asyncio.wait_for(entered.wait(), 1)
            adapter._latch_unverified_stop()
            clock.advance_ms(500)
            await asyncio.sleep(0)
            assert len(probe.recorder.rows) == 1
            assert probe.control._fault is None
            assert probe.control.mode is TeleopMode.READY
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task


async def test_no_consumers_still_allows_existing_pending_stop_transition():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        probe.control.machine.stale()
        probe.control._pending_stop_completion = True
        tasks.append(probe.start())
        try:
            await probe.gate.wait(1)
            assert probe.recorder.rows[0][0].mode is TeleopMode.STALE
            assert probe.control.mode is TeleopMode.DISARMED
            assert probe.compose().control.mode is TeleopMode.DISARMED
            assert probe.control.control_generation == 1
            assert not probe.control._pending_stop_completion
        finally:
            await probe.close()


@pytest.mark.parametrize("failure_phase", ["read", "record"])
async def test_observer_failure_invalidates_display_propagates_and_does_not_restart(failure_phase):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        tasks.append(probe.start())

        async def fail():
            raise RuntimeError("offline_observer_failed")

        try:
            await probe.gate.wait(1)
            if failure_phase == "read":
                adapter.get_state = fail
            else:
                probe.recorder.on_write = fail
            probe.gate.tokens.put_nowait(None)
            with pytest.raises(RuntimeError, match="offline_observer_failed"):
                await asyncio.wait_for(probe.task, 1)
            assert probe.compose().status == "disconnected"
            assert probe.completed == 1
            assert len(probe.gate.delays) == 1
            assert probe.control._fault is None  # Owner cleanup is still required.
            with pytest.raises(RuntimeError, match="prior_observer_not_joined"):
                probe.start()
        finally:
            with pytest.raises(RuntimeError, match="offline_observer_failed"):
                await probe.close()


async def test_recorder_unavailable_keeps_existing_authoritative_failure_response():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        probe.control.machine.mode = TeleopMode.ACTIVE

        async def fail():
            raise RecorderUnavailable("offline recorder unavailable")

        probe.recorder.on_write = fail
        tasks.append(probe.start())
        try:
            await probe.gate.wait(1)
            assert client.write_calls == [("stop_move",)]
            assert probe.control._fault == "recording_unavailable"
            assert probe.compose().fault == "recording_unavailable"
            assert probe.control.mode is TeleopMode.DISARMED
            assert probe.completed == 1
        finally:
            await probe.close()


async def test_close_joins_cancel_resistant_read_before_new_generation_can_start():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        entered, cancelled, release = asyncio.Event(), asyncio.Event(), asyncio.Event()
        original = client.get_kin_data

        async def resistant_read():
            entered.set()
            try:
                await release.wait()
            except asyncio.CancelledError:
                cancelled.set()
                await release.wait()
            return await original()

        client.get_kin_data = resistant_read
        tasks.append(probe.start())
        closing = None
        try:
            await asyncio.wait_for(entered.wait(), 1)
            old_generation = probe.generation
            closing = asyncio.create_task(probe.close())
            await asyncio.wait_for(cancelled.wait(), 1)
            assert not closing.done()
            assert probe.compose().status == "disconnected"
            with pytest.raises(RuntimeError, match="prior_observer_not_joined"):
                probe.start()
            release.set()
            await asyncio.wait_for(closing, 1)
            assert (old_generation, False) in probe.publications
            assert len(probe.recorder.rows) == 1  # Old authority may finish; join matters.
            client.get_kin_data = original
            await adapter.disconnect()
            await adapter.connect()
            tasks.append(probe.start())
            await probe.gate.wait(1)
            assert probe.generation != old_generation
            assert probe.compose().status == "available"
        finally:
            release.set()
            if closing is not None:
                await closing
            await probe.close()


@pytest.mark.parametrize("shared,consumers,reads", [(False, 1, 1), (True, 1, 1), (False, 3, 3), (True, 3, 1)])
@pytest.mark.parametrize("operation", ["motion", "stop"])
async def test_contention_comparison_includes_one_authoritative_observation(shared, consumers, reads, operation):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        probe = SharedObserverProbe(adapter, clock)
        before = client.read_calls.count("get_kin_data")
        await adapter._sdk_lock.acquire()
        try:
            if shared:
                tasks.append(probe.start())
                for _ in range(consumers):
                    assert probe.compose().status == "missing"
            else:
                for _ in range(consumers):
                    tasks.append(asyncio.create_task(probe.control.state_message()))
            await _wait_until(lambda: waiter_count(adapter) == reads)
            if operation == "motion":
                await adapter.command_tcp(_target(), 1)
            else:
                tasks.append(asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED)))
            await _wait_until(lambda: waiter_count(adapter) == reads + 1)
        finally:
            adapter._sdk_lock.release()
        try:
            if shared:
                await probe.gate.wait(1)
            kind = "pvat_sent" if operation == "motion" else "stop_diagnostics"
            await _wait_until(lambda: any(e["kind"] == kind for e in events))
            event = next(e for e in events if e["kind"] == kind)
            assert len(probe.recorder.rows) == reads
            if operation == "motion":
                assert event["timing_ms"]["snapshot_lock_wait"] == reads * 16
                assert event["timing_ms"]["handler_to_send_complete"] == 32 + reads * 16
                assert client.read_calls.count("get_kin_data") - before == reads + 1
            else:
                assert event["stop_move_lock"]["wait_ns"] == reads * 16_000_000
                assert client.read_calls.count("get_kin_data") - before == reads + len(event["samples"])
                rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
                assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
                assert client.write_calls == [("stop_move",)]
        finally:
            if shared:
                await probe.close()
