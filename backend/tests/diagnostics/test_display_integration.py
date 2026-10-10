"""Offline integration harness; no production publisher or sender is replaced."""

import asyncio
from dataclasses import asdict
from unittest.mock import AsyncMock

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.diagnostics.display_snapshot import DisplaySnapshotView
from app.diagnostics.display_state import LiveDisplayControl, compose_display_state
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from app.schemas.messages import RobotStateMessage, TeleopMode
from tests.robots.test_lebai_adapter_control import _frame, _target, _wait_until
from tests.robots.test_snapshot_contention import delayed_adapter, waiter_count


class DisplayHarness:
    """Test-only lifecycle/read hook; generations captured BEFORE async reads."""

    def __init__(self, adapter, clock):
        self.adapter = adapter
        self.clock = clock
        self.latest = LatestVRFrame()
        self.control = RobotControl(backend=adapter, latest=self.latest,
                                    clock=clock, recorder=NoopRecorder())
        self.view = DisplaySnapshotView(clock=clock.now_ns, max_local_age_ms=80)
        self.generation = self.view.begin_connection()
        self.publications = []
        self.original_read = adapter._read_snapshot

        async def publish_read(**kwargs):
            generation = self.generation
            snapshot = await self.original_read(**kwargs)
            self.publish(generation, snapshot)
            return snapshot

        adapter._read_snapshot = publish_read
        self.publish(self.generation, adapter._snapshot)

    def publish(self, generation, snapshot):
        measurement = RobotStateMessage(
            server_mono_ns=snapshot.captured_ns, mode=TeleopMode.READY,
            actual_tcp=snapshot.actual_tcp, actual_q=snapshot.actual_q,
            robot_state=snapshot.robot_state, gripper=snapshot.gripper,
            fault=snapshot.estop, backend="LEBAI", real_robot_mode="control",
            # Deliberately wrong cached metadata: composition must discard it.
            ack_seq=999, sample_age_ms=999, translation_scale=9,
        )
        accepted = self.view.publish(
            generation, measurement, read_started_ns=snapshot.captured_ns,
            read_completed_ns=snapshot.read_completed_ns, read_purpose=snapshot.read_purpose,
        )
        self.publications.append((generation, accepted))

    def compose(self):
        received = self.latest.snapshot()
        now = self.clock.now_ns()
        current = LiveDisplayControl(
            server_mono_ns=now, ack_seq=self.control.last_seq, mode=self.control.mode,
            vr_input_age_ms=None if received is None else max(0, now - received.received_ns) / 1e6,
            fault=self.control._fault or self.adapter._latched_fault
            or (str(self.adapter.pump_fault) if self.adapter.pump_fault else None),
            constraint=self.control._constraint, recovery_phase=self.control._recovery_phase,
            translation_scale=self.control.mapper.translation_scale,
            control_generation=self.control.control_generation,
        )
        return compose_display_state(self.view, current)


async def test_completed_read_publishes_and_composition_uses_current_control_metadata():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        await harness.control.connect()
        harness.latest.publish(_frame(42, grip=False), clock.now_ns())
        await harness.control.tick()
        clock.advance_ms(10)
        reads = list(client.read_calls)
        frame = harness.compose()
        assert frame.robot is not None
        assert frame.status == "available"
        assert frame.control.ack_seq == 42
        assert frame.control.mode is TeleopMode.READY
        assert frame.control.vr_input_age_ms == 10
        assert frame.control.translation_scale == 1
        assert frame.control.server_mono_ns == clock.now_ns()
        assert frame.local_read_age_ms == 26
        assert frame.read_purpose == "state"
        assert "ack_seq" not in asdict(frame.robot)
        assert "mode" not in asdict(frame.robot)
        assert "sample_age_ms" not in asdict(frame.robot)
        assert client.read_calls == reads


async def test_live_ack_and_vr_age_advance_while_robot_measurement_is_stale():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        await harness.control.connect()
        clock.advance_ms(100)
        harness.latest.publish(_frame(43, grip=False), clock.now_ns())
        await harness.control.tick()
        frame = harness.compose()
        assert frame.status == "stale"
        assert frame.control.ack_seq == 43
        assert frame.control.vr_input_age_ms == 0
        assert frame.local_read_age_ms == 116
        assert frame.source_freshness == "unknown"


async def test_arm_and_live_constraint_use_current_control_not_cached_ready_metadata():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        await harness.control.connect()
        harness.latest.publish(_frame(44, grip=False), clock.now_ns())
        await harness.control.tick()
        await harness.control.arm()
        harness.control._constraint = "tracking_lag"
        frame = harness.compose()
        assert frame.control.mode is TeleopMode.ARMED
        assert frame.control.constraint == "tracking_lag"
        assert frame.control.control_generation == harness.control.control_generation
        assert frame.control.ack_seq == 44
        assert frame.read_purpose == "preflight"


async def test_late_old_generation_publication_is_rejected_after_disconnect_and_reconnect():
    async with delayed_adapter(block_first_read=True) as (adapter, client, clock, events, tasks, entered, release):
        harness = DisplayHarness(adapter, clock)
        pending = asyncio.create_task(adapter.get_state())
        tasks.append(pending)
        await asyncio.wait_for(entered.wait(), 1)
        old = harness.generation
        harness.view.disconnect()
        assert harness.compose().status == "disconnected"
        # Reconnect the view only: model a new publisher epoch without a robot command.
        harness.generation = harness.view.begin_connection()
        assert harness.compose().status == "missing"
        release.set()
        await asyncio.wait_for(pending, 1)
        assert harness.publications[-1] == (old, False)
        assert harness.compose().status == "missing"
        await adapter.get_state()
        assert harness.publications[-1] == (harness.generation, True)
        assert harness.compose().status == "available"


async def test_fake_adapter_reconnect_keeps_old_read_out_of_new_display_generation():
    async with delayed_adapter(block_first_read=True) as (adapter, client, clock, events, tasks, entered, release):
        harness = DisplayHarness(adapter, clock)
        pending = asyncio.create_task(adapter.get_state())
        tasks.append(pending)
        await asyncio.wait_for(entered.wait(), 1)
        old = harness.generation
        harness.view.disconnect()
        await adapter.disconnect()
        harness.generation = harness.view.begin_connection()
        connecting = asyncio.create_task(adapter.connect())
        tasks.append(connecting)
        await _wait_until(lambda: waiter_count(adapter) == 1)
        release.set()
        await asyncio.wait_for(asyncio.gather(pending, connecting), 1)
        assert (old, False) in harness.publications
        assert harness.publications[-1] == (harness.generation, True)
        assert harness.compose().read_purpose == "connection"
        assert harness.compose().status == "available"
        assert client.network_calls == client.discovery_calls == 0


async def test_passive_composition_does_not_replace_backend_fault_latching_in_state_message():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        await harness.control.connect()
        adapter._latch_unverified_stop()
        passive = harness.compose()
        assert passive.fault == "stop_unverified"
        assert harness.control._fault is None
        assert harness.control.mode is TeleopMode.READY
        await harness.control.state_message()
        assert harness.control._fault == "stop_unverified"
        assert harness.control.mode is TeleopMode.FAULT


async def test_passive_composition_cannot_complete_pending_stop_from_cached_idle():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        await harness.control.connect()
        harness.control.machine.stale()
        harness.control._pending_stop_completion = True
        generation = harness.control.control_generation
        harness.compose()
        assert harness.control._pending_stop_completion
        assert harness.control.control_generation == generation
        # Existing authoritative path still owns this transition.
        await harness.control.state_message()
        assert not harness.control._pending_stop_completion
        assert harness.control.control_generation > generation


async def test_recording_remains_owned_by_existing_state_message():
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        await harness.control.connect()
        recorder = AsyncMock()
        harness.control.recorder = recorder
        harness.compose()
        recorder.write_robot_state.assert_not_awaited()
        await harness.control.state_message()
        recorder.write_robot_state.assert_awaited_once()


async def test_cancelled_consumer_does_not_cancel_blocked_read_or_its_publication():
    async with delayed_adapter(block_first_read=True) as (adapter, client, clock, events, tasks, entered, release):
        harness = DisplayHarness(adapter, clock)
        producing = asyncio.create_task(adapter.get_state())
        tasks.append(producing)
        await asyncio.wait_for(entered.wait(), 1)
        composed = asyncio.Event()

        async def consumer():
            harness.compose()
            composed.set()
            await asyncio.Event().wait()

        consuming = asyncio.create_task(consumer())
        tasks.append(consuming)
        await asyncio.wait_for(composed.wait(), 1)
        consuming.cancel()
        with pytest.raises(asyncio.CancelledError):
            await consuming
        assert not producing.done()
        release.set()
        await asyncio.wait_for(producing, 1)
        assert harness.publications[-1] == (harness.generation, True)
        assert harness.compose().read_purpose == "state"


@pytest.mark.parametrize("passive", [False, True])
async def test_three_display_consumers_compare_motion_wait_and_sdk_read_count(passive):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        before = client.read_calls.count("get_kin_data")
        await adapter._sdk_lock.acquire()
        try:
            if passive:
                for _ in range(3):
                    harness.compose()
            else:
                for _ in range(3):
                    tasks.append(asyncio.create_task(adapter.get_state()))
            readers = 0 if passive else 3
            await _wait_until(lambda: waiter_count(adapter) == readers)
            await adapter.command_tcp(_target(), 1)
            await _wait_until(lambda: waiter_count(adapter) == readers + 1)
        finally:
            adapter._sdk_lock.release()
        await _wait_until(lambda: any(e["kind"] == "pvat_sent" for e in events))
        await asyncio.gather(*tasks)
        event = next(e for e in events if e["kind"] == "pvat_sent")
        assert event["timing_ms"]["handler_to_send_complete"] == (32 if passive else 80)
        assert event["timing_ms"]["snapshot_lock_wait"] == (0 if passive else 48)
        assert client.read_calls.count("get_kin_data") - before == (1 if passive else 4)
        assert harness.compose().read_purpose == "motion"


@pytest.mark.parametrize("passive", [False, True])
async def test_three_display_consumers_compare_stop_lock_wait(passive):
    async with delayed_adapter() as (adapter, client, clock, events, tasks, _, _release):
        harness = DisplayHarness(adapter, clock)
        before = client.read_calls.count("get_kin_data")
        await adapter._sdk_lock.acquire()
        try:
            if passive:
                for _ in range(3):
                    harness.compose()
            else:
                for _ in range(3):
                    tasks.append(asyncio.create_task(adapter.get_state()))
            readers = 0 if passive else 3
            await _wait_until(lambda: waiter_count(adapter) == readers)
            stopping = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
            tasks.append(stopping)
            await _wait_until(lambda: waiter_count(adapter) == readers + 1)
        finally:
            adapter._sdk_lock.release()
        await asyncio.wait_for(asyncio.gather(*tasks), 1)
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in events))
        diagnostic = next(e for e in events if e["kind"] == "stop_diagnostics")
        rpc = next(e for e in events if e["kind"] == "stop_rpc_lifecycle")
        assert diagnostic["stop_move_lock"]["wait_ns"] == 0
        assert rpc["deadline_ns"] - rpc["started_ns"] == 200_000_000
        assert client.read_calls.count("get_kin_data") - before == len(diagnostic["samples"]) + readers
        assert client.write_calls == [("stop_move",)]
