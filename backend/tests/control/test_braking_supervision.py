"""Test-only braking policy through production control; injected backends only."""

import asyncio

import numpy as np
import pytest

from app.control.coordinate_mapper import CoordinateMapper
from app.control.robot_control import LatestVRFrame, RobotControl
from app.robots.base import BackendCommandError, StopReason
from app.schemas.messages import Pose, TeleopMode
from tests.control.braking_supervision_probe import SupervisedApproach
from tests.control.elapsed_step_probe import publish
from tests.control.test_robot_control import FakeBackend, FakeClock, RecordingRecorder
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _wait_until


async def active_control(backend=None, clock=None):
    backend = backend or FakeBackend()
    clock = clock or FakeClock()
    latest, recorder, limits = LatestVRFrame(), RecordingRecorder(), SupervisedApproach()
    control = RobotControl(
        backend=backend, latest=latest, clock=clock, recorder=recorder,
        mapper=CoordinateMapper(translation_scale=0.2, rotation_dead_zone_deg=0.5),
        limiter=limits,
    )
    await control.connect()
    publish(latest, clock, 1, False, 0, 0)
    await control.tick()
    await control.arm()
    publish(latest, clock, 2, True, 0, 0)
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    return control, latest, backend, clock, limits, recorder


async def warm(probe, channel, client=None):
    control, latest, backend, clock, limits, _ = probe
    # Original mapper and PoseFilter; accelerate for 400 ms without reaching goal.
    for seq in range(3, 23):
        clock.advance_ms(20)
        if client is not None:
            await backend.get_state()
            if seq == 22:
                client.block_ik = True
                client.ik_started.clear()
        publish(latest, clock, seq, True, 0.1 if channel == 'linear' else 0,
                0.4 if channel == 'angular' else 0)
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE, control._fault
    velocity = limits.candidate.linear_velocity if channel == 'linear' else limits.candidate.angular_velocity
    assert np.linalg.norm(velocity) > 0


def publish_reversal(probe, channel):
    _, latest, _, clock, _, _ = probe
    clock.advance_ms(20)
    publish(latest, clock, 24, True, -0.1 if channel == 'linear' else 0,
            -0.4 if channel == 'angular' else 0)


async def assert_no_recovery(probe, fault):
    control, latest, backend, clock, _, _ = probe
    for seq, grip in enumerate((False, True, False, True), 25):
        clock.advance_ms(20)
        publish(latest, clock, seq, grip, 0.1, 0.4)
        await control.tick()
        assert (await control.state_message()).fault == fault
    with pytest.raises(RuntimeError, match='arm_blocked_by_fault'):
        await control.arm()
    result = await control.reset_fault()
    assert not result.accepted
    assert result.reason == 'unrecoverable_fault'


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_infeasible_filtered_target_latches_and_never_submits(channel):
    probe = await active_control()
    control, _, backend, _, limits, recorder = probe
    await warm(probe, channel)
    targets = list(backend.targets)
    publish_reversal(probe, channel)
    await control.tick()
    assert len(limits.rejections) == 1
    assert control.mode is TeleopMode.FAULT
    assert backend.targets == targets
    assert backend.stops == [StopReason.FAULT]
    assert [e['kind'] for e in recorder.events][-3:] == ['robot_fault', 'stop_requested', 'stop_confirmed']
    assert (await control.state_message()).fault == 'target_braking_infeasible'
    assert control.mode is TeleopMode.DISARMED
    await assert_no_recovery(probe, 'target_braking_infeasible')
    assert backend.targets == targets
    assert backend.stops == [StopReason.FAULT]
    assert backend.home_phases == []


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_normal_grip_release_recaptures_actual_pose_and_zero_velocity(channel):
    probe = await active_control()
    control, latest, backend, clock, limits, _ = probe
    await warm(probe, channel)
    clock.advance_ms(20)
    publish(latest, clock, 23, False, 0.1, 0.4)
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    assert backend.stops == [StopReason.GRIP_RELEASED]
    backend.actual_tcp = Pose(p=(0.35, 0.02, 0.32), q=(0, 0, 0, 1))
    count = len(backend.targets)
    clock.advance_ms(20)
    publish(latest, clock, 24, True, -0.1, -0.4)
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    assert control.last_target == backend.actual_tcp
    assert len(backend.targets) == count
    assert np.linalg.norm(limits.candidate.linear_velocity) == 0
    assert np.linalg.norm(limits.candidate.angular_velocity) == 0
    clock.advance_ms(20)
    publish(latest, clock, 25, True, -0.1, -0.4)
    await control.tick()
    assert backend.targets[-1][1] == backend.actual_tcp
    assert not limits.rejections


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_failed_grip_release_stop_forbids_reanchor(channel):
    probe = await active_control()
    control, latest, backend, clock, limits, _ = probe
    await warm(probe, channel)
    targets = list(backend.targets)

    async def failed_stop(reason):
        backend.stops.append(reason)
        raise BackendCommandError('sdk_timeout:stop_move')

    backend.stop = failed_stop
    clock.advance_ms(20)
    publish(latest, clock, 23, False, 0.1, 0.4)
    await control.tick()
    assert control.mode is TeleopMode.FAULT
    assert not control.mapper.has_anchor
    assert control.last_target is None
    await assert_no_recovery(probe, 'stop_unverified')
    assert not control.mapper.has_anchor
    assert backend.targets == targets
    assert backend.stops == [StopReason.GRIP_RELEASED]
    assert not limits.rejections


@pytest.mark.parametrize('outcome', ['confirmed', 'incomplete', 'error', 'late', 'cancelled'])
@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_candidate_fault_uses_real_adapter_stop_transaction(channel, outcome):
    adapter, client, clock = await _connected_control_adapter()
    client.ik_results.extend([list(client.kin_data['actual_joint_pose']) for _ in range(100)])
    stop_started, release_stop = asyncio.Event(), asyncio.Event()
    task = None
    events = []

    async def record(event, timestamp):
        events.append(event)

    async def stop_move():
        client.write_calls.append(('stop_move',))
        stop_started.set()
        if outcome == 'error':
            raise RuntimeError('injected_stop_error')
        if outcome == 'late':
            clock.advance_ms(609)
        if outcome == 'cancelled':
            await release_stop.wait()

    adapter._event_callback = record
    client.stop_move = stop_move
    try:
        probe = await active_control(adapter, clock)
        control, _, _, _, limits, recorder = probe
        await warm(probe, channel, client)
        # Let the production pump start one IK request; newer accepted targets
        # remain pending. Stop must invalidate both before IK is released.
        await asyncio.wait_for(client.ik_started.wait(), 1)
        # A newer valid request is queued while the old IK is blocked.
        clock.advance_ms(20)
        publish(probe[1], clock, 23, True, 0.1 if channel == 'linear' else 0,
                0.4 if channel == 'angular' else 0)
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE
        assert adapter._pump.has_pending
        writes_before_stop = list(client.write_calls)
        if outcome == 'incomplete':
            client.kin_data['actual_joint_speed'] = [0.1] * 6
        publish_reversal(probe, channel)
        task = asyncio.create_task(control.tick())
        await _wait_until(lambda: adapter._stop_transaction is not None)
        client.release_ik.set()
        await asyncio.wait_for(stop_started.wait(), 1)
        if outcome == 'cancelled':
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            release_stop.set()
        else:
            await asyncio.wait_for(task, 2)
        assert len(limits.rejections) == 1
        assert not adapter._pump.has_pending
        expected = {'confirmed': 'target_braking_infeasible', 'incomplete': 'stop_incomplete'}.get(outcome, 'stop_unverified')
        state = await control.state_message()
        assert state.fault == expected
        assert control.mode is (TeleopMode.DISARMED if outcome == 'confirmed' else TeleopMode.FAULT)
        # Even subsequent stationary/IDLE feedback cannot clear a failed stop.
        client.kin_data['actual_joint_speed'] = [0.0] * 6
        await assert_no_recovery(probe, expected)
        assert client.write_calls == [*writes_before_stop, ('stop_move',)]
        kinds = [e['kind'] for e in recorder.events]
        assert ('stop_confirmed' in kinds) is (outcome == 'confirmed')
        assert ('stop_failed' in kinds) is (outcome != 'confirmed')
        if outcome == 'late':
            lifecycle = next(e for e in events if e['kind'] == 'stop_rpc_lifecycle')
            assert lifecycle['deadline_ns'] - lifecycle['started_ns'] == 200_000_000
            assert lifecycle['sdk_await_completed_ns'] - lifecycle['sdk_await_started_ns'] == 609_000_000
            assert lifecycle['outcome'] == 'returned_late'
    finally:
        client.release_ik.set()
        release_stop.set()
        if task is not None and not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()
    assert client.network_calls == client.discovery_calls == 0
    assert client.write_calls == [*writes_before_stop, ('stop_move',)]
