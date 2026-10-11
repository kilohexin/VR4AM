"""Offline dead-man release must survive newest-only coalescing and precede claw RPCs."""

import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.schemas.messages import TeleopMode
from app.control.robot_control import LatestVRFrame
from tests.control.test_robot_control import RecordingRecorder, connect_release_arm, frame, make_control


async def active_control():
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True, trigger=0), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    clock.advance_ms(100)
    return control, latest, backend, clock


@pytest.mark.parametrize('coalesced', [False, True])
async def test_grip_release_stops_before_new_trigger_command(coalesced):
    control, latest, backend, clock = await active_control()
    old_targets = list(backend.targets)
    latest.publish(frame(3, False, trigger=0.8), clock.now_ns())
    if coalesced:
        latest.publish(frame(4, True, trigger=0.8), clock.now_ns())

    await control.tick()

    assert backend.gripper_commands == []
    assert backend.targets == old_targets
    assert backend.stops == [StopReason.GRIP_RELEASED]
    assert control.mode is TeleopMode.HOLD
    assert not control.mapper.has_anchor
    assert latest.depth == 1
    if coalesced:
        assert latest.snapshot().frame.seq == 4
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE
        assert control.mapper.has_anchor
        assert backend.stops == [StopReason.GRIP_RELEASED]


@pytest.mark.parametrize('fault', ['stop_unverified', 'stop_incomplete'])
async def test_coalesced_release_stop_failure_never_resumes_or_sends_claw(fault):
    control, latest, backend, clock = await active_control()

    async def failed_stop(reason):
        backend.stops.append(reason)
        raise BackendCommandError(fault)

    backend.stop = failed_stop
    latest.publish(frame(3, False, trigger=1), clock.now_ns())
    latest.publish(frame(4, True, trigger=1), clock.now_ns())
    await control.tick()
    state = await control.state_message()
    assert state.mode is TeleopMode.FAULT
    assert state.fault == fault
    assert backend.gripper_commands == []
    assert backend.targets == []
    stops = list(backend.stops)
    latest.publish(frame(5, True, trigger=0), clock.now_ns())
    await control.tick()
    assert backend.stops == stops
    assert backend.gripper_commands == []


async def test_trigger_held_at_arm_requires_release_then_deliberate_new_press():
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    for seq, trigger in [(2, 1), (3, 0), (4, 0.08)]:
        clock.advance_ms(100)
        latest.publish(frame(seq, False, trigger=trigger), clock.now_ns())
        await control.tick()
    assert backend.gripper_commands == []

    latest.publish(frame(5, False, trigger=0.5), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == [0.5]


async def test_regrip_does_not_reuse_trigger_press_from_before_hold():
    control, latest, backend, clock = await active_control()
    latest.publish(frame(3, True, trigger=0.5), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == [0.5]
    clock.advance_ms(100)
    latest.publish(frame(4, False, trigger=1), clock.now_ns())
    latest.publish(frame(5, True, trigger=1), clock.now_ns())
    await control.tick()  # Consume release and stop, not Trigger.
    await control.tick()  # New anchor after the verified fake stop.
    await control.tick()  # Held Trigger establishes baseline, not claw intent.
    assert backend.gripper_commands == [0.5]
    latest.publish(frame(6, True, trigger=0), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == [0.5]
    latest.publish(frame(7, True, trigger=0.6), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == [0.5, 0.6]


@pytest.mark.parametrize('change', ['expired', 'invalid', 'hidden', 'new_session'])
async def test_release_does_not_downgrade_stale_or_session_change_stop(change):
    control, latest, backend, clock = await active_control()
    latest.publish(frame(3, False), clock.now_ns())
    latest.publish(frame(
        4, True, tracking_valid=change != 'invalid',
        visibility='hidden' if change == 'hidden' else 'visible',
        session_id='new' if change == 'new_session' else 'test',
    ), clock.now_ns())
    if change == 'expired':
        clock.advance_ms(100)

    await control.tick()
    expected = StopReason.DISCONNECT if change == 'new_session' else StopReason.STALE
    assert backend.stops == [expected]
    assert control.mode is (TeleopMode.DISARMED if change == 'new_session' else TeleopMode.STALE)
    assert backend.gripper_commands == []


async def test_disarm_during_release_recording_prevents_a_duplicate_stop():
    control, latest, backend, clock = await active_control()
    recorder = RecordingRecorder()
    original = recorder.write_critical_event

    async def recording(kind, payload, now):
        await original(kind, payload, now)
        if kind == 'state_transition' and payload == {'from': 'ACTIVE', 'to': 'HOLD'}:
            await control.disarm()

    recorder.write_critical_event = recording
    control.recorder = recorder
    latest.publish(frame(3, False), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    assert backend.stops == [StopReason.GRIP_RELEASED]


async def test_concurrent_tick_cannot_regrip_while_release_stop_is_pending():
    control, latest, backend, clock = await active_control()
    started, finish = asyncio.Event(), asyncio.Event()

    async def stopping(reason):
        backend.stops.append(reason)
        started.set()
        await finish.wait()

    backend.stop = stopping
    latest.publish(frame(3, False), clock.now_ns())
    latest.publish(frame(4, True), clock.now_ns())
    stopping_tick = asyncio.create_task(control.tick())
    try:
        await started.wait()
        await control.tick()
        assert control.mode is TeleopMode.HOLD
        assert not control.mapper.has_anchor
        assert backend.gripper_commands == []
        assert backend.targets == []
    finally:
        finish.set()
        await stopping_tick
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE


def test_release_count_only_tracks_accepted_same_session_edges_with_capacity_one():
    latest = LatestVRFrame()
    latest.publish(frame(2, True), 10)
    latest.publish(frame(2, False), 20)
    latest.publish(frame(1, False), 30)
    assert latest.grip_release_count == 0
    latest.publish(frame(3, False), 40)
    latest.publish(frame(4, False), 50)
    latest.publish(frame(5, True), 60)
    latest.publish(frame(6, False), 70)
    assert latest.grip_release_count == 2
    latest.publish(frame(1, True, session_id='new'), 80)
    latest.publish(frame(7, False), 90)  # Retired session cannot add an event.
    assert latest.grip_release_count == 2
    assert latest.depth == 1
    assert latest.snapshot().frame.session_id == 'new'


async def test_pending_release_revokes_target_and_direct_gripper_authority():
    control, latest, backend, clock = await active_control()
    latest.publish(frame(3, False), clock.now_ns())
    latest.publish(frame(4, True), clock.now_ns())
    received = latest.snapshot()
    assert not control._target_frame_is_current(received, control.control_generation)
    assert not control._target_context().authorized
    with pytest.raises(RuntimeError, match='gripper_command_requires_armed'):
        await control.command_gripper(1)
    assert backend.gripper_commands == []


async def test_cancellation_before_release_rpc_latches_unconfirmed_stop():
    control, latest, backend, clock = await active_control()
    recorder = RecordingRecorder()
    waiting = asyncio.Event()
    original = recorder.write_critical_event

    async def recording(kind, payload, now):
        await original(kind, payload, now)
        if kind == 'state_transition' and payload == {'from': 'ACTIVE', 'to': 'HOLD'}:
            waiting.set()
            await asyncio.Future()

    recorder.write_critical_event = recording
    control.recorder = recorder
    latest.publish(frame(3, False), clock.now_ns())
    latest.publish(frame(4, True), clock.now_ns())
    stopping_tick = asyncio.create_task(control.tick())
    await waiting.wait()
    stopping_tick.cancel()
    with pytest.raises(asyncio.CancelledError):
        await stopping_tick
    assert backend.stops == []
    state = await control.state_message()
    assert state.mode is TeleopMode.FAULT
    assert state.fault == 'stop_unverified'
    await control.tick()
    assert backend.targets == []
    assert backend.gripper_commands == []
    assert backend.stops == []  # No automatic retry after cancellation.


@pytest.mark.parametrize('coalesced', [False, True])
async def test_stop_wait_does_not_reinterpret_accepted_release_but_future_input_still_expires(coalesced):
    control, latest, backend, clock = await active_control()

    async def slow_stop(reason):
        backend.stops.append(reason)
        clock.advance_ms(350)

    backend.stop = slow_stop
    latest.publish(frame(3, False), clock.now_ns())
    if coalesced:
        latest.publish(frame(4, True), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    assert backend.stops == [StopReason.GRIP_RELEASED]
    assert control.last_seq == (2 if coalesced else 3)

    await control.tick()
    assert control.mode is TeleopMode.STALE
    assert backend.stops == [StopReason.GRIP_RELEASED, StopReason.STALE]
    assert backend.targets == []


@pytest.mark.parametrize('pursuit', [False, True])
async def test_transactional_fake_sdk_releases_before_coalesced_regrip(pursuit):
    from tests.control.test_transactional_vr_tick import rig

    async with rig() as r:
        await r.warm(pursuit=pursuit)
        before = r.control.coordinator.snapshot()
        request_count = len(r.control.requests)
        r.publish(grip=False, x=-0.2)
        r.publish(grip=True, x=-0.2)
        await r.control.tick()
        assert r.control.mode is TeleopMode.HOLD
        assert len(r.control.requests) == request_count
        assert not r.control.mapper.has_anchor
        assert any(call[0] == 'stop_move' for call in r.run.client.write_calls)

        await r.tick(x=-0.2)  # A fresh sample after the fake verified stop.
        assert r.control.mode is TeleopMode.ACTIVE
        after = r.control.coordinator.snapshot()
        assert after.version == before.version + 1
        assert after.linear_velocity == after.angular_velocity == (0.0, 0.0, 0.0)
        assert r.control.latest_requested is None
        assert r.control.pursuit is None
        assert len(r.control.requests) == request_count
