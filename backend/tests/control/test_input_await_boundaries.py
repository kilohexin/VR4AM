"""In-memory input/authority changes at VR activation await boundaries."""

import asyncio

import pytest

from app.recording.commissioning import RecorderUnavailable
from app.recording.noop import NoopRecorder
from app.schemas.messages import TeleopMode
from app.robots.base import StopReason
from tests.control.test_robot_control import connect_release_arm, frame, make_control


STAGES = ['vr_record', 'preflight', 'preflight_record', 'anchor_read', 'activation_record']


@pytest.mark.parametrize('stage', STAGES)
@pytest.mark.parametrize('change', ['replaced', 'expired', 'disarmed', 'rearmed'])
async def test_activation_wait_cannot_apply_invalidated_input(stage, change):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, False, trigger=0), clock.now_ns())
    await control.tick()
    if stage in {'preflight', 'preflight_record'}:
        latest.publish(frame(3, True), clock.now_ns())
        await control.tick()
        latest.publish(frame(4, False), clock.now_ns())
        await control.tick()
        assert control.mode is TeleopMode.HOLD
    expected_seq = control.last_seq
    captures = []
    capture = control.mapper.capture

    def measured_capture(*args):
        captures.append(clock.now_ns())
        return capture(*args)

    control.mapper.capture = measured_capture
    fired = False
    captures_at_change = None

    async def intervene():
        nonlocal fired, expected_seq, captures_at_change
        if fired:
            return
        fired = True
        captures_at_change = len(captures)
        if change == 'replaced':
            latest.publish(frame(11, True, p=(0.1, 1.2, -0.3)), clock.now_ns())
        elif change == 'expired':
            clock.advance_ms(100)
        else:
            await control.disarm()
            if change == 'rearmed':
                latest.publish(frame(11, False), clock.now_ns())
                await control.tick()
                await control.arm()
                expected_seq = control.last_seq

    class Recorder(NoopRecorder):
        async def write_vr_frame(self, message, ns):
            if stage == 'vr_record':
                await intervene()

        async def write_critical_event(self, kind, payload, ns):
            if (stage == 'preflight_record' and kind == 'preflight_result') or (
                stage == 'activation_record' and kind == 'state_transition'
                and payload['to'] == 'ACTIVE'
            ):
                await intervene()

    original_preflight, original_state = backend.preflight, backend.get_state

    async def preflight():
        result = await original_preflight()
        if stage == 'preflight':
            await intervene()
        return result

    async def state():
        result = await original_state()
        if stage == 'anchor_read':
            await intervene()
        return result

    backend.preflight, backend.get_state = preflight, state
    control.recorder = Recorder()
    latest.publish(frame(10, True, trigger=0.5), clock.now_ns())
    await control.tick()
    assert fired
    assert len(captures) == captures_at_change
    assert control.last_seq == expected_seq
    assert not backend.targets
    if stage == 'vr_record':
        assert not backend.gripper_commands
    if change in {'disarmed', 'rearmed'}:
        assert control.mode is (TeleopMode.DISARMED if change == 'disarmed' else TeleopMode.ARMED)
        assert not control.mapper.has_anchor
        assert control.last_target is None
    else:
        # ACTIVE is only allowed if a valid anchor was established before the
        # transition recorder yielded; never leave ACTIVE without an anchor.
        if control.mode is TeleopMode.ACTIVE:
            assert control.mapper.has_anchor and control.last_target is not None
        if change == 'expired':
            await control.tick()
            assert control.mode is TeleopMode.STALE
        else:
            await control.tick()
            assert control.mode is TeleopMode.ACTIVE
            assert control.mapper.has_anchor
            assert control.last_seq == 11


@pytest.mark.parametrize('stage', ['preflight', 'preflight_record'])
@pytest.mark.parametrize('rearm', [False, True])
async def test_explicit_claw_command_rechecks_authority_before_write(stage, rearm):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    fired = False

    async def revoke():
        nonlocal fired
        if not fired:
            fired = True
            await control.disarm()
            if rearm:
                latest.publish(frame(20, False), clock.now_ns())
                await control.tick()
                await control.arm()

    original = backend.preflight

    async def preflight():
        result = await original()
        if stage == 'preflight':
            await revoke()
        return result

    class Recorder(NoopRecorder):
        async def write_critical_event(self, kind, payload, ns):
            if stage == 'preflight_record' and kind == 'preflight_result':
                await revoke()

    backend.preflight = preflight
    control.recorder = Recorder()
    with pytest.raises(RuntimeError, match='gripper_command_authority_changed'):
        await control.command_gripper(0.5)
    assert not backend.gripper_commands
    assert control.mode is (TeleopMode.ARMED if rearm else TeleopMode.DISARMED)


@pytest.mark.parametrize('stage', ['vr_record', 'activation_record'])
async def test_recording_failure_remains_a_fault(stage):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)

    class Broken(NoopRecorder):
        async def write_vr_frame(self, message, ns):
            if stage == 'vr_record':
                raise RecorderUnavailable('offline')

        async def write_critical_event(self, kind, payload, ns):
            if stage == 'activation_record' and kind == 'state_transition' and payload['to'] == 'ACTIVE':
                raise RecorderUnavailable('offline')

    control.recorder = Broken()
    latest.publish(frame(10, True), clock.now_ns())
    await control.tick()
    assert control._fault == 'recording_unavailable'
    assert control.mode is TeleopMode.FAULT
    assert not backend.targets
    assert backend.stops


@pytest.mark.parametrize('hold', [False, True])
async def test_anchor_read_does_not_publish_active_without_an_anchor(hold):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    if hold:
        latest.publish(frame(2, True), clock.now_ns())
        await control.tick()
        latest.publish(frame(3, False), clock.now_ns())
        await control.tick()
    entered, release = asyncio.Event(), asyncio.Event()
    original = backend.get_state

    async def blocked_state():
        entered.set()
        await release.wait()
        return await original()

    backend.get_state = blocked_state
    latest.publish(frame(10, True), clock.now_ns())
    task = asyncio.create_task(control.tick())
    try:
        await asyncio.wait_for(entered.wait(), 1)
        assert control.mode is (TeleopMode.HOLD if hold else TeleopMode.ARMED)
        assert not control.mapper.has_anchor
        release.set()
        await asyncio.wait_for(task, 1)
        assert control.mode is TeleopMode.ACTIVE
        assert control.mapper.has_anchor and control.last_target is not None
    finally:
        release.set()
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('expired', [False, True])
async def test_grip_release_stop_is_not_skipped_after_transition_record(expired):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()

    class Recorder(NoopRecorder):
        async def write_critical_event(self, kind, payload, ns):
            if kind == 'state_transition' and payload['to'] == 'HOLD':
                if expired:
                    clock.advance_ms(100)
                else:
                    latest.publish(frame(11, True), clock.now_ns())

    control.recorder = Recorder()
    latest.publish(frame(10, False), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    assert backend.stops == [StopReason.GRIP_RELEASED]
    assert not control.mapper.has_anchor
    assert not backend.targets


async def test_accumulated_hold_resume_waits_expire_the_frame():
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, False), clock.now_ns())
    await control.tick()
    preflight, state = backend.preflight, backend.get_state

    async def delayed_preflight():
        clock.advance_ms(40)
        return await preflight()

    async def delayed_state():
        clock.advance_ms(40)
        return await state()

    class Recorder(NoopRecorder):
        async def write_critical_event(self, kind, payload, ns):
            if kind == 'preflight_result':
                clock.advance_ms(40)

    backend.preflight, backend.get_state = delayed_preflight, delayed_state
    control.recorder = Recorder()
    latest.publish(frame(10, True), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    assert not control.mapper.has_anchor
    assert not backend.targets
    assert control.last_seq == 3
    await control.tick()
    assert control.mode is TeleopMode.STALE


@pytest.mark.parametrize('stage', ['preflight', 'preflight_record', 'arm_record'])
@pytest.mark.parametrize('change', ['new_generation', 'fault', 'shutdown', 'loop_failed'])
async def test_arm_request_cannot_survive_authority_change(stage, change):
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    fired = False

    async def intervene():
        nonlocal fired
        if fired:
            return
        fired = True
        if change == 'new_generation':
            await control.disarm()
            latest.publish(frame(2, False), clock.now_ns())
            await control.tick()
            assert control.mode is TeleopMode.READY
        elif change == 'fault':
            control._fault = 'offline_fault'
        elif change == 'shutdown':
            control._shutdown_started = True
        else:
            control._loop_failed = True

    original = backend.preflight

    async def preflight():
        result = await original()
        if stage == 'preflight':
            await intervene()
        return result

    class Recorder(NoopRecorder):
        async def write_critical_event(self, kind, payload, ns):
            if (stage == 'preflight_record' and kind == 'preflight_result') or (
                stage == 'arm_record' and kind == 'state_transition' and payload['to'] == 'ARMED'
            ):
                await intervene()

    backend.preflight = preflight
    control.recorder = Recorder()
    with pytest.raises(RuntimeError, match='arm_authority_changed'):
        await control.arm()
    assert fired
    assert control.mode is not TeleopMode.ARMED
    assert not backend.targets and not backend.gripper_commands
