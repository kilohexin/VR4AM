"""Server arrival timing only; explicit fake backend, no SDK or robot I/O."""
import math

import pytest
from scipy.spatial.transform import Rotation

from app.control.safety import SafetyLimiter
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.target_strategy import DefaultTargetStrategy
from app.robots.base import BackendCommandError
from tests.control.test_robot_control import (
    FakeBackend, FakeClock, TargetSampleRecorder, connect_release_arm, frame, make_control,
)


class StepSpy(DefaultTargetStrategy):
    def __init__(self):
        self.steps = []

    def prepare(self, previous, requested, dt, *, limiter):
        self.steps.append(dt)
        return super().prepare(previous, requested, dt, limiter=limiter)


async def setup():
    recorder = TargetSampleRecorder()
    limiter = SafetyLimiter(max_linear_speed=.005, max_linear_accel=.02,
        max_angular_speed=.1, max_angular_accel=.2)
    latest, backend, clock = LatestVRFrame(), FakeBackend(), FakeClock()
    control = RobotControl(backend=backend, latest=latest, clock=clock, recorder=recorder,
        limiter=limiter, bounded_target_timing=True)
    control.mapper.set_translation_scale(.2)
    control.target_strategy = StepSpy()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    return control, latest, backend, clock, recorder


@pytest.mark.parametrize(('interval_ms', 'expected'), [(0, .02), (10, .02),
    (20, .02), (25, .025), (40, .04), (80, .04), (99, .04)])
async def test_filter_and_limiter_share_bounded_server_arrival_step(monkeypatch, interval_ms, expected):
    control, latest, backend, clock, recorder = await setup()
    filter_steps = []
    original = control.filter.update
    def filtered(pose, dt):
        filter_steps.append(dt)
        return original(pose, dt)
    monkeypatch.setattr(control.filter, 'update', filtered)
    clock.advance_ms(20)
    latest.publish(frame(3, True, p=(.03, 1.2, -.3), client_mono_ms=1e9), clock.now_ns())
    await control.tick()
    clock.advance_ms(interval_ms)
    latest.publish(frame(4, True, p=(.03, 1.2, -.3), client_mono_ms=1), clock.now_ns())
    # Force sampling in this unit case; production retains its 100 ms sampling cap.
    control._last_target_sample_ns = None
    await control.tick()
    assert control.target_strategy.steps == pytest.approx([.02, expected])
    assert filter_steps == pytest.approx([.02, expected])
    sample = recorder.samples[-1]
    assert sample['control_dt_s'] == pytest.approx(expected)
    assert sample['target_input_interval_ns'] == interval_ms * 1_000_000
    assert sample['control_dt_cap_s'] == .04
    assert len(backend.targets) == 2


async def trajectory(period_ms):
    control, latest, backend, clock, _recorder = await setup()
    anchor = control.last_target.model_copy(deep=True)
    sequence = 2
    for elapsed in range(20, 1001, 20):
        clock.advance_ms(20)
        if elapsed % period_ms == 0:
            sequence += 1
            latest.publish(frame(sequence, True, p=(.03, 1.2, -.3)), clock.now_ns())
        await control.tick()
        previous_count = len(backend.targets)
        await control.tick()  # Duplicate input cannot integrate a second step.
        assert len(backend.targets) == previous_count
    return math.dist(anchor.p, control.last_target.p), backend.targets


async def test_25hz_target_progress_is_close_to_50hz_without_duplicate_commands():
    at50, commands50 = await trajectory(20)
    at25, commands25 = await trajectory(40)
    assert len(commands50) == 50 and len(commands25) == 25
    assert .95 <= at25 / at50 <= 1.05, '25 Hz must not halve the wall-clock target progress'
    assert .004 < at50 < .005 and .004 < at25 < .005


@pytest.mark.parametrize('recording_delay', [False, True])
async def test_arrival_interval_cannot_credit_time_overlapping_previous_dispatch(recording_delay):
    control, latest, _backend, clock, recorder = await setup()
    clock.advance_ms(20)
    if recording_delay:
        original = recorder.write_event
        async def delayed(event, timestamp):
            await original(event, timestamp)
            clock.advance_ms(70)
        recorder.write_event = delayed
        latest.publish(frame(3, True, p=(.03, 1.2, -.3)), clock.now_ns())
        await control.tick()
        recorder.write_event = original
        clock.advance_ms(10)
        receipt = clock.now_ns() - 40_000_000
    else:
        # First input ages 60 ms; following input ages 40 ms. Arrival delta
        # is 40 ms although current dispatches are only 20 ms apart.
        latest.publish(frame(3, True, p=(.03, 1.2, -.3)), clock.now_ns() - 60_000_000)
        await control.tick()
        clock.advance_ms(20)
        receipt = clock.now_ns() - 40_000_000
    latest.publish(frame(4, True, p=(.03, 1.2, -.3)), receipt)
    await control.tick()
    assert control.target_strategy.steps == pytest.approx([.02, .02]), 'input cadence cannot spend overlapping dispatch time'


@pytest.mark.parametrize('motion', ['translation', 'rotation', 'combined'])
async def test_larger_step_still_respects_original_hard_caps(motion):
    control, latest, backend, clock, _recorder = await setup()
    control.limiter.max_linear_step_m = .000015
    control.limiter.max_angular_step_rad = .0003
    previous = control.last_target.model_copy(deep=True)
    for seq, gap in [(3, 20), (4, 80), (5, 40)]:
        clock.advance_ms(gap)
        position = (.03, 1.2, -.3) if motion != 'rotation' else (0, 1.2, -.3)
        incoming = frame(seq, True, p=position)
        if motion != 'translation':
            incoming.right.q = tuple(Rotation.from_euler('z', 10, degrees=True).as_quat())
        latest.publish(incoming, clock.now_ns())
        await control.tick()
        target = backend.targets[-1][1]
        assert math.dist(previous.p, target.p) <= .000015 + 1e-12
        angle = (Rotation.from_quat(target.q) * Rotation.from_quat(previous.q).inv()).magnitude()
        assert angle <= .0003 + 1e-12
        previous = target
    assert control.target_strategy.steps == pytest.approx([.02, .04, .04])


@pytest.mark.parametrize('boundary', ['release', 'disarm', 'disconnect'])
async def test_new_anchor_never_inherits_old_timing(boundary):
    control, latest, _backend, clock, recorder = await setup()
    for seq, gap in [(3, 20), (4, 40)]:
        clock.advance_ms(gap)
        latest.publish(frame(seq, True, p=(.03, 1.2, -.3)), clock.now_ns())
        await control.tick()
    if boundary == 'disarm':
        await control.disarm()
    elif boundary == 'disconnect':
        await control.on_disconnect()
        await control.connect()
    clock.advance_ms(40)
    latest.publish(frame(20, False, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    if boundary != 'release':
        await control.arm()
    latest.publish(frame(21, True, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    assert control._last_target_input_ns is control._last_target_step_ns is None
    clock.advance_ms(80)
    latest.publish(frame(22, True, p=(.04, 1.2, -.3)), clock.now_ns())
    control._last_target_sample_ns = None
    await control.tick()
    assert control.target_strategy.steps[-1] == .02
    assert recorder.samples[-1]['target_input_interval_ns'] is None
    assert recorder.samples[-1]['target_processing_interval_ns'] is None


async def test_soft_ik_rejection_consumes_time_without_advancing_target():
    control, latest, backend, clock, _recorder = await setup()
    clock.advance_ms(20)
    latest.publish(frame(3, True, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    previous = control.last_target.model_copy(deep=True)
    original = backend.command_tcp
    async def reject(*_args):
        raise BackendCommandError('ik_tracking_lag')
    backend.command_tcp = reject
    clock.advance_ms(80)
    latest.publish(frame(4, True, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    assert control.last_target == previous
    assert control._constraint == 'tracking_lag'
    backend.command_tcp = original
    clock.advance_ms(20)
    latest.publish(frame(5, True, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    assert control.target_strategy.steps == pytest.approx([.02, .04, .02])
    assert math.dist(previous.p, control.last_target.p) == pytest.approx(.000008)


async def test_workspace_hold_consumes_interval_without_a_target():
    control, latest, backend, clock, recorder = await setup()
    control.limiter.workspace_half_extent_m = .001
    clock.advance_ms(20)
    latest.publish(frame(3, True, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    assert backend.targets == []
    assert recorder.samples[-1]['workspace_hold']
    clock.advance_ms(20)
    latest.publish(frame(4, True, p=(.001, 1.2, -.3)), clock.now_ns())
    await control.tick()
    assert control.target_strategy.steps == [.02]
    assert len(backend.targets) == 1


@pytest.mark.parametrize('invalid', ['expired', 'future'])
async def test_invalid_input_still_stops_without_timing_credit(invalid):
    control, latest, backend, clock, _recorder = await setup()
    clock.advance_ms(20)
    latest.publish(frame(3, True, p=(.03, 1.2, -.3)), clock.now_ns())
    await control.tick()
    if invalid == 'expired':
        clock.advance_ms(100)
    else:
        latest.publish(frame(4, True, p=(.03, 1.2, -.3)), clock.now_ns() + 1)
    await control.tick()
    assert len(backend.targets) == 1
    assert backend.stops
    assert control._last_target_input_ns is control._last_target_step_ns is None


async def test_nonadvancing_server_arrival_never_increases_step():
    control, latest, _backend, clock, recorder = await setup()
    clock.advance_ms(20)
    previous_received = clock.now_ns()
    latest.publish(frame(3, True, p=(.03, 1.2, -.3)), previous_received)
    await control.tick()
    clock.advance_ms(40)
    latest.publish(frame(4, True, p=(.03, 1.2, -.3)), previous_received - 1_000_000)
    control._last_target_sample_ns = None
    await control.tick()
    assert control.target_strategy.steps == [.02, .02]
    assert control._last_target_input_ns == previous_received
    assert recorder.samples[-1]['target_input_interval_ns'] == -1_000_000


async def test_default_controller_requires_explicit_selection_for_larger_step():
    recorder = TargetSampleRecorder()
    control, latest, _backend, clock = make_control(recorder=recorder)
    control.target_strategy = StepSpy()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    for seq in [3, 4]:
        clock.advance_ms(40)
        latest.publish(frame(seq, True, p=(.03, 1.2, -.3)), clock.now_ns())
        await control.tick()
    assert control.target_strategy.steps == [.02, .02]
    assert recorder.samples[-1]['target_timing_policy'] == 'nominal20'
    assert recorder.samples[-1]['control_dt_cap_s'] == .02


@pytest.mark.parametrize('bad_option', [None, 1, 'true'])
def test_timing_selection_requires_boolean(bad_option):
    with pytest.raises(ValueError, match='^bounded_target_timing_must_be_bool$'):
        RobotControl(backend=FakeBackend(), latest=LatestVRFrame(), clock=FakeClock(),
            recorder=TargetSampleRecorder(), bounded_target_timing=bad_option)


@pytest.mark.parametrize('bounded', [False, True])
async def test_timing_pressure_comparison_keeps_joint_caps_and_records_backpressure(bounded):
    import asyncio
    import json
    from dataclasses import replace
    from collections import Counter
    from app.commissioning.low_speed_profile import LOW_SPEED_CAPS
    from app.control.coordinate_mapper import CoordinateMapper
    from app.recording.noop import NoopRecorder
    from app.schemas.messages import ControllerState, VRFrame
    from tests.robots.real_settings import control_settings
    from tests.robots.test_lebai_backpressure_experiments import Experiment, joints
    from tests.robots.test_vr_profile_experiments import limiter

    settings = control_settings()
    run = Experiment(replace(settings, control=replace(settings.control, **LOW_SPEED_CAPS)))
    async def synthetic_ik(pose, seed):
        run.client.ik_calls.append((dict(pose), list(seed)))
        return joints(50 * (pose['x'] - .3))
    run.client.kinematics_inverse = synthetic_ik
    latest = LatestVRFrame()
    control = RobotControl(backend=run.adapter, latest=latest, clock=run.clock,
        recorder=NoopRecorder(), mapper=CoordinateMapper(translation_scale=.2),
        limiter=limiter(), bounded_target_timing=bounded)
    def publish(seq, grip, x):
        latest.publish(VRFrame(v=1, type='vr_frame', session_id='timing-pressure-fake',
            seq=seq, client_mono_ms=run.clock.now_ns()/1e6, tracking_valid=True, visibility='visible',
            right=ControllerState(p=(x, 0, 0), q=(0, 0, 0, 1), grip=grip, trigger=0)), run.clock.now_ns())
    counts = Counter()
    try:
        await run.adapter.connect()
        await control.connect()
        publish(1, False, 0)
        await control.tick()
        await control.arm()
        publish(2, True, 0)
        await control.tick()
        for step in range(1, 101):
            run.clock.advance(.02)
            await control.tick()
            assert run.sent.empty()
            run.clock.advance(.02)
            publish(step + 2, True, .03 * min(step/25, 1))
            await control.tick()
            event = await asyncio.wait_for(run.sent.get(), 1)
            counts[event['pvat_mode']] += 1
            assert max(abs(v) for v in event['v']) <= .05 + 1e-12
            assert max(abs(a) for a in event['a']) <= .2 + 1e-12
            assert run.adapter.pump_fault is None
            run.integrate_feedback(event, .25)
        assert counts['catch_up'] == (59 if bounded else 18)
        assert sum(counts.values()) == 100
        assert all(c[0] == 'move_pvat' for c in run.client.write_calls)
        print('TIMING_PRESSURE_RESULT ' + json.dumps(dict(bounded=bounded, counts=dict(counts),
            duration_s=4., accepted_target_progress_mm=(event['target_tcp']['p'][0]-.3)*1000,
            hardware_verified=False, model='affine_fake_fixed_tcp_not_physical')))
    finally:
        run.client.kin_data['actual_joint_speed'] = [0.] * 6
        await run.adapter.disconnect()
        run.factory.assert_awaited_once()
        assert run.client.network_calls == run.client.discovery_calls == 0
