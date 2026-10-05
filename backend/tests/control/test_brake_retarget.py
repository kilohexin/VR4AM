"""Continuous-input experiments in command space, never physical stop tests."""

import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose, TeleopMode
from app.robots.base import StopReason
from tests.control.brake_retarget_probe import BrakeBeforeRetarget
from tests.control.braking_approach_probe import InfeasibleApproach
from tests.control.test_braking_approach import ORIGIN, pose_delta
from tests.control.test_braking_supervision import active_control, warm
from tests.control.elapsed_step_probe import publish
from tests.robots.test_vr_profile_experiments import limiter


PATTERNS = [(0.02,), (0.04,), (0.02, 0.06), (0.02, 0.08)]


def measured_step(candidate, previous, target, dt):
    old_v, old_w = candidate.linear_velocity.copy(), candidate.angular_velocity.copy()
    result = candidate.limit_motion(previous, target, dt)
    v = (np.asarray(result.p) - previous.p) / dt
    w = (Rotation.from_quat(result.q) * Rotation.from_quat(previous.q).inv()).as_rotvec() / dt
    assert np.linalg.norm(v) <= 0.005 + 1e-10
    assert np.linalg.norm(w) <= 0.05 + 1e-10
    assert np.linalg.norm(v - old_v) <= 0.02 * dt + 1e-10
    assert np.linalg.norm(w - old_w) <= 0.1 * dt + 1e-10
    assert np.linalg.norm(v) * dt <= 0.0005 + 1e-12
    assert np.linalg.norm(w) * dt <= math.radians(0.2) + 1e-12
    assert v == pytest.approx(candidate.linear_velocity, abs=1e-11)
    assert w == pytest.approx(candidate.angular_velocity, abs=1e-11)
    return result


def warm_candidate():
    candidate, previous = BrakeBeforeRetarget(limiter()), ORIGIN
    target = pose_delta(ORIGIN, 0.1, 0.5)
    for _ in range(30):
        previous = measured_step(candidate, previous, target, 0.02)
    return candidate, previous


@pytest.mark.parametrize('pattern', PATTERNS)
@pytest.mark.parametrize('path', ['reverse', 'near', 'sideways', 'changing_during_brake'])
def test_decelerates_before_latest_target_and_eventually_arrives(pattern, path):
    candidate, previous = warm_candidate()
    start = previous
    target = pose_delta(start, -0.003, -0.05)
    if path == 'near':
        target = pose_delta(start, 0.00001, 0.0001)
    if path == 'sideways':
        target = Pose(p=(start.p[0], 0.003, start.p[2]),
                      q=tuple((Rotation.from_rotvec((0.05, 0, 0)) * Rotation.from_quat(start.q)).as_quat()))
    elapsed, brake_time, drift, angular_drift = 0., None, 0., 0.
    zero_seen = False
    for index in range(400):
        dt = pattern[index % len(pattern)]
        elapsed += dt
        if path == 'changing_during_brake' and index == 2:
            target = pose_delta(start, 0.004, 0.06)
        result = measured_step(candidate, previous, target, dt)
        if not zero_seen:
            drift = max(drift, float(np.linalg.norm(np.asarray(result.p) - start.p)))
            angular_drift = max(angular_drift, (Rotation.from_quat(result.q) * Rotation.from_quat(start.q).inv()).magnitude())
            if np.linalg.norm(candidate.linear_velocity) == np.linalg.norm(candidate.angular_velocity) == 0:
                zero_seen, brake_time = True, elapsed
            else:
                # Both channels remain on their OLD rays until fully at rest,
                # even if a target changes again during the braking episode.
                assert candidate.linear_velocity[0] >= 0
                assert candidate.linear_velocity[1:] == pytest.approx([0, 0])
                assert candidate.angular_velocity[:2] == pytest.approx([0, 0])
                assert candidate.angular_velocity[2] >= 0
        previous = result
    assert zero_seen
    assert brake_time <= 0.5 + max(pattern) + 1e-10
    assert drift <= 0.005 ** 2 / (2 * 0.02) + 1e-12
    assert angular_drift <= 0.05 ** 2 / (2 * 0.1) + 1e-12
    assert previous.p == pytest.approx(target.p, abs=1e-10)
    assert (Rotation.from_quat(previous.q) * Rotation.from_quat(target.q).inv()).magnitude() < 1e-10
    assert candidate.brake_episodes == 1
    print('RETARGET_RESULT ' + json.dumps(dict(path=path, pattern_ms=[int(t * 1000) for t in pattern],
        brake_time_s=brake_time, old_direction_drift_mm=drift * 1000,
        old_direction_rotation_deg=math.degrees(angular_drift), brake_episodes=candidate.brake_episodes)))


@pytest.mark.parametrize('dt', [0, -0.01, 0.019, 0.081, float('nan'), float('inf')])
def test_invalid_interval_during_braking_preserves_state(dt):
    candidate, previous = warm_candidate()
    target = pose_delta(previous, -0.003, -0.05)
    previous = candidate.limit_motion(previous, target, 0.02)
    before = candidate.linear_velocity.copy(), candidate.angular_velocity.copy(), candidate.braking
    with pytest.raises(ValueError, match='interval'):
        candidate.limit_motion(previous, target, dt)
    assert np.array_equal(candidate.linear_velocity, before[0])
    assert np.array_equal(candidate.angular_velocity, before[1])
    assert candidate.braking is before[2]


@pytest.mark.parametrize('boundary', ['translation', 'rotation'])
def test_braking_step_cannot_cross_workspace_and_failure_is_atomic(boundary):
    candidate, previous = warm_candidate()
    candidate.limits.set_pose_anchor(previous)
    if boundary == 'translation':
        candidate.limits.workspace_half_extent_m = 0.000001
    else:
        candidate.limits.max_rotation_from_anchor_rad = 0.000001
    before_v, before_w = candidate.linear_velocity.copy(), candidate.angular_velocity.copy()
    with pytest.raises(InfeasibleApproach, match='generated_step_outside_workspace'):
        candidate.limit_motion(previous, pose_delta(previous, -0.003, -0.05), 0.02)
    assert np.array_equal(candidate.linear_velocity, before_v)
    assert np.array_equal(candidate.angular_velocity, before_w)
    assert not candidate.braking
    assert candidate.brake_episodes == 0


def test_reset_discards_braking_epoch_only_when_called_by_owner():
    candidate, previous = warm_candidate()
    target = pose_delta(previous, -0.003, -0.05)
    previous = candidate.limit_motion(previous, target, 0.02)
    assert candidate.braking
    candidate.reset_motion()
    assert not candidate.braking
    result = measured_step(candidate, previous, target, 0.02)
    assert result.p[0] < previous.p[0]


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_original_control_filter_handles_reversal_and_release(channel):
    probe = await active_control()
    control, latest, backend, clock, bridge, _ = probe
    bridge.candidate = BrakeBeforeRetarget(bridge.limits)
    await warm(probe, channel)
    for seq in range(23, 223):
        clock.advance_ms(20)
        publish(latest, clock, seq, True, -0.02 if channel == 'linear' else 0,
                -0.05 if channel == 'angular' else 0)
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE
    assert bridge.candidate.brake_episodes >= 1
    assert not bridge.rejections
    assert not backend.stops
    clock.advance_ms(20)
    publish(latest, clock, 223, False, 0, 0)
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    assert len(backend.stops) == 1


@pytest.mark.parametrize('pattern', PATTERNS)
@pytest.mark.parametrize('path', ['smooth_wave', 'slow_wave', 'corner', 'jitter', 'circle', 'slow_circle'])
def test_continuous_input_reports_follow_error_and_settles(pattern, path):
    candidate = BrakeBeforeRetarget(limiter())
    initial = Pose(p=ORIGIN.p, q=tuple(Rotation.from_euler('xyz', (0.2, -0.3, 0.1)).as_quat()))
    previous = initial
    elapsed = brake_time = peak_error = peak_angle = squared_error = moving_time = 0.
    target = initial
    index = 0
    moving_episodes = 0
    while elapsed < 10:
        dt = pattern[index % len(pattern)]
        elapsed += dt
        if elapsed <= 4:
            frequency = 0.2 if path.startswith('slow_') else 1.0
            phase = 2 * math.pi * frequency * elapsed
            if path in {'smooth_wave', 'slow_wave'}:
                delta = np.asarray((0.002 * math.sin(phase), 0, 0))
            elif path in {'circle', 'slow_circle'}:
                delta = np.asarray((0.002 * math.sin(phase), 0.002 * (1 - math.cos(phase)), 0))
            elif path == 'corner':
                delta = np.asarray((0.002, 0, 0) if int(elapsed * 2) % 2 == 0 else (0, 0.002, 0))
            else:
                delta = np.asarray((0.0001 * (-1) ** index, 0, 0))
            target = Pose(p=tuple(np.asarray(initial.p) + delta),
                          q=tuple((Rotation.from_rotvec(delta * 10) * Rotation.from_quat(initial.q)).as_quat()))
        else:
            target = initial
        was_braking = candidate.braking
        previous = measured_step(candidate, previous, target, dt)
        if elapsed <= 4:
            error = float(np.linalg.norm(np.asarray(previous.p) - target.p))
            angle = (Rotation.from_quat(previous.q) * Rotation.from_quat(target.q).inv()).magnitude()
            peak_error, peak_angle = max(peak_error, error), max(peak_angle, angle)
            squared_error += error ** 2 * dt
            moving_time += dt
            brake_time += dt * (was_braking or candidate.braking)
            moving_episodes = candidate.brake_episodes
        index += 1
    assert previous.p == pytest.approx(initial.p, abs=1e-10)
    assert (Rotation.from_quat(previous.q) * Rotation.from_quat(initial.q).inv()).magnitude() < 1e-10
    assert np.linalg.norm(candidate.linear_velocity) < 1e-12
    assert np.linalg.norm(candidate.angular_velocity) < 1e-12
    # Metrics are observations, not a claim these tracking errors are acceptable.
    print('CONTINUOUS_RESULT ' + json.dumps(dict(path=path,
        pattern_ms=[int(t * 1000) for t in pattern], total_brake_episodes=candidate.brake_episodes,
        moving_brake_episodes=moving_episodes,
        measured_moving_window_s=moving_time, brake_fraction=brake_time / moving_time,
        input_frequency_hz=0.2 if path.startswith('slow_') else 1.0 if path in {'smooth_wave', 'circle'} else None,
        peak_follow_error_mm=peak_error * 1000,
        rms_follow_error_mm=math.sqrt(squared_error / moving_time) * 1000,
        peak_orientation_error_deg=math.degrees(peak_angle))))


@pytest.mark.parametrize('channel', ['linear', 'angular'])
@pytest.mark.parametrize('interrupt', ['release', 'stale'])
async def test_existing_stop_preempts_candidate_mid_brake(channel, interrupt):
    probe = await active_control()
    control, latest, backend, clock, bridge, _ = probe
    bridge.candidate = BrakeBeforeRetarget(bridge.limits)
    await warm(probe, channel)
    clock.advance_ms(20)
    publish(latest, clock, 23, True, -0.1 if channel == 'linear' else 0,
            -0.4 if channel == 'angular' else 0)
    await control.tick()
    assert bridge.candidate.braking
    count = len(backend.targets)
    if interrupt == 'release':
        clock.advance_ms(20)
        publish(latest, clock, 24, False, 0, 0)
    else:
        clock.advance_ms(100)
    await control.tick()
    assert len(backend.targets) == count
    assert backend.stops == [StopReason.GRIP_RELEASED if interrupt == 'release' else StopReason.STALE]
    assert control.mode is (TeleopMode.HOLD if interrupt == 'release' else TeleopMode.STALE)
    if interrupt == 'release':
        clock.advance_ms(20)
        publish(latest, clock, 25, True, 0, 0)
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE
        assert not bridge.candidate.braking
        assert np.linalg.norm(bridge.candidate.linear_velocity) == 0
        assert np.linalg.norm(bridge.candidate.angular_velocity) == 0
        assert len(backend.targets) == count


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_generated_boundary_rejection_still_uses_fault_stop(channel):
    probe = await active_control()
    control, latest, backend, clock, bridge, _ = probe
    bridge.candidate = BrakeBeforeRetarget(bridge.limits)
    await warm(probe, channel)
    # Tiny sphere around the LAST generated pose. The new request is projected
    # inside, but continuing the previous velocity would leave that sphere.
    bridge.limits.set_pose_anchor(control.last_target)
    bridge.limits.workspace_radius = 0.000001
    if channel == 'angular':
        bridge.limits.max_rotation_from_anchor_rad = 0.000001
        bridge.limits.workspace_boundary_mode = 'axis_clamp'
    count = len(backend.targets)
    clock.advance_ms(20)
    publish(latest, clock, 23, True, -0.1 if channel == 'linear' else 0,
            -0.4 if channel == 'angular' else 0)
    await control.tick()
    assert bridge.rejections == ['generated_step_outside_workspace']
    assert len(backend.targets) == count
    assert backend.stops == [StopReason.FAULT]
    assert control.mode is TeleopMode.FAULT
