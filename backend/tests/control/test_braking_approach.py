"""Bounded, test-only approach policy; no robot or default limiter changes."""

import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from tests.control.braking_approach_probe import BrakingApproach, InfeasibleApproach
from tests.robots.test_vr_profile_experiments import limiter


ORIGIN = Pose(p=(0.3, 0, 0.3), q=(0, 0, 0, 1))


def pose_delta(previous, dx=0.0, angle=0.0, axis=(0, 0, 1)):
    return Pose(p=(previous.p[0] + dx, previous.p[1], previous.p[2]),
                q=tuple((Rotation.from_rotvec(np.asarray(axis) * angle)
                         * Rotation.from_quat(previous.q)).as_quat()))


@pytest.mark.parametrize('pattern', [(0.02,), (0.04,), (0.02, 0.06), (0.06, 0.02)])
@pytest.mark.parametrize('direction', [1, -1])
def test_static_target_arrives_without_snap_or_overshoot(pattern, direction):
    candidate = BrakingApproach(limiter())
    previous = ORIGIN
    target = pose_delta(ORIGIN, direction * 0.006, direction * 0.1)
    prior_v, prior_w = np.zeros(3), np.zeros(3)
    for index in range(500):
        dt = pattern[index % len(pattern)]
        result = candidate.limit_motion(previous, target, dt)
        velocity = (np.asarray(result.p) - previous.p) / dt
        omega = (Rotation.from_quat(result.q) * Rotation.from_quat(previous.q).inv()).as_rotvec() / dt
        assert np.linalg.norm(velocity) <= 0.005 + 1e-10
        assert np.linalg.norm(omega) <= 0.05 + 1e-10
        assert np.linalg.norm(velocity - prior_v) <= 0.02 * dt + 1e-10
        assert np.linalg.norm(omega - prior_w) <= 0.1 * dt + 1e-10
        assert np.linalg.norm(np.asarray(result.p) - previous.p) <= 0.0005 + 1e-12
        assert np.linalg.norm(omega) * dt <= math.radians(0.2) + 1e-12
        assert -1e-12 <= direction * (result.p[0] - ORIGIN.p[0]) <= 0.006 + 1e-12
        assert (Rotation.from_quat(result.q) * Rotation.from_quat(ORIGIN.q).inv()).magnitude() <= 0.1 + 1e-12
        previous, prior_v, prior_w = result, velocity, omega
    assert previous.p == pytest.approx(target.p, abs=1e-12)
    assert (Rotation.from_quat(previous.q) * Rotation.from_quat(target.q).inv()).magnitude() < 1e-12
    assert np.linalg.norm(candidate.linear_velocity) < 1e-12
    assert np.linalg.norm(candidate.angular_velocity) < 1e-12


def warm_candidate():
    candidate = BrakingApproach(limiter())
    previous = ORIGIN
    far = pose_delta(ORIGIN, 0.1, 0.5)
    for _ in range(30):
        previous = candidate.limit_motion(previous, far, 0.02)
    assert candidate.linear_velocity[0] == pytest.approx(0.005)
    assert candidate.angular_velocity[2] == pytest.approx(0.05)
    return candidate, previous, far


@pytest.mark.parametrize('case', ['near', 'reverse', 'sideways', 'angular_only_conflict'])
def test_unreachable_target_is_explicit_and_does_not_mutate_state(case):
    candidate, previous, far = warm_candidate()
    before_v, before_w = candidate.linear_velocity.copy(), candidate.angular_velocity.copy()
    if case == 'near':
        target = pose_delta(previous, 0.00001, 0.0001)
        # A one-step impossibility certificate for this collinear example:
        # even maximum deceleration still needs more distance than is left.
        assert (before_v[0] - 0.02 * 0.02) * 0.02 > 0.00001
        assert (before_w[2] - 0.1 * 0.02) * 0.02 > 0.0001
    elif case == 'reverse':
        target = pose_delta(previous, -0.001, -0.01)
    elif case == 'sideways':
        target = Pose(p=(previous.p[0], 0.01, 0.3), q=far.q)
    else:
        target = pose_delta(previous, 0.1, 0.0001)
    with pytest.raises(InfeasibleApproach):
        candidate.limit_motion(previous, target, 0.02)
    assert np.array_equal(candidate.linear_velocity, before_v)
    assert np.array_equal(candidate.angular_velocity, before_w)
    # This isn't a stop implementation: rejection leaves the prior velocity.


@pytest.mark.parametrize('dt', [0, -0.01, 0.019, 0.099, float('nan'), float('inf')])
def test_invalid_or_unmodelled_interval_does_not_mutate_candidate(dt):
    candidate, previous, far = warm_candidate()
    before = candidate.linear_velocity.copy()
    with pytest.raises(ValueError, match='interval'):
        candidate.limit_motion(previous, far, dt)
    assert np.array_equal(candidate.linear_velocity, before)


def test_reset_starts_a_new_zero_velocity_epoch():
    candidate, previous, _ = warm_candidate()
    candidate.reset_motion()
    target = pose_delta(previous, -0.001, -0.01)
    result = candidate.limit_motion(previous, target, 0.02)
    assert np.linalg.norm(np.asarray(result.p) - previous.p) <= 0.02 * 0.02 ** 2 + 1e-12
    assert np.linalg.norm(candidate.angular_velocity) <= 0.1 * 0.02 + 1e-12


@pytest.mark.parametrize('pattern', [(0.02,), (0.04,), (0.02, 0.06), (0.06, 0.02)])
@pytest.mark.parametrize('path', ['hold', 'reversal', 'sudden_near'])
@pytest.mark.parametrize('policy', ['legacy_fixed20', 'legacy_elapsed', 'braking_elapsed'])
def test_compare_command_space_progress_and_feasibility(pattern, path, policy):
    limits = limiter() if policy != 'braking_elapsed' else BrakingApproach(limiter())
    previous = ORIGIN
    prior_v, prior_w = np.zeros(3), np.zeros(3)
    elapsed = peak_a = peak_alpha = peak_v = peak_w = 0.0
    infeasible_at = None
    overshoots = 0
    count = 0
    near_target = None
    for index in range(1000):
        interval = pattern[index % len(pattern)]
        elapsed += interval
        if elapsed > 4 + 1e-9:
            break
        if path == 'sudden_near':
            if elapsed >= 1 and near_target is None:
                # State-relative adversarial input, not the same absolute
                # trajectory across policies; hold it fixed after the jump.
                near_target = pose_delta(previous, 0.00001, 0.0001)
            target = pose_delta(ORIGIN, 0.1, 0.5) if near_target is None else near_target
        else:
            ramp = min(elapsed, 1) if path == 'hold' else min(elapsed, 1) if elapsed <= 1 else max(2 - elapsed, 0)
            target = pose_delta(ORIGIN, 0.006 * ramp, 0.1 * ramp)
        dt = 0.02 if policy == 'legacy_fixed20' else interval
        try:
            result = limits.limit_motion(previous, target, dt)
        except InfeasibleApproach:
            infeasible_at = elapsed
            break  # Never retry or invent an automatic fallback.
        count += 1
        velocity = (np.asarray(result.p) - previous.p) / interval
        omega = (Rotation.from_quat(result.q) * Rotation.from_quat(previous.q).inv()).as_rotvec() / interval
        peak_a = max(peak_a, float(np.linalg.norm(velocity - prior_v) / interval))
        peak_alpha = max(peak_alpha, float(np.linalg.norm(omega - prior_w) / interval))
        peak_v = max(peak_v, float(np.linalg.norm(velocity)))
        peak_w = max(peak_w, float(np.linalg.norm(omega)))
        low, high = sorted((previous.p[0], target.p[0]))
        overshoots += int(result.p[0] < low - 1e-12 or result.p[0] > high + 1e-12)
        previous, prior_v, prior_w = result, velocity, omega
    if policy == 'braking_elapsed':
        assert peak_a <= 0.02 + 1e-10
        assert peak_alpha <= 0.1 + 1e-10
        assert overshoots == 0
        if path == 'sudden_near':
            assert infeasible_at is not None
        if path == 'hold':
            assert infeasible_at is None
            assert previous.p == pytest.approx(target.p, abs=1e-12)
            assert (Rotation.from_quat(previous.q) * Rotation.from_quat(target.q).inv()).magnitude() < 1e-12
            assert np.linalg.norm(limits.linear_velocity) < 1e-12
            assert np.linalg.norm(limits.angular_velocity) < 1e-12
    assert peak_v <= 0.005 + 1e-10
    assert peak_w <= 0.05 + 1e-10
    print('APPROACH_RESULT ' + json.dumps(dict(
        policy=policy, pattern_ms=[round(t * 1000) for t in pattern], path=path,
        processed=count, infeasible_at_s=infeasible_at,
        completed_window=infeasible_at is None,
        processed_final_mm=(previous.p[0] - 0.3) * 1000,
        peak_speed_mmps=peak_v * 1000, peak_angular_speed_radps=peak_w,
        peak_accel_mmps2=peak_a * 1000, peak_angular_accel_radps2=peak_alpha,
        steps_outside_previous_target_segment=overshoots,
    ), sort_keys=True))


@pytest.mark.parametrize('axis', [(1, 1, 1), (-1, 2, 1), (0, 1, 0)])
def test_diagonal_translation_and_nonidentity_orientation_keep_vector_budgets(axis):
    candidate = BrakingApproach(limiter())
    direction = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    initial = Pose(p=ORIGIN.p, q=tuple(Rotation.from_euler('xyz', (0.2, -0.3, 0.1)).as_quat()))
    target = Pose(p=tuple(np.asarray(initial.p) + direction * 0.006),
                  q=tuple((Rotation.from_rotvec(direction * 0.1) * Rotation.from_quat(initial.q)).as_quat()))
    previous = initial
    v, w = np.zeros(3), np.zeros(3)
    for index in range(200):
        dt = (0.02, 0.06)[index % 2]
        result = candidate.limit_motion(previous, target, dt)
        next_v = (np.asarray(result.p) - previous.p) / dt
        next_w = (Rotation.from_quat(result.q) * Rotation.from_quat(previous.q).inv()).as_rotvec() / dt
        assert np.linalg.norm(next_v - v) <= 0.02 * dt + 1e-10
        assert np.linalg.norm(next_w - w) <= 0.1 * dt + 1e-10
        assert np.linalg.norm(np.cross(np.asarray(result.p) - initial.p, direction)) < 1e-10
        progress = float(np.dot(np.asarray(result.p) - initial.p, direction))
        assert -1e-12 <= progress <= 0.006 + 1e-12
        previous, v, w = result, next_v, next_w
    assert previous.p == pytest.approx(target.p, abs=1e-12)
    assert (Rotation.from_quat(previous.q) * Rotation.from_quat(target.q).inv()).magnitude() < 1e-12


def test_infeasibility_neither_stops_nor_latches_the_candidate():
    candidate, previous, far = warm_candidate()
    target = pose_delta(previous, -0.001, -0.01)
    for _ in range(3):
        with pytest.raises(InfeasibleApproach):
            candidate.limit_motion(previous, target, 0.02)
    assert candidate.linear_velocity[0] == pytest.approx(0.005)
    continued = candidate.limit_motion(previous, far, 0.02)
    assert continued.p[0] > previous.p[0]  # No fault latch is implemented here.
    # A test reset represents a NEW stopped/reanchored epoch only, not an
    # automatic recovery of a moving robot or a substitute for stop evidence.
    candidate.reset_motion()
    candidate.limit_motion(continued, target, 0.02)
    assert candidate.linear_velocity[0] < 0
