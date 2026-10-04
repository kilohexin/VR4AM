"""Test-only elapsed-step candidates. Never installed in production control."""

import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.robots.base import StopReason
from app.schemas.messages import Pose, TeleopMode
from tests.control.elapsed_step_probe import StepPolicy, make_probe, publish
from tests.robots.test_vr_profile_experiments import limiter


@pytest.mark.parametrize('cap_ms', [40, 80])
def test_candidate_discards_excess_and_resets_anchor(cap_ms):
    policy = StepPolicy(cap_ms)
    policy.reset(1000_000_000)
    assert policy.consume(1099_000_000) == pytest.approx(cap_ms / 1000)
    assert policy.consume(1119_000_000) == pytest.approx(0.02)
    policy.reset(2000_000_000)
    assert policy.consume(2020_000_000) == pytest.approx(0.02)


@pytest.mark.parametrize('delta_ns', [0, -1])
def test_candidate_rejects_nonpositive_elapsed(delta_ns):
    policy = StepPolicy(40)
    policy.reset(1_000_000_000)
    with pytest.raises(ValueError, match='nonpositive_elapsed'):
        policy.consume(1_000_000_000 + delta_ns)


POLICIES = [None, 40, 80]  # None keeps the production 20 ms step.
PATTERNS = [(20,), (40,), (20, 60), (60, 20)]


@pytest.mark.parametrize('cap_ms', POLICIES)
@pytest.mark.parametrize('pattern', PATTERNS)
@pytest.mark.parametrize('path', ['ramp_hold', 'reversal'])
async def test_progress_and_real_time_limit_metrics(cap_ms, pattern, path):
    control, latest, backend, clock, probe = await make_probe(cap_ms)
    elapsed_ms = 0
    seq = 2
    one_second = None
    while elapsed_ms < 2000:
        gap = pattern[(seq - 2) % len(pattern)]
        elapsed_ms += gap
        clock.advance_ms(gap)
        t = elapsed_ms / 1000
        ramp = min(t, 1) if path == 'ramp_hold' else t if t <= 1 else 2 - t
        seq += 1
        publish(latest, clock, seq, True, 0.03 * ramp, 0.1 * ramp)
        await control.tick()
        assert control.mode is TeleopMode.ACTIVE
        if elapsed_ms == 1000:
            one_second = control.last_target.p[0] - 0.3
    rows = probe.rows
    assert len(rows) == len(backend.targets)
    assert not backend.stops
    assert all(row['dt'] == pytest.approx(0.02 if cap_ms is None else min(row['elapsed'], cap_ms / 1000)) for row in rows)
    # These are command-space speeds, never physical device measurements.
    assert max(row['step_m'] for row in rows) <= 0.0005 + 1e-12
    assert max(row['step_rad'] for row in rows) <= np.deg2rad(0.2) + 1e-12
    assert max(row['speed_mps'] for row in rows) <= 0.005 + 1e-12
    assert max(row['speed_radps'] for row in rows) <= 0.05 + 1e-12
    # Acceleration is measured, not assumed guaranteed: target snapping and
    # capped/uncapped interval transitions can break a wall-time interpretation.
    result = dict(
        policy='fixed20' if cap_ms is None else f'elapsed_cap{cap_ms}',
        pattern_ms=pattern, path=path, frames=len(rows),
        integration_ms=sum(row['dt'] for row in rows) * 1000,
        one_second_mm=None if one_second is None else one_second * 1000,
        final_mm=(control.last_target.p[0] - 0.3) * 1000,
        peak_speed_mmps=max(row['speed_mps'] for row in rows) * 1000,
        peak_angular_speed_radps=max(row['speed_radps'] for row in rows),
        peak_command_accel_mmps2=max(row['accel_mps2'] for row in rows) * 1000,
        peak_command_angular_accel_radps2=max(row['accel_radps2'] for row in rows),
        linear_accel_over_limit=sum(row['accel_mps2'] > 0.02 + 1e-10 for row in rows),
        angular_accel_over_limit=sum(row['accel_radps2'] > 0.1 + 1e-10 for row in rows),
        internal_linear_delta_over_limit=sum(row['internal_accel_mps2'] > 0.02 + 1e-10 for row in rows),
        internal_angular_delta_over_limit=sum(row['internal_accel_radps2'] > 0.1 + 1e-10 for row in rows),
    )
    if pattern == (40,) and path == 'ramp_hold':
        assert result['final_mm'] == pytest.approx(4.424 if cap_ms is None else 6, abs=1e-6)
    print('ELAPSED_RESULT ' + json.dumps(result, sort_keys=True))


@pytest.mark.parametrize('cap_ms', POLICIES)
@pytest.mark.parametrize('gap_ms', [99, 100, 250])
async def test_pause_keeps_freshness_gate_and_no_backlog(cap_ms, gap_ms):
    control, latest, backend, clock, probe = await make_probe(cap_ms)
    clock.advance_ms(20)
    publish(latest, clock, 3, True, 0.03, 0.1)
    await control.tick()
    before = len(backend.targets)
    clock.advance_ms(gap_ms)
    await control.tick()  # Observe stale BEFORE publishing a new frame.
    assert len(backend.targets) == before
    publish(latest, clock, 4, True, 0.03, 0.1)
    await control.tick()
    if gap_ms >= 100:
        assert StopReason.STALE in backend.stops
        assert control.mode is not TeleopMode.ACTIVE
        assert len(backend.targets) == before
    else:
        assert control.mode is TeleopMode.ACTIVE
        assert probe.rows[-1]['dt'] == pytest.approx(0.02 if cap_ms is None else cap_ms / 1000)
        clock.advance_ms(20)
        publish(latest, clock, 5, True, 0.03, 0.1)
        await control.tick()
        assert probe.rows[-1]['dt'] == pytest.approx(0.02)


@pytest.mark.parametrize('cap_ms', POLICIES)
async def test_regrip_recaptures_clock_and_pose(cap_ms):
    control, latest, backend, clock, probe = await make_probe(cap_ms)
    clock.advance_ms(40)
    publish(latest, clock, 3, True, 0.03, 0.1)
    await control.tick()
    backend.actual_tcp = control.last_target.model_copy(deep=True)
    backend.preflight_result = backend.preflight_result.__class__(
        **{**backend.preflight_result.__dict__, 'actual_tcp': backend.actual_tcp})
    clock.advance_ms(20)
    publish(latest, clock, 4, False, 0.03, 0.1)
    await control.tick()
    assert control.mode is TeleopMode.HOLD
    before = len(backend.targets)
    # Keep released frames fresh during a 500 ms HOLD; no synthetic stale bypass.
    for seq in range(5, 30):
        clock.advance_ms(20)
        publish(latest, clock, seq, False, 0.10, 0.2)
        await control.tick()
    publish(latest, clock, 30, True, 0.10, 0.2)
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    assert len(backend.targets) == before
    assert control.last_target == backend.actual_tcp
    clock.advance_ms(20)
    publish(latest, clock, 31, True, 0.13, 0.3)
    await control.tick()
    assert probe.rows[-1]['dt'] == pytest.approx(0.02)
    assert probe.rows[-1]['elapsed'] == pytest.approx(0.02)
    assert probe.rows[-1]['step_m'] <= 0.02 * 0.02 ** 2 + 1e-12


def saturated_limiter():
    limits = limiter()
    previous = Pose(p=(0.3, 0, 0.3), q=(0, 0, 0, 1))
    limits.set_pose_anchor(previous)
    far = Pose(p=(0.4, 0, 0.3), q=tuple(Rotation.from_rotvec((0, 0, 0.5)).as_quat()))
    for _ in range(30):
        previous = limits.limit_motion(previous, far, 0.02)
    assert limits.linear_velocity[0] == pytest.approx(0.005)
    assert limits.angular_velocity[2] == pytest.approx(0.05)
    return limits, previous, far


@pytest.mark.parametrize('dt', [0.02, 0.04, 0.06])
def test_existing_target_snap_resets_velocity_beyond_accel_budget(dt):
    limits, previous, _ = saturated_limiter()
    close = Pose(
        p=(previous.p[0] + 0.00001, 0, 0.3),
        q=tuple((Rotation.from_rotvec((0, 0, 0.0001)) * Rotation.from_quat(previous.q)).as_quat()),
    )
    before_v, before_w = limits.linear_velocity.copy(), limits.angular_velocity.copy()
    result = limits.limit_motion(previous, close, dt)
    assert result.p == pytest.approx(close.p)
    assert (Rotation.from_quat(result.q) * Rotation.from_quat(close.q).inv()).magnitude() < 1e-12
    assert np.linalg.norm(limits.linear_velocity) == 0
    assert np.linalg.norm(limits.angular_velocity) == 0
    assert np.linalg.norm(before_v) / dt > limits.max_linear_accel
    assert np.linalg.norm(before_w) / dt > limits.max_angular_accel


@pytest.mark.parametrize('cap_ms', POLICIES)
def test_interval_capping_causes_command_velocity_jump_without_target_snap(cap_ms):
    limits, previous, far = saturated_limiter()
    policy = StepPolicy(cap_ms)
    now = 1_000_000_000
    policy.reset(now)
    speeds = []
    for elapsed in (0.06, 0.02):
        now += round(elapsed * 1e9)
        dt = policy.consume(now)
        result = limits.limit_motion(previous, far, dt)
        speeds.append((result.p[0] - previous.p[0]) / elapsed)
        assert limits.linear_velocity[0] == pytest.approx(0.005)
        assert limits.angular_velocity[2] == pytest.approx(0.05)
        previous = result
    acceleration = abs(speeds[1] - speeds[0]) / 0.02
    expected = {None: 1 / 6, 40: 1 / 12, 80: 0}
    assert acceleration == pytest.approx(expected[cap_ms], abs=1e-12)
