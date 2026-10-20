"""Theoretical changing-state fixtures; no SDK, controller or physical proof."""
from dataclasses import replace
import importlib
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool, mdh_tcp_jacobian
from app.commissioning.native_jog_model_guard import ModelJogRegionGuard
from app.commissioning.native_jog_packet import plan_jog
from app.schemas.messages import Pose
from tests.commissioning.test_native_jog_mdh import independent_transform
from tests.commissioning.test_native_jog_region import numerical_jacobian
from tests.commissioning.test_native_jog_region_binding import fixture, SHA


def moving_type():
    try:
        return importlib.import_module('app.commissioning.native_jog_moving_guard').MovingModelJogRegionGuard
    except ModuleNotFoundError:
        pytest.fail('moving-start model guard missing')


def state(q, tool, speed=.001, acceleration=.004):
    """Generate coherent rates with an independently differenced Jdot."""
    q = np.array(q, dtype=float)
    transform = independent_transform(q, tool)
    _, j = mdh_tcp_jacobian(tuple(map(float, q)), tool)
    qd = np.linalg.solve(j, [0., 0., speed, 0., 0., 0.])
    dt = .001
    jp = mdh_tcp_jacobian(tuple(map(float, q+dt*qd)), tool)[1]
    jm = mdh_tcp_jacobian(tuple(map(float, q-dt*qd)), tool)[1]
    qdd = np.linalg.solve(j, np.array([0., 0., acceleration, 0., 0., 0.])-(jp-jm)@qd/(2*dt))
    pose = Pose(p=tuple(transform[:3, 3]), q=tuple(Rotation.from_matrix(transform[:3, :3]).as_quat()))
    return tuple(map(float, q)), tuple(map(float, qd)), tuple(map(float, qdd)), pose


def model_fixture(q=None, *, speed=.001, acceleration=.004):
    args, _ = fixture()
    q, qd, qdd, pose = state(args['reference_q'] if q is None else q, args['tool'], speed, acceleration)
    snapshot = SimpleNamespace(actual_q=q, target_q=q, actual_qd=qd, target_qd=qd,
        actual_qdd=qdd, target_qdd=qdd, actual_tcp=pose, target_tcp=pose,
        tcp_setting=Pose(p=args['tool'].translation_m, q=args['tool'].rotation_xyzw))
    packet = plan_jog((0, 0, 1), 1, limits=args['limits'], actual_p=pose.p,
        snapshot_ns=0, now_ns=0, generation=1, sequence=1, input_ns=0,
        input_expires_ns=100_000_000, duration_s=.04, config_sha256=SHA)
    return args, snapshot, packet


def check(guard, args, snapshot, packet):
    return guard.validate(packet, snapshot, settings=args['settings'], limits=args['limits'], config_sha256=SHA)


@pytest.mark.parametrize('q', [(0., -1., 1., 0., 1.57, 0.), (.2, -.8, 1.2, -.4, .5, .1)])
@pytest.mark.parametrize('qd', [(0.,)*6, (.01, -.02, .005, .03, -.01, .04)])
def test_analytic_jacobian_rate_matches_independent_elementary_differences(q, qd):
    from app.commissioning import native_jog_mdh as module
    derivative = getattr(module, 'mdh_tcp_jacobian_rate', None)
    assert derivative is not None, 'analytic changing-state kinematics missing'
    tool = MdhTool((.01, -.02, .12), (.5, .5, .5, .5))
    transform, j, jdot = derivative(q, qd, tool)
    dt = .001
    expected = (numerical_jacobian(np.array(q)+dt*np.array(qd), tool)
        - numerical_jacobian(np.array(q)-dt*np.array(qd), tool))/(2*dt)
    np.testing.assert_allclose(transform, independent_transform(q, tool), atol=2e-15, rtol=0)
    np.testing.assert_allclose(j, numerical_jacobian(np.array(q), tool), atol=2e-9, rtol=0)
    np.testing.assert_allclose(jdot, expected, atol=2e-8, rtol=0)


@pytest.mark.parametrize('speed', [.001, -.001])
def test_nonzero_translation_start_is_accepted_only_by_explicit_moving_guard(speed):
    args, snapshot, packet = model_fixture(speed=speed)
    with pytest.raises(ValueError, match='initial_motion_reported'):
        check(ModelJogRegionGuard(**args), args, snapshot, packet)
    guard = moving_type()(**args)
    check(guard, args, snapshot, packet)
    for flag in ('hardware_verified', 'joint_bounds_verified', 'braking_verified', 'continuous_control_admitted'):
        assert guard.assessment[flag] is False


def test_current_offset_is_checked_against_remaining_fixed_box_not_old_start_cap():
    args, _, _ = model_fixture()
    shifted = list(args['reference_q'])
    shifted[0] += .0005  # Beyond the old .0002 initial cap, inside the fixed .003 box.
    args, snapshot, packet = model_fixture(shifted)
    guard = moving_type()(**args)
    check(guard, args, snapshot, packet)
    with pytest.raises(ValueError, match='reference_changed'):
        check(ModelJogRegionGuard(**args), args, snapshot, packet)


@pytest.mark.parametrize('prefix', ['actual', 'target'])
@pytest.mark.parametrize('damage', ['velocity', 'acceleration', 'rotation', 'angular_acceleration',
    'cartesian_speed', 'cartesian_acceleration', 'remaining_region', 'nonfinite', 'feedback'])
def test_invalid_moving_state_is_rejected_without_recentring(prefix, damage):
    args, snapshot, packet = model_fixture()
    guard = moving_type()(**args)
    if damage == 'velocity':
        setattr(snapshot, prefix+'_qd', (.1, 0., 0., 0., 0., 0.))
    elif damage == 'acceleration':
        setattr(snapshot, prefix+'_qdd', (.3, 0., 0., 0., 0., 0.))
    elif damage == 'rotation':
        setattr(snapshot, prefix+'_qd', (0., 0., 0., 0., 0., .00001))
    elif damage == 'angular_acceleration':
        setattr(snapshot, prefix+'_qdd', (0., 0., 0., 0., 0., .00001))
    elif damage in ('cartesian_speed', 'cartesian_acceleration'):
        q, qd, qdd, pose = state(args['reference_q'], args['tool'],
            .00301 if damage == 'cartesian_speed' else .001,
            .01001 if damage == 'cartesian_acceleration' else .004)
        for field, value in [('q',q), ('qd',qd), ('qdd',qdd), ('tcp',pose)]:
            setattr(snapshot, prefix+'_'+field, value)
    elif damage == 'remaining_region':
        q = list(args['reference_q']); q[2] += .0029
        for field, value in zip(('q','qd','qdd','tcp'), state(q,args['tool'])):
            setattr(snapshot, prefix+'_'+field, value)
    elif damage == 'nonfinite':
        setattr(snapshot, prefix+'_qd', (float('nan'), 0., 0., 0., 0., 0.))
    else:
        setattr(snapshot, prefix+'_tcp', Pose(p=(0., 0., 0.), q=(0., 0., 0., 1.)))
    with pytest.raises(ValueError):
        check(guard, args, snapshot, packet)
    assert guard.reference_q == args['reference_q']


@pytest.mark.parametrize('field,value', [('reference_q', (0.,)*6), ('duration_s', .08),
    ('joint_radius_rad', (.01,)*6), ('initial_offset_rad', (.001,)*6)])
def test_in_place_model_changes_invalidate_dynamic_binding(field, value):
    args, snapshot, packet = model_fixture()
    guard = moving_type()(**args)
    object.__setattr__(guard, field, value)
    with pytest.raises(ValueError, match='binding_changed'):
        check(guard, args, snapshot, packet)


@pytest.mark.parametrize('host', ['localhost', '::1', '172.16.2.162'])
def test_moving_guard_refuses_every_nonliteral_loopback_host(host):
    args, _, _ = model_fixture()
    args['settings'] = replace(args['settings'], ip=host)
    with pytest.raises(ValueError):
        moving_type()(**args)


def test_curvature_can_exceed_cartesian_acceleration_when_j_times_qdd_alone_does_not():
    args, snapshot, packet = model_fixture(speed=.002)
    q = np.array(snapshot.actual_q); qd = np.array(snapshot.actual_qd)
    _,j = mdh_tcp_jacobian(tuple(map(float,q)),args['tool']); dt=.001
    jp = mdh_tcp_jacobian(tuple(map(float,q+dt*qd)),args['tool'])[1]
    jm = mdh_tcp_jacobian(tuple(map(float,q-dt*qd)),args['tool'])[1]
    curvature = (jp-jm)@qd/(2*dt)
    assert np.linalg.norm(curvature[:3]) > 1e-6
    linear = curvature[:3]/np.linalg.norm(curvature[:3])*.009999
    qdd = np.linalg.solve(j,np.r_[linear,-curvature[3:]])
    assert np.linalg.norm((j@qdd)[:3]) < .01
    assert np.linalg.norm((j@qdd+curvature)[:3]) > .01
    snapshot.actual_qdd = tuple(map(float,qdd))
    with pytest.raises(ValueError,match='cartesian_rates_exceeded'):
        check(moving_type()(**args),args,snapshot,packet)


@pytest.mark.parametrize('field,value', [('max_speed_mps',.03), ('max_acceleration_mps2',.1),
    ('tcp_max_m',(9.,9.,9.))])
def test_mutating_the_bound_limits_cannot_reuse_old_region_prediction(field,value):
    args,snapshot,packet = model_fixture()
    guard = moving_type()(**args)
    object.__setattr__(args['limits'],field,value)
    with pytest.raises(ValueError,match='binding_changed'):
        check(guard,args,snapshot,packet)
