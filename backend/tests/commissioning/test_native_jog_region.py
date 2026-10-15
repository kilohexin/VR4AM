"""Synthetic model estimates only; no SDK, field configuration, or sockets."""
import itertools
import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_region import assess_translation_region
from app.commissioning.native_jog_mdh import MdhTool, assess_translation_reference
from tests.commissioning.test_native_jog_mdh import Q, TOOL, independent_transform


def assessment(**changes):
    args=dict(joint_radius_rad=(.003,)*6, initial_offset_rad=(.0002,)*6,
        duration_s=.04,cartesian_speed_mps=.003,cartesian_acceleration_mps2=.01,
        joint_speed_limit_radps=.05,joint_acceleration_limit_radps2=.2)
    args.update(changes)
    return assess_translation_region(Q,TOOL,**args)


def numerical_jacobian(q,tool):
    # Differentiate the independently expanded elementary transform product;
    # do not use the production geometric-J or its derivatives as the oracle.
    result=np.zeros((6,6));h=1e-5
    for axis in range(6):
        delta=np.eye(6)[axis]*h
        plus=independent_transform(q+delta,tool)
        minus=independent_transform(q-delta,tool)
        result[:3,axis]=(plus[:3,3]-minus[:3,3])/(2*h)
        result[3:,axis]=Rotation.from_matrix(plus[:3,:3]@minus[:3,:3].T).as_rotvec()/(2*h)
    return result


def test_region_covers_sampled_directions_and_curvature():
    # Omitting angular/linear curvature or mixing Jacobian frames would make
    # this independent sampled-trajectory comparison fail. Sampling is QA,
    # not a uniform model proof or physical acceptance.
    r=assessment();v_bound=np.array(r['joint_velocity_bound_radps'])
    a_bound=np.array(r['joint_acceleration_bound_radps2'])
    directions=[np.array(v,dtype=float)/np.linalg.norm(v) for v in
        [(1,0,0),(0,1,0),(0,0,1),(1,1,1),(-1,1,-1)]]
    for corner in itertools.product((-1,1),repeat=6):
        q=np.array(Q)+np.array(corner)*.003
        j=numerical_jacobian(q,TOOL)
        for direction in directions:
            qd=np.linalg.solve(j,np.r_[direction*.003,[0.,0.,0.]])
            assert np.all(np.abs(qd)<=v_bound+1e-8)
            dt=1e-3
            jdot=(numerical_jacobian(q+dt*qd,TOOL)-numerical_jacobian(q-dt*qd,TOOL))/(2*dt)
            for acceleration in (np.zeros(3),direction*.01,-direction*.01):
                qdd=np.linalg.solve(j,np.r_[acceleration,[0.,0.,0.]]-jdot@qd)
                assert np.all(np.abs(qdd)<=a_bound+1e-6)


def test_model_success_never_admits_hardware():
    r=assessment()
    assert r['model_reference_eligible'] and r['model_rates_within_limits']
    assert r['reference_segment_within_region'] and 0<r['perturbation_contraction']<1
    for flag in ('hardware_verified','joint_bounds_verified','braking_verified',
            'continuous_control_admitted','factory_calibration_error_bounded','directed_rounding'):
        assert r[flag] is False
    assert r['scope']=='theoretical_region_only_not_controller_or_braking'
    assert r['model']=='manufacturer_theoretical_mdh_not_factory_calibration'
    assert len(r['max_reference_excursion_rad'])==6
    assert json.loads(json.dumps(r,allow_nan=False))==r


def test_curvature_survives_near_zero_cartesian_acceleration():
    r=assessment(cartesian_acceleration_mps2=1e-12)
    assert min(r['joint_acceleration_bound_radps2'])>1e-8
    slow=assessment(cartesian_speed_mps=.001,cartesian_acceleration_mps2=1e-12)
    np.testing.assert_allclose(r['joint_acceleration_bound_radps2'],
        np.array(slow['joint_acceleration_bound_radps2'])*9,atol=1e-9,rtol=0)


def test_reference_can_leave_region_despite_low_rates():
    r=assessment(joint_radius_rad=(.001,)*6,initial_offset_rad=(.0009,)*6,
        duration_s=.08,cartesian_speed_mps=.005,joint_speed_limit_radps=100.,
        joint_acceleration_limit_radps2=100.)
    assert r['model_rates_within_limits']
    assert not r['reference_segment_within_region'] and not r['model_reference_eligible']
    assert max(r['max_reference_excursion_rad'])>.001


def test_region_can_reject_rates_that_pass_at_reference():
    point=assess_translation_reference(Q,TOOL,cartesian_speed_mps=.005,
        cartesian_acceleration_mps2=.01,joint_speed_limit_radps=.05,
        joint_acceleration_limit_radps2=.2)
    region=assessment(joint_radius_rad=(.005,)*6,cartesian_speed_mps=.005,
        duration_s=.01,initial_offset_rad=(0.,)*6)
    assert point['limits_satisfied_at_reference']
    assert not region['model_rates_within_limits'] and not region['model_reference_eligible']
    assert max(region['joint_acceleration_bound_radps2'])>.2


def test_excess_reported_without_clipping():
    r=assessment(joint_speed_limit_radps=1e-8,joint_acceleration_limit_radps2=1e-8)
    usual=assessment()
    assert not r['model_rates_within_limits'] and not r['model_reference_eligible']
    assert r['joint_velocity_bound_radps']==usual['joint_velocity_bound_radps']
    assert r['joint_acceleration_bound_radps2']==usual['joint_acceleration_bound_radps2']


@pytest.mark.parametrize('name',['cartesian_speed_mps','cartesian_acceleration_mps2',
    'joint_speed_limit_radps','joint_acceleration_limit_radps2'])
@pytest.mark.parametrize('bad',[0,-1,True,math.nan,math.inf,'1',10**400])
def test_invalid_scalars_rejected(name,bad):
    with pytest.raises(ValueError):assessment(**{name:bad})


@pytest.mark.parametrize('bad',[0,-1,True,None,math.nan,math.inf,.009,.081,10**400])
def test_explicit_finite_duration_required(bad):
    with pytest.raises(ValueError):assessment(duration_s=bad)


@pytest.mark.parametrize('name',['joint_radius_rad','initial_offset_rad'])
@pytest.mark.parametrize('bad',[(0,)*5,(0,)*7,'000000',None,(True,)*6,(math.nan,)*6,(math.inf,)*6,(10**400,)*6])
def test_invalid_region_vectors_rejected(name,bad):
    with pytest.raises(ValueError):assessment(**{name:bad})


@pytest.mark.parametrize('radius,offset',[
    ((0.,)*6,(0.,)*6),((-.001,)*6,(0.,)*6),((.011,)*6,(0.,)*6),
    ((.003,)*6,(-.001,)*6),((.003,)*6,(.003,)*6),((.003,)*6,(.004,)*6),
])
def test_region_geometry_rejected(radius,offset):
    with pytest.raises(ValueError):assessment(joint_radius_rad=radius,initial_offset_rad=offset)


@pytest.mark.parametrize('q',[(0.,)*5,(True,*Q[1:]),(math.inf,*Q[1:]),None])
def test_invalid_reference_rejected(q):
    with pytest.raises(ValueError):assess_translation_region(q,TOOL,
        joint_radius_rad=(.003,)*6,initial_offset_rad=(0.,)*6,duration_s=.04,
        cartesian_speed_mps=.003,cartesian_acceleration_mps2=.01,
        joint_speed_limit_radps=.05,joint_acceleration_limit_radps2=.2)


@pytest.mark.parametrize('q,radius,reason',[
    ((0.,)*6,.003,'singular'),(Q,.01,'contractive'),
])
def test_singular_or_noncontractive_reference_has_no_prediction(q,radius,reason):
    with pytest.raises(ValueError,match=reason):assess_translation_region(q,TOOL,
        joint_radius_rad=(radius,)*6,initial_offset_rad=(0.,)*6,duration_s=.04,
        cartesian_speed_mps=.003,cartesian_acceleration_mps2=.01,
        joint_speed_limit_radps=.05,joint_acceleration_limit_radps2=.2)


def test_numerical_overflow_becomes_explicit_rejection():
    with pytest.raises(ValueError):assessment(cartesian_speed_mps=1e308)


def test_missing_explicit_tool_rejected():
    with pytest.raises(ValueError):assess_translation_region(Q,None,
        joint_radius_rad=(.003,)*6,initial_offset_rad=(0.,)*6,duration_s=.04,
        cartesian_speed_mps=.003,cartesian_acceleration_mps2=.01,
        joint_speed_limit_radps=.05,joint_acceleration_limit_radps2=.2)


def test_explicit_tool_translation_changes_prediction():
    alternate=MdhTool((.02,-.01,.12),(.5,.5,.5,.5))
    r=assess_translation_region(Q,alternate,joint_radius_rad=(.003,)*6,
        initial_offset_rad=(.0002,)*6,duration_s=.04,cartesian_speed_mps=.003,
        cartesian_acceleration_mps2=.01,joint_speed_limit_radps=.05,
        joint_acceleration_limit_radps2=.2)
    assert r['joint_velocity_bound_radps']!=assessment()['joint_velocity_bound_radps']


def test_inputs_and_outputs_never_alias_later_predictions():
    radius=[.003]*6;offset=[.0002]*6
    r=assessment(joint_radius_rad=radius,initial_offset_rad=offset)
    frozen=json.loads(json.dumps(r));radius[0]=0;offset[0]=9
    assert r==frozen
    r['joint_velocity_bound_radps'][0]=9
    assert assessment()==frozen


def test_nonuniform_radii_and_offsets_are_applied_per_joint():
    radius=(.001,.002,.003,.002,.001,.003)
    offset=(.0001,.0002,.0003,.0004,.0005,.0006)
    r=assessment(joint_radius_rad=radius,initial_offset_rad=offset)
    no_offset=assessment(joint_radius_rad=radius,initial_offset_rad=(0.,)*6)
    np.testing.assert_allclose(np.array(r['max_reference_excursion_rad'])-
        no_offset['max_reference_excursion_rad'],offset,atol=1e-18,rtol=0)
    q=np.array(Q)+np.array(radius)*np.array([1,-1,1,-1,1,-1])
    j=numerical_jacobian(q,TOOL)
    for axis in np.eye(3):
        qd=np.linalg.solve(j,np.r_[axis*.003,[0.,0.,0.]])
        assert np.all(np.abs(qd)<=np.array(r['joint_velocity_bound_radps'])+1e-8)


@pytest.mark.parametrize('duration',[.01,.08])
def test_duration_endpoints_are_accepted_without_rate_changes(duration):
    short=assessment(duration_s=.01)
    r=assessment(duration_s=duration)
    assert r['joint_velocity_bound_radps']==short['joint_velocity_bound_radps']
    assert r['joint_acceleration_bound_radps2']==short['joint_acceleration_bound_radps2']
    np.testing.assert_allclose(r['max_reference_excursion_rad'],
        np.array(r['joint_velocity_bound_radps'])*duration+.0002,atol=0,rtol=0)
