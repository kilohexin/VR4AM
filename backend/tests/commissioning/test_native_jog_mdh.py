"""Synthetic kinematics; no field poses, SDK, network or physical assertions."""
from dataclasses import FrozenInstanceError
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool, mdh_tcp_jacobian, assess_translation_reference

Q = (0.,-.8,1.2,-.4,.5,0.)
TOOL = MdhTool((0.,0.,.175),(0.,0.,0.,1.))


def independent_transform(q,tool):
    # Elementary transform product, independently expanded from manufacturer
    # Tx(a) Rx(alpha) Rz(theta) Tz(d); no helper from the implementation.
    t=np.eye(4)
    for theta,a,d,alpha in zip(q,[0,0,-.28,-.26,0,0],
            [.21583,0,0,.12063,.09833,.08343],[0,1.5708,0,0,1.5708,-1.5708]):
        tx=np.eye(4);tx[0,3]=a
        rx=np.eye(4);rx[1:3,1:3]=[[math.cos(alpha),-math.sin(alpha)],
            [math.sin(alpha),math.cos(alpha)]]
        rz=np.eye(4);rz[:2,:2]=[[math.cos(theta),-math.sin(theta)],
            [math.sin(theta),math.cos(theta)]]
        tz=np.eye(4);tz[2,3]=d
        t=t@tx@rx@rz@tz
    tcp=np.eye(4);tcp[:3,3]=tool.translation_m
    tcp[:3,:3]=Rotation.from_quat(tool.rotation_xyzw).as_matrix()
    return t@tcp


@pytest.mark.parametrize('q',[(0.,)*6,Q,(.3,-1.4,.9,.2,-.7,1.1)])
def test_mdh_fk_matches_independent_elementary_product(q):
    tool=MdhTool((.01,-.02,.12),(.5,.5,.5,.5))
    transform,_=mdh_tcp_jacobian(q,tool)
    np.testing.assert_allclose(transform,independent_transform(q,tool),atol=2e-15,rtol=0)


@pytest.mark.parametrize('h',[1e-5,1e-6])
def test_analytic_jacobian_matches_central_difference(h):
    _,j=mdh_tcp_jacobian(Q,TOOL)
    numerical=np.zeros((6,6))
    for i in range(6):
        delta=np.eye(6)[i]*h
        plus=independent_transform(np.array(Q)+delta,TOOL)
        minus=independent_transform(np.array(Q)-delta,TOOL)
        numerical[:3,i]=(plus[:3,3]-minus[:3,3])/(2*h)
        numerical[3:,i]=Rotation.from_matrix(plus[:3,:3]@minus[:3,:3].T).as_rotvec()/(2*h)
    np.testing.assert_allclose(j,numerical,atol=2e-9,rtol=0)


def assessment(**changes):
    args=dict(cartesian_speed_mps=.005,cartesian_acceleration_mps2=.02,
        joint_speed_limit_radps=.05,joint_acceleration_limit_radps2=.2)
    args.update(changes)
    return assess_translation_reference(Q,TOOL,**args)


def test_all_directions_bound_covers_combined_translation_and_is_tight():
    _,j=mdh_tcp_jacobian(Q,TOOL)
    g=np.linalg.solve(j,np.vstack([np.eye(3),np.zeros((3,3))]))
    result=assessment()
    peak=result['peak_joint_speed_radps']
    for direction in [(1,0,0),(0,1,0),(0,0,1),(1,1,1),(-1,1,-1)]:
        unit=np.array(direction,dtype=float);unit/=np.linalg.norm(unit)
        assert np.max(np.abs(g@unit*.005))<=peak+1e-15
    row=max(g,key=np.linalg.norm)
    worst=row/np.linalg.norm(row)
    assert np.max(np.abs(g@worst*.005))==pytest.approx(peak)
    assert result['peak_start_acceleration_radps2']==pytest.approx(peak*4)


def test_low_pointwise_values_never_admit_continuous_or_physical_operation():
    r=assessment(cartesian_speed_mps=1e-9,cartesian_acceleration_mps2=1e-9)
    assert r['limits_satisfied_at_reference'] is True
    assert r['offline_only'] is True
    for flag in ('hardware_verified','joint_bounds_verified','braking_verified','continuous_control_admitted'):
        assert r[flag] is False
    assert r['admission_error']=='native_jog_joint_bounds_unverified'
    assert r['acceleration_scope']=='instant_from_rest_only_no_jdot_bounds'


def test_pointwise_excess_is_reported_without_clipping_input():
    r=assessment(cartesian_speed_mps=1.,cartesian_acceleration_mps2=2.)
    assert not r['limits_satisfied_at_reference']
    assert r['peak_joint_speed_radps']>.05
    assert r['peak_start_acceleration_radps2']>.2
    assert r['continuous_control_admitted'] is False


def test_singular_reference_does_not_produce_a_feasible_speed():
    with pytest.raises(ValueError,match='singular'):
        assess_translation_reference((0.,)*6,TOOL,cartesian_speed_mps=.005,
            cartesian_acceleration_mps2=.02,joint_speed_limit_radps=.05,joint_acceleration_limit_radps2=.2)


@pytest.mark.parametrize('bad',[True,math.nan,math.inf,'0',None])
def test_bad_joint_components_rejected(bad):
    with pytest.raises(ValueError):mdh_tcp_jacobian((bad,*Q[1:]),TOOL)


@pytest.mark.parametrize('q',[(0,)*5,(0,)*7,'000000'])
def test_wrong_joint_shape_rejected(q):
    with pytest.raises(ValueError):mdh_tcp_jacobian(q,TOOL)


@pytest.mark.parametrize('name',['cartesian_speed_mps','cartesian_acceleration_mps2',
    'joint_speed_limit_radps','joint_acceleration_limit_radps2'])
@pytest.mark.parametrize('value',[0,-1,True,math.nan,math.inf])
def test_bad_assessment_parameters_rejected(name,value):
    with pytest.raises(ValueError):assessment(**{name:value})


@pytest.mark.parametrize('translation,rotation',[
    ((0,0), (0,0,0,1)),((0,0,True),(0,0,0,1)),
    ((0,0,0),(0,0,0,0)),((0,0,0),(0,0,0,2)),
    ((0,0,0),(True,0,0,1)),((0,0,0),(0,0,0,math.nan)),
])
def test_bad_tool_definition_rejected(translation,rotation):
    with pytest.raises(ValueError):MdhTool(translation,rotation)


def test_tool_and_returned_arrays_do_not_alias_callers():
    translation=[0,0,.175];rotation=[0,0,0,1]
    tool=MdhTool(translation,rotation)
    translation[2]=9;rotation[3]=0
    assert tool.translation_m==(0,0,.175)
    with pytest.raises(FrozenInstanceError):tool.translation_m=(0,0,9)
    first,j=mdh_tcp_jacobian(Q,tool);expected=first.copy()
    first[:]=0;j[:]=0
    np.testing.assert_array_equal(mdh_tcp_jacobian(Q,tool)[0],expected)
