"""Cruise/arrival arithmetic only, no physical plant or SDK simulation."""
from dataclasses import replace

import numpy as np
import pytest

from tests.robots.pvat_cruise_candidate import build_cruise_plan,fit_reference_window
from tests.robots.pvat_ramp_candidate import RampLimits

ZERO=(0.,)*6
axis=lambda x:(x,*ZERO[1:])


@pytest.mark.parametrize('distance',[-.5,-.001,.001,.5])
def test_rest_to_rest_cruise_plan_has_continuous_pva_and_monotonic_arrival(distance):
    plan=build_cruise_plan(ZERO,axis(distance))
    previous=None
    for segment in plan.segments:
        if previous is not None:
            np.testing.assert_allclose(segment.state(0.),previous.state(previous.duration_s),atol=1e-11,rtol=0)
        speed,acceleration=segment.peak_abs_derivatives()
        assert max(speed)<=.05+1e-10 and max(acceleration)<=.2+1e-10
        previous=segment
    states=np.array([plan.state(float(t)) for t in np.linspace(0.,plan.duration_s,301)])
    assert np.min(np.diff(states[:,0,0])*np.sign(distance))>=-1e-12
    assert plan.state(plan.duration_s)==(axis(distance),ZERO,ZERO)


def test_far_target_uses_cruise_instead_of_stretching_a_single_quintic():
    plan=build_cruise_plan(ZERO,axis(.5))
    assert plan.duration_s==pytest.approx(10.375)
    assert plan.state(2.)[0][0]==pytest.approx(.090625)
    assert plan.state(2.)[1][0]==pytest.approx(.05)
    assert len(plan.segments)==3


def test_near_target_omits_cruise_and_arrives_with_zero_derivatives():
    plan=build_cruise_plan(ZERO,axis(.001))
    assert plan.duration_s==pytest.approx(2*np.sqrt(1.5*.001/.2))
    assert len(plan.segments)==2
    assert plan.state(1.)==(axis(.001),ZERO,ZERO)


def test_all_six_joints_follow_the_same_scalar_progress():
    target=(.5,-.25,.1,0.,.2,-.05)
    plan=build_cruise_plan(ZERO,target)
    for t in np.linspace(0.,plan.duration_s,31):
        q,_,_=plan.state(float(t))
        np.testing.assert_allclose(q,np.array(target)*q[0]/.5,atol=1e-12,rtol=0)


def test_cross_phase_window_keeps_all_pieces_and_matches_the_full_reference():
    plan=build_cruise_plan(ZERO,axis(.5))
    window=plan.window(.35,.08)
    assert len(window.segments)==2
    for t in np.linspace(0.,.08,31):
        np.testing.assert_allclose(window.state(float(t)),plan.state(.35+float(t)),atol=1e-10,rtol=0)
    with pytest.raises(ValueError,match='invalid_sample_time'):
        window.state(.110)


def test_final_window_shortens_to_arrival_and_does_not_claim_postwindow_coverage():
    plan=build_cruise_plan(ZERO,axis(.001))
    window=plan.window(plan.duration_s-.01,.08)
    assert window.duration_s==pytest.approx(.01)
    np.testing.assert_allclose(window.state(window.duration_s),(axis(.001),ZERO,ZERO),atol=1e-10,rtol=0)
    with pytest.raises(ValueError,match='plan_complete'):
        plan.window(plan.duration_s,.08)


def test_stationary_plan_does_not_generate_acceleration_or_cruise():
    plan=build_cruise_plan(ZERO,ZERO)
    assert plan.state(.04)==(ZERO,ZERO,ZERO)


@pytest.mark.parametrize('initial,target,reason',[
    ((0.,)*5,ZERO,'invalid_vector'),(ZERO,axis(float('nan')),'invalid_vector'),
    (axis(3.1),ZERO,'initial_joint_limit'),(ZERO,axis(3.1),'target_joint_limit')])
def test_invalid_vectors_or_joint_bounds_reject(initial,target,reason):
    with pytest.raises(ValueError,match=reason):build_cruise_plan(initial,target)


@pytest.mark.parametrize('changes',[{'speed_cap':0.},{'acceleration_cap':float('inf')},{'joint_max':(-3.,)*6}])
def test_invalid_limits_reject(changes):
    with pytest.raises(ValueError,match='invalid_limits'):
        build_cruise_plan(ZERO,axis(.001),replace(RampLimits(),**changes))


@pytest.mark.parametrize('start',[.01,.5,10.1])
def test_single_phase_window_fit_matches_reference_inside_the_window(start):
    plan=build_cruise_plan(ZERO,axis(.5))
    fit=fit_reference_window(plan,start,.08)
    for time in np.linspace(0.,.08,31):
        np.testing.assert_allclose(fit.state(float(time)),plan.state(start+float(time)),atol=1e-10,rtol=0)


@pytest.mark.parametrize('start',[.35,.37,9.95,9.98])
def test_cross_phase_endpoint_fit_must_reject_if_its_interior_exceeds_caps(start):
    plan=build_cruise_plan(ZERO,axis(.5))
    with pytest.raises(ValueError,match='pvat_window_interpolation_limit'):
        fit_reference_window(plan,start,.08)


def test_roundoff_at_a_phase_boundary_does_not_emit_a_zero_duration_piece():
    plan=build_cruise_plan(ZERO,axis(.5))
    assert len(plan.window(.375,.08).segments)==1


@pytest.mark.parametrize('field',['initial_velocity','initial_acceleration'])
def test_moving_initial_state_cannot_be_silently_treated_as_rest(field):
    with pytest.raises(ValueError,match='cruise_requires_rest'):
        build_cruise_plan(ZERO,axis(.5),**{field:axis(.01)})
