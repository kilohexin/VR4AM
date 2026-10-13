"""Continuous reference arithmetic only; none of these are controller tests."""
from dataclasses import replace

import numpy as np
import pytest

from tests.robots.pvat_continuous_arrival_candidate import (
    build_continuous_arrival, retarget_continuous_arrival,
)
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import SettlingLimits, _position_range

ZERO = (0.,)*6
def axis(x):return (x, *ZERO[1:])


def assert_full_bounds(plan):
    limits=plan.limits.motion
    for segment in plan.segments:
        speed,acceleration=segment.peak_abs_derivatives()
        lower,upper=_position_range(segment)
        assert max(speed)<=limits.speed_cap+1e-10
        assert max(acceleration)<=limits.acceleration_cap+1e-10
        assert np.all(lower>=np.array(limits.joint_min)-1e-10)
        assert np.all(upper<=np.array(limits.joint_max)+1e-10)
    for a,b in zip(plan.segments,plan.segments[1:]):
        np.testing.assert_allclose(a.state(a.duration_s),b.state(0.),atol=1e-11,rtol=0)
    assert plan.state(plan.duration_s)==(plan.target,ZERO,ZERO)


@pytest.mark.parametrize('sign',[-1,1])
def test_nonzero_acceleration_bridge_enables_case_rejected_by_single_quintic(sign):
    plan=build_continuous_arrival(ZERO,axis(sign*.01),axis(sign*.1),axis(sign*.1))
    assert len(plan.segments)>=2
    assert plan.segments[0].duration_s==pytest.approx(.08)
    np.testing.assert_allclose(plan.state(0.),(ZERO,axis(sign*.01),axis(sign*.1)),atol=1e-12,rtol=0)
    assert_full_bounds(plan)


def test_bridge_matches_independent_integrals_of_linearly_decreasing_acceleration():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.1))
    bridge=plan.segments[0]
    assert bridge.duration_s==pytest.approx(.08)
    for t in [0.,.02,.04,.08]:
        q,v,a=bridge.state(t)
        assert q[0]==pytest.approx(.01*t+.1*t*t/2-.1*t**3/(6*.08),abs=1e-12)
        assert v[0]==pytest.approx(.01+.1*t-.1*t*t/(2*.08),abs=1e-12)
        assert a[0]==pytest.approx(.1*(1-t/.08),abs=1e-12)
    assert_full_bounds(plan)


def test_retarget_uses_elapsed_pva_before_bridge_and_does_not_zero_derivatives():
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.01))
    elapsed=.10
    initial=plan.state(elapsed)
    assert max(abs(x) for x in initial[2])>0
    replacement,cursor=retarget_continuous_arrival(plan,elapsed,axis(-.01))
    assert cursor==0 and replacement is not plan
    np.testing.assert_allclose(replacement.state(0.),initial,atol=1e-11,rtol=0)
    assert len(replacement.segments)>=2
    assert replacement.segments[0].duration_s==pytest.approx(.08)
    assert_full_bounds(replacement)


@pytest.mark.parametrize('elapsed',[.02,.04,.08])
def test_unchanged_target_reuses_plan_instead_of_restarting_arrival(elapsed):
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.01))
    same,cursor=retarget_continuous_arrival(plan,elapsed,axis(.01))
    assert same is plan and cursor==elapsed


def test_changed_limits_rebuild_instead_of_reusing_unchecked_plan():
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.01))
    limits=replace(plan.limits,motion=replace(plan.limits.motion,speed_cap=.04))
    other,cursor=retarget_continuous_arrival(plan,0.,axis(.01),limits)
    assert other is not plan and cursor==0
    assert_full_bounds(other)


def test_cross_bridge_window_preserves_both_pieces_and_exact_states():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.1))
    window=plan.window(.04,.08)
    assert len(window.segments)==2
    for t in np.linspace(0.,.08,31):
        np.testing.assert_allclose(window.state(float(t)),plan.state(.04+float(t)),atol=1e-10,rtol=0)


def test_issued_window_cannot_be_extrapolated_using_full_plan_prediction():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.1))
    window=plan.window(0.,.08)
    assert plan.state(.11)[0][0]>0
    with pytest.raises(ValueError,match='invalid_sample_time'):window.state(.11)


def test_acceleration_above_cap_rejects_instead_of_clipping_or_resetting_initial_state():
    with pytest.raises(ValueError,match='initial_acceleration_limit'):
        build_continuous_arrival(ZERO,axis(.01),axis(.201),axis(.1))


def test_outward_acceleration_at_speed_cap_rejects_unavoidable_bridge_overspeed():
    with pytest.raises(ValueError,match='no_feasible_acceleration_bridge'):
        build_continuous_arrival(ZERO,axis(.05),axis(.1),axis(-.1))


def test_bridge_duration_counts_toward_total_arrival_budget():
    limits=replace(SettlingLimits(),max_duration_s=.08)
    with pytest.raises(ValueError,match='no_feasible_continuous_arrival'):
        build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.1),limits)


def test_bridge_cannot_leave_joint_bounds_while_endpoints_are_in_bounds():
    limits=replace(SettlingLimits(),motion=replace(RampLimits(),joint_max=axis(.0001)))
    with pytest.raises(ValueError,match='no_feasible_acceleration_bridge'):
        build_continuous_arrival(ZERO,axis(.01),axis(-.1),axis(-.1),limits)


def test_rest_and_zero_acceleration_reuse_existing_single_segment_arrival():
    for v in [ZERO,axis(.01)]:
        plan=build_continuous_arrival(ZERO,v,ZERO,axis(.01))
        assert len(plan.segments)==1
        assert_full_bounds(plan)


def test_far_target_from_rest_uses_existing_cruise_family_instead_of_long_quintic():
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.5))
    assert plan.duration_s==pytest.approx(10.375)
    assert plan.state(2.)[0][0]==pytest.approx(.090625)
    assert_full_bounds(plan)


def test_moving_accelerating_far_target_has_continuous_brake_then_fast_cruise():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.5))
    assert plan.duration_s<11.
    np.testing.assert_allclose(plan.state(0.),(ZERO,axis(.01),axis(.1)),atol=1e-12,rtol=0)
    assert_full_bounds(plan)


def test_far_reversal_reports_old_direction_motion_without_resetting_velocity():
    plan=build_continuous_arrival(ZERO,axis(.03),axis(.1),axis(-.5))
    assert plan.duration_s<11.5
    assert plan.state(.04)[0][0]>0
    np.testing.assert_allclose(plan.state(0.),(ZERO,axis(.03),axis(.1)),atol=1e-12,rtol=0)
    assert_full_bounds(plan)


def test_cruise_alternative_can_fit_budget_rejected_by_single_settling_family():
    limits=replace(SettlingLimits(),max_duration_s=12.)
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.5),limits)
    assert plan.duration_s==pytest.approx(10.375)
    assert_full_bounds(plan)


def test_unchanged_target_during_bridge_reuses_full_plan_and_cursor():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.5))
    same,cursor=retarget_continuous_arrival(plan,.04,axis(.5))
    assert same is plan and cursor==.04


def test_mixed_joint_retargets_keep_full_pva_and_all_interval_bounds():
    initial=(ZERO,(.01,-.005,.003,0,0,0),(.1,-.05,.02,0,0,0))
    plan=build_continuous_arrival(*initial,(.1,-.05,.03,0,0,0))
    cursor=0.
    for interval,target in [(.02,(-.1,.05,-.03,0,0,0)),(.047,(.15,-.07,.01,0,0,0)),(.078,(.01,.02,-.04,0,0,0))]:
        state=plan.state(cursor+interval)
        plan,cursor=retarget_continuous_arrival(plan,cursor+interval,target)
        np.testing.assert_allclose(plan.state(0.),state,atol=1e-10,rtol=0)
        assert_full_bounds(plan)


def test_smooth_brake_equations_preserve_velocity_and_end_at_rest():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.5))
    brake=plan.segments[1]
    start=brake.state(0.)
    assert start[2]==pytest.approx(ZERO,abs=1e-12)
    for fraction in [0.,.25,.5,.75,1.]:
        q,v,a=brake.state(fraction*brake.duration_s)
        speed=start[1][0]
        assert q[0]==pytest.approx(start[0][0]+speed*brake.duration_s*(fraction-fraction**3+.5*fraction**4),abs=1e-12)
        assert v[0]==pytest.approx(speed*(1-3*fraction**2+2*fraction**3),abs=1e-12)
        assert a[0]==pytest.approx(speed/brake.duration_s*(-6*fraction+6*fraction**2),abs=1e-12)


def test_inputs_are_copied_and_completed_plan_holds_only_its_reference():
    q,v,a=np.zeros(6),np.array(axis(.01)),np.array(axis(.1))
    target=np.array(axis(.5))
    before=(q.copy(),v.copy(),a.copy(),target.copy())
    plan=build_continuous_arrival(q,v,a,target)
    for current,original in zip((q,v,a,target),before):np.testing.assert_array_equal(current,original)
    q[:]=1;v[:]=1;a[:]=1;target[:]=1
    np.testing.assert_allclose(plan.state(0.),(ZERO,axis(.01),axis(.1)),atol=1e-12,rtol=0)
    assert plan.state(plan.duration_s+1)==(axis(.5),ZERO,ZERO)


@pytest.mark.parametrize('elapsed',[-.01,float('nan'),float('inf')])
def test_invalid_elapsed_retarget_rejects_before_building_new_reference(elapsed):
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.01))
    with pytest.raises(ValueError,match='invalid_sample_time'):
        retarget_continuous_arrival(plan,elapsed,axis(.02))


def test_lowered_acceleration_cap_rejects_current_reference_instead_of_resetting_it():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.5))
    limits=replace(plan.limits,motion=replace(plan.limits.motion,acceleration_cap=.01))
    with pytest.raises(ValueError,match='initial_acceleration_limit'):
        retarget_continuous_arrival(plan,.02,axis(.5),limits)


@pytest.mark.parametrize('field', ['q','v','a','target'])
def test_nonfinite_initial_or_target_vectors_reject(field):
    values={'q':ZERO,'v':axis(.01),'a':axis(.1),'target':axis(.5)}
    values[field]=axis(float('nan'))
    with pytest.raises(ValueError,match='invalid_vector'):
        build_continuous_arrival(values['q'],values['v'],values['a'],values['target'])


def test_roundoff_rejection_of_cruise_preserves_existing_valid_settling_plan():
    # Brake's analytic q exceeds 3 by 5e-13 (within its range tolerance),
    # while the alternate cruise builder strictly rejects that start.
    # The independently found .08s settling path to exactly 3 must survive.
    plan=build_continuous_arrival(axis(2.9996000000005),axis(.01),ZERO,axis(3.))
    assert len(plan.segments)==1
    assert plan.duration_s==pytest.approx(.08)
    assert plan.state(0.)[0][0]==2.9996000000005
    assert plan.state(.08)==(axis(3.),ZERO,ZERO)
    assert_full_bounds(plan)


def test_out_of_bounds_observed_start_is_not_hidden_by_candidate_fallback():
    with pytest.raises(ValueError,match='initial_joint_limit'):
        build_continuous_arrival(axis(3.0000000000005),axis(.01),ZERO,axis(3.))


def test_short_cruise_can_fit_remaining_budget_below_settling_minimum():
    limits=replace(SettlingLimits(),max_duration_s=.12)
    plan=build_continuous_arrival(ZERO,axis(-.0046875),axis(.1171875),axis(-.000124),limits)
    assert plan.duration_s==pytest.approx(.08+2*np.sqrt(1.5e-6/.2),abs=1e-12)
    assert plan.duration_s<.12
    np.testing.assert_allclose(plan.state(0.),(ZERO,axis(-.0046875),axis(.1171875)),atol=1e-12,rtol=0)
    assert_full_bounds(plan)


def test_final_subpicosecond_reference_window_is_nonempty_and_does_not_extrapolate():
    plan=build_continuous_arrival(ZERO,ZERO,ZERO,axis(.5))
    elapsed=plan.duration_s-5e-13
    window=plan.window(elapsed,.08)
    assert len(window.segments)>0 and window.duration_s>0
    np.testing.assert_allclose(window.state(0.),plan.state(elapsed),atol=1e-10,rtol=0)
    with pytest.raises(ValueError,match='invalid_sample_time'):window.state(.000001)
