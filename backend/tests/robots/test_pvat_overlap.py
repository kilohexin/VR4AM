"""Declared quintic rolling references and explicit neutralization/deceleration."""
from dataclasses import replace

import numpy as np
import pytest

from tests.control.pvat_window_supervision_probe import WindowGuard
from tests.robots.pvat_cruise_candidate import build_cruise_plan
from tests.robots.pvat_overlap_probe import build_brake_reserve,start_overlap,renew_overlap
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import _position_range

ZERO=(0.,)*6
ACTUAL=RampLimits()
REFERENCE=replace(ACTUAL,speed_cap=.045,acceleration_cap=.18)


def axis(value): return (value,*ZERO[1:])


@pytest.mark.parametrize('sign',[-1.,1.])
def test_brake_branch_preserves_nonzero_initial_pva_and_stops_with_continuous_pva(sign):
    phases=build_brake_reserve(axis(.1),axis(sign*.02),axis(sign*.1))
    first,last=phases
    np.testing.assert_allclose(first.state(0.),(axis(.1),axis(sign*.02),axis(sign*.1)),atol=1e-12,rtol=0)
    # Integral of a(t)=a0*(1-t/.08) then smooth velocity reduction.
    q1=.1+sign*(.02*.08+.1*.08**2/3)
    v1=sign*.024
    np.testing.assert_allclose(first.state(.08),(axis(q1),axis(v1),ZERO),atol=1e-12,rtol=0)
    np.testing.assert_allclose(last.state(0.),first.state(.08),atol=1e-12,rtol=0)
    assert last.duration_s==pytest.approx(.18)
    np.testing.assert_allclose(last.state(last.duration_s),
                              (axis(q1+v1*.18/2),ZERO,ZERO),atol=1e-12,rtol=0)
    for phase in phases:
        speed,acceleration=phase.peak_abs_derivatives()
        assert max(speed)<=.05+1e-10 and max(acceleration)<=.2+1e-10


@pytest.mark.parametrize('q,v,a,reason',[
    (ZERO,axis(.049),axis(.1),'pvat_brake_reserve_limit'),
    (axis(2.999),axis(.02),ZERO,'pvat_brake_reserve_limit'),
    (ZERO,axis(.06),ZERO,'pvat_brake_reserve_limit'),
    (ZERO,ZERO,axis(.3),'pvat_brake_reserve_limit'),
    (ZERO,ZERO,axis(float('nan')),'invalid_vector'),
])
def test_unavailable_brake_branch_rejects_without_clamping_state(q,v,a,reason):
    with pytest.raises(ValueError,match=reason): build_brake_reserve(q,v,a)


@pytest.mark.parametrize('sign',[-1.,1.])
def test_brake_branch_interior_joint_violation_rejects_even_if_both_endpoints_are_inside(sign):
    # Neutralization ends at 2.99993333, but first moves past +3 internally.
    with pytest.raises(ValueError,match='pvat_brake_reserve_limit'):
        build_brake_reserve(axis(sign*2.99996),axis(sign*.005),axis(-sign*.2))


@pytest.mark.parametrize('target',[-.5,-.001,.001,.5])
def test_twenty_ms_overlaps_cross_all_phases_and_preserve_original_arrival_reference(target):
    plan=build_cruise_plan(ZERO,axis(target),REFERENCE)
    current=start_overlap(plan,0,ACTUAL)
    guard=WindowGuard();guard.commit(current.packet.segment,0)
    transitions=0
    for index in range(1,int(np.ceil(plan.duration_s/.02))+6):
        now=index*20_000_000
        inherited=current.packet.state((now-current.origin_ns)/1e9)
        guard.observe(*inherited)
        assert guard.check(now) is None
        proposed=renew_overlap(current,now)
        assert proposed.plan is plan and proposed.reference_origin_ns==0
        np.testing.assert_allclose(proposed.packet.state(0.),inherited,atol=1e-10,rtol=0)
        guard.commit(proposed.packet.segment,now)
        assert guard.observation is None
        speed,acceleration=proposed.packet.segment.peak_abs_derivatives()
        assert max(speed)<=.05+1e-10 and max(acceleration)<=.2+1e-10
        for phase in proposed.brake_reserve:
            lower,upper=_position_range(phase)
            assert np.all(lower>=-3.-1e-12) and np.all(upper<=3.+1e-12)
        transitions+=1;current=proposed
    assert transitions>10
    np.testing.assert_allclose(current.packet.state(current.packet.duration_s),
                              (axis(target),ZERO,ZERO),atol=1e-8,rtol=0)


def test_target_change_uses_covered_velocity_and_acceleration_then_keeps_new_plan():
    current=start_overlap(build_cruise_plan(ZERO,axis(.5),REFERENCE),0)
    inherited=current.packet.state(.04)
    assert inherited[1][0]>0 and inherited[2][0]>0
    changed=renew_overlap(current,40_000_000,axis(-.001))
    assert changed.plan.target==axis(-.001) and changed.reference_origin_ns==40_000_000
    np.testing.assert_allclose(changed.packet.state(0.),inherited,atol=1e-10,rtol=0)
    following=renew_overlap(changed,60_000_000,axis(-.001))
    assert following.plan is changed.plan
    assert following.reference_origin_ns==40_000_000
    assert current.plan.target==axis(.5) and current.origin_ns==0


@pytest.mark.parametrize('now',[-1,0,80_000_000,110_000_000,1.5,True])
def test_old_window_cannot_be_extrapolated_reordered_or_retimed(now):
    current=start_overlap(build_cruise_plan(ZERO,axis(.5),REFERENCE),0)
    with pytest.raises(ValueError,match='pvat_overlap_time'):
        renew_overlap(current,now)


@pytest.mark.parametrize('offset',[20_000_000,40_000_000,79_000_000])
def test_unchanged_explicit_target_does_not_restart_the_arrival_clock(offset):
    plan=build_cruise_plan(ZERO,axis(.5),REFERENCE)
    current=start_overlap(plan,0)
    updated=renew_overlap(current,offset,axis(.5))
    assert updated.plan is plan and updated.reference_origin_ns==0
    assert updated.origin_ns==offset


def test_no_margin_reference_rejects_missing_brake_reserve_before_cross_phase_fit():
    plan=build_cruise_plan(ZERO,axis(.5))
    current=start_overlap(plan,0)
    for index in range(1,12): current=renew_overlap(current,index*20_000_000)
    # At 240 ms, desired endpoint 320 ms has v=.04708883 and a=.10012444.
    # Neutralization for 80 ms would increase speed to .05109381 > .05.
    with pytest.raises(ValueError,match='pvat_brake_reserve_limit'):
        renew_overlap(current,240_000_000)
    assert current.origin_ns==220_000_000


def test_reference_exceeding_actual_curve_caps_rejects_without_clamping_endpoints():
    plan=build_cruise_plan(ZERO,axis(.5),replace(ACTUAL,acceleration_cap=.4))
    with pytest.raises(ValueError,match='pvat_window_interpolation_limit'):
        start_overlap(plan,0)


def test_six_axis_renewal_and_braking_keep_full_joint_vectors():
    target=(.5,-.25,.1,0.,.2,-.05)
    current=start_overlap(build_cruise_plan(ZERO,target,REFERENCE),0)
    for index in range(1,24): current=renew_overlap(current,index*20_000_000)
    for vector in current.packet.state(.04):
        np.testing.assert_allclose(vector,np.array(target)*vector[0]/.5,atol=1e-9,rtol=0)
    assert len(current.brake_reserve)==2


def test_large_epoch_origin_keeps_relative_plan_time_without_restarting_or_float_epoch_loss():
    origin=1_000_000_000_000_000_000
    plan=build_cruise_plan(ZERO,axis(.5),REFERENCE)
    current=start_overlap(plan,origin)
    updated=renew_overlap(current,origin+20_000_000)
    assert updated.reference_origin_ns==origin and updated.packet.start_s==.02
    np.testing.assert_allclose(updated.packet.state(0.),current.packet.state(.02),atol=1e-12,rtol=0)


@pytest.mark.parametrize('changes',[{'speed_cap':float('nan')},{'duration_s':0.},{'joint_max':(-3.,)*6}])
def test_invalid_limits_do_not_disable_overlap_or_braking_checks(changes):
    with pytest.raises(ValueError,match='invalid_motion_limits'):
        start_overlap(build_cruise_plan(ZERO,axis(.001),REFERENCE),0,replace(ACTUAL,**changes))
