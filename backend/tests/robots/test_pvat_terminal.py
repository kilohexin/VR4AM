"""Finite exact restrictions eliminate terminal refit overshoot without a snap."""
from dataclasses import replace

import numpy as np
import pytest

from tests.control.pvat_window_supervision_probe import WindowGuard
from tests.robots.pvat_cruise_candidate import build_cruise_plan
from tests.robots.pvat_interpolation_probe import quintic_segment
from tests.robots.pvat_overlap_probe import start_overlap,renew_overlap
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import _position_range
from tests.robots.pvat_terminal_probe import start_terminal,renew_terminal

ZERO=(0.,)*6
ACTUAL=RampLimits()
REFERENCE=replace(ACTUAL,speed_cap=.045,acceleration_cap=.18)


def axis(value):return (value,*ZERO[1:])


def before_terminal(target):
    plan=build_cruise_plan(ZERO,target,REFERENCE)
    current=start_overlap(plan,0);now=20_000_000
    while plan.duration_s-now/1e9>.16:
        current=renew_overlap(current,now);now+=20_000_000
    return plan,current,now


@pytest.mark.parametrize('distance',[-.5,-.001,.001,.5])
def test_finite_terminal_windows_preserve_pva_and_never_cross_fixed_target(distance):
    plan,current,now=before_terminal(axis(distance))
    inherited=current.packet.state((now-current.origin_ns)/1e9)
    terminal=start_terminal(current,now)
    guard=WindowGuard();guard.commit(current.packet.segment,current.origin_ns)
    guard.commit(terminal.packet.segment,now)
    deadline=round(plan.duration_s*1e9)
    assert terminal.deadline_ns==deadline
    np.testing.assert_allclose(terminal.packet.state(0.),inherited,atol=1e-10,rtol=0)
    seen_final=False;refreshes=0
    while now<deadline:
        qmin,qmax=_position_range(terminal.packet.segment)
        assert min(0.,distance)-1e-12<=qmin[0]
        assert qmax[0]<=max(0.,distance)+1e-12
        speed,acceleration=terminal.packet.segment.peak_abs_derivatives()
        assert max(speed)<=.05+1e-10 and max(acceleration)<=.2+1e-10
        for time in np.linspace(0.,terminal.packet.duration_s,9):
            reference_time=terminal.packet.start_s+float(time)
            assert reference_time<=terminal.segment.duration_s+1e-12
            np.testing.assert_allclose(terminal.packet.state(float(time)),
                terminal.segment.state(min(reference_time,terminal.segment.duration_s)),atol=1e-9,rtol=0)
        guard.observe(*terminal.packet.state((now-terminal.origin_ns)/1e9))
        assert guard.check(now) is None
        now+=20_000_000
        if now>=deadline:break
        prior=terminal;terminal=renew_terminal(terminal,now)
        if terminal.refresh:
            guard.commit(terminal.packet.segment,terminal.origin_ns);refreshes+=1
        else:
            assert terminal.packet is prior.packet and terminal.origin_ns==prior.origin_ns
            seen_final=True
    assert seen_final and refreshes>0
    assert terminal.origin_ns+round(terminal.packet.duration_s*1e9)==deadline
    np.testing.assert_allclose(terminal.packet.state(terminal.packet.duration_s),
                              (axis(distance),ZERO,ZERO),atol=1e-10,rtol=0)
    assert guard.check(deadline)=='pvat_window_expired'
    with pytest.raises(ValueError,match='pvat_terminal_time'):renew_terminal(terminal,deadline)


def test_final_window_is_retained_instead_of_sending_a_tiny_tail_or_extending_coverage():
    _,current,now=before_terminal(axis(.001))
    terminal=start_terminal(current,now)
    for timestamp in (60,80,100,120):terminal=renew_terminal(terminal,timestamp*1_000_000)
    assert terminal.refresh and terminal.origin_ns==120_000_000
    assert terminal.packet.duration_s==pytest.approx(.062574186)
    prior=terminal
    for timestamp in (140,160,180):
        terminal=renew_terminal(terminal,timestamp*1_000_000)
        assert not terminal.refresh and terminal.packet is prior.packet
        assert terminal.deadline_ns==182_574_186
    assert terminal.deadline_ns-terminal.checked_ns==2_574_186
    with pytest.raises(ValueError,match='pvat_terminal_time'):renew_terminal(terminal,200_000_000)


def test_capture_preserves_nonzero_acceleration_and_reconstructed_wire_endpoints():
    _,current,now=before_terminal(axis(.001))
    initial=current.packet.state((now-current.origin_ns)/1e9)
    assert initial[1][0]>0 and initial[2][0]>0
    terminal=start_terminal(current,now)
    np.testing.assert_allclose(terminal.segment.state(0.),initial,atol=1e-12,rtol=0)
    args=terminal.packet.sdk_arguments()
    fit=quintic_segment(*initial,args['p'],args['v'],args['a'],args['t'])
    for time in np.linspace(0.,args['t'],9):
        np.testing.assert_allclose(fit.state(float(time)),terminal.packet.state(float(time)),atol=1e-10,rtol=0)


@pytest.mark.parametrize('now',[-1,0,80_000_000,110_000_000,1.5,True])
def test_capture_never_borrows_a_stale_or_retimed_old_window(now):
    previous=start_overlap(build_cruise_plan(ZERO,axis(.001),REFERENCE),0)
    with pytest.raises(ValueError,match='pvat_terminal_time'):start_terminal(previous,now)


def test_capture_before_declared_terminal_region_rejects_without_restarting_full_target():
    previous=start_overlap(build_cruise_plan(ZERO,axis(.5),REFERENCE),0)
    with pytest.raises(ValueError,match='pvat_terminal_region'):start_terminal(previous,20_000_000)


@pytest.mark.parametrize('now',[39_999_999,40_000_000,182_574_186,200_000_000,True,float('nan')])
def test_terminal_renewal_rejects_obsolete_or_expired_timestamps(now):
    _,current,origin=before_terminal(axis(.001));terminal=start_terminal(current,origin)
    with pytest.raises(ValueError,match='pvat_terminal_time'):renew_terminal(terminal,now)


def test_target_change_cannot_silently_reuse_stale_terminal_reference():
    _,current,origin=before_terminal(axis(.001));terminal=start_terminal(current,origin)
    with pytest.raises(ValueError,match='pvat_terminal_target_changed'):
        renew_terminal(terminal,60_000_000,axis(-.001))
    updated=renew_terminal(terminal,60_000_000,axis(.001))
    assert updated.segment is terminal.segment and updated.deadline_ns==terminal.deadline_ns


def test_late_nonfinal_refresh_rejects_short_new_window_without_clamping():
    _,current,origin=before_terminal(axis(.001));terminal=start_terminal(current,origin)
    terminal=renew_terminal(terminal,100_000_000) # Window ends at 180 ms, before arrival.
    with pytest.raises(ValueError,match='pvat_terminal_short_window'):
        renew_terminal(terminal,175_000_000)


@pytest.mark.parametrize('minimum',[0.,float('nan'),.2])
def test_invalid_minimum_cannot_disable_terminal_window_checks(minimum):
    _,current,origin=before_terminal(axis(.001))
    with pytest.raises(ValueError,match='invalid_terminal_limits'):
        start_terminal(current,origin,min_duration_s=minimum)


def test_capture_rejects_existing_reversal_instead_of_snapping_position_or_velocity():
    _,current,origin=before_terminal(axis(.001))
    # Same valid covered window, but target lies behind its positive velocity.
    changed=replace(current,plan=replace(current.plan,target=axis(-.001)))
    with pytest.raises(ValueError,match='pvat_terminal_not_monotonic'):
        start_terminal(changed,origin)


def test_terminal_renewal_keeps_all_six_axes_and_declared_bounds():
    target=(.001,-.0005,.0002,0.,.0004,-.0001)
    _,current,origin=before_terminal(target);terminal=start_terminal(current,origin)
    for timestamp in (60,80,100,120):terminal=renew_terminal(terminal,timestamp*1_000_000)
    for vector in terminal.packet.state(.03):
        np.testing.assert_allclose(vector,np.array(target)*vector[0]/.001,atol=1e-10,rtol=0)
    np.testing.assert_allclose(terminal.packet.state(terminal.packet.duration_s),
                              (target,ZERO,ZERO),atol=1e-10,rtol=0)
