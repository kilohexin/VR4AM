"""No SDK calls: exact declared reference encoding and covered-state retargeting."""
from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from tests.robots.pvat_cruise_candidate import build_cruise_plan
from tests.robots.pvat_interpolation_probe import QuinticSegment, quintic_segment
from tests.robots.pvat_phase_encoding_probe import encode_phases, retarget_window
from tests.robots.pvat_ramp_candidate import RampLimits

ZERO = (0.,) * 6


def axis(value):
    return (value, *ZERO[1:])


def constant_velocity_window():
    return encode_phases((quintic_segment(
        ZERO, axis(.02), ZERO, axis(.0016), axis(.02), ZERO, .08),))[0]


@pytest.mark.parametrize('distance,count', [(-.5,131), (-.001,4), (.001,4), (.5,131)])
def test_balanced_phase_windows_keep_duration_and_continuity_without_tiny_tail(distance,count):
    plan = build_cruise_plan(ZERO, axis(distance))
    packets = encode_phases(plan.segments)
    assert len(packets) == count
    assert sum(p.duration_s for p in packets) == pytest.approx(plan.duration_s)
    assert all(.02 <= p.duration_s <= .08 for p in packets)
    for left,right in zip(packets,packets[1:]):
        assert right.start_s == pytest.approx(left.start_s+left.duration_s)
        np.testing.assert_allclose(left.state(left.duration_s),right.state(0.),atol=1e-10,rtol=0)
    np.testing.assert_allclose(packets[-1].state(packets[-1].duration_s),
                               (axis(distance),ZERO,ZERO),atol=1e-10,rtol=0)


def analytic_far_state(time):
    # Independent integrals for V=.05, R=.375, H=10.375.
    if time < .375:
        u = time/.375
        return (axis(.01875*(u**3-.5*u**4)),
                axis(.05*(3*u*u-2*u**3)), axis(.8*u*(1-u)))
    if time <= 10.:
        return axis(.009375+.05*(time-.375)),axis(.05),ZERO
    q,v,a = analytic_far_state(max(0.,10.375-time))
    return axis(.5-q[0]),v,axis(-a[0])


def test_every_encoded_curve_matches_independent_piecewise_integrals_and_caps():
    packets = encode_phases(build_cruise_plan(ZERO,axis(.5)).segments)
    for packet in packets:
        speed,acceleration = packet.segment.peak_abs_derivatives()
        assert max(speed) <= .05+1e-10
        assert max(acceleration) <= .2+1e-10
        for time in np.linspace(0.,packet.duration_s,9):
            np.testing.assert_allclose(packet.state(float(time)),
                analytic_far_state(packet.start_s+float(time)),atol=1e-9,rtol=0)
        for boundary in (.375,10.):
            assert not packet.start_s+1e-12 < boundary < packet.start_s+packet.duration_s-1e-12


def test_explicit_endpoints_encode_all_six_axes_and_arguments_are_independent_copies():
    plan = build_cruise_plan(ZERO,(.5,-.25,.1,0.,.2,-.05))
    packet = encode_phases(plan.segments)[0]
    args = packet.sdk_arguments()
    assert set(args) == {'p','v','a','t'}
    assert args['t'] == pytest.approx(.075)
    np.testing.assert_allclose((args['p'],args['v'],args['a']),packet.state(.075),atol=1e-10,rtol=0)
    args['p'][0] = 999.
    assert packet.sdk_arguments()['p'][0] != 999.
    with pytest.raises(FrozenInstanceError): packet.start_s = 1.
    for time in (0.,.03,.075):
        q,v,a = packet.state(time)
        for vec in (q,v,a):
            np.testing.assert_allclose(vec,np.array(plan.target)*vec[0]/.5,atol=1e-10,rtol=0)


@pytest.mark.parametrize('options', [
    {'min_duration_s':0.}, {'max_duration_s':float('nan')},
    {'min_duration_s':.1}, {'max_points':0}, {'max_points':1.5}, {'max_points':True}])
def test_invalid_encoding_limits_reject(options):
    with pytest.raises(ValueError,match='invalid_encoding_limits'):
        encode_phases(build_cruise_plan(ZERO,axis(.001)).segments,**options)


def test_tiny_phase_rejects_instead_of_sending_below_declared_minimum():
    with pytest.raises(ValueError,match='pvat_phase_too_short'):
        encode_phases(build_cruise_plan(ZERO,axis(.00001)).segments)


def test_packet_budget_rejects_before_returning_any_partial_sequence():
    with pytest.raises(ValueError,match='pvat_encoding_budget'):
        encode_phases(build_cruise_plan(ZERO,axis(.5)).segments,max_points=130)


def test_discontinuous_input_phases_cannot_be_encoded_as_one_reference():
    first = quintic_segment(ZERO,ZERO,ZERO,ZERO,ZERO,ZERO,.08)
    other = quintic_segment(axis(.001),ZERO,ZERO,axis(.001),ZERO,ZERO,.08)
    with pytest.raises(ValueError,match='pvat_reference_discontinuous'):
        encode_phases((first,other))


@pytest.mark.parametrize('segments', [(),
    (QuinticSegment(((0.,)*6,)*5,.08),),
    (QuinticSegment(((float('nan'),0.,0.,0.,0.,0.),)*6,.08),),
    (QuinticSegment(((0.,)*6,)*6,0.),)])
def test_malformed_phase_data_rejects(segments):
    with pytest.raises(ValueError,match='invalid_phases'): encode_phases(segments)


@pytest.mark.parametrize('change', [
    {'speed_cap':float('nan')}, {'acceleration_cap':0.}, {'joint_min':(0.,)*5},
    {'joint_min':(3.,)*6}])
def test_invalid_motion_limits_cannot_disable_curve_checks(change):
    with pytest.raises(ValueError,match='invalid_motion_limits'):
        encode_phases(build_cruise_plan(ZERO,axis(.001)).segments,replace(RampLimits(),**change))


@pytest.mark.parametrize('kind', ['speed','acceleration','position'])
def test_interior_or_endpoint_violations_reject_entire_sequence(kind):
    if kind == 'speed':
        segment = quintic_segment(ZERO,axis(.1),ZERO,axis(.008),axis(.1),ZERO,.08)
    elif kind == 'acceleration':
        segment = quintic_segment(ZERO,ZERO,ZERO,axis(.001),ZERO,ZERO,.08)
    else:
        # Endpoint positions are in range; interior leaves the joint interval.
        segment = quintic_segment(axis(3.),axis(.001),ZERO,axis(3.),axis(-.001),ZERO,.08)
    with pytest.raises(ValueError,match='pvat_window_interpolation_limit'):
        encode_phases((segment,))


def test_retarget_inherits_nonzero_velocity_and_brakes_before_reversing():
    packet = constant_velocity_window()
    updated = retarget_window(packet,.04,axis(-.001))
    np.testing.assert_allclose(updated[0].state(0.),(axis(.0008),axis(.02),ZERO),atol=1e-10,rtol=0)
    q = [p.state(0.)[0][0] for p in updated]
    assert max(q) > .0008  # Braking travel is preserved, not snapped to new target.
    assert any(p.state(0.)[1][0] < 0 for p in updated)
    for p in updated:
        speed,acceleration = p.segment.peak_abs_derivatives()
        assert max(speed) <= .05+1e-10 and max(acceleration) <= .2+1e-10
    np.testing.assert_allclose(updated[-1].state(updated[-1].duration_s),
                               (axis(-.001),ZERO,ZERO),atol=1e-10,rtol=0)


def test_retarget_preserves_acceleration_from_covered_reference():
    packet = encode_phases(build_cruise_plan(ZERO,axis(.5)).segments)[0]
    updated = retarget_window(packet,.04,axis(.001))
    u = .04/.375
    expected = (axis(.01875*(u**3-.5*u**4)),axis(.05*(3*u*u-2*u**3)),axis(.8*u*(1-u)))
    assert expected[2][0] > 0
    np.testing.assert_allclose(updated[0].state(0.),expected,atol=1e-10,rtol=0)


@pytest.mark.parametrize('elapsed',[-.001,.08,.11,float('nan')])
def test_retarget_cannot_predict_from_expired_or_invalid_issued_window(elapsed):
    with pytest.raises(ValueError,match='pvat_retarget_window_expired'):
        retarget_window(constant_velocity_window(),elapsed,axis(-.001))


def test_retarget_keeps_known_moving_state_family_rejection_instead_of_zeroing_feedback():
    segment = quintic_segment(ZERO,axis(.01),axis(.1),axis(.00112),axis(.018),axis(.1),.08)
    packet = encode_phases((segment,))[0]
    with pytest.raises(ValueError,match='no_feasible_settling_plan'):
        retarget_window(packet,0.,axis(.1))
