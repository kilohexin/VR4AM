"""Explicit simulated ages/generations/PVA; no controller timing evidence."""
from dataclasses import replace

import numpy as np
import pytest

from tests.robots.pvat_continuous_arrival_candidate import build_continuous_arrival
from tests.robots.pvat_interpolation_probe import quintic_segment
from tests.robots.pvat_phase_encoding_probe import encode_phases
from tests.robots.pvat_window_decision_probe import retarget_arrival_window

ZERO=(0.,)*6
def axis(x):return (x,*ZERO[1:])


def packet():
    return encode_phases((quintic_segment(ZERO,axis(.02),ZERO,axis(.0016),axis(.02),ZERO,.08),))[0]


def decide(p=None,elapsed=.04,**changes):
    p=p or packet()
    options={'issued_generation':3,'current_generation':3,'frame_age_ns':40_000_000,
             'observed_pva':p.state(.04)}
    options.update(changes)
    return retarget_arrival_window(p,elapsed,axis(-.1),**options)


@pytest.mark.parametrize('issued,current',[(3,4),(3,2),(True,1),(-1,-1),(None,3)])
def test_generation_change_or_unknown_discards_reference_without_replacement(issued,current):
    d=decide(issued_generation=issued,current_generation=current)
    assert d.discard_reference and d.reason=='pvat_reference_generation_changed'
    assert d.packets==()


@pytest.mark.parametrize('age',[100_000_000,100_000_001,-1,None,True])
def test_old_or_unknown_vr_age_discards_without_renewing_ttl(age):
    d=decide(frame_age_ns=age)
    assert d.discard_reference and d.reason=='pvat_vr_frame_expired'
    assert d.packets==()


@pytest.mark.parametrize('elapsed',[.08,.080058,.11,-.001,float('nan'),None])
def test_late_or_unknown_execution_time_never_predicts_from_full_plan(elapsed):
    d=decide(elapsed=elapsed,frame_age_ns=0)
    assert d.discard_reference and d.reason=='pvat_issued_window_expired'
    assert d.packets==()


@pytest.mark.parametrize('component',[0,1,2])
def test_feedback_mismatch_discards_instead_of_replanning_from_prediction(component):
    observed=np.array(packet().state(.04))
    observed[component,0]+=.001
    d=decide(observed_pva=observed)
    assert d.discard_reference and d.reason=='pvat_reference_feedback_mismatch'
    assert d.packets==()


@pytest.mark.parametrize('feedback',[None,((0.,)*6,)*2,((float('nan'),)*6,)*3])
def test_missing_or_invalid_feedback_has_no_replacement_packets(feedback):
    d=decide(observed_pva=feedback)
    assert d.discard_reference and d.reason=='pvat_reference_feedback_unknown'
    assert d.packets==()


@pytest.mark.parametrize('tolerance',[(1e-3,)*2,(-1.,0.,0.),(float('nan'),)*3])
def test_invalid_feedback_tolerance_cannot_disable_comparison(tolerance):
    d=decide(tolerances=tolerance)
    assert d.discard_reference and d.reason=='invalid_reference_tolerances'
    assert d.packets==()


def test_matching_simulated_feedback_can_generate_continuous_phase_local_packets():
    p=packet()
    d=decide(p)
    assert d.discard_reference is False and d.reason is None and d.packets
    np.testing.assert_allclose(d.packets[0].state(0.),p.state(.04),atol=1e-10,rtol=0)
    for left,right in zip(d.packets,d.packets[1:]):
        np.testing.assert_allclose(left.state(left.duration_s),right.state(0.),atol=1e-10,rtol=0)
    np.testing.assert_allclose(d.packets[-1].state(d.packets[-1].duration_s),(axis(-.1),ZERO,ZERO),atol=1e-10,rtol=0)


def test_nonzero_acceleration_uses_new_continuous_arrival_instead_of_old_family_rejection():
    p=encode_phases((quintic_segment(ZERO,axis(.01),axis(.1),axis(.00112),axis(.018),axis(.1),.08),))[0]
    d=decide(p,elapsed=0.,observed_pva=p.state(0.))
    assert d.discard_reference is False and d.packets
    np.testing.assert_allclose(d.packets[0].state(0.),(ZERO,axis(.01),axis(.1)),atol=1e-10,rtol=0)


def test_reference_tolerance_boundary_is_inclusive_but_excess_is_not():
    observed=np.array(packet().state(.04));observed[0,0]+=.0005
    assert decide(observed_pva=observed,tolerances=(.000500001,0.,0.)).discard_reference is False
    assert decide(observed_pva=observed,tolerances=(.000499999,0.,0.)).discard_reference is True


def test_new_vr_frame_does_not_extend_expired_issued_window():
    d=decide(elapsed=.11,frame_age_ns=0)
    assert d.discard_reference and d.packets==()


def test_sdk_endpoint_fields_reconstruct_single_phase_of_complete_arrival_reference():
    plan=build_continuous_arrival(ZERO,axis(.01),axis(.1),axis(.5))
    encoded=encode_phases(plan.segments,plan.limits.motion)
    boundaries=np.cumsum([s.duration_s for s in plan.segments])[:-1]
    for p in encoded:
        assert not any(p.start_s+1e-12<b<p.start_s+p.duration_s-1e-12 for b in boundaries)
        args=p.sdk_arguments()
        reconstructed=quintic_segment(*p.state(0.),args['p'],args['v'],args['a'],args['t'])
        for t in [0.,p.duration_s/2,p.duration_s]:
            np.testing.assert_allclose(reconstructed.state(t),plan.state(p.start_s+t),atol=1e-9,rtol=0)
        speed,accel=reconstructed.peak_abs_derivatives()
        assert max(speed)<=.05+1e-10 and max(accel)<=.2+1e-10


@pytest.mark.parametrize('bad',[None,'packet'])
def test_unknown_issued_reference_has_no_replacement(bad):
    d=retarget_arrival_window(bad,.04,axis(.1),issued_generation=3,current_generation=3,
        frame_age_ns=0,observed_pva=(ZERO,ZERO,ZERO))
    assert d.discard_reference and d.reason=='invalid_issued_reference' and d.packets==()


def test_oversized_packet_is_not_repartitioned_and_mistaken_for_issued_coverage():
    p=replace(packet(),segment=quintic_segment(ZERO,axis(.02),ZERO,axis(.0018),axis(.02),ZERO,.09))
    d=decide(p)
    assert d.discard_reference and d.reason=='invalid_issued_reference' and d.packets==()


def test_tiny_phase_rejection_discards_entire_plan_instead_of_merging_phases():
    p=encode_phases((quintic_segment(ZERO,ZERO,ZERO,ZERO,ZERO,ZERO,.08),))[0]
    d=retarget_arrival_window(p,.04,axis(.000001),issued_generation=3,current_generation=3,
        frame_age_ns=0,observed_pva=(ZERO,ZERO,ZERO))
    assert d.discard_reference and d.reason=='pvat_phase_too_short' and d.packets==()


def test_used_feedback_and_endpoint_argument_copies_do_not_alias_caller_arrays():
    p=packet();feedback=np.array(p.state(.04));before=feedback.copy()
    d=decide(p,observed_pva=feedback)
    assert not d.discard_reference
    np.testing.assert_array_equal(feedback,before)
    args=d.packets[0].sdk_arguments();args['p'][0]=999
    assert d.packets[0].sdk_arguments()['p'][0]!=999
    feedback[:]=999
    np.testing.assert_allclose(d.packets[0].state(0.),before,atol=1e-10,rtol=0)


def test_feedback_within_declared_tolerance_remains_the_actual_planner_input():
    observed=np.array(packet().state(.04));observed[0,0]+=.0001
    d=decide(observed_pva=observed,tolerances=(.001,0.,0.))
    assert not d.discard_reference
    np.testing.assert_allclose(d.packets[0].state(0.),observed,atol=1e-10,rtol=0)


def test_simulated_age_just_inside_original_ttl_is_not_renewed():
    assert not decide(frame_age_ns=99_999_999).discard_reference
    d=decide(frame_age_ns=100_000_000)
    assert d.discard_reference and d.packets==()


@pytest.mark.parametrize('target',[{},None,(0.,)*5,(float('nan'),)*6])
def test_invalid_target_returns_discard_decision_instead_of_escaping_interface(target):
    p=packet()
    d=retarget_arrival_window(p,.04,target,issued_generation=3,current_generation=3,
        frame_age_ns=0,observed_pva=p.state(.04))
    assert d.discard_reference and d.reason=='invalid_reference_target' and d.packets==()


@pytest.mark.parametrize('limits',[{},False,'limits'])
def test_invalid_limit_object_returns_discard_decision_without_attribute_error(limits):
    d=decide(limits=limits)
    assert d.discard_reference and d.reason=='invalid_motion_limits' and d.packets==()
