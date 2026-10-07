"""Synthetic time hypotheses are explicit; observed consistency is not arrival."""
from dataclasses import replace

import numpy as np
import pytest

from app.control.target_strategy import TargetContext
from app.robots.motion_view import AcceptedTarget, JointObservation, MotionView
from tests.control.pvat_alignment_probe import (
    ReferenceWindow, SyntheticTiming, TerminalFeedbackProbe, compare_reference,
)
from tests.control.pvat_observation_probe import ObservationBinding, qualify_observation
from tests.control.test_pvat_observation import COMMAND
from tests.control.pvat_window_supervision_probe import WindowGuard
from tests.robots.pvat_interpolation_probe import quintic_segment

ZERO = (0.,) * 6
ORIGIN = 1_000_000_000
axis = lambda value: (value, *ZERO[1:])
TIMING = SyntheticTiming(ORIGIN, ORIGIN, True, True)


def linear_reference():
    return ReferenceWindow(quintic_segment(ZERO, axis(.02), ZERO, axis(.0016), axis(.02), ZERO, .08),
                           ORIGIN, 7, COMMAND)


def fixed_reference():
    return ReferenceWindow(quintic_segment(axis(.001), ZERO, ZERO, axis(.001), ZERO, ZERO, .08),
                           ORIGIN, 7, COMMAND)


def observed(reference=None, *, sample_ms=20, captured_ms=15, completed_ms=25, checked_ms=40, state=None):
    reference = reference or linear_reference()
    q, v, a = state if state is not None else reference.segment.state(sample_ms / 1000)
    sample = JointObservation(COMMAND.generation, ORIGIN + captured_ms * 1_000_000,
                              ORIGIN + completed_ms * 1_000_000, 'state', q, v, a)
    now = ORIGIN + checked_ms * 1_000_000
    view = MotionView(now, COMMAND.generation, True, True, None, None, True, False,
                      AcceptedTarget(COMMAND, (.3, 0., .3), (0., 0., 0., 1.), ZERO), sample)
    return qualify_observation(view, TargetContext(7, now, True, True),
                               ObservationBinding(7, COMMAND, ORIGIN), max_age_ns=60_000_000)


def compare(reference=None, evidence=None, **kwargs):
    reference = reference or linear_reference()
    return compare_reference(reference, evidence or observed(reference), timing=kwargs.pop('timing', TIMING), **kwargs)


def test_cached_sample_is_compared_to_read_interval_not_view_time():
    reference = linear_reference(); evidence = observed(reference)
    old = WindowGuard(); old.commit(reference.segment, ORIGIN)
    old.observe(evidence.sample.actual_q, evidence.sample.actual_qd, evidence.sample.actual_qdd)
    assert old.check(evidence.checked_ns) == 'pvat_reference_position_mismatch'
    result = compare(reference, evidence)
    assert result.possible_reference_time_s[0] == pytest.approx(.015)
    assert result.possible_reference_time_s[1] == pytest.approx(.025)
    assert .015 <= result.witness_time_s <= .025
    assert result.scope == 'synthetic_time_compatibility'
    assert evidence.sample.captured_ns == ORIGIN + 15_000_000


@pytest.mark.parametrize('timing', [None, replace(TIMING, sample_within_read=False),
                                   replace(TIMING, synchronous_pva=False)])
def test_sdk_local_timestamps_alone_cannot_assert_controller_sample_time(timing):
    with pytest.raises(ValueError, match='pvat_alignment_time_unknown'):
        compare(timing=timing)


def test_execution_origin_uncertainty_is_kept_instead_of_chosen_midpoint():
    timing = replace(TIMING, execution_origin_earliest_ns=ORIGIN+5_000_000,
                     execution_origin_latest_ns=ORIGIN+10_000_000)
    result = compare(evidence=observed(sample_ms=15, captured_ms=20), timing=timing)
    assert result.possible_reference_time_s == pytest.approx((.010, .020))


@pytest.mark.parametrize('field,value', [('origin_ns', True), ('owner_generation', True),
                                       ('origin_ns', -1)])
def test_invalid_reference_metadata_rejects(field, value):
    with pytest.raises(ValueError, match='invalid_alignment_reference'):
        compare(replace(linear_reference(), **{field: value}))


@pytest.mark.parametrize('field,value', [('execution_origin_earliest_ns', True),
                                       ('execution_origin_latest_ns', ORIGIN-1)])
def test_invalid_execution_interval_rejects(field, value):
    with pytest.raises(ValueError, match='invalid_alignment_timing'):
        compare(timing=replace(TIMING, **{field: value}))


@pytest.mark.parametrize('change', ['owner', 'command'])
def test_qualified_observation_cannot_bind_another_reference(change):
    reference = linear_reference()
    reference = replace(reference, **({'owner_generation': 8} if change == 'owner' else
                                     {'command': replace(COMMAND, submission_id=10)}))
    with pytest.raises(ValueError, match='pvat_alignment_binding'):
        compare(reference)


@pytest.mark.parametrize('kind', ['read_before_start', 'read_after_end', 'check_at_deadline'])
def test_no_reference_extrapolation_or_coverage_extension(kind):
    if kind == 'read_before_start':
        kwargs = {'timing': replace(TIMING, execution_origin_latest_ns=ORIGIN+20_000_000)}
    elif kind == 'read_after_end':
        kwargs = {'timing': replace(TIMING, execution_origin_earliest_ns=ORIGIN-60_000_000,
                                    execution_origin_latest_ns=ORIGIN-60_000_000)}
    else:
        kwargs = {'evidence': observed(captured_ms=30, completed_ms=40, checked_ms=80)}
    with pytest.raises(ValueError, match='pvat_alignment_coverage'):
        compare(**kwargs)


def test_joint_ranges_cannot_mix_different_times_into_one_sample():
    q1 = (.02*.016, .02*.024, *ZERO[2:])
    v = (.02, .02, *ZERO[2:])
    reference = ReferenceWindow(quintic_segment(ZERO, v, ZERO, (.0016,.0016,*ZERO[2:]), v, ZERO,.08),
                                ORIGIN,7,COMMAND)
    with pytest.raises(ValueError, match='pvat_alignment_incompatible'):
        compare(reference, observed(reference,state=(q1,v,ZERO)), tolerances=(1e-5,1e-4,1e-3))


def test_position_and_velocity_must_also_share_one_reference_time():
    reference = ReferenceWindow(quintic_segment(ZERO,ZERO,axis(.1),axis(.00032),axis(.008),axis(.1),.08),
                                ORIGIN,7,COMMAND)
    mixed = (reference.segment.state(.016)[0], reference.segment.state(.024)[1], axis(.1))
    with pytest.raises(ValueError, match='pvat_alignment_incompatible'):
        compare(reference, observed(reference,state=mixed), tolerances=(1e-7,1e-5,1e-3))


@pytest.mark.parametrize('time_ms',[15,20,25])
def test_compatible_accelerating_state_including_interval_endpoints(time_ms):
    reference = ReferenceWindow(quintic_segment(ZERO,ZERO,axis(.1),axis(.00032),axis(.008),axis(.1),.08),
                                ORIGIN,7,COMMAND)
    result = compare(reference, observed(reference,sample_ms=time_ms), tolerances=(1e-8,1e-6,1e-5))
    actual = np.array((result.observation.sample.actual_q,result.observation.sample.actual_qd,
                       result.observation.sample.actual_qdd))
    assert np.all(np.abs(actual-np.array(reference.segment.state(result.witness_time_s))) <=
                  np.array((1e-8,1e-6,1e-5))[:,None]+1e-12)


@pytest.mark.parametrize('tolerances', [(0,1,1),(1,-1,1),(1,float('nan'),1),(1,1)])
def test_invalid_tolerance_never_turns_unknown_state_into_compatible(tolerances):
    with pytest.raises(ValueError,match='invalid_alignment_tolerances'):
        compare(tolerances=tolerances)


def test_two_independent_close_samples_only_report_consistency_not_completion():
    reference = fixed_reference(); probe = TerminalFeedbackProbe(reference,axis(.001))
    first = observed(reference,captured_ms=10,completed_ms=11,checked_ms=12)
    second = observed(reference,captured_ms=30,completed_ms=31,checked_ms=32)
    assert probe.observe(first,timing=TIMING) == 'awaiting_feedback'
    assert probe.observe(second,timing=TIMING) == 'feedback_consistent'
    assert probe.good_samples == 2
    assert probe.check(ORIGIN+80_000_000) == 'pvat_window_expired'
    assert probe.phase == 'fault'


def test_repeated_cache_and_overlapping_reads_do_not_accumulate_settle_samples():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    first=observed(reference,captured_ms=10,completed_ms=25,checked_ms=25)
    probe.observe(first,timing=TIMING)
    assert probe.observe(first,timing=TIMING)=='awaiting_feedback'
    probe.observe(observed(reference,captured_ms=20,completed_ms=26,checked_ms=27),timing=TIMING)
    assert probe.good_samples==1


def test_long_sample_gap_restarts_evidence_run_without_moving_deadline():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    probe.observe(observed(reference,captured_ms=10,completed_ms=11,checked_ms=12),timing=TIMING)
    assert probe.observe(observed(reference,captured_ms=45,completed_ms=46,checked_ms=47),timing=TIMING)=='awaiting_feedback'
    assert probe.good_samples==1 and probe.deadline_ns==ORIGIN+80_000_000


def test_terminal_target_change_faults_without_zeroing_state_or_replanning():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    assert probe.retarget(axis(-.001),ORIGIN+20_000_000)=='pvat_terminal_target_changed'
    assert probe.target==axis(.001) and probe.deadline_ns==ORIGIN+80_000_000
    probe.revoke()
    assert probe.check(ORIGIN+21_000_000)=='pvat_terminal_target_changed'


def test_revocation_discards_feedback_and_cannot_reactivate_reference():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    probe.observe(observed(reference),timing=TIMING);probe.revoke()
    assert probe.phase=='revoked' and probe.good_samples==0
    assert probe.observe(observed(reference),timing=TIMING)=='revoked'


def test_missing_time_model_faults_terminal_probe_instead_of_claiming_completion():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    assert probe.observe(observed(reference))=='pvat_alignment_time_unknown'
    assert probe.phase=='fault' and probe.good_samples==0


def test_terminal_deadline_has_priority_over_late_matching_feedback():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    evidence=observed(reference,captured_ms=60,completed_ms=70,checked_ms=80)
    assert probe.observe(evidence,timing=TIMING)=='pvat_window_expired'
    assert probe.good_samples==0


@pytest.mark.parametrize('field,value', [('owner_generation',True),('command',replace(COMMAND,generation=True))])
def test_tampered_qualified_binding_is_rejected_instead_of_integer_aliasing(field,value):
    reference=replace(linear_reference(),owner_generation=1) if field=='owner_generation' else linear_reference()
    evidence=replace(observed(),**{field:value})
    if field=='command':
        reference=replace(reference,command=replace(COMMAND,generation=1))
        evidence=replace(evidence,sample=replace(evidence.sample,generation=1))
    with pytest.raises(ValueError,match='pvat_alignment_binding'):
        compare(reference,evidence)


def test_fractional_nanosecond_duration_does_not_allow_sampling_past_polynomial_end():
    original=fixed_reference();reference=replace(original,segment=replace(original.segment,duration_s=.0799999996))
    evidence=observed(reference,captured_ms=15,completed_ms=25,checked_ms=40)
    timing=replace(TIMING,execution_origin_earliest_ns=ORIGIN-55_000_000,
                   execution_origin_latest_ns=ORIGIN-55_000_000)
    with pytest.raises(ValueError,match='pvat_alignment_coverage'):
        compare(reference,evidence,timing=timing)


def test_large_monotonic_epoch_preserves_small_relative_read_interval():
    offset=10**18
    reference=replace(linear_reference(),origin_ns=ORIGIN+offset)
    evidence=observed();sample=evidence.sample
    evidence=replace(evidence,checked_ns=evidence.checked_ns+offset,
                     sample=replace(sample,captured_ns=sample.captured_ns+offset,
                                    read_completed_ns=sample.read_completed_ns+offset))
    timing=replace(TIMING,execution_origin_earliest_ns=ORIGIN+offset,execution_origin_latest_ns=ORIGIN+offset)
    assert compare(reference,evidence,timing=timing).possible_reference_time_s==pytest.approx((.015,.025))


def test_terminal_state_still_moving_resets_close_sample_run():
    reference=ReferenceWindow(quintic_segment(ZERO,axis(.01),ZERO,axis(.001),ZERO,ZERO,.08),ORIGIN,7,COMMAND)
    probe=TerminalFeedbackProbe(reference,axis(.001))
    assert probe.observe(observed(reference,sample_ms=20,captured_ms=19,completed_ms=21,checked_ms=22),
                         timing=TIMING)=='awaiting_feedback'
    assert probe.good_samples==0


def test_same_terminal_target_does_not_extend_deadline_or_erase_feedback():
    reference=fixed_reference();probe=TerminalFeedbackProbe(reference,axis(.001))
    probe.observe(observed(reference,captured_ms=10,completed_ms=11,checked_ms=12),timing=TIMING)
    assert probe.retarget(axis(.001),ORIGIN+20_000_000)=='awaiting_feedback'
    assert probe.good_samples==1 and probe.deadline_ns==ORIGIN+80_000_000


def test_terminal_clock_reversal_latches_and_cannot_be_cleared_by_revoke():
    probe=TerminalFeedbackProbe(fixed_reference(),axis(.001))
    probe.check(ORIGIN+20_000_000)
    assert probe.check(ORIGIN+19_000_000)=='pvat_reference_clock_reversed'
    probe.revoke()
    assert probe.check(ORIGIN+21_000_000)=='pvat_reference_clock_reversed'


def test_existing_finite_terminal_cannot_fake_twenty_ms_of_settled_feedback_before_expiry():
    from tests.robots.pvat_cruise_candidate import build_cruise_plan
    from tests.robots.pvat_overlap_probe import start_overlap,renew_overlap
    from tests.robots.pvat_ramp_candidate import RampLimits
    from tests.robots.pvat_terminal_probe import start_terminal,renew_terminal
    plan=build_cruise_plan(ZERO,axis(.001),replace(RampLimits(),speed_cap=.045,acceleration_cap=.18))
    terminal=start_terminal(renew_overlap(start_overlap(plan,0),20_000_000),40_000_000)
    for ms in (60,80,100,120):terminal=renew_terminal(terminal,ms*1_000_000)
    assert terminal.packet.duration_s==pytest.approx(.062574186)
    reference=ReferenceWindow(terminal.packet.segment,ORIGIN,7,COMMAND)
    probe=TerminalFeedbackProbe(reference,axis(.001))
    for ms in (60,61,62):
        result=probe.observe(observed(reference,sample_ms=ms,captured_ms=ms,completed_ms=ms,checked_ms=ms),
                             timing=TIMING)
        assert result=='awaiting_feedback'
    assert probe.good_samples==2  # 61/62 ms are close, but only 1 ms apart.
    assert probe.check(ORIGIN+62_574_186)=='pvat_window_expired'


def test_narrow_feasible_interval_is_found_without_fixed_time_sampling():
    sample_ms=19.123456
    # Use the qualified integer read interval, with a deliberately off-grid PVA.
    evidence=observed(sample_ms=sample_ms)
    result=compare(evidence=evidence,tolerances=(1e-9,1e-8,1e-7))
    assert abs(result.witness_time_s-sample_ms/1000)<=5.01e-8
