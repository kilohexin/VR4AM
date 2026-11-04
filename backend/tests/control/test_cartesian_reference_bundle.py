import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import app.control.cartesian_reference_bundle as module
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.commissioning.native_jog_mdh import MdhTool


def rig(**options):
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start=JointReference((0.,-.8,1.4,-.6,1.57,0.))
    supervisor=module.OfflineReferenceSupervisor(model,start,**options)
    base=model.pose(start.q)
    target=base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',.15)).as_quat())})
    return model,supervisor,target


def test_each_admitted_reference_carries_an_immutable_contiguous_zero_rate_tail():
    _,s,target=rig()
    for _ in range(12):
        before=s.state;packet=s.advance(target)
        assert packet.segment.start==before and packet.segment.end==s.state
        assert packet.braking_tail[0].start==packet.segment.end
        chain=(packet.segment,)+packet.braking_tail
        assert all(a.end==b.start for a,b in zip(chain,chain[1:]))
        assert np.linalg.norm(chain[-1].end.v)<1e-8 and np.linalg.norm(chain[-1].end.a)<1e-8
        assert not packet.native_execution_verified


def test_rejected_new_candidate_consumes_the_previous_approved_tail_and_requires_capture(monkeypatch):
    _,s,target=rig();accepted=s.advance(target);tail=accepted.braking_tail
    def unavailable(*a,**kw):raise ValueError('cartesian_brake_no_feasible_plan')
    monkeypatch.setattr(module,'plan_cartesian_braking',unavailable)
    packets=[s.advance(target)]
    while s.phase!='ready':packets.append(s.advance(target))
    assert tuple(p.segment for p in packets)==tail
    assert all(p.mode=='braking' and p.reason=='no_candidate_with_braking_tail' for p in packets)
    with pytest.raises(ValueError,match='requires_capture'):s.advance(target)


def test_release_needs_no_new_planning_and_never_retargets_to_the_release_pose(monkeypatch):
    _,s,target=rig();packet=s.advance(target)
    monkeypatch.setattr(module,'plan_cartesian_braking',lambda *a,**kw:pytest.fail('tail already approved'))
    released=s.finish()
    assert released.segment==packet.braking_tail[0] and released.reason=='release'


def test_model_change_refuses_to_reuse_a_previously_approved_packet():
    model,s,target=rig();s.advance(target);model.upper[0]-=.01
    with pytest.raises(ValueError,match='model_changed'):s.finish()
    assert s.phase=='fault'


def test_unknown_planning_error_is_not_treated_as_a_known_mathematical_backoff(monkeypatch):
    _,s,target=rig();before=s.state
    def unknown(*a,**kw):raise RuntimeError('unknown')
    monkeypatch.setattr(module,'plan_cartesian_braking',unknown)
    with pytest.raises(RuntimeError):s.advance(target)
    assert s.state==before and s.phase=='fault'


@pytest.mark.parametrize('code',['cartesian_reference_nonfinite_or_shape','stop_unverified',
    'cartesian_reference_bundle_chain'])
def test_bad_data_unknown_stop_and_internal_chain_errors_do_not_use_mathematical_backoff(monkeypatch,code):
    _,s,target=rig();before=s.state
    def failure(*a,**kw):raise ValueError(code)
    monkeypatch.setattr(module,'plan_cartesian_braking',failure)
    with pytest.raises(ValueError,match=code):s.advance(target)
    assert s.state==before and s.phase=='fault'


def test_execution_head_mismatch_faults_instead_of_splicing_an_old_braking_tail():
    _,s,target=rig();s.advance(target)
    s.state=JointReference(tuple(np.array(s.state.q)+.001))
    with pytest.raises(ValueError,match='bundle_chain'):s.finish()
    assert s.phase=='fault'


def test_known_refusal_can_take_an_approved_deceleration_step_without_installing_the_bad_candidate(monkeypatch):
    _,s,target=rig(continue_on_known_rejection=True)
    accepted=s.advance(target);planner=module.plan_cartesian_braking
    def refuse(*a,**kw):raise ValueError('cartesian_brake_no_feasible_plan')
    monkeypatch.setattr(module,'plan_cartesian_braking',refuse)
    delayed=s.advance(target)
    assert delayed.segment==accepted.braking_tail[0]
    assert delayed.mode=='governed_deceleration' and s.phase=='following'
    monkeypatch.setattr(module,'plan_cartesian_braking',planner)
    resumed=s.advance(target)
    assert resumed.segment.start==delayed.segment.end and resumed.mode=='following'


def test_release_stays_latched_even_when_known_refusal_continuation_is_selected():
    _,s,target=rig(continue_on_known_rejection=True);s.advance(target)
    packet=s.finish()
    assert packet.mode=='braking'
    while s.phase!='ready':assert s.advance(target).mode=='braking'
    with pytest.raises(ValueError,match='requires_capture'):s.advance(target)


def test_continuation_option_never_recovers_unknown_stop_or_bad_data(monkeypatch):
    _,s,target=rig(continue_on_known_rejection=True)
    def refuse(*a,**kw):raise ValueError('stop_unverified')
    monkeypatch.setattr(module,'plan_cartesian_braking',refuse)
    with pytest.raises(ValueError,match='stop_unverified'):s.advance(target)
    assert s.phase=='fault'


def test_reseed_error_after_a_quiet_deceleration_remains_a_fault(monkeypatch):
    _,s,target=rig(continue_on_known_rejection=True)
    s._tail=();s._tail_checks=()
    def unknown(*a,**kw):raise RuntimeError('unknown')
    monkeypatch.setattr(module,'plan_cartesian_braking',unknown)
    with pytest.raises(RuntimeError):s.advance(target)
    assert s.phase=='fault'


def test_work_exhaustion_consumes_only_a_previously_validated_tail_and_does_not_retry_scales():
    _,s,target=rig(continue_on_known_rejection=True,braking_segment_limit=1)
    tail=s._tail
    packet=s.advance(target)
    assert packet.mode=='governed_deceleration' and packet.segment==tail[0]
    assert s.last_braking_segment_solves==1
    assert len(s.last_rejections)==1 and s.last_rejections[0]['code']=='cartesian_brake_search_budget_exhausted'
    assert not packet.native_execution_verified


def test_work_budget_is_shared_across_demand_scales(monkeypatch):
    _,s,target=rig(continue_on_known_rejection=True,braking_segment_limit=2)
    def expensive(*args,segment_budget=None,**kwargs):
        segment_budget.consume();segment_budget.consume()
        raise ValueError('cartesian_brake_no_feasible_plan')
    monkeypatch.setattr(module,'plan_cartesian_braking',expensive)
    packet=s.advance(target)
    assert packet.mode=='governed_deceleration' and s.last_braking_segment_solves==2
    assert len(s.last_rejections)==2
    assert s.last_rejections[-1]['code']=='cartesian_brake_search_budget_exhausted'


def test_curve_exhaustion_keeps_original_tail_without_claiming_segment_limit_was_reached():
    _,s,target=rig(continue_on_known_rejection=True,braking_segment_limit=24,
        braking_curve_limit=1,braking_method='joint_fit')
    for _ in range(4):
        before=s.state;packet=s.advance(target)
        assert packet.segment.start==before and s.last_braking_curve_attempts<=1
        assert s.last_braking_segment_solves<=24


def test_unowned_budget_error_does_not_recover_as_a_known_refusal(monkeypatch):
    _,s,target=rig(continue_on_known_rejection=True)
    def unknown(*a,**kw):raise ValueError('cartesian_brake_search_budget_exhausted')
    monkeypatch.setattr(module,'plan_cartesian_braking',unknown)
    with pytest.raises(ValueError,match='search_budget_exhausted'):s.advance(target)
    assert s.phase=='fault'


def test_fitted_braking_candidate_keeps_paired_chains_and_release_latch():
    _,s,target=rig(continue_on_known_rejection=True,braking_segment_limit=24,braking_method='joint_fit')
    for _ in range(24):
        before=s.state;packet=s.advance(target)
        assert packet.segment.start==before
        chain=(packet.segment,)+packet.braking_tail
        assert all(a.end==b.start for a,b in zip(chain,chain[1:]))
        assert s.last_braking_segment_solves<=24
    while s.phase!='ready':assert s.finish().mode=='braking'
    with pytest.raises(ValueError,match='requires_capture'):s.advance(target)
