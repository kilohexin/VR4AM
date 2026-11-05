"""Transport-free execution ownership; never imports a physical SDK."""
from dataclasses import replace

import pytest

from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference, MdhReferenceModel


@pytest.fixture(scope='module')
def contract():
    from app.commissioning.real_cartesian_pvat import build_contract
    model = MdhReferenceModel(MdhTool((.01, -.02, .2), (0., 0., 0., 1.)), (-3.,)*6, (3.,)*6)
    start = JointReference((0., -.8, 1.4, -.6, 1.57, 0.))
    p = model.pose(start.q).p
    return build_contract(model, start, workspace_lower=tuple(x-.01 for x in p),
                          workspace_upper=tuple(x+.01 for x in p), config_sha256='a'*64,
                          source_sha256='b'*64, tool_report_sha256='c'*64)


def window(contract):
    from app.commissioning.real_cartesian_pvat import FinitePvatWindow
    owner = object()
    w = FinitePvatWindow(contract, owner, 7)
    w.admit(owner, 7, contract.sha256, contract.tool_sha256,
            contract.origin, contract.origin, 'IDLE', 0, 1)
    return w, owner


def test_contract_binds_exact_path_workspace_and_does_not_grant_motion(contract):
    assert len(contract.points) == 20
    assert contract.points[-1].end.v + contract.points[-1].end.a == (0.,)*12
    assert not contract.motion_admitted
    assert len(contract.sha256) == 64
    assert replace(contract, config_sha256='d'*64).sha256 != contract.sha256
    assert replace(contract, workspace_upper=tuple(x+.001 for x in contract.workspace_upper)).sha256 != contract.sha256


def test_workspace_cannot_recenter_to_make_candidate_fit():
    from app.commissioning.real_cartesian_pvat import build_contract
    model = MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start = JointReference((0.,-.8,1.4,-.6,1.57,0.)); p=model.pose(start.q).p
    with pytest.raises(ValueError, match='workspace'):
        build_contract(model,start,workspace_lower=tuple(x-.01 for x in p),
            workspace_upper=(p[0]+.01,p[1]+.01,p[2]+.001),config_sha256='a'*64,
            source_sha256='b'*64,tool_report_sha256='c'*64)


@pytest.mark.parametrize('change', ['owner','generation','contract','tool','stale','future','moving','target_rate','actual_rate','origin'])
def test_admission_refuses_mismatch_without_any_entry(contract,change):
    from app.commissioning.real_cartesian_pvat import FinitePvatWindow
    owner=object();w=FinitePvatWindow(contract,owner,7)
    args=[owner,7,contract.sha256,contract.tool_sha256,contract.origin,contract.origin,'IDLE',0,1]
    if change=='owner':args[0]=object()
    if change=='generation':args[1]=8
    if change=='contract':args[2]='d'*64
    if change=='tool':args[3]='d'*64
    if change=='stale':args[8]=100_000_000
    if change=='future':args[7]=2
    if change=='moving':args[6]='MOVING'
    if change=='target_rate':args[5]=JointReference(contract.origin.q,(.001,)*6)
    if change=='actual_rate':args[4]=JointReference(contract.origin.q,(.001,)*6)
    if change=='origin':args[4]=JointReference(tuple(x+.01 for x in contract.origin.q))
    with pytest.raises(ValueError):w.admit(*args)
    assert w.entered==0 and w.fault and not w.motion_unknown
    with pytest.raises(ValueError):w.admit(owner,7,contract.sha256,contract.tool_sha256,contract.origin,contract.origin,'IDLE',0,1)


def test_twenty_returns_are_not_arrival_or_physical_acceptance(contract):
    w,owner=window(contract)
    for i in range(20):
        now=1+i*79_000_000
        point=w.enter(owner,7,i+1,now)
        assert point==contract.points[i]
        w.returned(i+1,now+1)
    assert w.returned_count==20 and w.phase=='awaiting_arrival'
    assert not w.motion_unknown and not w.passed
    with pytest.raises(ValueError):w.enter(owner,7,21,1_600_000_000)


def test_pending_write_and_late_return_cannot_be_hidden_by_revocation(contract):
    w,owner=window(contract);w.enter(owner,7,1,1)
    w.revoke('operator_release')
    assert w.motion_unknown and w.pending_index==1
    w.returned(1,2)
    assert w.motion_unknown and w.fault=='operator_release' and w.pending_index is None
    assert not w.passed


@pytest.mark.parametrize('kind',['pending','cadence','early_queue','wrong_index','wrong_owner','clock','reply_timeout'])
def test_invalid_dispatch_latches_and_never_returns_next_point(contract,kind):
    w,owner=window(contract);w.enter(owner,7,1,10)
    if kind=='reply_timeout':
        w.returned(1,60_000_010)
        assert w.motion_unknown and w.fault=='pvat_reply_deadline'
        return
    if kind!='pending':w.returned(1,11)
    args=[owner,7,2,12]
    if kind=='cadence':args[3]=80_000_010
    if kind=='wrong_index':args[2]=3
    if kind=='wrong_owner':args[0]=object()
    if kind=='clock':args[3]=9
    if kind=='early_queue':
        w.enter(*args);w.returned(2,13);args=[owner,7,3,14]
    before=w.entered
    with pytest.raises(ValueError):w.enter(*args)
    assert w.entered==before and w.fault


def test_stop_return_does_not_clear_unknown_or_authorize_another_stop(contract):
    w,owner=window(contract);w.enter(owner,7,1,1);w.revoke('connection_lost')
    assert w.claim_exception_stop(owner,7,2)
    w.stop_returned(3)
    assert w.motion_unknown and w.stop_unverified and not w.passed
    with pytest.raises(ValueError):w.claim_exception_stop(owner,7,4)


def test_zero_write_refusal_never_requests_a_stop(contract):
    w,owner=window(contract);w.revoke('operator_release')
    assert not w.claim_exception_stop(owner,7,2)
    assert not w.stop_unverified and not w.motion_unknown


def feedback_rig(contract):
    from app.commissioning.real_cartesian_pvat import FinitePvatFeedback
    model=MdhReferenceModel(MdhTool((.01,-.02,.2),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    w,owner=window(contract)
    return w,owner,model,FinitePvatFeedback(w,model)


def complete_returns(w,owner,f=None,model=None):
    for i in range(20):
        t=1+i*79_000_000;w.enter(owner,7,i+1,t);w.returned(i+1,t+1)
        if f is not None:
            r=w.contract.points[i].end
            f.observe(owner,7,r,r,model.pose(r.q),'MOVING',t+2,t+2)


def test_feedback_requires_stable_arrival_and_full_fixed_anchor_tail(contract):
    w,owner,model,f=feedback_rig(contract);complete_returns(w,owner,f,model)
    final=contract.points[-1].end;tcp=model.pose(final.q)
    for t in range(1600,3801,20):
        f.observe(owner,7,final,final,tcp,'IDLE',t*1_000_000,t*1_000_000+1)
        if t<3800:assert not f.sampled_passed
    assert f.sampled_passed and f.arrival_ns==1_800_000_000
    assert not f.hardware_verified and not w.passed


@pytest.mark.parametrize('kind',['drop','wrong_tool','stale','rate','unknown','stalled','tail_drift','gap','clock'])
def test_feedback_never_accepts_returns_or_later_quiet_to_hide_failure(contract,kind):
    w,owner,model,f=feedback_rig(contract)
    if kind=='wrong_tool':
        from app.commissioning.real_cartesian_pvat import FinitePvatFeedback
        with pytest.raises(ValueError):FinitePvatFeedback(w,MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6))
        return
    complete_returns(w,owner,f,model);final=contract.points[-1].end;tcp=model.pose(final.q)
    if kind=='unknown':w.revoke('connection_lost')
    if kind=='drop':tcp=tcp.model_copy(update={'p':(tcp.p[0],tcp.p[1],tcp.p[2]-.01)})
    if kind=='rate':final=JointReference(final.q,(.1,)*6)
    if kind=='stalled':
        final=contract.origin;tcp=model.pose(final.q)
        with pytest.raises(ValueError,match='arrival_deadline'):
            for t in range(1600,5021,20):f.observe(owner,7,final,final,tcp,'IDLE',t*1_000_000,t*1_000_000+1)
    elif kind in ('tail_drift','gap','clock'):
        for t in range(1600,1821,20):f.observe(owner,7,final,final,tcp,'IDLE',t*1_000_000,t*1_000_000+1)
        if kind=='tail_drift':tcp=tcp.model_copy(update={'p':(tcp.p[0]+.002,tcp.p[1],tcp.p[2])})
        t=1950 if kind=='gap' else (1800 if kind=='clock' else 1840)
        with pytest.raises(ValueError):f.observe(owner,7,final,final,tcp,'IDLE',t*1_000_000,t*1_000_000+1)
    else:
        with pytest.raises(ValueError):f.observe(owner,7,final,final,tcp,'IDLE',1_500_000_000 if kind=='stale' else 1_600_000_000,1_600_000_001)
    assert not f.sampled_passed and w.fault


def test_arrival_requires_target_to_agree_with_terminal_and_actual_orientation(contract):
    w,owner,model,f=feedback_rig(contract);complete_returns(w,owner,f,model)
    final=contract.points[-1].end;tcp=model.pose(final.q)
    for t in range(1600,1901,20):
        f.observe(owner,7,final,contract.origin,tcp,'IDLE',t*1_000_000,t*1_000_000+1)
    assert f.arrival_ns is None


def test_feedback_budget_is_finite_even_with_fresh_quiet_samples_before_motion(contract):
    w,owner,model,f=feedback_rig(contract)
    with pytest.raises(ValueError,match='budget'):
        for i in range(513):
            t=2+i*1_000_000
            f.observe(owner,7,contract.origin,contract.origin,model.pose(contract.origin.q),'IDLE',t,t)
    assert w.entered==0


def test_first_send_uses_original_capture_age_not_admission_age(contract):
    from app.commissioning.real_cartesian_pvat import FinitePvatWindow
    owner=object();w=FinitePvatWindow(contract,owner,7)
    w.admit(owner,7,contract.sha256,contract.tool_sha256,contract.origin,contract.origin,'IDLE',0,99_000_000)
    with pytest.raises(ValueError,match='snapshot_age'):w.enter(owner,7,1,100_000_000)
    assert w.entered==0


def test_capture_lag_cannot_shorten_stable_or_tail_observation(contract):
    w,owner,model,f=feedback_rig(contract);complete_returns(w,owner,f,model)
    final=contract.points[-1].end;tcp=model.pose(final.q)
    for t in range(1600,1801,20):
        capture=t if t==1600 else t-10
        f.observe(owner,7,final,final,tcp,'IDLE',capture*1_000_000,t*1_000_000+1)
    assert f.arrival_ns is None
    f.observe(owner,7,final,final,tcp,'IDLE',1_820_000_000,1_820_000_001)
    for t in range(1840,3821,20):
        f.observe(owner,7,final,final,tcp,'IDLE',(t-10)*1_000_000,t*1_000_000+1)
    assert not f.sampled_passed


def test_replacing_a_contract_cannot_bypass_path_validation(contract):
    with pytest.raises(ValueError):replace(contract,joint_lower=(10.,)*6,joint_upper=(11.,)*6)
    from app.control.cartesian_pvat_reference import PvatSegment
    stationary=PvatSegment(contract.origin,contract.origin,.08)
    with pytest.raises(ValueError):replace(contract,points=(stationary,)*20)
    with pytest.raises(ValueError):replace(contract,tool_sha256='f'*64)
