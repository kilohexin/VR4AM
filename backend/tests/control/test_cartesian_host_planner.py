import numpy as np
import pytest

from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_clock import ControllerClock
from app.control.cartesian_host_planner import CartesianHostPlanner
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor


@pytest.fixture
def host():
    time=[0]
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start=JointReference((0.,-.8,1.4,-.6,1.57,0.))
    h=CartesianHostPlanner(model,start,session='15'*16,model_sha256='a9'*32,
        generation=1,clock=lambda:time[0],braking_segment_limit=24,braking_curve_limit=4)
    pose=model.pose(start.q)
    target=pose.model_copy(update={'p':(pose.p[0]+.01,pose.p[1],pose.p[2])})
    h.offer(target,captured_ms=0,sequence=1,generation=1)
    return h,time,target


def result(host):
    h,_,_=host
    job=h.begin(80)
    assert job is not None
    return h.calculate(job)


def test_work_does_not_commit_until_controller_ack_and_tail_is_complete(host):
    h,_,_=host;origin=h.head
    planned=result(host);packet=h.complete(planned)
    assert packet is not None and packet.lead.start==origin
    assert packet.expand_tail()==planned.checkpoint._tail
    assert h.head==origin and h.sequence==0
    assert h.begin(80) is None
    h.acknowledge(packet.sequence,True)
    assert h.sequence==1
    h.observe_head(packet.lead.end,1,generation=1)
    assert h.begin(80) is not None


@pytest.mark.parametrize('reason',['release','tracking_lost','disconnect'])
def test_release_during_calculation_retires_late_result(host,reason):
    h,_,_=host;planned=result(host);h.close(reason)
    assert h.complete(planned) is None and h.reason==reason
    with pytest.raises(ValueError,match='retired'):h.begin(80)


def test_late_math_cannot_install_even_with_fresh_new_input(host):
    h,time,target=host;planned=result(host)
    time[0]=81;h.offer(target,captured_ms=81,sequence=2,generation=1)
    assert h.complete(planned) is None and h.sequence==0
    assert not h.closed


def test_input_expiry_is_permanent_and_unknown_publish_is_not_erased(host):
    h,time,_=host;planned=result(host);packet=h.complete(planned)
    h.close('disconnect')
    assert h.pending is planned
    with pytest.raises(ValueError,match='cartesian_publish_unknown'):
        h.acknowledge(packet.sequence,False,known=False)
    assert h.closed and h.reason=='disconnect'


def test_finished_brake_resumes_from_exact_terminal_and_same_preparation_box(host):
    h,time,target=host;planned=result(host);packet=h.complete(planned)
    lower=h.supervisor.lower.copy();upper=h.supervisor.upper.copy()
    h.acknowledge(packet.sequence,True)
    tick=1+packet.braking_ticks
    h.observe_head(packet.terminal,tick,generation=1)
    for seq in range(2,9):
        time[0]=(seq-1)*90;h.offer(target,captured_ms=time[0],sequence=seq,generation=1)
    job=h.begin(710)
    assert job.source.state==packet.terminal
    np.testing.assert_array_equal(job.source.lower,lower)
    np.testing.assert_array_equal(job.source.upper,upper)


def test_expired_input_cannot_be_revived_by_new_offer(host):
    h,time,target=host;time[0]=500
    with pytest.raises(ValueError,match='input_expired'):
        h.offer(target,captured_ms=500,sequence=2,generation=1)
    assert h.closed


def test_future_resume_ack_does_not_erase_current_native_terminal_hold(host):
    h,_,_=host;planned=result(host);packet=h.complete(planned);h.acknowledge(packet.sequence,True)
    tick=1+packet.braking_ticks
    h.observe_owned_cursor(tick,packet.sequence,'holding',generation=1)
    assert h.planning_head()==(tick+1,packet.terminal)
    resumed=h.calculate(h.begin(100));next_packet=h.complete(resumed)
    assert next_packet is not None and next_packet.parent_tick==tick+1
    h.acknowledge(next_packet.sequence,True)
    h.observe_owned_cursor(tick+1,packet.sequence,'holding',generation=1)
    assert not h.closed and h.head==packet.terminal


def test_owned_dynamic_braking_state_can_plan_without_waiting_for_terminal(host):
    h,_,_=host;planned=result(host);packet=h.complete(planned);h.acknowledge(packet.sequence,True)
    assert packet.braking_ticks>1
    h.observe_owned_cursor(2,packet.sequence,'active',generation=1)
    assert h.head==packet.expand_tail()[0].end and any(x!=0 for x in h.head.v)
    next_job=h.begin(160)
    assert next_job is not None and next_job.source.state==h.head
    assert next_job.source._tail==packet.expand_tail()[1:]
    resumed=h.calculate(next_job)
    assert resumed.bundle is not None and resumed.bundle.lead.start==h.head
    assert resumed.bundle.expand_tail()==resumed.checkpoint._tail


def test_rejected_first_packet_cannot_pin_the_controller_reference_epoch(host):
    h,_,_=host;first=h.complete(result(host))
    h.acknowledge(first.sequence,False)
    assert h.reference_epoch is None
    next_result=h.calculate(h.begin(240));next_packet=h.complete(next_result)
    assert next_packet is not None and next_packet.parent_tick==0
    h.acknowledge(next_packet.sequence,True)
    assert h.reference_epoch==240


def test_unowned_or_wrong_generation_head_closes_session(host):
    h,_,_=host
    with pytest.raises(ValueError,match='unowned_head'):
        h.observe_head(JointReference((.1,)*6),1,generation=1)
    assert h.closed


def test_fixed_preparation_box_does_not_follow_new_origin(host):
    h,_,_=host
    with pytest.raises(ValueError,match='workspace'):
        OfflineReferenceSupervisor(h.supervisor.model,h.head,workspace_span=.01,workspace_center=(2.,2.,2.))


def test_clock_bounds_use_opposite_roundtrip_endpoints():
    c=ControllerClock(1_000_000_000,1_020_000_000,500)
    assert c.capture_lower_ms(1_030_000_000)==509
    assert c.sample_lower_ns(530)==1_029_000_000
    assert c.check_pair(1_040_000_000,1_060_000_000,540).sampled_ms==540
    with pytest.raises(ValueError,match='drift'):c.check_pair(1_200_000_000,1_220_000_000,540)
    with pytest.raises(ValueError,match='rollback'):c.sample_lower_ns(499)
    with pytest.raises(ValueError,match='pair'):ControllerClock(0,51_000_000,0)
