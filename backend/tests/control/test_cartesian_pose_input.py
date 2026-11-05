from dataclasses import replace

import pytest
from scipy.spatial.transform import Rotation

from app.control.cartesian_pose_input import CartesianPoseInput
from app.control.finite_position_types import PositionBinding,PositionIntent
from app.schemas.messages import Pose


def fixture():
    clock=[1_000_000_000];binding=PositionBinding(object(),7,'a9'*32,'15'*32)
    hand=Pose(p=(.1,.2,.3),q=(0.,0.,0.,1.))
    tcp=Pose(p=(.3,0.,.4),q=(.5,-.5,-.5,.5))
    intent=PositionIntent('xr-finite-cartesian',7,1,clock[0],'held',(0.,0.,0.),hand_pose=hand)
    inputs=CartesianPoseInput(binding,intent,tcp,tcp_ns=clock[0],clock=lambda:clock[0])
    return clock,inputs,intent,tcp


def test_capture_uses_full_pose_and_fixed_original_anchor_with_one_to_one_scale():
    now,inputs,first,tcp=fixture()
    now[0]+=20_000_000
    moved=first.hand_pose.model_copy(update={'p':(.13,.24,.25),
        'q':tuple(Rotation.from_euler('x',.2).as_quat())})
    inputs.offer(replace(first,seq=2,received_ns=now[0],hand_pose=moved))
    sample=inputs.sample()
    assert sample.pose.p==pytest.approx((.33,.05,.44))
    assert sample.seq==2 and sample.captured_ns==now[0]
    assert Rotation.from_quat(sample.pose.q).apply((0.,0.,1.))[2]>0
    assert inputs.anchor_tcp==tcp


def test_sampling_never_renews_old_input_and_does_not_expose_mutable_cached_pose():
    now,inputs,first,_=fixture()
    sample=inputs.sample();sample.pose.p=(9.,9.,9.)
    assert inputs.sample().pose.p!=(9.,9.,9.)
    for _ in range(5):
        now[0]+=19_000_000
        assert inputs.sample().captured_ns==first.received_ns
    now[0]=first.received_ns+100_000_000
    assert inputs.sample() is None and not inputs.closed
    now[0]=first.received_ns+500_000_000
    assert inputs.sample() is None and inputs.closed and inputs.reason=='input_expired'
    with pytest.raises(ValueError,match='retired'):
        inputs.offer(replace(first,seq=2,received_ns=now[0]))


@pytest.mark.parametrize('change',[
    {'generation':8},{'session_id':'xr-finite-other'},{'seq':1},{'received_ns':999_999_999},
    {'received_ns':1_000_000_001},
])
def test_bad_identity_order_or_time_retires_input_without_recapture(change):
    _,inputs,first,_=fixture()
    with pytest.raises(ValueError):inputs.offer(replace(first,seq=2,**change) if 'seq' not in change else replace(first,**change))
    assert inputs.closed and inputs.sample() is None


def test_release_before_executor_start_is_permanent_and_keeps_first_reason():
    _,inputs,first,_=fixture()
    inputs.offer(replace(first,seq=2,kind='release',hand_pose=None))
    assert inputs.closed and inputs.reason=='release' and inputs.sample() is None
    inputs.close('disconnect')
    assert inputs.reason=='release'
    with pytest.raises(ValueError,match='retired'):inputs.offer(replace(first,seq=3))


def test_clock_rollback_cannot_make_expired_input_fresh():
    now,inputs,_,_=fixture();now[0]-=1
    with pytest.raises(ValueError,match='clock'):inputs.sample()
    assert inputs.closed
