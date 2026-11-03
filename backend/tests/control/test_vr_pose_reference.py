import math
import pytest
from scipy.spatial.transform import Rotation
from app.control.vr_pose_reference import VRPoseReference
from app.schemas.messages import Pose


def rig():
    now=[1_000_000_000]
    mapper=VRPoseReference(clock=lambda:now[0])
    hand=Pose(p=(.1,.2,.3),q=(0.,0.,0.,1.))
    tcp=Pose(p=(.3,0.,.4),q=tuple(Rotation.from_euler('xyz',(.2,.3,.4)).as_quat()))
    mapper.capture(hand,tcp,generation=1,hand_ns=now[0],tcp_ns=now[0])
    return now,mapper,hand,tcp


def test_hand_70_degrees_means_70_degree_tcp_orientation_not_seventy_per_joint():
    now,m,hand,tcp=rig()
    rotation=Rotation.from_euler('y',70,degrees=True)
    goal=m.target(hand.model_copy(update={'q':tuple(rotation.as_quat())}),generation=1,seq=1,captured_ns=now[0])
    actual=Rotation.from_quat(goal.q)*Rotation.from_quat(tcp.q).inv()
    expected=m.axes*rotation*m.axes.inv()
    assert (actual.inv()*expected).magnitude()<1e-10
    assert actual.magnitude()==pytest.approx(math.radians(70))
    assert goal.p==pytest.approx(tcp.p)
    # An unmoving held hand keeps one endpoint instead of integrating velocity.
    for seq in range(2,50):
        repeated=m.target(hand.model_copy(update={'q':tuple(rotation.as_quat())}),generation=1,seq=seq,captured_ns=now[0])
        assert repeated.q==pytest.approx(goal.q)


def test_translation_maps_axes_and_scales_from_the_original_anchor():
    now,m,hand,tcp=rig()
    goal=m.target(hand.model_copy(update={'p':(.11,.22,.27)}),generation=1,seq=1,captured_ns=now[0])
    assert goal.p==pytest.approx((tcp.p[0]+.02,tcp.p[1]+.06,tcp.p[2]+.04))


def test_stale_input_retires_anchor_and_late_input_cannot_resume():
    now,m,hand,_=rig();now[0]+=100_000_000
    with pytest.raises(ValueError,match='expired'):m.target(hand,generation=1,seq=1,captured_ns=now[0]-100_000_000)
    with pytest.raises(ValueError,match='requires_capture'):m.target(hand,generation=1,seq=2,captured_ns=now[0])


def test_release_and_new_capture_do_not_replay_the_old_endpoint():
    now,m,hand,tcp=rig();m.release()
    with pytest.raises(ValueError):m.target(hand,generation=1,seq=1,captured_ns=now[0])
    new_hand=hand.model_copy(update={'p':(.8,.8,.8)})
    m.capture(new_hand,tcp,generation=2,hand_ns=now[0],tcp_ns=now[0])
    assert m.target(new_hand,generation=2,seq=1,captured_ns=now[0]).p==pytest.approx(tcp.p)
