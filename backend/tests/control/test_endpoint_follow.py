import pytest
from app.schemas.messages import Pose


def make():
    from app.control.endpoint_follow import EndpointMailbox
    return EndpointMailbox('demo',7,(-.05,)*3,(.05,)*3)


def test_latest_input_replaces_unsent_targets_and_release_keeps_final_endpoint():
    m=make();m.offer('demo',7,1,0,Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.)),0)
    first=m.take(1);assert first.seq==1
    m.offer('demo',7,2,2,Pose(p=(.02,0.,0.),q=(0.,0.,0.,1.)),2)
    m.offer('demo',7,3,3,Pose(p=(.03,0.,0.),q=(0.,0.,0.,1.)),3)
    assert m.take(4) is None
    m.release();m.returned(1,100,5)
    last=m.take(6);assert last.seq==3
    m.returned(3,101,7)
    assert m.phase=='holding' and m.final_pose.p==(.03,0.,0.)
    assert m.take(8) is None and m.completed_writes==2


def test_timeout_is_sticky_and_late_return_does_not_restart():
    m=make();m.offer('demo',7,1,0,Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.)),0);m.take(1)
    m.fail('write_deadline')
    m.returned(1,100,2)
    assert m.fault=='write_deadline' and m.unknown and m.take(3) is None


def test_disconnect_never_submits_pending_target_or_opens_gripper():
    m=make();m.offer('demo',7,1,0,Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.)),0)
    m.fail('disconnect');assert m.take(1) is None and m.completed_writes==0


@pytest.mark.parametrize('kind',['stale','bounds','generation','sequence','clock'])
def test_bad_input_cannot_replace_an_endpoint(kind):
    m=make();p=Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.));m.offer('demo',7,1,0,p,1)
    args=['demo',7,2,2,p,2]
    if kind=='stale':args[5]=100_000_002
    if kind=='bounds':args[4]=p.model_copy(update={'p':(.06,0.,0.)})
    if kind=='generation':args[1]=8
    if kind=='sequence':args[2]=1
    if kind=='clock':args[5]=0
    with pytest.raises(ValueError):m.offer(*args)
    assert m.fault and m.take(100_000_003) is None


def test_old_pending_target_is_not_sent_after_slow_return():
    m=make();p=Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.));m.offer('demo',7,1,0,p,0);m.take(1)
    m.offer('demo',7,2,2,p,2);m.release();m.returned(1,100,100_000_003)
    assert m.take(100_000_004) is None and m.final_pose.p==p.p
    assert m.phase=='holding' and not m.fault


def test_empty_exception_message_still_latches_failure():
    m=make();m.fail(str(TimeoutError()))
    with pytest.raises(ValueError):m.offer('demo',7,1,0,Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.)),0)
    assert m.take(1) is None and m.fault


def test_late_known_reply_is_sticky_unknown_and_never_drains_next_target():
    m=make();p=Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.));m.offer('demo',7,1,0,p,0);m.take(1)
    m.offer('demo',7,2,2,p,2);m.returned(1,100,200_000_001)
    assert m.fault and m.unknown and m.take(200_000_002) is None


def test_mutated_pose_shape_is_rejected_and_owned_pose_is_not_mutable():
    m=make();p=Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.))
    with pytest.raises(ValueError):m.offer('demo',7,1,0,p.model_copy(update={'p':(.01,)}),0)
    n=make();n.offer('demo',7,1,0,p,0);item=n.take(1)
    item.pose.p=(.04,0.,0.);n.returned(1,100,2)
    assert n.final_pose.p==(.01,0.,0.)


def test_missing_receipt_is_not_silently_left_pending_forever():
    m=make();p=Pose(p=(.01,0.,0.),q=(0.,0.,0.,1.));m.offer('demo',7,1,0,p,0);m.take(1)
    assert m.take(200_000_001) is None and m.unknown and m.fault
