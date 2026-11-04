import asyncio
from types import SimpleNamespace as NS

import pytest
from scipy.spatial.transform import Rotation

from app.control.docker_pvat_input import ExecutorLease
from app.control.finite_position_types import PositionIntent
from app.control.vr_pose_input import VRPoseInput
from app.control.vr_pose_mailbox import PoseMailboxReader
from app.schemas.messages import Pose


class Signals:
    def __init__(self):
        self.values = {i: 0 for i in range(240, 256)}
        self.writes = []
        self.hook = None

    async def get_signal(self, i):
        return self.values[i]

    async def get_signals(self, i, n):
        return [self.values[j] for j in range(i, i+n)]

    async def set_signal(self, i, value):
        self.values[i] = value
        self.writes.append((i, value))
        if self.hook:
            await self.hook(i)

    async def set_signals(self, i, values):
        for j, value in enumerate(values, i):
            self.values[j] = value
        self.writes.append((i, tuple(values)))
        if self.hook:
            await self.hook(i)


async def rig():
    ns = [1_000_000_000]
    io, lease = Signals(), ExecutorLease()
    stream = VRPoseInput(io, lease, NS(generation=4), clock=lambda: ns[0],
        stamp_for_capture=lambda t: t//1_000_000, authorize=lambda: None, authorize_cleanup=lambda: None)
    hand = Pose(p=(.1,.2,.3), q=(0.,0.,0.,1.))
    tcp = Pose(p=(.3,0.,.4), q=(0.,0.,0.,1.))
    await stream.open()
    stream.capture(hand,tcp,hand_ns=ns[0],tcp_ns=ns[0])
    return ns,io,lease,stream,hand,tcp


def intent(ns, seq, pose=None, **kw):
    return PositionIntent(kw.pop('session_id','xr-finite-test'),kw.pop('generation',4),seq,ns,
        'held' if pose else 'release',(0,0,0),hand_pose=pose,**kw)


async def spin_until(predicate):
    async with asyncio.timeout(1):
        while not predicate():
            await asyncio.sleep(.001)


async def test_complete_xr_pose_through_mapping_bulk_transport_and_known_cleanup():
    ns,io,lease,s,hand,tcp = await rig()
    s.offer(intent(ns[0],1,hand));s.start()
    await spin_until(lambda:s.writer.written_seq==1)
    ns[0]+=20_000_000
    q=tuple(Rotation.from_euler('y',70,degrees=True).as_quat())
    moved=Pose(p=(.11,.22,.27),q=q)
    s.offer(intent(ns[0],2,moved))
    await spin_until(lambda:s.writer.written_seq==2)
    reader=PoseMailboxReader(io,controller_ms=lambda:ns[0]//1_000_000,not_before_ms=1000,
                             authorize=lambda:None,bulk=True)
    goal=(await reader.read_latest()).pose
    assert goal.p==pytest.approx((.32,.06,.44))
    assert Rotation.from_quat(goal.q).magnitude()==pytest.approx(70*3.141592653589793/180,abs=2e-7)
    assert len(io.writes)==6  # Two publications, each invalidate/payload/commit.
    s.offer(intent(ns[0],3));await s.withdraw()
    assert lease.owner is s.writer.token
    with pytest.raises(ValueError,match='closed'):s.offer(intent(ns[0],4,moved))
    await s.complete(controller_ok=True,quiet_observed=True,program_exited=True)
    assert lease.owner is None and s.completed and io.values[240]==io.values[248]==0


async def test_diagnostics_bound_history_and_keep_original_capture_times():
    ns,io,lease,s,hand,_=await rig()
    s.offer(intent(ns[0],1,hand));s.start()
    await spin_until(lambda:s.writer.written_seq==1)
    for seq in range(2,25):
        ns[0]+=20_000_000
        s.offer(intent(ns[0],seq,hand))
        await spin_until(lambda:s.writer.written_seq==seq)
    await s.withdraw()
    assert len(s.publish_tail)==16 and s.publish_tail[0]['seq']==9
    assert s.publish_tail[-1]['capture_ns']==ns[0] and s.publish_tail[-1]['age_ms']==0
    assert s.max_offer_gap_ns==20_000_000 and s.max_publish_gap_ns==20_000_000
    # Diagnostic completion never releases a controller lease or clears signals.
    assert lease.owner is s.writer.token and not s.completed


async def test_normal_release_waits_for_known_inflight_write_without_cancelling_it():
    ns,io,lease,s,hand,_=await rig()
    entered,finish=asyncio.Event(),asyncio.Event()
    async def block(i):
        if i==241:
            entered.set();await finish.wait()
    io.hook=block;s.offer(intent(ns[0],1,hand));s.start();await entered.wait()
    s.offer(intent(ns[0],2))
    task=asyncio.create_task(s.withdraw());await asyncio.sleep(0)
    assert not task.done() and lease.owner is s.writer.token
    finish.set();await task
    assert not s.writer.unknown and io.values[240]==0  # No final commit after release.
    await s.complete(controller_ok=True,quiet_observed=True,program_exited=True)


async def test_unknown_batch_write_prevents_cleanup_and_lease_release():
    ns,io,lease,s,hand,_=await rig()
    async def fail(i):
        if i==241:raise TimeoutError('unknown payload outcome')
    io.hook=fail;s.offer(intent(ns[0],1,hand));s.start()
    await spin_until(lambda:s.closed)
    count=len(io.writes)
    with pytest.raises(ValueError,match='completion_unverified'):
        await s.complete(controller_ok=True,quiet_observed=True,program_exited=True)
    assert len(io.writes)==count and lease.owner is s.writer.token and s.writer.unknown


async def test_input_expiry_retires_even_when_no_new_frame_arrives():
    ns,io,lease,s,hand,_=await rig()
    s.offer(intent(ns[0],1,hand));s.start();await spin_until(lambda:s.writer.written_seq==1)
    ns[0]+=100_000_000
    await spin_until(lambda:s.closed)
    assert s.reason=='vr_pose_input_expired' and lease.owner is s.writer.token
    with pytest.raises(ValueError,match='closed'):s.offer(intent(ns[0],2,hand))
    await s.withdraw()


@pytest.mark.parametrize('changes',[{'generation':5},{'session_id':'desktop'}])
async def test_other_source_or_generation_cannot_use_captured_pose(changes):
    ns,io,lease,s,hand,_=await rig()
    with pytest.raises(ValueError,match='identity'):s.offer(intent(ns[0],1,hand,**changes))
    assert not io.writes and s.closed and lease.owner is s.writer.token


@pytest.mark.parametrize('field',['controller_ok','quiet_observed','program_exited'])
async def test_partial_completion_never_clears_signal_or_releases_owner(field):
    _,io,lease,s,_,_=await rig()
    flags=dict(controller_ok=True,quiet_observed=True,program_exited=True);flags[field]=False
    with pytest.raises(ValueError,match='unverified'):await s.complete(**flags)
    assert not io.writes and lease.owner is s.writer.token


async def test_outdated_robot_sample_cannot_capture_or_publish():
    ns,io,lease,s,hand,tcp=await rig()
    s.reference.release()
    with pytest.raises(ValueError,match='capture_expired'):
        s.capture(hand,tcp,hand_ns=ns[0],tcp_ns=ns[0]-100_000_000)
    assert s.closed and not io.writes


async def test_reader_uses_current_packet_after_slow_work_not_expired_previous_cache():
    ns,io,_,s,hand,_=await rig()
    reader=PoseMailboxReader(io,controller_ms=lambda:ns[0]//1_000_000,not_before_ms=1000,
                            authorize=lambda:None,bulk=True)
    s.offer(intent(ns[0],1,hand));await s.writer.publish_latest()
    await reader.read_latest()
    # Source never misses its lease while consumer does other known work.
    for seq in range(2,5):
        ns[0]+=40_000_000
        s.offer(intent(ns[0],seq,hand));await s.writer.publish_latest()
    assert (await reader.read_latest()).sequence_mod==4
    # Actually observing no fresh packet at expiry still retires permanently.
    ns[0]+=99_000_000
    with pytest.raises(ValueError,match='expired'):await reader.read_latest()
    assert reader.closed


async def test_identity_change_before_commit_leaves_inactive_payload_unpublished():
    ns,io,lease,s,hand,_=await rig()
    def reject():raise ValueError('configuration_changed')
    s.writer.before_commit=reject
    s.offer(intent(ns[0],1,hand));s.start();await spin_until(lambda:s.closed)
    assert s.reason=='vr_pose_pump_rejected' and io.values[240]==0
    assert len(io.writes)==2 and not s.writer.unknown and lease.owner is s.writer.token


async def test_unknown_admission_read_cannot_clear_headers_or_release_lease():
    ns,io,lease,s,hand,_=await rig()
    from app.control.vr_pose_input import VRPoseInput
    lease.release(s.writer.token)
    async def unknown(i):raise TimeoutError('read outcome not known')
    io.get_signal=unknown
    pending=VRPoseInput(io,lease,NS(generation=4),clock=lambda:ns[0],stamp_for_capture=lambda n:1000,
        authorize=lambda:None,authorize_cleanup=lambda:None)
    with pytest.raises(TimeoutError):await pending.open()
    assert pending.writer.read_unknown
    with pytest.raises(ValueError,match='unverified'):
        await pending.complete(controller_ok=True,quiet_observed=True,program_exited=True)
    assert not io.writes and lease.owner is pending.writer.token
