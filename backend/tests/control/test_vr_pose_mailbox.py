import asyncio
import math

import pytest
from scipy.spatial.transform import Rotation

from app.control.docker_pvat_input import ExecutorLease
from app.control.vr_pose_mailbox import (
    BASE, BANK_SIZE, STAMP_MOD, CapturedPose, PoseMailboxReader,
    PoseMailboxWriter, header, pack, unpack,
)
from app.control.vr_pose_reference import VRPoseReference
from app.schemas.messages import Pose


class Signals:
    def __init__(self):
        self.values = {i: 0 for i in range(BASE, BASE + 2 * BANK_SIZE)}
        self.writes = []
        self.after_read = self.after_write = None

    async def get_signal(self, channel):
        value = self.values[channel]
        if self.after_read:
            await self.after_read(channel)
        return value

    async def set_signal(self, channel, value):
        self.values[channel] = value
        self.writes.append((channel, value))
        if self.after_write:
            await self.after_write(channel, value)


class Rig:
    def __init__(self, ms=1000,ttl_ms=100):
        self.ns = ms * 1_000_000
        self.authorized = True
        self.io = Signals()
        self.lease = ExecutorLease()
        self.writer = PoseMailboxWriter(
            self.io, self.lease, clock=lambda: self.ns,
            stamp_for_capture=lambda ns: ns // 1_000_000, authorize=self.authorize,ttl_ms=ttl_ms,
        )
        self.reader = PoseMailboxReader(
            self.io, controller_ms=lambda: self.ns // 1_000_000,
            not_before_ms=ms, authorize=self.authorize,
        )

    def authorize(self):
        if not self.authorized:
            raise ValueError('test_revoked')

    def offer(self, seq, pose=None):
        pose = pose or Pose(p=(seq * .001, .2, .3), q=(0., 0., 0., 1.))
        self.writer.offer(CapturedPose(seq, self.ns, pose))

    async def publish(self, seq):
        self.offer(seq)
        assert await self.writer.publish_latest()


async def test_latest_input_is_selected_after_inactive_header_invalidation():
    r=Rig();await r.writer.open();r.offer(1)
    async def replace(channel,value):
        if channel==BASE and value==0:
            r.ns+=20_000_000;r.offer(2)
    r.io.after_write=replace
    assert await r.writer.publish_latest()
    result=await r.reader.read_latest()
    assert result.pose.p[0]==.002 and result.captured_ms==1020
    assert r.writer.written_seq==2 and r.writer.last_written_capture_ns==r.ns


@pytest.mark.parametrize('same',[False,True])
async def test_commit_uses_new_capture_only_for_bit_identical_payload(same):
    r=Rig();await r.writer.open();p=Pose(p=(.001,.2,.3),q=(0.,0.,0.,1.));r.offer(1,p)
    async def replace(channel,value):
        if channel==BASE+7:
            r.ns+=20_000_000;r.offer(2,p if same else None)
    r.io.after_write=replace
    assert await r.writer.publish_latest()
    result=await r.reader.read_latest()
    assert result.pose.p[0]==.001  # Never label a different pose as the new sample.
    assert result.captured_ms==(1020 if same else 1000)
    assert r.writer.written_seq==(2 if same else 1)


@pytest.mark.parametrize('ttl,age,accepted',[(100,150,False),(250,150,True),(250,250,False)])
async def test_explicit_mailbox_budget_preserves_capture_and_default_deadline(ttl,age,accepted):
    r=Rig(ttl_ms=ttl);await r.writer.open();r.offer(1);r.ns+=age*1_000_000
    if accepted:
        assert await r.writer.publish_latest()
        assert r.writer.last_written_capture_ns==1_000_000_000
        assert r.io.values[BASE]==header(1000,1)
    else:
        with pytest.raises(ValueError,match='input_expired'):await r.writer.publish_latest()
        assert not r.io.writes and not r.writer.unknown


@pytest.mark.parametrize('ttl,delay,known',[(100,.12,False),(250,.12,True),(250,.28,False)])
async def test_simulator_rpc_budget_joins_known_response_and_keeps_unknown_timeout(ttl,delay,known):
    r=Rig(ttl_ms=ttl);await r.writer.open();r.offer(1)
    async def slow_once(channel,value):
        r.io.after_write=None
        await asyncio.sleep(delay)
        r.ns+=round(delay*1e9)
    r.io.after_write=slow_once
    if known:
        assert await r.writer.publish_latest()
        assert not r.writer.unknown and r.writer.written_seq==1
    else:
        with pytest.raises(TimeoutError):await r.writer.publish_latest()
        assert r.writer.unknown and r.lease.owner is r.writer.token


@pytest.mark.asyncio
async def test_reference_to_mailbox_retains_70_degree_goal_and_two_times_position():
    r = Rig()
    await r.writer.open()
    reference = VRPoseReference(clock=lambda: r.ns)
    hand = Pose(p=(.1, .2, .3), q=(0., 0., 0., 1.))
    tcp = Pose(p=(.3, 0., .4), q=tuple(Rotation.from_euler('xyz', (.2, .3, .4)).as_quat()))
    reference.capture(hand, tcp, generation=4, hand_ns=r.ns, tcp_ns=r.ns)
    changed = Pose(p=(.11, .22, .27), q=tuple(Rotation.from_euler('y', 70, degrees=True).as_quat()))
    previous = None
    for seq in range(1, 20):
        r.ns += 10_000_000
        goal = reference.target(changed, generation=4, seq=seq, captured_ns=r.ns)
        r.offer(seq, goal)
        await r.writer.publish_latest()
        received = await r.reader.read_latest()
        assert received.captured_ms == r.ns // 1_000_000
        assert received.pose.p == pytest.approx((.32, .06, .44), abs=5.1e-7)
        rotation = Rotation.from_quat(received.pose.q) * Rotation.from_quat(tcp.q).inv()
        assert rotation.magnitude() == pytest.approx(math.radians(70), abs=2e-7)
        assert (Rotation.from_quat(received.pose.q).inv() * Rotation.from_quat(goal.q)).magnitude() < 2e-7
        if previous:
            assert received.pose == previous
        previous = received.pose
    reference.release()
    r.writer.revoke()
    assert r.lease.owner is r.writer.token  # No implied native completion.


@pytest.mark.asyncio
async def test_every_write_boundary_exposes_only_complete_old_or_new_pose():
    r = Rig()
    await r.writer.open()
    await r.publish(1)
    r.ns += 1_000_000
    observed = []

    async def inspect(channel, value):
        sample = await r.reader.read_latest()
        observed.append(sample.pose.p[0])

    r.io.after_write = inspect
    await r.publish(2)
    assert observed == [.001] * 8 + [.002]
    assert r.io.writes[9] == (BASE + BANK_SIZE, 0)


@pytest.mark.asyncio
@pytest.mark.parametrize('payload_index', range(1, 8))
async def test_bank_replaced_during_payload_read_is_never_returned(payload_index):
    r = Rig()
    await r.writer.open()
    await r.publish(1)
    changed = False

    async def overwrite(channel):
        nonlocal changed
        if channel == BASE + payload_index and not changed:
            changed = True
            r.ns += 1_000_000
            await r.publish(2)
            r.ns += 1_000_000
            await r.publish(3)  # Reuses selected bank while reader is inside it.

    r.io.after_read = overwrite
    assert await r.reader.read_latest() is None
    r.io.after_read = None
    sample = await r.reader.read_latest()
    assert sample.sequence_mod == 3
    assert sample.pose.p[0] == .003


@pytest.mark.asyncio
async def test_other_bank_commit_does_not_starve_coherent_selected_bank():
    r = Rig()
    await r.writer.open()
    await r.publish(1)

    async def publish_other(channel):
        if channel == BASE + 1:
            r.io.after_read = None
            r.ns += 1_000_000
            await r.publish(2)

    r.io.after_read = publish_other
    assert (await r.reader.read_latest()).sequence_mod == 1
    assert (await r.reader.read_latest()).sequence_mod == 2


@pytest.mark.asyncio
async def test_inflight_target_finishes_then_skips_intermediate_offers():
    r = Rig()
    await r.writer.open()
    r.offer(1)

    async def replace_pending(channel, value):
        if channel == BASE + 1:
            r.io.after_write = None
            r.ns += 1_000_000
            r.offer(2)
            r.ns += 1_000_000
            r.offer(3)
            with pytest.raises(ValueError, match='write_inflight'):
                await r.writer.publish_latest()

    r.io.after_write = replace_pending
    await r.writer.publish_latest()
    assert (await r.reader.read_latest()).sequence_mod == 1
    await r.writer.publish_latest()
    assert (await r.reader.read_latest()).sequence_mod == 3
    assert not await r.writer.publish_latest()
    commits = [value for channel, value in r.io.writes if channel in (BASE, BASE + BANK_SIZE) and value]
    assert commits == [header(1000, 1), header(1002, 3)]


@pytest.mark.asyncio
async def test_pose_mutation_after_offer_cannot_change_transaction():
    r = Rig()
    await r.writer.open()
    pose = Pose(p=(.001, .2, .3), q=(0., 0., 0., 1.))
    r.offer(1, pose)
    pose.p = (.9, .9, .9)
    await r.writer.publish_latest()
    assert (await r.reader.read_latest()).pose.p == (.001, .2, .3)


@pytest.mark.asyncio
async def test_expiry_during_payload_write_never_commits_or_retries():
    r = Rig()
    await r.writer.open()
    r.offer(1)

    async def slow(channel, value):
        r.ns += 20_000_000

    r.io.after_write = slow
    with pytest.raises(ValueError, match='input_expired'):
        await r.writer.publish_latest()
    assert r.io.values[BASE] == 0
    count = len(r.io.writes)
    with pytest.raises(ValueError, match='closed'):
        await r.writer.publish_latest()
    assert len(r.io.writes) == count
    assert r.writer.closed and not r.writer.unknown
    assert r.lease.owner is r.writer.token


@pytest.mark.asyncio
async def test_unknown_write_retains_lease_and_never_clears_or_retries():
    r = Rig()
    await r.writer.open()
    r.offer(1)

    async def fail_after_effect(channel, value):
        raise TimeoutError('unconfirmed write')

    r.io.after_write = fail_after_effect
    with pytest.raises(TimeoutError):
        await r.writer.publish_latest()
    assert r.writer.unknown and r.writer.closed
    assert r.lease.owner is r.writer.token
    assert len(r.io.writes) == 1
    with pytest.raises(ValueError, match='closed'):
        await r.writer.publish_latest()
    with pytest.raises(ValueError, match='occupied'):
        PoseMailboxWriter(r.io, r.lease, clock=lambda: r.ns, stamp_for_capture=lambda ns: 1000, authorize=r.authorize)
    assert len(r.io.writes) == 1


@pytest.mark.asyncio
async def test_revocation_during_write_is_sticky_even_if_authority_returns():
    r = Rig()
    await r.writer.open()
    r.offer(1)

    async def revoke(channel, value):
        r.authorized = False

    r.io.after_write = revoke
    with pytest.raises(ValueError, match='test_revoked'):
        await r.writer.publish_latest()
    r.authorized = True
    with pytest.raises(ValueError, match='closed'):
        await r.writer.publish_latest()
    assert r.io.writes == [(BASE, 0)]


@pytest.mark.asyncio
async def test_expired_reader_cannot_resume_with_late_new_input():
    r = Rig()
    await r.writer.open()
    await r.publish(1)
    await r.reader.read_latest()
    r.ns += 99_000_000
    with pytest.raises(ValueError, match='input_expired'):
        await r.reader.read_latest()
    await r.publish(2)
    with pytest.raises(ValueError, match='retired'):
        await r.reader.read_latest()


@pytest.mark.asyncio
@pytest.mark.parametrize('failure', ['timeout', 'revoked', 'nan'])
async def test_failed_read_retires_instance(failure):
    r = Rig()

    async def fail(channel):
        if failure == 'timeout':
            raise TimeoutError()
        if failure == 'revoked':
            r.authorized = False
        else:
            r.io.values[BASE + BANK_SIZE] = float('nan')

    r.io.after_read = fail
    with pytest.raises((ValueError, TimeoutError)):
        await r.reader.read_latest()
    assert r.reader.closed
    r.authorized = True
    r.io.after_read = None
    with pytest.raises(ValueError, match='retired'):
        await r.reader.read_latest()


@pytest.mark.asyncio
async def test_old_bank_cannot_roll_target_back_and_duplicate_cannot_mutate():
    r = Rig()
    await r.writer.open()
    await r.publish(1)
    r.ns += 1_000_000
    await r.publish(2)
    await r.reader.read_latest()
    r.io.values[BASE + BANK_SIZE] = 0
    with pytest.raises(ValueError, match='rollback'):
        await r.reader.read_latest()
    reader = PoseMailboxReader(r.io, controller_ms=lambda: r.ns // 1_000_000, not_before_ms=1000, authorize=r.authorize)
    await reader.read_latest()
    r.io.values[BASE + 1] += 1
    with pytest.raises(ValueError, match='mutated'):
        await reader.read_latest()


@pytest.mark.asyncio
async def test_epoch_and_sequence_wrap_do_not_reanimate_previous_session():
    r = Rig(STAMP_MOD - 1)
    await r.writer.open()
    await r.publish(1023)
    assert (await r.reader.read_latest()).sequence_mod == 1023
    r.ns += 2_000_000
    await r.publish(1024)
    sample = await r.reader.read_latest()
    assert sample.sequence_mod == 0 and sample.captured_ms == STAMP_MOD + 1
    r.ns += (STAMP_MOD // 2) * 1_000_000
    with pytest.raises(ValueError, match='clock'):
        await r.reader.read_latest()
    assert header(STAMP_MOD - 1, 1023) == 2**31 - 1
    with pytest.raises(ValueError, match='reserved_epoch'):
        header(STAMP_MOD, 1024)


@pytest.mark.asyncio
async def test_nonzero_admission_and_capture_before_epoch_are_rejected():
    r = Rig()
    r.io.values[BASE] = header(999, 1)
    assert await r.reader.read_latest() is None
    with pytest.raises(ValueError, match='occupied'):
        await r.writer.open()
    assert r.writer.closed and not r.io.writes


@pytest.mark.parametrize('pose', [
    Pose.model_construct(p=(float('nan'), 0., 0.), q=(0., 0., 0., 1.)),
    Pose.model_construct(p=(0., 0., 0.), q=(0., 0., 0., 0.)),
    Pose(p=(3000., 0., 0.), q=(0., 0., 0., 1.)),
])
def test_invalid_pose_is_rejected_before_writes(pose):
    with pytest.raises(ValueError, match='pose_mailbox'):
        pack(pose)


def test_invalid_quaternion_payload_is_not_normalized_into_a_goal():
    with pytest.raises(ValueError, match='quaternion'):
        unpack((0, 0, 0, 0, 0, 0, 0))


@pytest.mark.asyncio
async def test_new_frame_after_input_gap_cannot_revive_writer():
    r = Rig()
    await r.writer.open()
    await r.publish(1)
    r.ns += 100_000_000
    count = len(r.io.writes)
    with pytest.raises(ValueError, match='expired'):
        r.offer(2)
    with pytest.raises(ValueError, match='closed'):
        await r.writer.publish_latest()
    assert len(r.io.writes) == count


@pytest.mark.asyncio
async def test_cancelling_inflight_write_retains_unknown_and_ownership():
    r = Rig()
    await r.writer.open()
    r.offer(1)
    entered = asyncio.Event()

    async def pending(channel, value):
        entered.set()
        await asyncio.Event().wait()

    r.io.after_write = pending
    task = asyncio.create_task(r.writer.publish_latest())
    await entered.wait()
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert r.writer.unknown and r.writer.closed and r.lease.owner is r.writer.token
    assert len(r.io.writes) == 1


@pytest.mark.asyncio
async def test_commit_response_after_deadline_cannot_be_consumed():
    r = Rig()
    await r.writer.open()
    r.offer(1)

    async def delayed_commit(channel, value):
        if channel == BASE and value:
            r.ns += 100_000_000

    r.io.after_write = delayed_commit
    with pytest.raises(ValueError, match='expired'):
        await r.writer.publish_latest()
    assert r.io.values[BASE] != 0  # Known write, but its capture time is stale.
    assert await r.reader.read_latest() is None
    assert r.writer.closed and not r.writer.unknown


@pytest.mark.asyncio
async def test_read_crossing_deadline_retires_without_returning_pose():
    r = Rig()
    await r.writer.open()
    await r.publish(1)

    async def slow(channel):
        r.ns += 10_000_000

    r.io.after_read = slow
    with pytest.raises(ValueError, match='expired'):
        await r.reader.read_latest()
    assert r.reader.closed


@pytest.mark.asyncio
async def test_concurrent_read_is_rejected_without_disturbing_inflight_read():
    r = Rig()
    await r.writer.open()
    await r.publish(1)

    async def reenter(channel):
        r.io.after_read = None
        with pytest.raises(ValueError, match='read_inflight'):
            await r.reader.read_latest()

    r.io.after_read = reenter
    assert (await r.reader.read_latest()).sequence_mod == 1
    assert not r.reader.closed


@pytest.mark.asyncio
async def test_conflicting_same_capture_time_is_rejected():
    r = Rig()
    await r.writer.open()
    await r.publish(1)
    r.io.values[BASE + BANK_SIZE] = header(1000, 2)
    with pytest.raises(ValueError, match='ambiguous'):
        await r.reader.read_latest()
