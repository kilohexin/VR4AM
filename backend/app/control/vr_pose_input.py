"""Injected host pipeline: XR pose -> clutch reference -> latest pose mailbox.

No endpoint factory or product executor selection. The caller binds a fresh
robot sample and controller clock before capture; controller completion is a
separate obligation from withdrawing hand input.
"""
import asyncio

from app.control.finite_position_types import PositionIntent
from app.control.vr_pose_mailbox import BASE, BANK_SIZE, TTL_NS, CapturedPose, PoseMailboxWriter
from app.control.vr_pose_reference import VRPoseReference


class VRPoseInput:
    def __init__(self, client, lease, binding, *, clock, stamp_for_capture,
                 authorize, authorize_cleanup, translation_scale=2., authorize_publish=None):
        self.client, self.lease, self.binding = client, lease, binding
        self.clock, self.authorize, self.authorize_cleanup = clock, authorize, authorize_cleanup
        self.reference = VRPoseReference(clock=clock, translation_scale=translation_scale)
        self.writer = PoseMailboxWriter(client, lease, clock=clock,
            stamp_for_capture=stamp_for_capture, authorize=authorize, bulk=True,
            before_commit=authorize_publish or authorize)
        self.closed = False
        self.reason = None
        self.worker = None
        self.unknown_cleanup = False
        self.completed = False
        self.last_seq = -1
        self.last_ns = None
        self.session = None
        self.max_publish_ns = 0
        self.max_offer_gap_ns = self.max_offer_age_ns = self.max_publish_gap_ns = 0
        self.last_published_ns = None
        self.publish_tail = []
        self.wake = asyncio.Event()

    def _check(self):
        if self.closed:
            raise ValueError('vr_pose_input_closed')
        try:
            self.authorize()
            self.lease.require(self.writer.token)
        except BaseException:
            self._retire('vr_pose_authority_revoked')
            raise

    def _retire(self, reason):
        self.closed = True
        self.reason = self.reason or reason
        self.reference.release()
        self.writer.revoke()
        self.wake.set()

    async def open(self):
        self._check()
        try:
            await self.writer.open()
        except BaseException:
            self._retire('vr_pose_open_failed')
            raise

    def capture(self, hand, tcp, *, hand_ns, tcp_ns):
        self._check()
        if not self.writer.opened or self.reference.generation is not None:
            raise ValueError('vr_pose_capture_not_available')
        try:
            self.reference.capture(hand, tcp, generation=self.binding.generation,
                                   hand_ns=hand_ns, tcp_ns=tcp_ns)
        except BaseException:
            self._retire('vr_pose_capture_expired')
            raise
        self.last_ns = hand_ns

    def offer(self, intent):
        self._check()
        if (type(intent) is not PositionIntent or intent.generation != self.binding.generation
                or not intent.session_id.startswith('xr-finite-')
                or self.session not in (None, intent.session_id) or intent.seq <= self.last_seq
                or (self.last_ns is not None and intent.received_ns < self.last_ns)):
            self._retire('vr_pose_input_identity')
            raise ValueError('vr_pose_input_identity')
        if self.last_ns is not None:
            self.max_offer_gap_ns=max(self.max_offer_gap_ns,intent.received_ns-self.last_ns)
        self.max_offer_age_ns=max(self.max_offer_age_ns,self.clock()-intent.received_ns)
        if (not 0 <= self.clock() - intent.received_ns < TTL_NS
                or self.last_ns is None or not 0 <= self.clock() - self.last_ns < TTL_NS):
            self._retire('vr_pose_input_expired')
            raise ValueError('vr_pose_input_expired')
        self.session, self.last_seq, self.last_ns = intent.session_id, intent.seq, intent.received_ns
        if intent.kind == 'release':
            self._retire('vr_pose_released')
            return
        if intent.hand_pose is None:
            self._retire('vr_pose_requires_full_pose')
            raise ValueError('vr_pose_requires_full_pose')
        try:
            goal = self.reference.target(intent.hand_pose, generation=intent.generation,
                                         seq=intent.seq, captured_ns=intent.received_ns)
            self.writer.offer(CapturedPose(intent.seq, intent.received_ns, goal))
        except BaseException:
            self._retire('vr_pose_target_rejected')
            raise
        self.wake.set()

    def inherit_capture(self, reference, *, latest_ns):
        """Keep the zero captured at Grip, not a later moving-hand position."""
        self._check()
        if (type(reference) is not VRPoseReference or reference.generation != self.binding.generation
                or reference.clock != self.clock or not self.writer.opened
                or not 0 <= self.clock()-latest_ns < TTL_NS):
            self._retire('vr_pose_capture_not_available')
            raise ValueError('vr_pose_capture_not_available')
        self.reference = reference
        self.last_ns = latest_ns

    def start(self):
        self._check()
        if self.worker is not None or self.reference.generation is None:
            raise ValueError('vr_pose_start_not_available')
        self.worker = asyncio.create_task(self._run(), name='vr-pose-latest-input')

    async def _run(self):
        try:
            while not self.closed:
                self._check()
                if self.last_ns is None or not 0 <= self.clock() - self.last_ns < TTL_NS:
                    self._retire('vr_pose_input_expired')
                    return
                self.wake.clear()
                started=self.clock()
                if await self.writer.publish_latest():
                    now=self.clock()
                    gap=now-self.last_published_ns if self.last_published_ns is not None else 0
                    self.max_publish_gap_ns=max(self.max_publish_gap_ns,gap)
                    self.last_published_ns=now
                    self.max_publish_ns=max(self.max_publish_ns,now-started)
                    self.publish_tail.append(dict(seq=self.writer.written_seq,host_ns=now,
                        capture_ns=self.writer.last_written_capture_ns,
                        age_ms=(now-self.writer.last_written_capture_ns)/1e6,
                        duration_ms=(now-started)/1e6,gap_ms=gap/1e6))
                    self.publish_tail=self.publish_tail[-16:]
                try:
                    await asyncio.wait_for(self.wake.wait(), .02)
                except TimeoutError:
                    pass
        except asyncio.CancelledError:
            self._retire('vr_pose_pump_cancelled')
            raise
        except Exception:
            self._retire('vr_pose_write_unknown' if self.writer.unknown else 'vr_pose_pump_rejected')

    async def withdraw(self):
        if not self.closed:
            self._retire('vr_pose_withdrawn')
        # Joining does not cancel or pretend a started native write completed.
        if self.worker is not None:
            while not self.worker.done():
                try:
                    await asyncio.shield(self.worker)
                except asyncio.CancelledError:
                    if self.worker.done():
                        break
                    continue
            await self.worker

    def revoke(self, reason='vr_pose_withdrawn'):
        self._retire(reason)

    async def complete(self, *, controller_ok, quiet_observed, program_exited):
        await self.withdraw()
        if self.completed:
            raise ValueError('vr_pose_already_completed')
        if (controller_ok is not True or quiet_observed is not True or program_exited is not True
                or self.writer.unknown or self.writer.read_unknown or self.unknown_cleanup or self.writer.busy):
            raise ValueError('vr_pose_completion_unverified')
        self.lease.require(self.writer.token)
        self.authorize_cleanup()
        # Native motion is known finished. Unknown cleanup still retains lease.
        try:
            for channel in (BASE, BASE + BANK_SIZE):
                self.authorize_cleanup()
                await asyncio.wait_for(self.client.set_signal(channel, 0), .1)
            for channel in (BASE, BASE + BANK_SIZE):
                self.authorize_cleanup()
                value = await asyncio.wait_for(self.client.get_signal(channel), .1)
                self.authorize_cleanup()
                if type(value) is not int or value != 0:
                    raise ValueError('vr_pose_cleanup_unverified')
        except BaseException:
            self.unknown_cleanup = True
            raise
        self.lease.release(self.writer.token)
        self.completed = True
