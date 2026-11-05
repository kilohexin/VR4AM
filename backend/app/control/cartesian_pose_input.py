"""Clutch capture and genuine XR input for the new Cartesian candidate.

Pure input state only: no SDK, executor selection or automatic recapture.
Reading the latest target never creates a new timestamp or renews a lease.
"""
from app.control.finite_position_types import PositionBinding,PositionIntent
from app.control.vr_pose_mailbox import CapturedPose
from app.control.vr_pose_reference import VRPoseReference


class CartesianPoseInput:
    def __init__(self,binding,intent,tcp,*,tcp_ns,clock,translation_scale=1.):
        if type(binding) is not PositionBinding:raise ValueError('cartesian_input_binding')
        self.binding=binding;self.clock=clock;self.closed=False;self.reason=None
        self.session=None;self.last_seq=-1;self.last_ns=None;self.latest=None
        self.last_now=clock();self.anchor_tcp=tcp.model_copy(deep=True)
        self.reference=VRPoseReference(clock=clock,translation_scale=translation_scale,orientation_mode='tool')
        if type(intent) is not PositionIntent or intent.kind!='held' or intent.hand_pose is None:
            raise ValueError('cartesian_input_capture')
        self.reference.capture(intent.hand_pose,tcp,generation=binding.generation,
            hand_ns=intent.received_ns,tcp_ns=tcp_ns)
        self.offer(intent)

    def close(self,reason='release'):
        self.closed=True;self.reason=self.reason or reason;self.latest=None
        self.reference.release()

    def _now(self):
        now=self.clock()
        if type(now) is not int or now<self.last_now:
            self.close('cartesian_input_clock');raise ValueError(self.reason)
        self.last_now=now
        if self.last_ns is not None and now-self.last_ns>=500_000_000:
            self.close('input_expired')
        return now

    def offer(self,intent):
        now=self._now()
        if self.closed:raise ValueError('cartesian_input_retired')
        if (type(intent) is not PositionIntent or intent.generation!=self.binding.generation
                or not intent.session_id.startswith('xr-finite-')
                or self.session not in (None,intent.session_id) or intent.seq<=self.last_seq
                or self.last_ns is not None and intent.received_ns<self.last_ns
                or not 0<=now-intent.received_ns<100_000_000):
            self.close('cartesian_input_identity_or_age');raise ValueError(self.reason)
        self.session=intent.session_id;self.last_seq=intent.seq;self.last_ns=intent.received_ns
        if intent.kind=='release':self.close('release');return
        if intent.hand_pose is None:
            self.close('cartesian_input_pose_required');raise ValueError(self.reason)
        try:
            target=self.reference.target(intent.hand_pose,generation=intent.generation,
                seq=intent.seq,captured_ns=intent.received_ns)
        except Exception:
            self.close('cartesian_input_mapping');raise
        self.latest=CapturedPose(intent.seq,intent.received_ns,target)

    def sample(self):
        now=self._now()
        if self.closed or self.latest is None or now-self.latest.captured_ns>=100_000_000:return None
        s=self.latest
        return CapturedPose(s.seq,s.captured_ns,s.pose.model_copy(deep=True))
