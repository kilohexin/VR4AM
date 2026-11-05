"""Latest endpoint ownership, no IK, interpolation, SDK or motion admission.

Release stops input and may drain ONE still-fresh latest endpoint after an
already known write. The accepted finite target is then held, not cancelled.
`holding` describes input ownership, not measured physical standstill.
"""
from dataclasses import dataclass
import math

from app.schemas.messages import Pose


@dataclass(frozen=True)
class Endpoint:
    seq: int
    captured_ns: int
    pose: Pose


class EndpointMailbox:
    def __init__(self,session,generation,lower,upper):
        if type(session) is not str or not session or type(generation) is not int or generation<0:
            raise ValueError('endpoint_identity')
        if any(len(v)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in v) for v in (lower,upper)):
            raise ValueError('endpoint_workspace')
        if any(a>=b for a,b in zip(lower,upper)):raise ValueError('endpoint_workspace')
        self.session=session;self.generation=generation;self.lower=tuple(lower);self.upper=tuple(upper)
        self.phase='active';self.fault=None;self.unknown=False;self.completed_writes=0
        self.latest=None;self.inflight=None;self.last_seq=-1;self.last_capture=None;self.last_now=None
        self.released=False;self._final_pose=None;self.last_motion_id=None
        self.entered_ns=None

    @property
    def final_pose(self):return self._final_pose.model_copy(deep=True) if self._final_pose else None

    def fail(self,reason):
        reason=reason if type(reason) is str and reason else 'endpoint_failure'
        self.fault=self.fault or reason;self.unknown |= self.inflight is not None
        self.latest=None;self.phase='fault'

    def _reject(self,reason):
        self.fail(reason);raise ValueError(reason)

    def _clock(self,now):
        if type(now) is not int or now<0 or self.last_now is not None and now<self.last_now:
            self._reject('endpoint_clock')
        self.last_now=now
        if self.inflight is not None and now-self.entered_ns>=200_000_000:
            self.fail('endpoint_reply_deadline')

    def offer(self,session,generation,seq,captured_ns,pose,now_ns):
        self._clock(now_ns)
        if self.fault or self.released:self._reject('endpoint_retired')
        if (session!=self.session or type(generation) is not int or generation!=self.generation
                or type(seq) is not int or seq<=self.last_seq
                or type(captured_ns) is not int or not 0<=captured_ns<=now_ns
                or now_ns-captured_ns>=100_000_000
                or self.last_capture is not None and captured_ns<self.last_capture):
            self._reject('endpoint_identity_or_age')
        if (type(pose) is not Pose or len(pose.p)!=3 or len(pose.q)!=4
                or any(type(x) not in (float,int) or not math.isfinite(x) for x in pose.p+pose.q)
                or abs(math.hypot(*pose.q)-1)>1e-6
                or any(x<lo or x>hi for x,lo,hi in zip(pose.p,self.lower,self.upper))):
            self._reject('endpoint_target')
        self.latest=Endpoint(seq,captured_ns,pose.model_copy(deep=True))
        self.last_seq=seq;self.last_capture=captured_ns

    def release(self):
        self.released=True
        if not self.fault:self.phase='draining' if self.inflight or self.latest else 'holding'

    def take(self,now_ns):
        self._clock(now_ns)
        if self.fault or self.inflight is not None:return None
        if self.latest is not None and now_ns-self.latest.captured_ns>=100_000_000:self.latest=None
        if self.latest is None:
            if self.released:self.phase='holding'
            return None
        item=self.latest;self.latest=None
        self.inflight=Endpoint(item.seq,item.captured_ns,item.pose.model_copy(deep=True))
        self.entered_ns=now_ns
        # Callers cannot modify the internally owned command via returned Pose.
        return Endpoint(item.seq,item.captured_ns,item.pose.model_copy(deep=True))

    def returned(self,seq,motion_id,now_ns):
        self._clock(now_ns)
        if (self.inflight is None or type(seq) is not int or seq!=self.inflight.seq
                or type(motion_id) is not int or not 0<=motion_id<2**53):
            self._reject('endpoint_receipt')
        if now_ns-self.entered_ns>=200_000_000:self.fail('endpoint_reply_deadline')
        item=self.inflight;self.inflight=None;self.completed_writes+=1
        if self.fault:return  # Late return cannot erase earlier uncertainty.
        self._final_pose=item.pose.model_copy(deep=True);self.last_motion_id=motion_id
        if self.released:self.phase='draining' if self.latest else 'holding'
