"""Latest intent only; release is a barrier, never remote cancellation."""
import math
from app.control.finite_position_types import PositionBinding,PositionIntent,integer,INPUT_NS,SESSION_NS


class PositionGate:
    def __init__(self,binding,*,clock,diagnostic_limits=True,direction_tolerance_rad=0.):
        self.binding,self.clock=binding,clock
        self.diagnostic_limits=diagnostic_limits
        if not 0<=direction_tolerance_rad<=math.radians(5):raise ValueError('position_direction_tolerance')
        self.direction_tolerance_rad=direction_tolerance_rad
        self.latest=self.active=None
        self.started_ns=None;self.deadline_ns=None
        self.entered_steps=0;self.released=True;self.reason=None
        self._last_seq=-1;self._last_stamp=-1;self._session=None
        self._barrier=0;self._taken_barrier=0;self._entered=False
        self._finish_seq=-1;self._finish_ns=-1

    def _live(self):
        now=integer(self.clock())
        if self.reason:raise ValueError(self.reason)
        if self.started_ns is not None and (now<self.started_ns or self.deadline_ns is not None and now>=self.deadline_ns):
            raise ValueError('position_session_expired')
        return now

    def offer(self,intent):
        now=self._live()
        if (type(intent) is not PositionIntent or intent.generation!=self.binding.generation
                or intent.seq<=self._last_seq or intent.received_ns<self._last_stamp
                or not 0<=now-intent.received_ns<INPUT_NS
                or self._session not in (None,intent.session_id)):
            raise ValueError('position_input_rejected')
        if self.started_ns is None:
            self.started_ns=now;self.deadline_ns=now+SESSION_NS if self.diagnostic_limits else None;self._session=intent.session_id
        self._last_seq,self._last_stamp=intent.seq,intent.received_ns
        self.latest=intent;self.released=intent.kind=='release'
        if self.released:self._barrier+=1

    def take_latest(self):
        now=self._live()
        if self.active is not None or self.released or self.latest is None or self.exhausted:return None
        if not 0<=now-self.latest.received_ns<INPUT_NS:raise ValueError('position_input_expired')
        if self.latest.seq<=self._finish_seq or self.latest.received_ns<=self._finish_ns:return None
        self.active=self.latest;self._taken_barrier=self._barrier;self._entered=False
        return self.active

    def require_entry(self,intent):
        now=self._live()
        direction_matches=self.latest is not None and self.latest.vector==intent.vector
        if (not direction_matches and self.latest is not None and self.direction_tolerance_rad
                and any(self.latest.vector) and any(intent.vector)):
            direction_matches=sum(a*b for a,b in zip(self.latest.vector,intent.vector))>=math.cos(self.direction_tolerance_rad)-1e-12
        if (intent is not self.active or self._entered or self.exhausted
                or self.released or self.latest is None or not direction_matches
                or self.latest.pitch!=intent.pitch or self.latest.spin!=intent.spin
                or self._taken_barrier!=self._barrier
                or not 0<=now-self.latest.received_ns<INPUT_NS):
            raise ValueError('position_entry_revoked')

    def mark_entered(self,intent):
        self.require_entry(intent);self._entered=True;self.entered_steps+=1

    @property
    def exhausted(self):return self.diagnostic_limits and self.entered_steps>=10

    def finish(self):
        self._finish_seq=self._last_seq;self._finish_ns=self.clock()
        self.active=None;self._entered=False

    def revoke(self,reason):
        self.reason=reason;self.latest=None;self.released=True;self._barrier+=1
