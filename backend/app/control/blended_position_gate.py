"""Geometry/ownership gate for Docker blended translation experiments.

No SDK access. A reservation must be revalidated at native entry. Unknown writes
permanently revoke this instance; task completion alone never reanchors motion.
"""
from dataclasses import dataclass
import math
from app.control.finite_position_types import integer,vector,INPUT_NS


@dataclass(frozen=True)
class BlendTarget:
    position: tuple
    direction: tuple
    barrier: int


class BlendedPositionGate:
    step=.004
    max_lead=.008
    def __init__(self,binding,lower,upper,*,clock):
        self.binding=binding;self.clock=clock
        self.lower=vector(lower,3);self.upper=vector(upper,3)
        if any(a>=b for a,b in zip(self.lower,self.upper)):raise ValueError('blend_region')
        self.latest=None;self.seq=-1;self.stamp=-1;self.direction=None
        self.tail=None;self.pending=[];self.reserved=None;self.ids=set()
        self.barrier=0;self.draining=False;self.revoked=False;self.quiet_since=None
        self.observed_stamp=-1;self.quiet_anchor=None

    def _live(self,binding):
        if binding is not self.binding or self.revoked:raise ValueError('blend_revoked')
        return integer(self.clock())

    def offer(self,binding,seq,stamp,direction):
        now=self._live(binding);integer(seq);integer(stamp);d=vector(direction,3)
        if seq<=self.seq or stamp<self.stamp or not 0<=now-stamp<INPUT_NS:raise ValueError('blend_input_expired')
        n=math.hypot(*d);d=tuple(x/n for x in d) if n>.15 else None
        if d!=self.latest:
            self.barrier+=1
            if self.tail is not None:self.draining=True
        self.seq,self.stamp,self.latest=seq,stamp,d

    def observe(self,binding,actual,stamp,*,finished=(),idle=False,stationary=False):
        now=self._live(binding);actual=vector(actual,3);integer(stamp)
        if not 0<=now-stamp<INPUT_NS or stamp<=self.observed_stamp:
            raise ValueError('blend_feedback_expired')
        if any(x not in self.ids for x in finished):raise ValueError('blend_foreign_task')
        if (stamp-self.observed_stamp>=INPUT_NS or
                self.quiet_anchor is not None and math.dist(actual,self.quiet_anchor)>.0001):
            self.quiet_since=None;self.quiet_anchor=None
        self.observed_stamp=stamp
        self.pending=[x for x in self.pending if x not in finished]
        if self.tail is not None and not self.pending and self.reserved is None and idle and stationary:
            if self.quiet_since is None:self.quiet_since=stamp;self.quiet_anchor=actual
            if stamp-self.quiet_since>=200_000_000:
                self.tail=None;self.direction=None;self.draining=False
        else:self.quiet_since=None;self.quiet_anchor=None

    def reserve(self,binding,actual,stamp):
        now=self._live(binding);actual=vector(actual,3)
        if not 0<=now-stamp<INPUT_NS:raise ValueError('blend_feedback_expired')
        if self.latest is None or self.draining or self.reserved is not None or len(self.pending)>=2:return None
        if not 0<=now-self.stamp<INPUT_NS:raise ValueError('blend_input_expired')
        if self.direction is not None and self.latest!=self.direction:return None
        anchor=self.tail or actual
        target=tuple(x+self.step*d for x,d in zip(anchor,self.latest))
        if math.dist(actual,anchor)+self.step>self.max_lead+1e-12:return None
        if any(not lo<=x<=hi for x,lo,hi in zip(target,self.lower,self.upper)):return None
        self.reserved=BlendTarget(target,self.latest,self.barrier)
        return self.reserved

    def require_entry(self,binding,target,actual,stamp):
        now=self._live(binding);actual=vector(actual,3)
        if (target is not self.reserved or target.barrier!=self.barrier or self.draining
                or target.direction!=self.latest or not 0<=now-self.stamp<INPUT_NS
                or not 0<=now-stamp<INPUT_NS or len(self.pending)>=2
                or math.dist(actual,self.tail or actual)+self.step>self.max_lead+1e-12):
            raise ValueError('blend_entry_revoked')

    def acknowledge(self,binding,target,motion_id):
        self._live(binding)
        # A reply after release still belongs to us and must drain.
        if target is not self.reserved or type(motion_id) is not int or motion_id<=0 or motion_id in self.ids:
            self.revoke();raise ValueError('blend_reply_unknown')
        self.ids.add(motion_id);self.pending.append(motion_id)
        self.tail=target.position;self.direction=target.direction;self.reserved=None
        if target.barrier!=self.barrier:self.draining=True

    def cancel_unsent(self,target):
        if target is self.reserved:self.reserved=None

    def revoke(self):self.revoked=True;self.latest=None;self.barrier+=1
