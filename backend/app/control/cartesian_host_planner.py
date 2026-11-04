"""Transactional host planning for a controller-owned Cartesian chain.

Pure host state: no sockets/SDK, no physical target, no implicit recovery.
Only one calculation can exist. A late result cannot advance the committed
reference. Native braking may finish without another Grip/zero capture;
planning then resumes from the exact known terminal with the ORIGINAL box.
"""
from copy import copy
from dataclasses import dataclass

from app.control.cartesian_bundle_wire import CompactBundle
from app.control.cartesian_pvat_reference import JointReference
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor


@dataclass(frozen=True)
class PlanningJob:
    identity: int
    parent_tick: int
    due_ms: int
    captured_ms: int
    source: OfflineReferenceSupervisor
    target: object


@dataclass(frozen=True)
class PlannedBundle:
    job: PlanningJob
    bundle: CompactBundle|None
    checkpoint: OfflineReferenceSupervisor
    terminal_checkpoint: OfflineReferenceSupervisor|None=None


def _branch(supervisor):
    result=copy(supervisor)
    result.options=dict(supervisor.options)
    result.lower=supervisor.lower.copy();result.upper=supervisor.upper.copy()
    return result


class CartesianHostPlanner:
    def __init__(self,model,start,*,session,model_sha256,generation,clock,
            workspace_span=.35,**options):
        self.clock=clock;self.session=session;self.model_sha256=model_sha256;self.generation=generation
        self.supervisor=OfflineReferenceSupervisor(model,start,workspace_span=workspace_span,
            braking_method='joint_fit',continue_on_known_rejection=True,**options)
        # Validate identities using the codec before any work can be started.
        segment=self.supervisor._tail[0]
        CompactBundle(session,model_sha256,generation,1,0,1,0,segment,start,1)
        self.checkpoints={start:_branch(self.supervisor)}
        self.allowed_states={0:{start}};self.terminal_hold=(0,start);self.head=start;self.head_tick=0
        self.tip=start;self.tip_tick=0
        self.native_phase=None
        self.target=None;self.captured_ms=None;self.input_sequence=0
        self.job=None;self.pending=None;self.sequence=0;self.jobs=0
        self.closed=False;self.reason=None;self.last_now=self.clock()
        self.publish_unknown=False
        self.accepted={}

    def _now(self):
        value=self.clock()
        if type(value) is not int or value<self.last_now or value<0:
            self.close('cartesian_host_clock');raise ValueError(self.reason)
        self.last_now=value;return value

    def _live(self):
        if self.closed:raise ValueError('cartesian_host_retired')
        return self._now()

    def close(self,reason='release'):
        self.closed=True
        if self.reason is None:self.reason=reason
        # Preserve any unsettled publish obligation for the transport owner.
        # Running work is retired, never considered cancelled/safe-to-reuse.

    def offer(self,target,*,captured_ms,sequence,generation):
        now=self._live()
        if self.captured_ms is not None and now-self.captured_ms>=500:
            self.close('input_expired');raise ValueError(self.reason)
        if generation!=self.generation or type(generation) is not int:
            self.close('cartesian_host_generation');raise ValueError(self.reason)
        if (type(captured_ms) is not int or not 0<=now-captured_ms<100
                or self.captured_ms is not None and captured_ms<self.captured_ms
                or type(sequence) is not int or sequence<=self.input_sequence):
            self.close('cartesian_host_input');raise ValueError(self.reason)
        # Pose schema already enforces finite, normalized, immutable tuples.
        from app.schemas.messages import Pose
        if not isinstance(target,Pose):
            self.close('cartesian_host_target');raise ValueError(self.reason)
        self.target=target.model_copy(deep=True);self.captured_ms=captured_ms;self.input_sequence=sequence

    def observe_head(self,state,tick,*,generation):
        self._live()
        if (generation!=self.generation or type(generation) is not int or type(tick) is not int
                or tick<self.head_tick or not isinstance(state,JointReference)
                or tick==self.head_tick and state!=self.head):
            self.close('cartesian_host_head');raise ValueError(self.reason)
        # Lua evaluates the owned quintic with a different FP operation order.
        # It may report an ULP-different intermediate brake state. Recognize it
        # for monitoring, but NEVER plan from that approximate match: wait for
        # the exact wire-carried terminal instead.
        candidates=self.allowed_states.get(tick,set())
        if self.terminal_hold is not None and tick>=self.terminal_hold[0]:
            candidates=candidates|{self.terminal_hold[1]}
        if state not in candidates:
            if not any(max(abs(a-b) for a,b in zip(state.q+state.v+state.a,s.q+s.v+s.a))<=1e-10
                    for s in candidates):
                self.close('cartesian_host_unowned_head');raise ValueError(self.reason)
        self.head=state;self.head_tick=tick
        if tick>=self.tip_tick:self.tip=state;self.tip_tick=tick
        self.allowed_states={t:states for t,states in self.allowed_states.items() if t>=tick}

    def planning_head(self):
        """At most two queued leads, including a not-yet-emitted ACKed tip."""
        if self.tip_tick-self.head_tick>=2:return None
        if self.tip_tick==self.head_tick and self.native_phase=='holding':
            return self.head_tick+1,self.head
        return self.tip_tick,self.tip

    def observe_owned_cursor(self,tick,sequence,phase,*,generation):
        """Controller-owned reference metadata, NEVER measured robot PVA."""
        if sequence==0 and tick==0:state=self.supervisor.state
        else:
            packet=self.accepted.get(sequence)
            if packet is None:
                self.close('cartesian_host_cursor');raise ValueError(self.reason)
            offset=tick-packet.parent_tick-1
            if offset==0:state=packet.lead.end
            elif 1<=offset<=packet.braking_ticks:state=packet.expand_tail()[offset-1].end
            elif offset>packet.braking_ticks and phase in ('holding','ready'):
                state=packet.terminal
                # A newly ACKed future lead must not erase the presently
                # emitted packet's exact zero-rate hold while its idle prefix
                # is consumed. The cursor identifies WHICH certified tail.
                self.allowed_states.setdefault(tick,set()).add(state)
            else:
                self.close('cartesian_host_cursor');raise ValueError(self.reason)
        self.native_phase=phase;self.observe_head(state,tick,generation=generation)

    def begin(self,due_ms):
        now=self._live()
        if self.job is not None or self.pending is not None:return None
        if self.captured_ms is None or now-self.captured_ms>=500:
            self.close('input_expired');return None
        tip=self.planning_head()
        if now-self.captured_ms>=100 or tip is None or tip[1] not in self.checkpoints:return None
        if type(due_ms) is not int or not now<due_ms<=now+500:
            raise ValueError('cartesian_host_deadline')
        self.jobs+=1
        self.job=PlanningJob(self.jobs,tip[0],due_ms,self.captured_ms,
            _branch(self.checkpoints[tip[1]]),self.target)
        return self.job

    def calculate(self,job):
        if job is not self.job:raise ValueError('cartesian_host_job')
        checkpoint=_branch(job.source)
        approved=checkpoint.advance(job.target)
        if approved.mode!='following':return PlannedBundle(job,None,checkpoint)
        packet=CompactBundle(self.session,self.model_sha256,self.generation,self.sequence+1,
            job.parent_tick,job.due_ms,job.captured_ms,approved.segment,
            approved.braking_tail[-1].end,len(approved.braking_tail))
        if packet.expand_tail()!=approved.braking_tail:
            raise ValueError('cartesian_host_compaction')
        # Only create a lazy mathematical checkpoint. If native braking ever
        # reaches this EXACT zero-rate terminal, advance() first validates a
        # new constant hold through its existing empty-tail path. Do not pay
        # for that unused hold at every successful following point.
        resumed=_branch(checkpoint);resumed.state=packet.terminal
        resumed._tail=();resumed._tail_checks=();resumed.phase='following';resumed.reason=None
        return PlannedBundle(job,packet,checkpoint,resumed)

    def complete(self,result):
        # A cancelled/released calculation still has to finish; only this
        # boundary can retire its slot. No second concurrent calculation.
        if result.job is not self.job:raise ValueError('cartesian_host_job')
        self.job=None
        if self.closed:return None
        now=self._live()
        if self.captured_ms is None or now-self.captured_ms>=500:
            self.close('input_expired');return None
        tip=self.planning_head()
        quiet_prefix=(self.native_phase=='holding' and self.tip_tick==self.head_tick
            and result.job.parent_tick in (self.head_tick,self.head_tick+1)
            and result.job.source.state==self.head)
        if (now>=result.job.due_ms or now-result.job.captured_ms>=100 or tip is None
                or not quiet_prefix and (result.job.parent_tick!=tip[0] or result.job.source.state!=tip[1])):
            return None
        if result.bundle is None:return None
        self.pending=result;return result.bundle

    def acknowledge(self,sequence,accepted,*,known=True):
        self._now()
        if not known:
            self.publish_unknown=True;self.close('cartesian_publish_unknown')
            raise ValueError('cartesian_publish_unknown')
        result=self.pending
        if (result is None or sequence!=result.bundle.sequence or type(accepted) is not bool):
            self.close('cartesian_host_ack');raise ValueError(self.reason)
        self.pending=None;self.sequence=sequence  # A known rejection also consumes the sequence.
        if not accepted or self.closed:return
        branch=result.checkpoint;packet=result.bundle
        self.accepted[sequence]=packet
        if len(self.accepted)>4:del self.accepted[min(self.accepted)]
        self.tip=packet.lead.end;self.tip_tick=packet.parent_tick+1
        self.checkpoints[packet.lead.end]=_branch(branch)
        self.allowed_states.setdefault(packet.parent_tick+1,set()).add(packet.lead.end)
        for index,segment in enumerate(packet.expand_tail(),packet.parent_tick+2):
            self.allowed_states.setdefault(index,set()).add(segment.end)
        self.terminal_hold=(packet.parent_tick+1+packet.braking_ticks,packet.terminal)
        # A known terminal may resume without moving the preparation box or
        # capturing a new hand zero. Quiet initialization validates it afresh.
        self.checkpoints={packet.lead.end:_branch(branch),packet.terminal:result.terminal_checkpoint}
