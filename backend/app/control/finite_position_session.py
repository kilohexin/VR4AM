"""One offline finite goal at a time; normal release drains, faults stop."""
import asyncio
from app.control.finite_position_gate import PositionGate
from app.control.finite_position_types import PositionView,INPUT_NS
from app.robots.base import StopReason


class FinitePositionSession:
    def __init__(self,adapter,binding,*,clock,recorder,authorize):
        self.adapter,self.binding,self.clock,self.recorder,self.authorize=adapter,binding,clock,recorder,authorize
        self.gate=PositionGate(binding,clock=clock);self.worker=None
        self.phase='ready';self.reason=None;self.motion_id=None;self.closed=False
        self._last_seq=-1;self._session=None;self._rearm=False

    @property
    def busy(self):return self.worker is not None

    def offer(self,intent):
        self.authorize()
        if self.closed or self.phase=='fault':raise ValueError('position_session_closed')
        if intent.seq<=self._last_seq or self._session not in (None,intent.session_id):
            raise ValueError('position_input_rejected')
        if self.phase=='hold' and not self.busy:
            if intent.kind=='release':
                self._rearm=True;self._last_seq=intent.seq;self._session=intent.session_id;return
            elif self._rearm:
                self.gate=PositionGate(self.binding,clock=self.clock);self.phase='ready';self.reason=None;self._rearm=False
        self.gate.offer(intent);self._last_seq=intent.seq;self._session=intent.session_id
        if intent.kind=='release' and self.adapter.position_busy:self.phase='draining'

    def _monitor(self):
        self.authorize()
        if self.closed or self.gate.reason:raise ValueError('position_session_revoked')
        now=self.clock()
        if self.gate.deadline_ns is not None and now>=self.gate.deadline_ns:
            if not self.busy and self.gate.released:
                self.phase='hold';self._rearm=True;return
            raise ValueError('position_session_expired')
        if not self.gate.released and self.gate.latest is not None:
            if not 0<=now-self.gate.latest.received_ns<INPUT_NS:raise ValueError('position_input_expired')
        if (getattr(self.recorder,'fatal_error',None) is not None
                or getattr(self.recorder,'dropped_normal_events',0)):
            raise ValueError('position_recording_unavailable')

    async def _run_step(self,intent):
        try:
            def entry_authorize():
                self._monitor();self.gate.require_entry(intent)
            plan=await self.adapter.plan_step(intent,authorize=entry_authorize)
            def entered():
                entry_authorize();self.gate.mark_entered(intent);self.phase='executing'
            result=await self.adapter.execute_step(plan,before_entry=entry_authorize,
                monitor_authorize=self._monitor,on_entered=entered)
            self.motion_id=result.motion_id
            if not result.complete:
                if not result.entered and result.reason in ('position_entry_revoked','position_snapshot_expired'):
                    self.phase='ready';self.reason=result.reason
                else:self.phase='fault';self.reason=result.reason
            else:self.phase='ready';self.reason=None
            await self.recorder.write_event(dict(kind='finite_position_result',complete=result.complete,
                entered=result.entered,remote_unknown=result.remote_unknown,motion_id=result.motion_id,
                reason=result.reason),self.clock())
        except asyncio.CancelledError:raise
        except Exception as error:
            reason=str(error)
            if reason in ('position_entry_revoked','position_unresolvable_step','position_snapshot_expired'):
                self.phase='hold' if reason=='position_unresolvable_step' else 'ready';self.reason=reason
            else:
                self.phase='fault';self.reason=reason if reason.startswith('position_') else 'position_session_failed'
                if self.adapter.position_busy:await self.adapter.stop(StopReason.FAULT)
        finally:self.gate.finish()

    async def advance(self):
        if self.closed:return
        if self.worker is not None and self.worker.done():
            try:self.worker.result()
            except asyncio.CancelledError:pass
            self.worker=None
        if self.phase=='fault':return
        try:self._monitor()
        except Exception as error:
            self.reason=str(error) if str(error).startswith('position_') else 'position_revoked'
            await self.abort(StopReason.STALE);return
        if self.worker is not None:return
        if self.phase=='hold':return
        if self.gate.entered_steps>=10:self.phase='hold';return
        intent=self.gate.take_latest()
        if intent is not None:
            self.phase='planning';self.reason=None
            self.worker=asyncio.create_task(self._run_step(intent),name='finite-position-step')

    async def abort(self,reason):
        if self.closed:return
        self.closed=True;self.phase='fault';self.reason=self.reason or 'position_aborted'
        self.gate.revoke(self.reason);self.adapter.position_aborted=True
        if self.adapter.position_busy:
            try:await self.adapter.stop(reason)
            except BaseException:self.reason='stop_unverified'
        if self.worker is not None and not self.worker.done():
            self.worker.cancel()
            await asyncio.wait({self.worker},timeout=.5)
        if self.adapter._latched_fault:self.reason=self.adapter._latched_fault

    def view(self):
        return PositionView(self.phase,self.binding.generation,self.gate.entered_steps,self.motion_id,self.reason)
