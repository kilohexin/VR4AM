"""Unregistered Docker translation session, using product web/XR intents.

The injected port must audit Cartesian paths and enforce entry at the native
boundary. No native client or application selector is provided by this module.
"""
import asyncio
from dataclasses import dataclass

from app.control.blended_position_gate import BlendedPositionGate
from app.control.finite_position_types import INPUT_NS, PositionIntent, PositionView


@dataclass(frozen=True)
class BlendFeedback:
    position: tuple
    captured_ns: int
    finished: tuple = ()
    idle: bool = False
    stationary: bool = False


class BlendedPositionSession:
    def __init__(self, port, binding, *, lower, upper, clock, authorize):
        if port.policy != 'docker_blended_candidate':
            raise ValueError('blend_docker_candidate_required')
        self.port, self.binding, self.clock, self.authorize = port, binding, clock, authorize
        self.gate = BlendedPositionGate(binding, lower, upper, clock=clock)
        self.phase, self.reason = 'ready', None
        self.closed = False
        self.worker = None
        self.motion_id = None
        self.entered_steps = 0
        self.intent = None
        self._session_id = None
        self._stop_task = None
        self._obligation = False

    @property
    def busy(self):
        return self.worker is not None or self.gate.tail is not None

    def offer(self, intent: PositionIntent):
        self.authorize()
        if self.closed:
            raise ValueError('position_session_closed')
        if (intent.generation != self.binding.generation
                or self._session_id not in (None, intent.session_id)):
            raise ValueError('position_input_rejected')
        # Rotation must use its separately audited executor. Never silently
        # treat pitch/spin as translation or release during an owned stream.
        if intent.pitch or intent.spin:
            raise ValueError('blend_translation_only')
        self.gate.offer(self.binding, intent.seq, intent.received_ns, intent.vector)
        self.intent = intent
        self._session_id = intent.session_id
        if self.gate.draining:
            self.phase = 'draining'

    def _monitor(self):
        self.authorize()
        if self.closed:
            raise ValueError('position_session_revoked')
        if (self.intent is not None and self.intent.kind != 'release'
                and not 0 <= self.clock()-self.intent.received_ns < INPUT_NS):
            raise ValueError('position_input_expired')

    async def _read(self):
        self._monitor()
        sample = await self.port.read(tuple(self.gate.pending))
        self._monitor()
        self.gate.observe(self.binding, sample.position, sample.captured_ns,
                          finished=sample.finished, idle=sample.idle, stationary=sample.stationary)
        if self.gate.tail is None and not self.gate.pending:
            self._obligation = False
        return sample

    async def _tick(self):
        target = None
        native_entered = False
        try:
            sample = await self._read()
            target = self.gate.reserve(self.binding, sample.position, sample.captured_ns)
            if target is None:
                self.phase = 'draining' if self.gate.tail is not None else 'ready'
                return
            self.phase = 'planning'
            plan = await self.port.audit(target)
            self._monitor()
            sample = await self._read()

            def entering():
                nonlocal native_entered
                self._monitor()
                if native_entered:
                    raise ValueError('blend_duplicate_entry')
                self.gate.require_entry(self.binding, target, sample.position, sample.captured_ns)
                native_entered = True
                self._obligation = True
                self.entered_steps += 1

            # Port owns timeouts/unknown results and invokes entering exactly
            # at native entry, before any write. Release during the await may
            # invalidate entry; a reply after entry still belongs to us.
            ident = await self.port.submit(plan, entering=entering)
            if not native_entered:
                raise ValueError('blend_entry_not_confirmed')
            self.gate.acknowledge(self.binding, target, ident)
            self.motion_id = ident
            self.phase = 'draining' if self.gate.draining else 'executing'
        except asyncio.CancelledError:
            if not self.closed:
                await self.abort('position_worker_cancelled')
            raise
        except Exception as error:
            if not self.closed:
                if str(error) == 'blend_entry_revoked' and not native_entered and not self._obligation:
                    self.phase = 'ready'
                elif str(error) == 'blend_entry_revoked' and not native_entered and self.gate.draining:
                    self.phase = 'draining'
                else:
                    await self.abort(str(error))
        finally:
            if target is not None:
                self.gate.cancel_unsent(target)

    async def advance(self):
        if self.closed:
            return
        if self.worker is not None and self.worker.done():
            try:
                self.worker.result()
            except asyncio.CancelledError:
                pass
            self.worker = None
        try:
            self._monitor()
        except Exception as error:
            await self.abort(str(error))
            return
        if self.worker is None and (self.intent is not None or self.gate.tail is not None):
            self.worker = asyncio.create_task(self._tick(), name='docker-blended-tick')

    async def abort(self, reason):
        if not self.closed:
            self.closed = True
            self.phase, self.reason = 'fault', reason
            self.gate.revoke()
            if self._obligation:
                self._stop_task = asyncio.create_task(self.port.stop(), name='docker-blended-stop')
        if self._stop_task is not None:
            try:
                confirmed = await asyncio.shield(self._stop_task)
                if confirmed is not True:
                    self.reason = 'stop_unverified'
            except Exception:
                self.reason = 'stop_unverified'
        # An in-flight port remains owned until its own bounded call returns;
        # closing never clears its unknown-result evidence or renews a binding.

    def view(self):
        return PositionView(self.phase, self.binding.generation, self.entered_steps,
                            self.motion_id, self.reason)
