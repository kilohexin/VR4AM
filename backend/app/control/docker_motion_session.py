"""Opt-in Docker translation blending; rotations retain the finite executor."""
from dataclasses import replace
from app.control.blended_position_session import BlendedPositionSession
from app.control.finite_position_session import FinitePositionSession
from app.robots.blended_position_docker import DockerBlendPort
from app.robots.blended_rotation_docker import DockerRotationPort


class DockerMotionSession:
    def __init__(self, adapter, binding, *, clock, recorder, authorize):
        self.adapter, self.binding = adapter, binding
        self.clock, self.recorder = clock, recorder
        def checked():
            authorize()
            if (getattr(recorder,'fatal_error',None) is not None
                    or getattr(recorder,'dropped_normal_events',0)):
                raise ValueError('position_recording_unavailable')
        self.authorize = checked
        self.active = None
        self.kind = None
        self.seq = -1
        self.source = None
        self.completed_steps = 0
        self._select('translation')

    def _select(self, kind):
        if self.active is not None:
            self.authorize()
            if self.active.busy or self.active.phase != 'ready' or self.active.closed:
                raise ValueError('position_busy')
            self.completed_steps += self.active.view().entered_steps
            if self.kind in ('translation','pitch','spin'):
                self.active.port.check()
                self.active.port.closed = True
        self.kind = kind
        if kind in ('translation','pitch','spin'):
            if kind=='translation':
                port = DockerBlendPort(self.adapter, self.binding, authorize=self.authorize)
                lower,upper=self.adapter.settings.startup_tcp_min_m,self.adapter.settings.startup_tcp_max_m
            else:
                port=DockerRotationPort(self.adapter,self.binding,authorize=self.authorize,axis=kind)
                lower,upper=(-port.coordinate_limit,)*3,(port.coordinate_limit,)*3
            self.active = BlendedPositionSession(port, self.binding,
                lower=lower, upper=upper,
                clock=self.clock, authorize=self.authorize)
        else:
            self.active = FinitePositionSession(self.adapter, self.binding, clock=self.clock,
                recorder=self.recorder, authorize=self.authorize)

    @property
    def busy(self):return self.active.busy
    @property
    def worker(self):return self.active.worker
    @property
    def phase(self):return self.active.phase
    @property
    def gate(self):return self.active.gate

    def offer(self, intent):
        self.authorize()
        if intent.seq <= self.seq or self.source not in (None, intent.session_id):
            raise ValueError('position_input_rejected')
        desired = 'rotation' if intent.pitch or intent.spin else 'translation'
        if getattr(self.adapter,'position_blended_rotation',False) is True and not any(intent.vector):
            if intent.pitch and not intent.spin:desired='pitch'
            elif intent.spin and not intent.pitch:desired='spin'
        if intent.kind != 'release' and desired != self.kind:
            if not self.active.busy and self.active.phase == 'ready' and not self.active.closed:
                self._select(desired)
            else:
                # Finish the existing mode, never run two executors together.
                # A later fresh held frame must request the new mode again.
                intent = replace(intent, kind='release', vector=(0.,0.,0.), pitch=0., spin=0.)
        if self.kind in ('pitch','spin') and intent.kind!='release':
            value=intent.pitch if self.kind=='pitch' else intent.spin
            intent=replace(intent,vector=(value,0.,0.) if self.kind=='pitch' else (0.,0.,value),pitch=0.,spin=0.)
        self.active.offer(intent)
        self.seq, self.source = intent.seq, intent.session_id

    async def advance(self):await self.active.advance()
    async def abort(self, reason):await self.active.abort(reason)
    def view(self):
        view = self.active.view()
        return replace(view, entered_steps=view.entered_steps+self.completed_steps)
