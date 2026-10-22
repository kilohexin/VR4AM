"""Explicit real web selector; shares finite planning and original fault stops.

Construction never connects. The launcher supplies the approved scope guard;
the ordinary application and the Fake selector cannot opt in implicitly.
"""
import asyncio
from pathlib import Path
from app.robots.base import BackendCommandError
from app.robots.finite_position_adapter import _FinitePositionCore


class _WebClient:
    READS=frozenset({'is_connected','get_robot_state','get_estop_reason','get_kin_data',
        'get_tcp','get_claw','get_running_motion','get_motion_state',
        'kinematics_inverse','kinematics_forward','get_velocity_factor'})

    def __init__(self,native,authorize,entry_check,claw_entry_check):
        self.native=native;self.authorize=authorize;self.entry_check=entry_check
        self.claw_entry_check=claw_entry_check
        self.move_entries=0;self.claw_entries=0;self.move_unknown=False;self.move_returned=True

    def __getattr__(self,name):
        if name in self.READS:return getattr(self.native,name)
        raise AttributeError(name)

    async def movej(self,*args):
        self.authorize()
        def entering():
            self.entry_check()
            self.move_entries+=1;self.move_returned=False
        guarded=getattr(self.native,'_position_guarded_movej',None)
        before=self.move_entries
        try:
            if callable(guarded):result=await guarded(args,entering)
            else:entering();result=await self.native.movej(*args)
            self.move_returned=True
            return result
        except BaseException:
            if self.move_entries>before:self.move_unknown=True
            raise

    async def set_claw(self,*args):
        self.authorize()
        def entering():
            self.claw_entry_check();self.claw_entries+=1
        guarded=getattr(self.native,'_position_guarded_claw',None)
        if callable(guarded):return await guarded(args,entering)
        entering()
        return await self.native.set_claw(*args)

    async def stop_move(self):
        # Expiry/revocation never removes the original exception stop route.
        return await self.native.stop_move()

    async def move_pvat(self,*args):raise BackendCommandError('position_web_stream_disabled')
    async def stop_sys(self):raise BackendCommandError('position_web_reset_disabled')


class RealFinitePositionAdapter(_FinitePositionCore):
    position_candidate=True
    position_policy='finite_position_web'

    def __init__(self,settings,*,client_factory,position_model,authorize,**kwargs):
        if (settings.mode!='control' or settings.ip in ('digital-twin','127.0.0.1','localhost')
                or not callable(authorize) or position_model is None
                or not 0<settings.control.max_joint_speed_radps<=.05
                or not 0<settings.control.max_joint_acceleration_radps2<=.2):
            raise ValueError('position_web_explicit_profile_required')
        self._web_authorize=authorize
        self._web_prepare_authorize=None
        self._web_entry_plan=None;self._web_before_entry=None;self._web_on_entered=None;self._web_move_floor=0
        async def factory(ip):
            authorize()
            native=await client_factory(ip)
            return _WebClient(native,authorize,self._web_native_entry_check,self._web_claw_entry_check)
        super().__init__(settings,client_factory=factory,position_model=position_model,
            backend_label='LEBAI',arrival_policy='coarse_grasp',**kwargs)
        self._position_sources.add(Path(__file__))
        self._position_identity=self._source_identity();self._position_metadata=self._source_metadata()
        self._finite_snapshot_wrappers=set()

    async def connect(self):
        self._web_authorize()
        return await super().connect()

    def bind_position_generation(self,generation):
        self._web_authorize()
        return super().bind_position_generation(generation)

    def _position_health(self,binding):
        self._web_authorize()
        return super()._position_health(binding)

    def _web_native_entry_check(self):
        plan=self._web_entry_plan
        if plan is None:
            if self._web_prepare_authorize is None:raise BackendCommandError('position_web_prepare_not_owned')
            self._web_prepare_authorize()
            self._web_fresh_feedback()
            return
        self._web_before_entry()
        self._position_entry_snapshot(plan)
        from app.control.finite_position_types import INPUT_NS
        if not 0<=self._clock()-plan.start.captured_ns<INPUT_NS:
            raise ValueError('position_snapshot_expired')
        self._web_on_entered()

    async def execute_step(self,plan,**kwargs):
        if self._web_entry_plan is not None:raise BackendCommandError('position_busy')
        self._web_entry_plan=plan;self._web_before_entry=kwargs['before_entry']
        self._web_on_entered=kwargs['on_entered']
        # Reserve the gesture only at the final native boundary. Before that,
        # the same input can still be released/replaced during scope checking.
        kwargs['on_entered']=lambda:None
        self._web_move_floor=self._client.move_entries
        try:return await super().execute_step(plan,**kwargs)
        finally:self._web_entry_plan=None;self._web_before_entry=None;self._web_on_entered=None

    def _position_native_entered(self):return self._client.move_entries>self._web_move_floor

    def _position_native_unknown(self):
        return self._client.move_unknown or self._position_native_entered() and not self._client.move_returned

    def _web_claw_entry_check(self):
        self._require_gripper_authority()
        if self.position_busy:raise BackendCommandError('position_busy')
        self._web_fresh_feedback()

    def _web_fresh_feedback(self):
        from app.control.finite_position_types import INPUT_NS
        if self._snapshot is None:raise BackendCommandError('position_feedback_missing')
        if not 0<=self._clock()-self._snapshot.captured_ns<INPUT_NS:
            raise BackendCommandError('position_snapshot_expired')

    def web_idle_close_safe(self):
        """Narrow proof for page-close draining before this connection actuates."""
        return bool(self._client is not None and self._client.move_entries==0
            and self._client.claw_entries==0 and not self.position_busy
            and self._web_prepare_authorize is None and self._web_entry_plan is None
            and not self._gripper_active and not self._motion_accepted
            and not self._motion_write_stop_required and not self._latched_fault
            and not self._snapshot_calls.unknown_methods and not self._motion_calls.unknown_methods
            and not self._motion_calls.tasks and self._snapshot is not None
            and self._snapshot.robot_state.value=='IDLE' and self._snapshot.running_motion is None)

    async def prepare(self,options,on_phase,*,authorize=None):
        if self._web_prepare_authorize is not None:raise BackendCommandError('position_busy')
        self._web_prepare_authorize=authorize or (lambda:None)
        try:return await super().prepare(options,on_phase,authorize=authorize)
        finally:self._web_prepare_authorize=None

    async def _read_snapshot_fields(self,client,latencies,deadline_ns,**kwargs):
        if kwargs['read_purpose']=='stop_verification':
            return await super()._read_snapshot_fields(client,latencies,deadline_ns,**kwargs)
        owner=self._snapshot_calls;generation=self._pump.generation
        def check():
            if (client is not self._client or owner is not self._snapshot_calls or owner.closed
                    or owner.unknown_methods
                    or (kwargs['read_purpose']!='state' and generation!=self._pump.generation)):
                raise BackendCommandError('position_snapshot_binding_changed')
        from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
        return await FiniteWindowDiagnosticAdapter._parallel_snapshot_fields(self,client,latencies,
            deadline_ns,check=check,include_raw_kinematics=kwargs['include_raw_kinematics'])

    async def disconnect(self):
        for task in tuple(self._finite_snapshot_wrappers):
            if not task.done():task.cancel()
        return await super().disconnect()

    async def home(self,*args,**kwargs):raise BackendCommandError('position_web_home_disabled')
    async def submit_tcp(self,*args,**kwargs):raise BackendCommandError('position_web_stream_disabled')
    async def command_tcp(self,*args,**kwargs):raise BackendCommandError('position_web_stream_disabled')
    async def submit_catch_up(self,*args,**kwargs):raise BackendCommandError('position_web_stream_disabled')
