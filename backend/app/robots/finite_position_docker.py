"""Official Docker adapter; it cannot select a non-loopback native endpoint."""
import asyncio
import time
from pathlib import Path
from app.robots.finite_position_web import RealFinitePositionAdapter
from app.robots.lebai_sdk_bridge import connect_docker_simulator_client
from app.commissioning.finite_position_site import MdhPositionModel


class DockerFinitePositionAdapter(RealFinitePositionAdapter):
    position_policy='finite_position_docker'
    gripper_provider='local_virtual'
    _stop_rpc_timeout_s=1.
    _stop_verification_ns=2_000_000_000
    _stop_cleanup_grace_s=3.5
    position_offline_prepare_scheduler=True
    position_idle_input_guard=True

    def __init__(self,settings,*,client_factory=None,**kwargs):
        if (settings.ip!='127.0.0.1' or settings.mode!='control'
                or not 0<settings.control.max_joint_speed_radps<=.05
                or not 0<settings.control.max_joint_acceleration_radps2<=.2):
            raise ValueError('docker_simulator_profile_required')
        self.position_vr_enabled=True;self.position_pitch_enabled=True
        kwargs.setdefault('clock',time.perf_counter_ns)
        self._docker_native=client_factory is None;self._docker_admission=None;self._docker_ready_client=None
        from app.robots.lebai_motion_calls import OwnedMotionCalls
        self._docker_bootstrap_calls=OwnedMotionCalls(verify_completion_authority=True)
        self._docker_bootstrap_closed=False
        self._docker_connecting=False
        self._docker_finished_task=None
        async def factory(ip):
            if ip!='127.0.0.1':raise ValueError('docker_simulator_loopback_required')
            if self._docker_ready_client is None:raise ValueError('docker_simulator_not_initialized')
            return self._docker_ready_client
        self._initialize_web_core(settings,client_factory=client_factory or factory,
            position_model=MdhPositionModel.from_settings(settings),authorize=lambda:None,
            backend_label='LEBAI_DOCKER',**kwargs)
        self._position_sources.add(Path(__file__))
        self._position_identity=self._source_identity();self._position_metadata=self._source_metadata()

    async def connect(self):
        if self._docker_connecting:raise ValueError('docker_bootstrap_pending')
        self._docker_connecting=True
        try:return await self._connect_once()
        finally:self._docker_connecting=False

    async def _connect_once(self):
        if self._docker_native:
            if self._docker_bootstrap_calls.unknown_methods:
                raise ValueError('docker_bootstrap_result_unknown')
            if self._docker_bootstrap_closed:
                raise ValueError('docker_bootstrap_closed')
            if self._docker_bootstrap_calls.tasks:
                raise ValueError('docker_bootstrap_pending')
            if self._docker_ready_client is not None:
                return await super().connect()
            from app.digital_twin.docker_runtime import verify_local_docker,ensure_docker_idle
            self._docker_admission=await asyncio.to_thread(verify_local_docker)
            async def initialize():
                client=await connect_docker_simulator_client()
                await ensure_docker_idle(client)
                from app.digital_twin.virtual_claw import DockerArmWithVirtualClaw
                return DockerArmWithVirtualClaw(client,clock=self._clock)
            self._docker_ready_client=await self._docker_bootstrap_calls.call(
                'initialize_simulator',initialize,15,revoked=lambda:self._docker_bootstrap_closed)
        return await super().connect()

    async def disconnect(self):
        self._docker_bootstrap_closed=True
        self._docker_bootstrap_calls.close()
        return await super().disconnect()

    def docker_idle_input_safe(self):
        """No owned actuator operation exists; this is not a stopping result."""
        if self.position_idle_input_guard is not True or not self.web_idle_close_safe():return False
        try:self._check_known_snapshot_fault(self._client,self._pump.generation)
        except Exception:return False
        # This only proves this connection never acquired actuator obligations.
        # An old display sample does not create an obligation to issue stop.
        # It grants no readiness: unlock/prepare keep their fresh preflight.
        return True

    def docker_quiet_close_safe(self):
        """Completed owned actions permit draining a read, never skipping stop."""
        s=self._snapshot
        return bool(self._client is not None and s is not None and s.robot_state.value=='IDLE'
            and s.running_motion is None and not self.position_busy and not self._stop_lock.locked()
            and not self._gripper_active and not self._motion_accepted
            and not self._motion_write_stop_required and not self._latched_fault
            and self._web_prepare_authorize is None and self._web_entry_plan is None
            and not self._motion_calls.tasks and not self._motion_calls.unknown_methods
            and not self._snapshot_calls.unknown_methods
            and not self._snapshot_calls.closed and not self._pump.has_pending
            and all(x==0 for v in (s.actual_qd,s.target_qd,s.actual_qdd,s.target_qdd) for x in v))

    async def _read_snapshot_fields(self,client,latencies,deadline_ns,**kwargs):
        # Connection is read before this group and retained task state after it.
        # Keep all fields and original stop predicates; only scheduling differs.
        return await self._web_parallel_snapshot_fields(client,latencies,deadline_ns,**kwargs)

    async def _position_progress(self,motion_id,authorize):
        # Task IDs refer to one immutable owned target. Read its state first,
        # then new kinematics so waiting for the status cannot age that sample.
        # Arrival still requires FINISHED, IDLE, matching task and stable feedback.
        key=self._docker_task_key(motion_id)
        if self._docker_finished_task==key:
            state='FINISHED'
        else:
            state=await self._position_math('get_motion_state',lambda:self._client.get_motion_state(motion_id),authorize)
            if str(state).upper()=='FINISHED':self._docker_finished_task=key
        authorize();self._position_health(self.position_binding)
        sample=await self._read_snapshot(read_purpose='position_progress')
        return sample,state

    def _docker_task_key(self,motion_id):
        return (self._client,self._snapshot_calls,self._pump.generation,self.position_binding,motion_id)

    async def _read_retained_task_state(self,client,motion_id,latencies,deadline_ns,read_purpose):
        # A FINISHED task is terminal, not evidence of physical settling. Only
        # reuse its identity while the same finite step owns this connection.
        # All pose/rate/fault samples remain newly read, with unchanged limits.
        if (client is self._client and self.position_busy and not self.position_aborted
                and self._docker_finished_task==self._docker_task_key(motion_id)
                and not self._snapshot_calls.unknown_methods):
            return 'FINISHED'
        return await super()._read_retained_task_state(client,motion_id,latencies,deadline_ns,read_purpose)

    async def execute_step(self,*args,**kwargs):
        self._docker_finished_task=None
        try:return await super().execute_step(*args,**kwargs)
        finally:self._docker_finished_task=None

    async def _position_progress_pause(self,started_ns):
        # Reads already consume the sample period. Do not add 20 ms on top.
        await self._sleep(max(0,20_000_000-(self._clock()-started_ns))/1_000_000_000)

    async def _wait_for_next_stop_sample(self,read_started_ns):
        await self._position_progress_pause(read_started_ns)
