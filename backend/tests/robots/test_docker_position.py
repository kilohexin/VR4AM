"""The official simulator selector never grants a real connection path."""
from dataclasses import replace
import pytest
from app.recording.noop import NoopRecorder


def test_docker_settings_use_separate_loopback_zero_tool_and_mdh_ready():
    from app.digital_twin.docker_runtime import docker_settings
    s=docker_settings()
    assert s.lebai.ip=='127.0.0.1' and s.lebai.mode=='control'
    assert s.lebai.expected_tcp.z==0 and s.lebai.teleop_ready_q[5]==0
    assert s.lebai.control.max_joint_speed_radps==.3
    assert s.lebai.control.max_tcp_speed_mps==.03
    assert s.lebai.gripper.max_force_percent==30


def test_docker_selector_is_explicit_and_exposes_its_own_identity():
    from app.digital_twin.docker_runtime import docker_settings
    from app.main import build_backend
    a=build_backend(docker_settings(),NoopRecorder(),backend_label='LEBAI_DOCKER',docker_simulation=True)
    assert a._backend_label=='LEBAI_DOCKER' and a.position_policy=='finite_position_docker'
    assert a.position_vr_enabled and a.position_pitch_enabled
    assert a._position_joint_speed_cap==.3


@pytest.mark.parametrize('speed',[0,float('nan'),float('inf'),.301])
def test_docker_profile_still_rejects_invalid_or_unbounded_joint_speed(speed):
    from app.digital_twin.docker_runtime import docker_settings
    from app.robots.finite_position_docker import DockerFinitePositionAdapter
    s=docker_settings().lebai
    with pytest.raises(ValueError,match='profile_required'):
        DockerFinitePositionAdapter(replace(s,control=replace(s.control,max_joint_speed_radps=speed)))


def test_real_web_constructor_does_not_accept_the_docker_speed_profile():
    from app.digital_twin.docker_runtime import docker_settings
    from app.robots.finite_position_web import RealFinitePositionAdapter
    s=replace(docker_settings().lebai,ip='192.0.2.1')
    with pytest.raises(ValueError,match='explicit_profile_required'):
        RealFinitePositionAdapter(s,client_factory=lambda _:None,position_model=object(),authorize=lambda:None)


def test_virtual_timing_is_distinct_from_physical_stop_contract():
    from app.robots.finite_position_docker import DockerFinitePositionAdapter
    from app.robots.lebai_adapter import RealLebaiAdapter
    assert RealLebaiAdapter._stop_rpc_timeout_s==.2
    assert RealLebaiAdapter._stop_verification_ns==500_000_000
    assert DockerFinitePositionAdapter._stop_rpc_timeout_s==1.
    assert DockerFinitePositionAdapter._stop_verification_ns==2_000_000_000


@pytest.mark.parametrize('damage',['target','label','real','fake'])
def test_docker_selector_rejects_mixed_profiles_without_connecting(damage):
    from app.digital_twin.docker_runtime import docker_settings
    from app.main import build_backend
    s=docker_settings();kw=dict(backend_label='LEBAI_DOCKER',docker_simulation=True)
    if damage=='target':s=replace(s,lebai=replace(s.lebai,ip='172.16.2.162'))
    if damage=='label':kw['backend_label']='LEBAI'
    if damage=='real':kw['real_finite_position']=True
    if damage=='fake':kw['offline_finite_position']=True
    with pytest.raises(ValueError):build_backend(s,NoopRecorder(),**kw)


def test_docker_status_roundtrips_without_being_labelled_real_or_fake():
    from app.schemas.messages import RobotStateMessage,FinitePositionStatus
    message=RobotStateMessage(v=1,type='robot_state',server_mono_ns=1,mode='READY',robot_state='IDLE',
        actual_tcp=dict(p=[0,0,0],q=[0,0,0,1]),actual_q=[0]*6,gripper=0,
        backend='LEBAI_DOCKER',finite_position=FinitePositionStatus(policy='finite_position_docker',
            phase='ready',generation=1,entered_steps=0))
    assert RobotStateMessage.model_validate_json(message.model_dump_json()).backend=='LEBAI_DOCKER'


async def test_cancelled_virtual_start_remains_owned_and_cannot_be_reconnected(monkeypatch):
    import asyncio
    from types import SimpleNamespace
    from app.robots import finite_position_docker as d
    from app.digital_twin import docker_runtime as runtime
    entered=asyncio.Event();release=asyncio.Event();writes=[];connections=[]
    async def state():return 'STOP'
    async def start():
        writes.append('start_sys');entered.set()
        try:await release.wait()
        except asyncio.CancelledError:await release.wait()
    async def factory():connections.append('connect');return SimpleNamespace(get_robot_state=state,start_sys=start)
    monkeypatch.setattr(d,'connect_docker_simulator_client',factory)
    monkeypatch.setattr(runtime,'verify_local_docker',lambda:{'target':'127.0.0.1'})
    a=d.DockerFinitePositionAdapter(runtime.docker_settings().lebai)
    task=asyncio.create_task(a.connect());await entered.wait();task.cancel()
    try:
        done,_=await asyncio.wait({task},timeout=.1)
        assert task in done, 'cancelled startup must return without waiting for native cancellation'
        with pytest.raises(asyncio.CancelledError):await task
        assert a._docker_bootstrap_calls.unknown_methods=={'initialize_simulator'}
        with pytest.raises(Exception,match='bootstrap_result_unknown'):await a.connect()
        assert writes==['start_sys'] and connections==['connect']
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)
        if hasattr(a,'_docker_bootstrap_calls'):
            await asyncio.gather(*tuple(a._docker_bootstrap_calls.tasks),return_exceptions=True)
    assert a._docker_bootstrap_calls.unknown_methods=={'initialize_simulator'}


async def test_disconnect_during_start_never_attaches_the_late_client(monkeypatch):
    import asyncio
    from types import SimpleNamespace
    from app.robots import finite_position_docker as d
    from app.digital_twin import docker_runtime as runtime
    entered=asyncio.Event();release=asyncio.Event();states=iter(['STOP','IDLE'])
    async def state():return next(states)
    async def start():
        entered.set()
        try:await release.wait()
        except asyncio.CancelledError:return
    async def factory():return SimpleNamespace(get_robot_state=state,start_sys=start)
    monkeypatch.setattr(d,'connect_docker_simulator_client',factory)
    monkeypatch.setattr(runtime,'verify_local_docker',lambda:{})
    a=d.DockerFinitePositionAdapter(runtime.docker_settings().lebai)
    task=asyncio.create_task(a.connect());await entered.wait()
    try:
        await a.disconnect();release.set()
        with pytest.raises((TimeoutError,ValueError,RuntimeError)):await task
        assert a._docker_ready_client is None and a._client is None
        assert a._docker_bootstrap_calls.unknown_methods
    finally:
        release.set();await asyncio.gather(task,*tuple(a._docker_bootstrap_calls.tasks),return_exceptions=True)


async def test_parallel_connect_is_reserved_before_container_inspection(monkeypatch):
    import asyncio,threading
    from types import SimpleNamespace
    from app.robots import finite_position_docker as d
    from app.digital_twin import docker_runtime as runtime
    entered=threading.Event();release=threading.Event();connections=[]
    def inspect():entered.set();release.wait(2);return {}
    async def state():return 'IDLE'
    async def fk(q):
        from app.schemas.messages import Pose
        from app.robots.lebai_codec import pose_to_lebai
        p,r=a.position_model.forward(q);return pose_to_lebai(Pose(p=p,q=r))
    async def factory():
        connections.append(1);return SimpleNamespace(get_robot_state=state,kinematics_forward=fk)
    async def connected(self):return 'connected'
    monkeypatch.setattr(d,'connect_docker_simulator_client',factory)
    monkeypatch.setattr(runtime,'verify_local_docker',inspect)
    monkeypatch.setattr(d.RealFinitePositionAdapter,'connect',connected)
    a=d.DockerFinitePositionAdapter(runtime.docker_settings().lebai)
    first=asyncio.create_task(a.connect())
    while not entered.is_set():await asyncio.sleep(0)
    try:
        with pytest.raises(ValueError,match='pending'):await a.connect()
    finally:release.set();await first
    assert connections==[1]


@pytest.mark.parametrize('where',['connect','fk'])
async def test_late_bootstrap_result_cannot_dispatch_another_native_call(monkeypatch,where):
    import asyncio
    from types import SimpleNamespace
    from app.robots import finite_position_docker as d
    from app.digital_twin import docker_runtime as runtime
    from app.schemas.messages import Pose
    from app.robots.lebai_codec import pose_to_lebai
    entered=asyncio.Event();release=asyncio.Event();calls=[];started=False
    async def delayed():
        entered.set()
        try:await release.wait()
        except asyncio.CancelledError:await release.wait()
    async def state():calls.append('state');return 'IDLE' if where=='fk' or started else 'STOP'
    async def start():
        nonlocal started
        started=True;calls.append('start_sys')
    async def fk(q):
        calls.append('fk')
        if where=='fk':await delayed()
        p,r=a.position_model.forward(q);return pose_to_lebai(Pose(p=p,q=r))
    native=SimpleNamespace(get_robot_state=state,start_sys=start,kinematics_forward=fk)
    async def factory():
        if where=='connect':await delayed()
        return native
    monkeypatch.setattr(d,'connect_docker_simulator_client',factory)
    monkeypatch.setattr(runtime,'verify_local_docker',lambda:{})
    a=d.DockerFinitePositionAdapter(runtime.docker_settings().lebai)
    task=asyncio.create_task(a.connect());await entered.wait();task.cancel()
    try:
        with pytest.raises(asyncio.CancelledError):await task
        release.set();await asyncio.gather(*tuple(a._docker_bootstrap_calls.tasks),return_exceptions=True)
        assert calls==([] if where=='connect' else ['state','fk'])
        assert a._docker_bootstrap_calls.unknown_methods and a._docker_ready_client is None
    finally:release.set();await asyncio.gather(task,return_exceptions=True)


@pytest.mark.parametrize('failure',[False,True])
async def test_page_poll_waits_for_planning_entry_window_without_reusing_feedback(monkeypatch,failure):
    import asyncio
    from app.robots.finite_position_docker import DockerFinitePositionAdapter
    from app.robots.finite_position_web import RealFinitePositionAdapter
    from app.digital_twin.docker_runtime import docker_settings
    a=DockerFinitePositionAdapter(docker_settings().lebai)
    planning=asyncio.Event();finish=asyncio.Event();reads=[]
    async def plan(self,*args,**kwargs):
        planning.set();await finish.wait()
        if failure:raise ValueError('position_planning_expired')
        return 'owned-plan'
    async def execute(self,plan,**kwargs):
        assert plan=='owned-plan'
        kwargs['on_entered']()
        return 'completed'
    async def state(self):reads.append('new-controller-read');return 'new-state'
    monkeypatch.setattr(RealFinitePositionAdapter,'plan_step',plan)
    monkeypatch.setattr(RealFinitePositionAdapter,'execute_step',execute)
    monkeypatch.setattr(RealFinitePositionAdapter,'get_state',state)
    worker=asyncio.create_task(a.plan_step(None,authorize=lambda:None));await planning.wait()
    poll=asyncio.create_task(a.get_state());await asyncio.sleep(0);await asyncio.sleep(0)
    try:
        assert not poll.done() and reads==[], 'page polling must not queue ahead of fresh-entry send'
        finish.set()
        if failure:
            with pytest.raises(ValueError,match='planning_expired'):await worker
        else:
            result=await worker
            await asyncio.sleep(0)
            assert not poll.done() and reads==[]
            assert await a.execute_step(result,on_entered=lambda:None)=='completed'
        assert await asyncio.wait_for(poll,.2)=='new-state'
        assert reads==['new-controller-read']
    finally:
        finish.set();worker.cancel();poll.cancel()
        await asyncio.gather(worker,poll,return_exceptions=True)
