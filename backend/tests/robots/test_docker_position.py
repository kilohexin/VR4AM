"""The official simulator selector never grants a real connection path."""
from dataclasses import replace
import pytest
from app.recording.noop import NoopRecorder


def test_docker_settings_use_separate_loopback_zero_tool_and_mdh_ready():
    from app.digital_twin.docker_runtime import docker_settings
    s=docker_settings()
    assert s.lebai.ip=='127.0.0.1' and s.lebai.mode=='control'
    assert s.lebai.expected_tcp.z==0 and s.lebai.teleop_ready_q[5]==0
    assert s.lebai.control.max_joint_speed_radps==.05
    assert s.lebai.gripper.max_force_percent==30


def test_docker_selector_is_explicit_and_exposes_its_own_identity():
    from app.digital_twin.docker_runtime import docker_settings
    from app.main import build_backend
    a=build_backend(docker_settings(),NoopRecorder(),backend_label='LEBAI_DOCKER',docker_simulation=True)
    assert a._backend_label=='LEBAI_DOCKER' and a.position_policy=='finite_position_docker'
    assert a.position_vr_enabled and a.position_pitch_enabled


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
    async def factory():connections.append(1);return SimpleNamespace(get_robot_state=state)
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
