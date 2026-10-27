"""Before authority or writes, stale UI input only locks the Docker UI locally."""
import pytest
from app.control.robot_control import RobotControl,LatestVRFrame
from app.recording.noop import NoopRecorder
from app.robots.finite_position_docker import DockerFinitePositionAdapter
from app.digital_twin.docker_runtime import docker_settings
from tests.robots.finite_position_fixture import PositionClient
from tests.robots.test_lebai_adapter_control import FakeClock
from tests.control.test_robot_control import frame


async def rig():
    clock=FakeClock();client=PositionClient(clock);client.tcp=dict(x=0.,y=0.,z=0.,rx=0.,ry=0.,rz=0.)
    async def factory(ip):assert ip=='127.0.0.1';return client
    adapter=DockerFinitePositionAdapter(docker_settings().lebai,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    await adapter.connect();latest=LatestVRFrame()
    control=RobotControl(backend=adapter,latest=latest,clock=clock,recorder=NoopRecorder(),
        prepare_on_arm=True,explicit_preparation=True)
    await control.connect();latest.publish(frame(1,False),clock.now_ns());await control.tick()
    return control,adapter,client,clock,latest


async def test_initial_page_gap_without_authority_or_writes_does_not_send_stop():
    control,adapter,client,clock,latest=await rig()
    try:
        await clock.sleep(.11);await control.state_message();await control.tick()
        assert client.write_calls==[] and control._fault is None
        latest.publish(frame(2,False),clock.now_ns());await control.tick()
        assert control.mode.value=='READY' and control._fault is None
    finally:await adapter.disconnect()


async def test_old_idle_snapshot_cannot_create_stop_obligation_before_any_actuation():
    control,adapter,client,clock,_latest=await rig()
    try:
        await clock.sleep(.5);await control.tick()
        assert client.write_calls==[] and control.mode.value=='DISARMED'
        assert control._fault is None
    finally:await adapter.disconnect()


@pytest.mark.parametrize('damage',['unlock','prior_write','disabled'])
async def test_idle_skip_is_unavailable_after_authority_or_unproven_state(damage):
    control,adapter,client,clock,_latest=await rig()
    try:
        if damage=='unlock':await control.arm()
        if damage=='prior_write':adapter._client.move_entries=1
        if damage=='disabled':adapter.position_idle_input_guard=False
        await clock.sleep(.11)
        if damage!='old_snapshot':await control.state_message()
        await control.tick()
        assert client.write_calls or control._fault is not None
    finally:await adapter.disconnect()


async def test_docker_stop_reads_all_six_fields_concurrently_without_dropping_fields():
    import asyncio
    _control,adapter,client,_clock,_latest=await rig();entered=asyncio.Event();release=asyncio.Event()
    original=client.get_robot_state
    async def blocked():entered.set();await release.wait();return await original()
    client.get_robot_state=blocked;client.read_calls.clear()
    reader=asyncio.create_task(adapter._read_snapshot(read_purpose='stop_verification'))
    try:
        await entered.wait()
        for _ in range(15):await asyncio.sleep(0)
        assert {'get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion'}<=set(client.read_calls)
        release.set();sample=await reader
        assert sample.actual_qd==(0.,)*6 and sample.robot_state.value=='IDLE'
    finally:release.set();await asyncio.gather(reader,return_exceptions=True);await adapter.disconnect()


async def test_progress_observes_new_feedback_after_slow_task_status():
    _control,adapter,client,clock,_latest=await rig();adapter.bind_position_generation(1)
    async def slow_status(_motion_id):await clock.sleep(.12);return 'RUNNING'
    client.get_motion_state=slow_status
    try:
        sample,state=await adapter._position_progress(1,lambda:None)
        assert state=='RUNNING' and 0<=clock.now_ns()-sample.captured_ns<100_000_000
    finally:await adapter.disconnect()


@pytest.mark.parametrize('retained',[False,True])
async def test_slow_finished_status_still_completes_with_fresh_stable_feedback(retained):
    from dataclasses import replace
    from app.control.finite_position_types import PositionIntent
    from tests.robots.test_finite_position_adapter import execute
    from tests.robots.finite_position_fixture import fixture
    settings,client,clock=fixture()
    settings=replace(settings,ip='127.0.0.1',control=replace(settings.control,
        max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    adapter=DockerFinitePositionAdapter(settings,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    await adapter.connect()
    # The injected analytical fixture isolates RPC scheduling, not native geometry.
    adapter.position_model=client.model;adapter.bind_position_generation(1)
    plan=await adapter.plan_step(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)),authorize=lambda:None)
    original=client.advance;calls=[]
    def advance():
        original()
        if retained and client.start_ns is not None and client.robot_state=='IDLE':client.running_motion=7
    client.advance=advance
    async def slow_status(motion_id):
        calls.append(motion_id);await clock.sleep(.12);client.advance();return client.motion_state
    client.get_motion_state=slow_status
    original_kin=client.get_kin_data
    async def slow_kin():await clock.sleep(.08);return await original_kin()
    client.get_kin_data=slow_kin
    try:
        result=await execute(adapter,plan)
        assert result.complete,result
        assert [c[0] for c in client.write_calls]==['movej']
        assert len(calls)<=3
    finally:await adapter.disconnect()
