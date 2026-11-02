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


async def test_external_docker_motion_gets_a_new_stop_episode_after_confirmed_stop():
    from dataclasses import replace
    from app.robots.base import StopReason
    _control,a,c,clock,_latest=await rig()
    try:
        await a.stop(StopReason.DISCONNECT)
        old=a._stop_transaction
        assert old.confirmed
        # A fresh observed external pose, with no owned motion still pending.
        sample=await a._read_snapshot()
        a._snapshot=replace(sample,actual_q=(sample.actual_q[0]+.1,*sample.actual_q[1:]))
        await a.stop(StopReason.DISCONNECT)
        assert a._stop_transaction is not old and a._stop_transaction.confirmed
        assert [x[0] for x in c.write_calls]==['stop_move','stop_move']
    finally:await a.disconnect()


async def test_unchanged_docker_feedback_reuses_confirmed_stop():
    from app.robots.base import StopReason
    _control,a,c,_clock,_latest=await rig()
    try:
        await a.stop(StopReason.DISCONNECT);old=a._stop_transaction
        await a.stop(StopReason.DISCONNECT)
        assert a._stop_transaction is old
        assert [x[0] for x in c.write_calls]==['stop_move']
    finally:await a.disconnect()


@pytest.mark.parametrize('damage',['unconfirmed','fault','unknown_read','stale','owned_motion'])
async def test_external_motion_never_replaces_unresolved_or_owned_stop(damage):
    from dataclasses import replace
    from app.robots.base import StopReason
    _control,a,c,clock,_latest=await rig()
    try:
        await a.stop(StopReason.DISCONNECT);old=a._stop_transaction
        sample=await a._read_snapshot()
        a._snapshot=replace(sample,actual_q=(sample.actual_q[0]+.1,*sample.actual_q[1:]))
        if damage=='unconfirmed':old.confirmed=False
        if damage=='fault':a._latched_fault='stop_unverified'
        if damage=='unknown_read':a._snapshot_calls.unknown_methods.add('get_tcp')
        if damage=='stale':await clock.sleep(.2)
        if damage=='owned_motion':a._motion_write_stop_required=True
        a._ensure_stop_transaction()
        assert a._stop_transaction is old
    finally:await a.disconnect()


async def test_initial_page_gap_without_authority_or_writes_does_not_send_stop():
    control,adapter,client,clock,latest=await rig()
    try:
        await clock.sleep(.11);await control.state_message();await control.tick()
        assert client.write_calls==[] and control._fault is None
        latest.publish(frame(2,False),clock.now_ns());await control.tick()
        assert control.mode.value=='READY' and control._fault is None
    finally:await adapter.disconnect()


async def test_idle_display_budget_never_changes_motion_or_stop_deadlines():
    control,a,c,clock,_latest=await rig()
    try:
        assert control.idle_display_period_s()==.2
        assert a._snapshot_read_timeout_ns('state')==1_000_000_000
        for purpose in ('prepare','preflight','position_progress','stop_verification'):
            assert a._snapshot_read_timeout_ns(purpose)==300_000_000
        await control.arm()
        assert control.idle_display_period_s()==0
        a._client.move_entries=1
        assert a._snapshot_read_timeout_ns('state')==300_000_000
    finally:await a.disconnect()


async def test_slow_idle_display_finishes_but_cannot_supply_fresh_motion_feedback():
    import asyncio,time
    from app.robots.base import BackendCommandError
    _control,a,c,_clock,_latest=await rig();original=c.get_tcp
    a._clock=time.perf_counter_ns
    async def delayed():await asyncio.sleep(.4);return await original()
    c.get_tcp=delayed
    try:
        state=await a.get_state()
        assert time.perf_counter_ns()-state.server_mono_ns>=350_000_000
        assert not a._snapshot_calls.unknown_methods and not c.write_calls
        with pytest.raises(BackendCommandError,match='snapshot_expired'):a._web_fresh_feedback()
    finally:await a.disconnect()


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
