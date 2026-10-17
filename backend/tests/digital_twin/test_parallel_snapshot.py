"""Offline six-field scheduling; no robot/network factory."""
import asyncio
from dataclasses import replace
from unittest.mock import AsyncMock
from types import SimpleNamespace
import pytest
from app.robots.base import BackendCommandError, StopReason
from app.robots.segmented_adapter import SegmentedLebaiAdapter
from tests.control.test_segmented_control import OPTIONS
from tests.commissioning.test_finite_diagnostic import AuditClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock, _target, _wait_until

FIELDS=('get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion')

async def rig():
    client=AuditClient();clock=FakeClock()
    settings=replace(control_settings(),ip='digital-twin')
    adapter=SegmentedLebaiAdapter(settings,settling=OPTIONS,client_factory=AsyncMock(return_value=client),
        backend_label='LEBAI_FAKE',offline_parallel_snapshot_reads=True,parallel_fk=True,
        clock=clock.now_ns,pvat_clock=clock.now_ns,sleep=clock.sleep)
    await adapter.connect();assert (await adapter.preflight()).ready
    return adapter,client,clock

def block_fields(client):
    entered=[];all_started=asyncio.Event();release=asyncio.Event()
    for name in FIELDS:
        original=getattr(client,name)
        async def query(name=name,original=original):
            entered.append(name)
            if len(entered)==6:all_started.set()
            await release.wait()
            return await original()
        setattr(client,name,query)
    return entered,all_started,release

async def test_state_starts_all_fields_before_install_and_keeps_connection_first():
    adapter,client,_=await rig();before=adapter._snapshot
    entered,ready,release=block_fields(client);client.read_calls.clear()
    task=asyncio.create_task(adapter._read_snapshot(read_purpose='state'))
    try:
        await asyncio.wait_for(ready.wait(),1)
        assert set(entered)==set(FIELDS) and adapter._snapshot is before
        assert client.read_calls==['is_connected'] and not client.write_calls
        release.set();snapshot=await task
        assert snapshot is adapter._snapshot and snapshot.read_purpose=='state'
        assert set(client.read_calls)=={'is_connected',*FIELDS}
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True);await adapter.disconnect()

async def test_motion_uses_the_same_six_owned_reads_and_original_four_fk():
    adapter,client,_=await rig();adapter.begin_segments()
    entered,ready,release=block_fields(client)
    receipt=await adapter.submit_segment(_target(.3001),1)
    try:
        await asyncio.wait_for(ready.wait(),1)
        assert not client.ik_calls and not client.write_calls
        release.set();done=await asyncio.wait_for(receipt.completion.wait(),2)
        assert done.sdk_outcome=='returned' and len(client.fk_calls)==4
        assert len([c for c in client.write_calls if c[0]=='move_pvat'])==1
    finally:release.set();await adapter.disconnect()

@pytest.mark.parametrize('purpose',['stop_verification','preflight','prepare','home','connection'])
async def test_other_purposes_keep_sequential_reads(purpose):
    adapter,client,_=await rig();entered,ready,release=block_fields(client)
    task=asyncio.create_task(adapter._read_snapshot(read_purpose=purpose))
    try:
        await _wait_until(lambda:bool(entered))
        for _ in range(10):await asyncio.sleep(0)
        assert entered==[FIELDS[0]] and not ready.is_set()
        release.set();await task
        assert entered==list(FIELDS)
    finally:release.set();await asyncio.gather(task,return_exceptions=True);await adapter.disconnect()

@pytest.mark.parametrize('method',FIELDS)
async def test_any_failed_field_never_installs_partial_state(method):
    adapter,client,_=await rig();before=adapter._snapshot
    async def fail():raise RuntimeError('injected')
    setattr(client,method,fail)
    try:
        with pytest.raises(BackendCommandError):await adapter._read_snapshot(read_purpose='state')
        assert adapter._snapshot is before and not client.write_calls
    finally:await adapter.disconnect()

async def test_cancelled_group_retains_stubborn_read_owner_and_never_installs_late_state():
    adapter,client,_=await rig();before=adapter._snapshot;release=asyncio.Event();started=asyncio.Event()
    original=client.get_claw
    async def stubborn():
        started.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return await original()
    client.get_claw=stubborn
    task=asyncio.create_task(adapter._read_snapshot(read_purpose='state'))
    try:
        await asyncio.wait_for(started.wait(),1);task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        assert adapter._snapshot is before
        assert 'get_claw' in adapter._snapshot_calls.unknown_methods
        assert any(not t.done() for t in adapter._snapshot_calls.tasks)
        release.set();await _wait_until(lambda:not adapter._snapshot_calls.tasks)
        assert adapter._snapshot is before
        with pytest.raises(BackendCommandError):await adapter._read_snapshot(read_purpose='state')
        assert not client.write_calls
    finally:release.set();await adapter.disconnect()

@pytest.mark.parametrize('change',['closed','expired'])
async def test_changed_owner_or_expired_read_cannot_install_a_completed_group(change):
    adapter,client,clock=await rig();before=adapter._snapshot
    entered,ready,release=block_fields(client)
    task=asyncio.create_task(adapter._read_snapshot(read_purpose='state'))
    try:
        await asyncio.wait_for(ready.wait(),1)
        if change=='closed':adapter._snapshot_calls.close()
        if change=='expired':clock.value+=301_000_000
        release.set()
        with pytest.raises(BackendCommandError):await task
        assert adapter._snapshot is before and not client.write_calls
    finally:release.set();await asyncio.gather(task,return_exceptions=True);await adapter.disconnect()

async def test_normal_stop_during_state_read_keeps_session_feedback_but_never_relabels_motion_evidence():
    adapter,client,_=await rig();old_generation=adapter._pump.generation
    old_observation=adapter._motion_observation
    _,ready,release=block_fields(client)
    read=asyncio.create_task(adapter._read_snapshot(read_purpose='state'))
    stop=None
    try:
        await asyncio.wait_for(ready.wait(),1)
        stop=asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
        await _wait_until(lambda:adapter._pump.generation!=old_generation)
        release.set();snapshot=await read
        # Ordinary feedback can finish across a stop, as before the candidate.
        # It must not be relabelled as the new epoch's motion observation.
        assert snapshot.read_purpose=='state'
        # Stop may already have installed its own fresh verification sample;
        # the old state sample itself cannot have the new generation.
        observation=adapter._motion_observation
        assert observation is old_observation or observation.read_purpose=='stop_verification'
        await stop
        await adapter._read_snapshot(read_purpose='state')
        assert adapter._motion_observation.generation==adapter._pump.generation
    finally:
        release.set();await asyncio.gather(read,*([stop] if stop else []),return_exceptions=True)
        await adapter.disconnect()

async def test_state_sender_continues_after_normal_stop_overlaps_parallel_feedback():
    from app.api.teleop_ws import state_sender
    adapter,client,_=await rig();generation=adapter._pump.generation
    _,ready,release=block_fields(client);sent=[]
    async def state():
        sample=await adapter._read_snapshot(read_purpose='state')
        return SimpleNamespace(model_dump=lambda **kw:{'state':sample.robot_state})
    async def send(payload):
        sent.append(payload)
        if len(sent)==2:raise asyncio.CancelledError()
    sender=asyncio.create_task(state_sender(SimpleNamespace(send_json=send),SimpleNamespace(state_message=state)))
    stop=None
    try:
        await asyncio.wait_for(ready.wait(),1)
        stop=asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
        await _wait_until(lambda:adapter._pump.generation!=generation)
        release.set()
        with pytest.raises(asyncio.CancelledError):await asyncio.wait_for(sender,1)
        assert len(sent)==2
        await stop
    finally:
        release.set();sender.cancel()
        await asyncio.gather(sender,*([stop] if stop else []),return_exceptions=True)
        await adapter.disconnect()

async def test_retained_motion_state_query_waits_for_all_six_fields():
    adapter,client,_=await rig();client.running_motion=123
    _,ready,release=block_fields(client)
    task=asyncio.create_task(adapter._read_snapshot(read_purpose='state'))
    try:
        await asyncio.wait_for(ready.wait(),1)
        assert 'get_motion_state' not in client.read_calls
        release.set();snapshot=await task
        assert snapshot.raw_running_motion==123 and snapshot.running_motion is None
        assert snapshot.motion_state=='FINISHED'
        assert client.read_calls[-1]=='get_motion_state'
    finally:release.set();await asyncio.gather(task,return_exceptions=True);await adapter.disconnect()

@pytest.mark.parametrize('method,payload',[('get_kin_data',{}),('get_tcp',{}),('get_claw',{})])
async def test_returned_malformed_fields_keep_original_decode_rejection(method,payload):
    adapter,client,_=await rig();before=adapter._snapshot
    async def malformed():return payload
    setattr(client,method,malformed)
    try:
        with pytest.raises(BackendCommandError):await adapter._read_snapshot(read_purpose='state')
        assert adapter._snapshot is before and not client.write_calls
    finally:await adapter.disconnect()

@pytest.mark.parametrize('runtime,ip', [('LEBAI','digital-twin'),('LEBAI_FAKE','192.0.2.1')])
def test_direct_adapter_cannot_enable_candidate_for_nonlocal_identity(runtime,ip):
    with pytest.raises(ValueError,match='offline_parallel_snapshot_reads_offline_only'):
        SegmentedLebaiAdapter(replace(control_settings(),ip=ip),settling=OPTIONS,
            client_factory=None,backend_label=runtime,offline_parallel_snapshot_reads=True)

@pytest.mark.parametrize('bad',[None,1,'true'])
def test_private_app_option_rejects_non_boolean(bad):
    from app.main import build_backend
    from app.digital_twin.segmented_demo import demo_settings
    from app.recording.noop import NoopRecorder
    with pytest.raises(ValueError,match='invalid_offline_parallel_snapshot_reads'):
        build_backend(demo_settings(),NoopRecorder(),None,'LEBAI_FAKE',offline_parallel_snapshot_reads=bad)

@pytest.mark.parametrize('case',['real','simulator','missing_policy','nonlocal'])
def test_app_flag_rejects_nonlocal_or_nonsegment_configuration_before_factory(case):
    from app.main import build_backend
    from app.digital_twin.segmented_demo import demo_settings
    from app.recording.noop import NoopRecorder
    settings=demo_settings();runtime='LEBAI_FAKE';factory=AsyncMock()
    if case=='real':runtime='LEBAI'
    if case=='simulator':settings=replace(settings,backend='simulator')
    if case=='missing_policy':settings=replace(settings,segmented_target=None)
    if case=='nonlocal':settings=replace(settings,lebai=replace(settings.lebai,ip='192.0.2.1'))
    with pytest.raises(ValueError,match='offline_parallel_snapshot_reads_offline_only'):
        build_backend(settings,NoopRecorder(),factory,runtime,offline_parallel_snapshot_reads=True)
    factory.assert_not_called()

@pytest.mark.parametrize('runtime',['LEBAI','LEBAI_FAKE'])
def test_ordinary_app_remains_sequential_by_default(runtime):
    from app.main import build_backend
    from app.digital_twin.segmented_demo import demo_settings
    from app.recording.noop import NoopRecorder
    adapter=build_backend(demo_settings(),NoopRecorder(),None,runtime)
    assert not adapter.offline_parallel_snapshot_reads and not adapter.parallel_motion_reads
