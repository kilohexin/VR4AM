"""Delivery demo uses local model and synthetic motion only."""
import asyncio
import sys

import pytest
from fastapi.testclient import TestClient

from tests.robots.test_lebai_adapter_control import FakeClock


def test_demo_reference_is_quintic_with_consistent_velocity_and_acceleration():
    from app.digital_twin.segmented_demo import SegmentedDemoClient, demo_settings
    clock=FakeClock()
    client=SegmentedDemoClient.idle(demo_settings().lebai,clock=clock.now_ns)
    start=client._q
    async def move():
        goal=[start[0]+.000128,*start[1:]]
        await client.move_pvat(goal,[0.]*6,[0.]*6,.08)
        clock.value=20_000_000
        data=await client.get_kin_data()
        assert data['actual_joint_pose'][0]-start[0]==pytest.approx(.00001325)
        assert data['actual_joint_speed'][0]==pytest.approx(.0016875)
        assert data['actual_joint_acc'][0]==pytest.approx(.1125)
        clock.value=80_000_000
        final=await client.get_kin_data()
        assert final['actual_joint_pose']==goal
        assert final['actual_joint_speed']==final['actual_joint_acc']==[0.]*6
        assert await client.get_robot_state()=='IDLE'
    asyncio.run(move())


async def test_demo_refuses_a_nonzero_start_instead_of_restarting_zero_pva():
    from app.digital_twin.segmented_demo import SegmentedDemoClient,demo_settings
    clock=FakeClock()
    client=SegmentedDemoClient.idle(demo_settings().lebai,clock=clock.now_ns)
    start=client._q
    await client.move_pvat([start[0]+.000128,*start[1:]],[0.]*6,[0.]*6,.08)
    clock.value=20_000_000
    with pytest.raises(ValueError,match='demo_zero_start_required'):
        await client.move_pvat(start,[0.]*6,[0.]*6,.08)
    assert len(client._write_calls)==1


async def test_cancelled_local_read_does_not_claim_an_unknown_native_rpc():
    from app.digital_twin.segmented_demo import SegmentedDemoClient,demo_settings
    from app.robots.segmented_adapter import SegmentedLebaiAdapter
    client=SegmentedDemoClient.idle(demo_settings().lebai)
    async def factory(ip):return client
    adapter=SegmentedLebaiAdapter(demo_settings().lebai,
        settling=demo_settings().segmented_target.settling,client_factory=factory)
    await adapter.connect()
    entered=asyncio.Event()
    async def local_read():
        entered.set()
        await asyncio.Event().wait()
    read=asyncio.create_task(adapter._timed('get_claw',local_read,{}))
    await entered.wait()
    read.cancel()
    with pytest.raises(asyncio.CancelledError):await read
    assert adapter._snapshot_calls.unknown_methods==set()
    await adapter.disconnect()


async def test_demo_fk_and_segment_adapter_use_the_local_model():
    from app.digital_twin.segmented_demo import SegmentedDemoClient,demo_settings
    from app.robots.segmented_adapter import SegmentedLebaiAdapter
    from app.schemas.messages import Pose
    clock=FakeClock()
    settings=demo_settings()
    client=SegmentedDemoClient.idle(settings.lebai,clock=clock.now_ns)
    async def factory(ip):
        assert ip=='digital-twin'
        return client
    adapter=SegmentedLebaiAdapter(settings.lebai,settling=settings.segmented_target.settling,
        client_factory=factory,clock=clock.now_ns,pvat_clock=clock.now_ns,sleep=clock.sleep)
    try:
        await adapter.connect()
        assert (await adapter.preflight()).ready
        adapter.begin_segments()
        anchor=(await adapter.get_state()).actual_tcp
        target=Pose(p=(anchor.p[0]+.0001,*anchor.p[1:]),q=anchor.q)
        admission=await adapter.submit_segment(target,1)
        done=await asyncio.wait_for(admission.completion.wait(),2)
        assert done.sdk_outcome=='returned',done.error
        assert client._write_calls[0][0]=='move_pvat'
        assert client._write_calls[0][2:4]==([0.]*6,[0.]*6)
        assert 'lebai_sdk' not in sys.modules
    finally:
        await adapter.disconnect()


def test_full_app_has_fake_identity_and_serves_compiled_page(tmp_path,monkeypatch):
    from app.digital_twin.segmented_demo import create_segmented_demo_app
    from app.recording.noop import NoopRecorder
    (tmp_path/'index.html').write_text('<html><title>demo fixture</title></html>')
    monkeypatch.setattr('app.main.build_recorder',lambda *a:NoopRecorder())
    app=create_segmented_demo_app(tmp_path)
    with TestClient(app) as client:
        identity=client.get('/demo/identity').json()
        assert identity['runtime']=='LEBAI_FAKE'
        assert identity['target_strategy']=='segmented_experimental'
        assert identity['hardware_verified'] is False
        assert identity['native_sdk_loaded'] is False
        assert client.get('/health').json()['real_robot_enabled'] is False
        assert 'demo fixture' in client.get('/').text


@pytest.mark.parametrize('env',[{'VR4ARM_REAL_ROBOT_CONFIRM':'x'},
    {'VR4ARM_CONFIG':'config/real-robot.local.yaml'},{'VITE_TELEOP_WS_URL':'ws://robot/ws'}])
def test_demo_environment_rejects_live_overrides(env):
    from app.digital_twin.segmented_demo import validate_demo_environment
    with pytest.raises(ValueError,match='unsafe_demo_environment'):
        validate_demo_environment(env)
