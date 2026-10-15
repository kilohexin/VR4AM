"""Local model preparation; no SDK import, network or hardware action."""
import asyncio
from dataclasses import replace
from unittest.mock import AsyncMock

import numpy as np
import pytest

from app.robots.base import BackendCommandError, HomeOptions
from app.robots.segmented_adapter import SegmentedLebaiAdapter
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_joint_move_speed_limit import Clock


OPTIONS = HomeOptions(.25, 15., .01, .02, .3)


async def test_segment_prepare_requires_explicit_selection_and_still_denies_home():
    from app.control.transactional_settings import PursuitSettings
    for enabled in (False, True):
        clock, client = Clock(), FakeLebaiClient.idle()
        original = client.movej
        async def converge(p,a,v,t,r):
            receipt = await original(p,a,v,t,r)
            client.kin_data['actual_joint_pose'] = list(p)
            return receipt
        client.movej = converge
        adapter = SegmentedLebaiAdapter(control_settings(),
            settling=PursuitSettings(500,60,60,.001,.002,3), allow_prepare=enabled,
            client_factory=AsyncMock(return_value=client),clock=clock.now_ns,sleep=clock.sleep)
        await adapter.connect()
        try:
            with pytest.raises(BackendCommandError, match='not_supported'):
                await adapter.home(OPTIONS, lambda _: None)
            if not enabled:
                with pytest.raises(BackendCommandError, match='not_supported'):
                    await adapter.prepare(OPTIONS, lambda _: None)
                assert client.write_calls == []
            else:
                phases=[]
                await adapter.prepare(OPTIONS, phases.append)
                assert [c[0] for c in client.write_calls] == ['movej']
                assert client.write_calls[0][1] == list(adapter.settings.teleop_ready_q)
                assert phases[0] == 'homing' and phases[-1] == 'stabilizing'
                assert clock.now_ns() >= 300_000_000
                with pytest.raises(BackendCommandError, match='finite_sdk_entry_not_admitted'):
                    adapter._begin_motion_epoch()
        finally:
            await adapter.disconnect()


async def test_fake_ready_move_is_visible_quintic_and_only_configured_posture():
    from app.digital_twin.segmented_demo import SegmentedDemoClient, demo_settings
    settings = demo_settings()
    assert settings.segmented_target.prepare_on_arm
    assert settings.lebai.teleop_ready_q != settings.lebai.home_q
    clock = Clock()
    client = SegmentedDemoClient.idle(settings.lebai, clock=clock.now_ns)
    start = np.array(client._q)
    goal = np.array(settings.lebai.teleop_ready_q)
    with pytest.raises(ValueError, match='demo_ready_pose_required'):
        await client.movej(list(start), 1., .25, 0., 0.)
    await client.movej(list(goal), 1., .25, 0., 0.)
    duration = client._motion.duration_ns
    assert duration >= 2_000_000_000
    clock.now = duration // 2
    middle = await client.get_kin_data()
    assert np.allclose(middle['actual_joint_pose'], (start + goal) / 2)
    assert max(abs(v) for v in middle['actual_joint_speed']) <= .25
    assert np.linalg.norm(np.array(middle['actual_joint_pose']) - start) > .1
    clock.now = duration
    final = await client.get_kin_data()
    assert np.allclose(final['actual_joint_pose'], goal)
    assert final['actual_joint_speed'] == final['actual_joint_acc'] == [0.] * 6
    assert [c[0] for c in client._write_calls] == ['movej']

async def test_fake_path_validation_yields_and_stop_prevents_a_late_move(monkeypatch):
    from app.digital_twin.segmented_demo import SegmentedDemoClient,demo_settings
    settings=demo_settings();client=SegmentedDemoClient.idle(settings.lebai)
    entered,finish=asyncio.Event(),asyncio.Event()
    async def check(*args,**kwargs):entered.set();await finish.wait()
    monkeypatch.setattr(asyncio,'to_thread',check)
    task=asyncio.create_task(client.movej(list(settings.lebai.teleop_ready_q),1.,.25,0.,0.))
    try:
        await asyncio.wait_for(entered.wait(),.5)
        await client.stop_move();finish.set()
        with pytest.raises(ValueError,match='demo_ready_revoked'):await task
        assert not any(call[0]=='movej' for call in client._write_calls)
    finally:
        finish.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True)
