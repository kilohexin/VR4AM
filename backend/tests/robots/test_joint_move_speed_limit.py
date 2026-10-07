import asyncio
from dataclasses import replace
from unittest.mock import AsyncMock

import pytest

from app.robots.base import HomeOptions
from app.robots.lebai_adapter import RealLebaiAdapter
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings


class Clock:
    def __init__(self):
        self.now = 0

    def now_ns(self):
        return self.now

    async def sleep(self, seconds):
        self.now += int(seconds * 1e9)
        await asyncio.sleep(0)


@pytest.mark.parametrize('action', ['prepare', 'home'])
@pytest.mark.parametrize(('requested', 'configured', 'expected'), [
    (.5, .15, .15),  # Current default HomeOptions vs saved site profile.
    (.5, .05, .05),  # Low-speed diagnostic profile must also cap joint moves.
    (.03, .15, .03),  # A tighter requested speed must be preserved.
    (.15, .15, .15),
])
async def test_joint_move_speed_respects_both_option_and_profile(action, requested, configured, expected):
    settings = control_settings()
    settings = replace(settings, control=replace(settings.control,
                                                 max_joint_speed_radps=configured))
    client = FakeLebaiClient.idle()
    original_movej = client.movej

    async def converging_movej(p, a, v, t, r):
        receipt = await original_movej(p, a, v, t, r)
        client.kin_data['actual_joint_pose'] = list(p)
        client.kin_data['target_joint_pose'] = list(p)
        return receipt

    client.movej = converging_movej
    clock = Clock()
    factory = AsyncMock(return_value=client)
    adapter = RealLebaiAdapter(settings, client_factory=factory,
                               clock=clock.now_ns, sleep=clock.sleep)
    options = HomeOptions(max_speed_radps=requested, timeout_s=10,
                          position_tolerance_rad=.01, velocity_tolerance_radps=.02,
                          stable_seconds=.3)
    await adapter.connect()
    try:
        await getattr(adapter, action)(options, lambda _phase: None)
        move_calls = [call for call in client.write_calls if call[0] == 'movej']
        assert len(move_calls) == 1
        command = move_calls[0]
        assert command[3] == pytest.approx(expected)
        assert command[2] == settings.control.max_joint_acceleration_radps2
        target = settings.teleop_ready_q if action == 'prepare' else settings.home_q
        assert command[1] == list(target)
        assert command[4:] == (0., 0.)
        assert factory.await_count == 1
        assert not any(call[0] in ('move_pvat', 'set_claw', 'stop_sys') for call in client.write_calls)
    finally:
        await adapter.disconnect()
