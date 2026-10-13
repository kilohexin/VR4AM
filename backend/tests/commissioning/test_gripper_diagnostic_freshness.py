"""One explicit commissioning claw intent gets a current receipt, never a longer TTL."""
import asyncio
from unittest.mock import AsyncMock

import pytest

from app.commissioning import smoke
from app.commissioning.actions import GripperAction, SmokeOptions
from app.config import REAL_ROBOT_CONFIRMATION
from tests.commissioning.test_actions import _control_config
from tests.robots.fake_lebai import FakeLebaiClient


@pytest.mark.parametrize('delay_s', [0, .12])
async def test_single_claw_intent_is_current_but_can_still_expire(tmp_path, monkeypatch, delay_s):
    client = FakeLebaiClient.idle()
    original = smoke.RobotControl
    observed = []

    class ObserveCommand(original):
        async def command_gripper(self, value):
            receipt = self.latest.snapshot()
            assert receipt.frame.right.trigger == value == 1.0
            assert receipt.frame.right.grip
            assert self.clock.now_ns() - receipt.received_ns < 100_000_000
            observed.append(receipt.frame.seq)
            if delay_s:
                await asyncio.sleep(delay_s)
            return await super().command_gripper(value)

    monkeypatch.setattr(smoke, 'RobotControl', ObserveCommand)
    monkeypatch.setattr(smoke, 'COMMISSIONING_LOG_ROOT', tmp_path / 'logs')
    option = SmokeOptions(_control_config(tmp_path), GripperAction('close'), REAL_ROBOT_CONFIRMATION)
    if delay_s:
        with pytest.raises(RuntimeError, match='^gripper_command_authority_changed$'):
            await smoke.run_smoke(option, AsyncMock(return_value=client))
        assert not any(call[0] == 'set_claw' for call in client.write_calls)
    else:
        await smoke.run_smoke(option, AsyncMock(return_value=client))
        assert [call for call in client.write_calls if call[0] == 'set_claw'] == [('set_claw', 30, 0)]
    assert observed == [3]
    assert all(call[0] in ('set_claw', 'stop_move') for call in client.write_calls)
    assert client.network_calls == client.discovery_calls == 0
