"""Actual scheduled claw entry must retain adapter fences; fake SDK, no network."""
import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _wait_until


@pytest.mark.parametrize('bound', [False, True])
@pytest.mark.parametrize('change,expected', [
    ('expired', 'robot_state_stale'),
    ('generation', 'preflight_not_ready'),
    ('preflight', 'preflight_not_ready'),
    ('client', 'robot_disconnected'),
    ('stop', 'preflight_not_ready'),
])
async def test_claw_scheduled_task_rechecks_adapter_fences(monkeypatch, bound, change, expected):
    adapter, client, clock = await _connected_control_adapter()
    if bound:
        adapter.bind_gripper_authority(lambda: True)
    original_wait_for = asyncio.wait_for
    intercepted = False
    stopping = None

    async def change_before_entry(awaitable, timeout):
        nonlocal intercepted, stopping
        code = getattr(awaitable, 'cr_code', None)
        if timeout == .20 and getattr(code, 'co_name', None) == 'send_if_authorized' and not intercepted:
            intercepted = True
            if change == 'expired':
                clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000 + 1)
            elif change == 'generation':
                adapter._pump.invalidate()
            elif change == 'preflight':
                adapter._preflight_ready = False
            elif change == 'client':
                adapter._client = None
            else:
                old = adapter._gripper_revocation_generation
                stopping = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
                await original_wait_for(_wait_until(lambda: adapter._gripper_revocation_generation > old), 1)
        return await original_wait_for(awaitable, timeout)

    monkeypatch.setattr(asyncio, 'wait_for', change_before_entry)
    try:
        result = (await asyncio.gather(adapter.set_gripper(.5), return_exceptions=True))[0]
        assert intercepted
        assert not any(call[0] == 'set_claw' for call in client.write_calls)
        assert isinstance(result, BackendCommandError)
        assert str(result) == expected
        if stopping is not None:
            await original_wait_for(stopping, 2)
            assert client.write_calls == [('stop_move',)]
    finally:
        adapter._client = client
        if stopping is not None:
            if not stopping.done():
                stopping.cancel()
            await asyncio.gather(stopping, return_exceptions=True)
        await adapter.disconnect()
        assert client.network_calls == client.discovery_calls == 0


@pytest.mark.parametrize('advance_ms', [0, 1])
async def test_claw_scheduled_healthy_entry_has_no_extra_reads(monkeypatch, advance_ms):
    adapter, client, clock = await _connected_control_adapter()
    original_wait_for = asyncio.wait_for
    intercepted = False

    async def advance_before_entry(awaitable, timeout):
        nonlocal intercepted
        if timeout == .20 and getattr(getattr(awaitable, 'cr_code', None), 'co_name', None) == 'send_if_authorized':
            intercepted = True
            clock.advance_ms(advance_ms)
        return await original_wait_for(awaitable, timeout)

    monkeypatch.setattr(asyncio, 'wait_for', advance_before_entry)
    client.read_calls.clear()
    try:
        await adapter.set_gripper(.5)
        assert intercepted
        assert client.write_calls == [('set_claw', 30, 50)]
        assert client.read_calls == ['is_connected', 'get_robot_state', 'get_estop_reason',
            'get_kin_data', 'get_tcp', 'get_claw', 'get_running_motion']
    finally:
        await adapter.disconnect()


async def test_native_claw_error_keeps_sdk_failure_classification():
    adapter, client, _ = await _connected_control_adapter()

    async def fail(_force, _amplitude):
        raise BackendCommandError('native synthetic failure')

    client.set_claw = fail
    try:
        with pytest.raises(BackendCommandError, match='^sdk_call_failed:set_claw$'):
            await adapter.set_gripper(.5)
    finally:
        await adapter.disconnect()
