import asyncio
from dataclasses import replace
from unittest.mock import AsyncMock

import pytest

from app.robots.base import BackendCommandError, HomeOptions
from app.robots.lebai_adapter import RealLebaiAdapter
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_joint_move_speed_limit import Clock


def settings():
    result = control_settings()
    return replace(result, control=replace(result.control, max_joint_speed_radps=.05,
                                           max_joint_acceleration_radps2=.2))


@pytest.mark.parametrize('action', ['prepare', 'home'])
@pytest.mark.parametrize('outcome', ['returned', 'timeout', 'error', 'cancelled'])
async def test_movej_records_capped_arguments_and_rpc_outcome_without_changing_stop(action, outcome):
    client = FakeLebaiClient.idle()
    clock = Clock()
    events = []

    async def record(event, _timestamp):
        events.append(dict(event))

    original = client.movej
    async def movej(p, a, v, t, r):
        result = await original(p, a, v, t, r)
        if outcome == 'timeout':
            await asyncio.Event().wait()
        if outcome == 'error':
            raise RuntimeError('private controller message must not be logged')
        if outcome == 'cancelled':
            raise asyncio.CancelledError
        client.kin_data['actual_joint_pose'] = list(p)
        client.kin_data['target_joint_pose'] = list(p)
        return result
    client.movej = movej
    adapter = RealLebaiAdapter(settings(), client_factory=AsyncMock(return_value=client),
                               clock=clock.now_ns, sleep=clock.sleep, event_callback=record)
    await adapter.connect()
    options = HomeOptions(max_speed_radps=.5, timeout_s=10, position_tolerance_rad=.01,
                          velocity_tolerance_radps=.02, stable_seconds=.3)
    try:
        call = getattr(adapter, action)(options, lambda _: None)
        if outcome == 'returned':
            await call
        elif outcome == 'cancelled':
            with pytest.raises(asyncio.CancelledError):
                await call
        else:
            with pytest.raises(BackendCommandError, match='sdk_timeout:movej' if outcome == 'timeout' else 'sdk_call_failed:movej'):
                await call
    finally:
        await adapter.disconnect()
    lifecycle = [e for e in events if e['kind'] == 'sdk_request_lifecycle' and e.get('method') == 'movej']
    assert len(lifecycle) == 1
    event = lifecycle[0]
    assert event['rpc_outcome'] == outcome
    assert event['v'] == .05
    assert event['a'] == .2
    assert event['t'] == 0
    assert event['r'] == 0
    assert event['p'] == client.write_calls[0][1]
    assert event['completed_ns'] >= event['started_ns']
    assert event['request_id'] > 0
    assert event['evidence_scope'] == 'local_sdk_call_not_remote_execution'
    assert event.get('error_type') == ('RuntimeError' if outcome == 'error' else None)
    assert 'private controller' not in str(event)
    assert [c[0] for c in client.write_calls] == (['movej'] if outcome == 'returned' else ['movej', 'stop_move'])


async def test_blocked_movej_diagnostic_does_not_delay_error_stop():
    client = FakeLebaiClient.idle()
    clock = Clock()
    release = asyncio.Event()
    entered = asyncio.Event()
    async def record(event, _timestamp):
        if event['kind'] == 'sdk_request_lifecycle':
            entered.set()
            try:
                await release.wait()
            except asyncio.CancelledError:
                await release.wait()

    original = client.movej
    async def failed(*args):
        await original(*args)
        raise RuntimeError('movej failed')
    client.movej = failed
    adapter = RealLebaiAdapter(settings(), client_factory=AsyncMock(return_value=client),
                               clock=clock.now_ns, sleep=clock.sleep, event_callback=record)
    await adapter.connect()
    try:
        with pytest.raises(BackendCommandError, match='sdk_call_failed:movej'):
            await asyncio.wait_for(adapter.prepare(HomeOptions(.5, 10, .01, .02, .3), lambda _: None), .4)
        await asyncio.wait_for(entered.wait(), .1)
        assert [c[0] for c in client.write_calls] == ['movej', 'stop_move']
    finally:
        release.set()
        await adapter.disconnect()
