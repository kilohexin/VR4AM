import asyncio

import pytest

from app.commissioning.static_system_stop import CONFIRM, run_probe
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings, readonly_settings
from tests.robots.test_lebai_adapter_control import FakeClock


async def run(client, **kwargs):
    clock = FakeClock()
    events = []
    result = await run_probe(client, kwargs.pop('settings', control_settings()),
                             confirm=kwargs.pop('confirm', CONFIRM), emit=events.append,
                             clock=clock.now_ns, sleep=clock.sleep, **kwargs)
    return result, events


async def test_single_system_stop_only_after_stable_baseline():
    c = FakeLebaiClient.idle()
    async def stop():
        c.write_calls.append(('stop_sys',))
        c.robot_state = 'STOP'
    c.stop_sys = stop
    result, events = await run(c)
    assert result['complete']
    assert c.write_calls == [('stop_sys',)]
    before = [e for e in events if e['kind'] == 'baseline_sample']
    assert before[-1]['sample']['completed_ns'] - before[0]['sample']['completed_ns'] >= 300_000_000
    assert len([e for e in events if e['kind'] == 'observation_sample']) > 100


@pytest.mark.parametrize('bad', ['HOLD', 'MOVING', 'estop', 'speed', 'limits', 'tcp', 'id', 'target', 'nan'])
async def test_invalid_baseline_never_writes(bad):
    c = FakeLebaiClient.idle()
    if bad in {'HOLD', 'MOVING'}:
        c.robot_state = 'STOP' if bad == 'HOLD' else bad
    elif bad == 'estop': c.estop_reason = 1
    elif bad == 'speed': c.kin_data['actual_joint_speed'][0] = .1
    elif bad == 'limits': c.kin_data['actual_joint_pose'][0] = 4
    elif bad == 'tcp': c.tcp['z'] = 1
    elif bad == 'id': c.running_motion, c.motion_state = 111, 'RUNNING'
    elif bad == 'target': c.kin_data['target_joint_pose'][0] += .1
    elif bad == 'nan': c.kin_data['actual_joint_pose'][0] = float('nan')
    result, _ = await run(c)
    assert not result['complete']
    assert c.write_calls == []


@pytest.mark.parametrize('options', [{'confirm': 'yes'}, {'settings': readonly_settings()}])
async def test_explicit_control_and_confirmation_required(options):
    c = FakeLebaiClient.idle()
    with pytest.raises(ValueError):
        await run(c, **options)
    assert c.read_calls == c.write_calls == []


async def test_system_stop_error_never_retries_or_adds_other_write():
    c = FakeLebaiClient.idle()
    async def fail():
        c.write_calls.append(('stop_sys',))
        raise RuntimeError('failed')
    c.stop_sys = fail
    result, _ = await run(c)
    assert not result['complete']
    assert c.write_calls == [('stop_sys',)]


async def test_pending_system_stop_does_not_resend():
    c = FakeLebaiClient.idle()
    async def hold():
        c.write_calls.append(('stop_sys',))
        await asyncio.Event().wait()
    c.stop_sys = hold
    result, events = await run(c)
    assert not result['complete']
    assert c.write_calls == [('stop_sys',)]
    assert any(e['kind'] == 'rpc_pending' for e in events)


async def test_position_change_remains_failed_without_recovery_write():
    c = FakeLebaiClient.idle()
    original = c.get_kin_data
    async def read():
        if c.write_calls:
            c.kin_data['actual_tcp_pose']['x'] = .31
        return await original()
    c.get_kin_data = read
    result, _ = await run(c)
    assert not result['complete']
    assert result['max_tcp_drift_m'] >= .009
    assert c.write_calls == [('stop_sys',)]


async def test_changed_config_at_dispatch_never_writes():
    c = FakeLebaiClient.idle()
    calls = 0
    def check():
        nonlocal calls
        calls += 1
        if calls == 2:
            raise ValueError('config_changed')
    result, _ = await run(c, before_write=check)
    assert not result['complete']
    assert c.write_calls == []


async def test_transient_fault_is_not_erased_by_final_stop():
    c = FakeLebaiClient.idle()
    original = c.get_robot_state
    reads = 0
    async def read():
        nonlocal reads
        if c.write_calls:
            reads += 1
            return 'ERROR' if reads == 3 else 'STOP'
        return await original()
    c.get_robot_state = read
    result, _ = await run(c)
    assert not result['complete']
    assert result['final_sample']['raw_state'] == 'STOP'
    assert c.write_calls == [('stop_sys',)]


async def test_target_speed_blocks_write():
    c = FakeLebaiClient.idle()
    c.kin_data['target_joint_speed'][0] = .1
    result, _ = await run(c)
    assert not result['complete']
    assert c.write_calls == []


@pytest.mark.parametrize('field', ['target_joint_pose', 'actual_joint_torque', 'target_joint_torque'])
async def test_missing_observation_diagnostic_channel_fails(field):
    c = FakeLebaiClient.idle()
    original = c.get_kin_data
    async def read():
        data = await original()
        if c.write_calls:
            data.pop(field)
        return data
    c.get_kin_data = read
    result, _ = await run(c)
    assert not result['complete']
    assert result['error_type'] == 'KeyError'
    assert c.write_calls == [('stop_sys',)]


async def test_gap_to_first_observation_fails_without_second_write():
    c = FakeLebaiClient.idle()
    clock = FakeClock()
    original = c.get_robot_state
    delayed = False
    async def read():
        nonlocal delayed
        if c.write_calls and not delayed:
            delayed = True
            await clock.sleep(.25)
        return await original()
    c.get_robot_state = read
    async def stop():
        c.write_calls.append(('stop_sys',))
        await clock.sleep(.1)
        c.robot_state = 'STOP'
    c.stop_sys = stop
    events = []
    result = await run_probe(c, control_settings(), confirm=CONFIRM,
                             emit=events.append, clock=clock.now_ns, sleep=clock.sleep)
    assert not result['complete']
    assert result['observation_completed']
    assert c.write_calls == [('stop_sys',)]


async def test_late_return_between_sampling_ticks_is_still_failure():
    c = FakeLebaiClient.idle()
    clock = FakeClock()
    async def stop():
        c.write_calls.append(('stop_sys',))
        await clock.sleep(5.05)
        c.robot_state = 'STOP'
    c.stop_sys = stop
    result = await run_probe(c, control_settings(), confirm=CONFIRM,
                             emit=lambda event: None, clock=clock.now_ns, sleep=clock.sleep)
    assert not result['complete']
    assert result['rpc_outcome'] == 'returned'
    assert result['rpc_wait_exceeded']
    assert result['rpc_elapsed_ns'] >= 5_000_000_000
