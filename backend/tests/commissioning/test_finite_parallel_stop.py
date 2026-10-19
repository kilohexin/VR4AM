"""Injected Fake clients only; no transport or physical stopping claim."""
import asyncio
from dataclasses import replace

import pytest

from app.commissioning.finite_window_run import run_window
from tests.commissioning.test_finite_pause_start_period import settings_for
from tests.commissioning.test_finite_pause_window import PauseZClient
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.test_lebai_adapter_control import FakeClock

FIELDS = ('get_robot_state', 'get_estop_reason', 'get_kin_data',
          'get_tcp', 'get_claw', 'get_running_motion')


class ConcurrentClient(PauseZClient):
    def __init__(self, clock, *, mode='finished', fault=None):
        super().__init__(clock, mode=mode)
        self.active = self.peak = 0
        self.calls = []
        self.fault = fault
        self.release = asyncio.Event()
        self.entered = asyncio.Event()
        for name in FIELDS:
            original = getattr(self, name)
            async def wrapped(name=name, original=original):
                if self.pause_ns is None:
                    return await original()
                self.calls.append(('start', name))
                self.active += 1
                self.peak = max(self.peak, self.active)
                try:
                    # Cooperative overlap is deterministic; no timing threshold.
                    await asyncio.sleep(0)
                    if name == 'get_kin_data' and self.fault == 'pending':
                        self.entered.set()
                        while not self.release.is_set():
                            try:
                                await self.release.wait()
                            except asyncio.CancelledError:
                                pass
                    if name == 'get_claw' and self.fault == 'error':
                        raise RuntimeError('injected read failure')
                    if name == 'get_kin_data' and self.fault == 'late':
                        self.clock.advance_ms(201)
                    return await original()
                finally:
                    self.active -= 1
                    self.calls.append(('end', name))
            setattr(self, name, wrapped)

    async def is_connected(self):
        if self.pause_ns is not None:
            assert self.active == 0
            self.calls.append(('connection',))
        return await super().is_connected()

    async def get_motion_state(self, motion_id):
        if self.pause_ns is not None:
            assert self.active == 0, 'task query must follow all six known results'
            self.calls.append(('task', motion_id))
        return await super().get_motion_state(motion_id)


async def exercise(*, enabled=True, client=None, clock=None, hot=lambda: None, report=None):
    clock = clock or FakeClock()
    client = client or ConcurrentClient(clock)
    recorder = Recorder()
    async def factory(ip):
        assert ip == '127.0.0.1'
        return client
    try:
        result = await run_window(settings_for(client), recorder, factory, lambda: None, hot,
            clock=clock.now_ns, deadline_clock=clock.now_ns, sleep=clock.sleep, report=report,
            offline_pause_comparison=True, offline_stop_start_period=True,
            offline_parallel_stop_reads=enabled)
        return result, client, recorder
    finally:
        client.release.set()
        for _ in range(8):
            await asyncio.sleep(0)
        assert client.network_calls == client.discovery_calls == 0


async def test_six_fields_overlap_across_repeated_stop_groups_with_task_query_last():
    result, client, recorder = await exercise()
    assert result['complete'] and result['stop_confirmed']
    assert result['stop_snapshot_read_policy'] == 'six_fields_parallel'
    assert client.peak == 6
    diagnostics = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')
    assert len(diagnostics['samples']) >= 16
    assert diagnostics['verification_deadline_ns'] - diagnostics['verification_started_ns'] == 500_000_000
    assert diagnostics['samples'][-1]['observed_ns'] - diagnostics['stable_since_ns'] >= 300_000_000
    assert diagnostics['stable_since_ns'] >= diagnostics['verification_started_ns']
    groups = []
    for item in client.calls:
        if item[0] == 'connection':
            groups.append([])
        else:
            groups[-1].append(item)
    expected = set(FIELDS)
    for group in groups[:len(diagnostics['samples'])]:
        assert {item[1] for item in group if item[0] == 'start'} == expected
        assert len([item for item in group if item[0] == 'end']) == 6
        assert group[-1][0] == 'task'
    assert [call[0] for call in client.write_calls] == ['move_pvat', 'pause_move']
    assert not result['hardware_verified'] and not result['motion_authorized']


async def test_default_path_remains_serial():
    result, client, _ = await exercise(enabled=False)
    assert result['complete'] and client.peak == 1
    assert result['stop_snapshot_read_policy'] == 'serial'


@pytest.mark.parametrize('value', [None, 0, 1, 'true', [], {}])
async def test_selector_is_strict_boolean_before_factory(value):
    client = ConcurrentClient(FakeClock())
    async def forbidden(ip):
        pytest.fail('invalid selector entered factory')
    with pytest.raises(TypeError, match='parallel_stop_reads_requires_boolean'):
        await run_window(settings_for(client), Recorder(), forbidden, lambda: None, lambda: None,
                         offline_parallel_stop_reads=value)


@pytest.mark.parametrize('case', ['no_pause', 'no_start_period', 'real_ip'])
async def test_candidate_cannot_escape_original_offline_scope(case):
    client = ConcurrentClient(FakeClock())
    settings = settings_for(client)
    if case == 'real_ip':
        settings = replace(settings, ip='172.16.2.162')
    async def forbidden(ip):
        pytest.fail('invalid candidate entered factory')
    with pytest.raises(ValueError, match='(parallel_stop_reads_requires_offline_pause|pause.*loopback)'):
        await run_window(settings, Recorder(), forbidden, lambda: None, lambda: None,
            offline_parallel_stop_reads=True, offline_pause_comparison=case != 'no_pause',
            offline_stop_start_period=case != 'no_start_period')


@pytest.mark.parametrize('mode', ['running', 'late_finish', 'drift', 'late_effect'])
async def test_parallelism_preserves_invalid_state_or_drift_failure(mode):
    clock = FakeClock()
    result, client, _ = await exercise(client=ConcurrentClient(clock, mode=mode), clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == 'stop_incomplete'
    assert [call[0] for call in client.write_calls] == ['move_pvat', 'pause_move']


@pytest.mark.parametrize('fault', ['error', 'late', 'pending'])
async def test_failed_or_unknown_parallel_read_does_not_install_or_restart_group(fault):
    clock = FakeClock()
    result, client, recorder = await exercise(client=ConcurrentClient(clock, fault=fault), clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == 'stop_unverified'
    assert [call[0] for call in client.write_calls] == ['move_pvat', 'pause_move']
    assert sum(item[0] == 'connection' for item in client.calls) == 1
    assert not any(item[0] == 'task' for item in client.calls)
    if fault == 'pending':
        assert result['read_remote_result_unknown']
    diagnostics = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')
    assert diagnostics['samples'] == []


async def test_cancellation_during_group_retains_unknown_and_single_pause():
    clock, report = FakeClock(), {}
    client = ConcurrentClient(clock, fault='pending')
    task = asyncio.create_task(exercise(client=client, clock=clock, report=report))
    await asyncio.wait_for(client.entered.wait(), 1)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert report['interrupted'] and not report['stop_confirmed']
    assert report['latched_fault'] == 'stop_unverified'
    assert report['read_remote_result_unknown']
    assert [call[0] for call in client.write_calls] == ['move_pvat', 'pause_move']


async def test_authorization_revocation_cannot_install_partial_group():
    clock = FakeClock()
    client = ConcurrentClient(clock)
    def hot():
        if client.pause_ns is not None and client.active:
            raise ValueError('revoked test read authority')
    result, _, recorder = await exercise(client=client, clock=clock, hot=hot)
    assert not result['stop_confirmed'] and result['latched_fault'] == 'stop_unverified'
    assert not any(item[0] == 'task' for item in client.calls)
    assert next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')['samples'] == []


async def test_legacy_site_sampling_runner_cannot_forward_candidate():
    from app.commissioning.finite_pause_sampling import run_sampling_pause_window
    clock = FakeClock()
    client = ConcurrentClient(clock)
    async def forbidden(ip):
        pytest.fail('site runner enabled Fake-only sampling')
    with pytest.raises(ValueError, match='parallel_stop_reads_requires_offline_pause'):
        await run_sampling_pause_window(settings_for(client, ip='172.16.2.162'), Recorder(),
            forbidden, lambda: None, lambda: None, clock=clock.now_ns,
            deadline_clock=clock.now_ns, write_check=lambda: None, offline_parallel_stop_reads=True)


async def test_verification_read_requires_returned_pause_transaction():
    from app.commissioning.finite_stop_parallel import ParallelStopPauseAdapter, ParallelStopScopeClient
    from app.commissioning.idle_stop_baseline import AdmissionRejected
    clock = FakeClock()
    client = ConcurrentClient(clock)
    owner = asyncio.current_task()
    proxy = ParallelStopScopeClient(client, lambda *a: None, lambda: None,
        lambda: None, owner, clock.now_ns, deadline_clock=clock.now_ns)
    async def factory(ip):
        return proxy
    adapter = ParallelStopPauseAdapter(settings_for(client), client_factory=factory,
        clock=clock.now_ns, sleep=clock.sleep, backend_label='LEBAI_FAKE',
        offline_stop_start_period=True)
    try:
        await adapter.connect()
        before = len(client.read_calls)
        with pytest.raises(AdmissionRejected, match='finite_parallel_stop_read_revoked'):
            await adapter._read_snapshot(read_purpose='stop_verification')
        assert len(client.read_calls) == before
        assert client.write_calls == []
    finally:
        await adapter.disconnect()
