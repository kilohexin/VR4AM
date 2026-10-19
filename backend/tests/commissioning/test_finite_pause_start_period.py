"""Synthetic timing cases only; not a replay or a physical stopping model."""
import asyncio
from dataclasses import replace

import pytest

from app.commissioning.finite_window_run import run_window
from tests.commissioning.test_finite_pause_window import PauseZClient
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


class TimedPauseClient(PauseZClient):
    def __init__(self, clock, *, read_ms=38., finished_ms=160., drift=False):
        super().__init__(clock, mode='running')
        self.read_ms, self.finished_ms, self.drift = read_ms, finished_ms, drift
        self.stop_read_starts = []

    async def is_connected(self):
        if self.pause_ns is not None:
            self.stop_read_starts.append(self.clock.now_ns())
        return await super().is_connected()

    async def get_kin_data(self):
        if self.pause_ns is not None:
            self.clock.advance_ms(self.read_ms)
            if self.drift:
                self.kin_data['actual_joint_pose'][0] += .002
        return await super().get_kin_data()

    async def get_motion_state(self, motion_id):
        if self.pause_ns is not None:
            elapsed_ms = (self.clock.now_ns() - self.pause_ns) / 1e6
            self.motion_state = 'FINISHED' if elapsed_ms >= self.finished_ms else 'RUNNING'
        return await super().get_motion_state(motion_id)


def settings_for(client, *, ip='127.0.0.1'):
    settings = control_settings()
    return replace(settings, ip=ip, teleop_ready_q=tuple(client.kin_data['actual_joint_pose']),
        control=replace(settings.control, max_joint_speed_radps=.05,
                        max_joint_acceleration_radps2=.2, max_tcp_step_m=.00005))


async def run_timed(*, candidate=True, client=None, clock=None, sleep=None, report=None):
    clock = clock or FakeClock()
    client = client or TimedPauseClient(clock)
    recorder = Recorder()
    async def factory(ip):
        assert ip == '127.0.0.1'
        return client
    options = {} if candidate is None else dict(offline_stop_start_period=candidate)
    result = await run_window(settings_for(client), recorder, factory, lambda: None, lambda: None,
        clock=clock.now_ns, deadline_clock=clock.now_ns, sleep=sleep or clock.sleep, report=report,
        offline_pause_comparison=True, **options)
    assert client.network_calls == client.discovery_calls == 0
    return result, client, recorder


async def test_original_post_read_wait_misses_synthetic_stability_window():
    result, client, recorder = await run_timed(candidate=None)
    assert not result['complete'] and result['latched_fault'] == 'stop_incomplete'
    samples = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')['samples']
    assert [s['observed_ns'] - client.pause_ns for s in samples[:4]] == [
        38_000_000, 96_000_000, 154_000_000, 212_000_000]
    assert samples[-1]['observed_ns'] - client.pause_ns == 502_000_000


async def test_start_period_preserves_all_reads_and_completes_synthetic_stability():
    result, client, recorder = await run_timed()
    assert result['complete'] and result['stop_confirmed']
    assert result['stop_sampling_policy'] == 'start_period_20ms'
    diagnostics = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')
    samples = diagnostics['samples']
    assert [s['observed_ns'] - client.pause_ns for s in samples[:5]] == [
        38_000_000, 76_000_000, 114_000_000, 152_000_000, 190_000_000]
    assert samples[-1]['observed_ns'] - client.pause_ns == 494_000_000
    assert samples[-1]['observed_ns'] - samples[4]['observed_ns'] == 304_000_000
    assert all(s['read_completed_ns'] - s['read_requested_ns'] == 38_000_000 for s in samples)
    groups = []
    for call in result['read_calls']:
        if call['started_ns'] >= client.pause_ns:
            if call['method'] == 'is_connected':
                groups.append([])
            groups[-1].append(call['method'])
    expected = ['is_connected', 'get_robot_state', 'get_estop_reason', 'get_kin_data',
                'get_tcp', 'get_claw', 'get_running_motion', 'get_motion_state']
    assert groups[:len(samples)] == [expected] * len(samples)
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    assert result['pvat_entries'] == result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert not result['hardware_verified'] and not result['motion_authorized']


async def test_original_stop_move_policy_keeps_fixed_post_read_wait():
    clock = FakeClock()
    client = TimedPauseClient(clock, finished_ms=0.)
    original = client.stop_move
    async def stop():
        await original()
        client.pause_ns = clock.now_ns()  # Start synthetic read cost instrumentation.
    client.stop_move = stop
    recorder = Recorder()
    async def factory(ip):
        return client
    result = await run_window(settings_for(client), recorder, factory, lambda: None, lambda: None,
        clock=clock.now_ns, deadline_clock=clock.now_ns, sleep=clock.sleep)
    assert result['complete'] and result['stop_sampling_policy'] == 'fixed_delay_after_read_20ms'
    samples = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')['samples']
    starts = [s['read_requested_ns'] for s in samples]
    assert all(b - a == 58_000_000 for a, b in zip(starts, starts[1:]))
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']


@pytest.mark.parametrize('read_ms,expected_gap_ns', [(0., 20_000_000), (5., 20_000_000), (38., 38_000_000)])
async def test_read_start_period_never_polls_faster_than_20ms(read_ms, expected_gap_ns):
    clock = FakeClock()
    client = TimedPauseClient(clock, read_ms=read_ms, finished_ms=0.)
    result, _, recorder = await run_timed(client=client, clock=clock)
    assert result['complete']
    samples = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')['samples']
    starts = [s['read_requested_ns'] for s in samples]
    assert all(b - a == expected_gap_ns for a, b in zip(starts, starts[1:]))


async def test_early_sleep_wakeup_cannot_start_a_read_before_20ms():
    clock = FakeClock()
    client = TimedPauseClient(clock, read_ms=5., finished_ms=0.)
    async def early_sleep(seconds):
        if seconds:
            clock.value += max(1, int(seconds * 1e9 / 2))
        await asyncio.sleep(0)
    result, _, recorder = await run_timed(client=client, clock=clock, sleep=early_sleep)
    assert result['complete']
    samples = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')['samples']
    starts = [s['read_requested_ns'] for s in samples]
    assert all(b - a >= 20_000_000 for a, b in zip(starts, starts[1:]))


@pytest.mark.parametrize('mode', ['never_finished', 'late_finished', 'drift', 'late_rpc'])
async def test_faster_polling_never_accepts_invalid_or_late_results(mode):
    clock = FakeClock()
    client = TimedPauseClient(clock, finished_ms=10000. if mode == 'never_finished'
                             else 260. if mode == 'late_finished' else 0., drift=mode == 'drift')
    if mode == 'late_rpc':
        original = client.pause_move
        async def pause():
            await original()
            clock.advance_ms(200.)
        client.pause_move = pause
    result, _, _ = await run_timed(client=client, clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == ('stop_unverified' if mode == 'late_rpc' else 'stop_incomplete')
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


async def test_unknown_verification_read_stays_latched_with_one_pause():
    clock, release = FakeClock(), asyncio.Event()
    client = TimedPauseClient(clock)
    original = client.get_kin_data
    async def pending():
        if client.pause_ns is not None:
            await release.wait()
        return await original()
    client.get_kin_data = pending
    try:
        result, _, _ = await run_timed(client=client, clock=clock)
        assert result['read_remote_result_unknown']
        assert result['latched_fault'] == 'stop_unverified' and not result['complete']
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    finally:
        release.set()
        for _ in range(5):
            await asyncio.sleep(0)


async def test_zero_extra_wait_still_yields_cancellation_and_keeps_one_pause():
    clock, report = FakeClock(), {}
    client = TimedPauseClient(clock)
    requested = False
    async def cancel_at_yield(seconds):
        nonlocal requested
        if seconds == 0. and client.pause_ns is not None and not requested:
            requested = True
            asyncio.current_task().cancel()
        await clock.sleep(seconds)
    task = asyncio.create_task(run_timed(client=client, clock=clock, sleep=cancel_at_yield, report=report))
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(task, 1.)
    assert requested and report['interrupted'] and not report['complete']
    assert report['latched_fault'] == 'stop_unverified'
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


@pytest.mark.parametrize('value', [None, 0, 1, 'true', [], {}])
async def test_start_period_selector_requires_strict_boolean_before_factory(value):
    async def forbidden(_ip):
        pytest.fail('invalid option connected')
    with pytest.raises(TypeError, match='stop_start_period.*boolean'):
        await run_window(control_settings(), Recorder(), forbidden, lambda: None, lambda: None,
                         offline_stop_start_period=value)


async def test_start_period_cannot_enable_on_original_stop_policy():
    client = TimedPauseClient(FakeClock())
    async def forbidden(_ip):
        pytest.fail('stop-only scope gained a new policy')
    with pytest.raises(ValueError, match='stop_start_period.*offline_pause'):
        await run_window(settings_for(client), Recorder(), forbidden, lambda: None, lambda: None,
                         offline_stop_start_period=True)


async def test_real_pause_entry_cannot_forward_start_period_even_with_fake_factory():
    from app.commissioning.finite_pause_window import run_pause_window
    clock = FakeClock()
    client = TimedPauseClient(clock)
    async def forbidden(_ip):
        pytest.fail('site path must reject this option before connection')
    with pytest.raises(ValueError, match='stop_start_period.*offline_pause'):
        await run_pause_window(settings_for(client, ip='172.16.2.162'), Recorder(), forbidden,
            lambda: None, lambda: None, clock=clock.now_ns, deadline_clock=clock.now_ns,
            write_check=lambda: None, offline_stop_start_period=True)
