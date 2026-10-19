"""Injected Fake only. These tests do not model controller braking."""
import asyncio
from dataclasses import replace

import pytest

from app.commissioning.finite_window_run import run_window
from tests.commissioning.test_finite_window_run import ZClient
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


class PauseZClient(ZClient):
    def __init__(self, clock, *, mode='finished'):
        super().__init__()
        self.clock, self.mode = clock, mode
        self.pause_ns = None

    async def move_pvat(self, *args):
        result = await super().move_pvat(*args)
        self.running_motion = result
        self.motion_state = 'RUNNING'
        return result

    async def pause_move(self):
        self.write_calls.append(('pause_move',))
        self.pause_ns = self.clock.now_ns()
        if self.mode == 'error':
            raise RuntimeError('injected pause failure')
        self.robot_state = 'PAUSED'
        if self.mode != 'running':
            self.motion_state = 'FINISHED'

    async def stop_move(self):
        await super().stop_move()
        self.robot_state = 'IDLE'
        self.running_motion = None
        self.motion_state = 'FINISHED'

    async def get_kin_data(self):
        if self.pause_ns is not None:
            elapsed = self.clock.now_ns() - self.pause_ns
            if self.mode == 'late_finish':
                self.motion_state = 'FINISHED' if elapsed >= 250_000_000 else 'RUNNING'
            if self.mode == 'drift':
                self.kin_data['actual_joint_pose'][0] += .002
            if self.mode == 'late_effect' and elapsed >= 100_000_000:
                self.robot_state = 'MOVING'
                self.motion_state = 'RUNNING'
            if self.mode == 'tail_drift' and elapsed >= 500_000_000:
                self.kin_data['actual_joint_pose'][0] += .002
        return await super().get_kin_data()


async def exercise(*, mode='finished', enabled=True, ip='127.0.0.1', client=None,
                   clock=None, recorder=None, hot=lambda: None, report=None):
    clock = clock or FakeClock()
    client = client or PauseZClient(clock, mode=mode)
    recorder = recorder or Recorder()
    settings = control_settings()
    settings = replace(settings, ip=ip,
        teleop_ready_q=tuple(client.kin_data['actual_joint_pose']),
        control=replace(settings.control, max_joint_speed_radps=.05,
                        max_joint_acceleration_radps2=.2, max_tcp_step_m=.00005))
    async def factory(requested_ip):
        assert requested_ip == ip
        return client
    kwargs = dict(offline_pause_comparison=enabled)
    result = await run_window(settings, recorder, factory, lambda: None, hot,
        clock=clock.now_ns, sleep=clock.sleep, report=report, **kwargs)
    assert client.network_calls == client.discovery_calls == 0
    return result, client, recorder


async def test_explicit_pause_reuses_finite_packet_and_original_stop_verifier():
    result, client, recorder = await exercise()
    assert result['complete'] and result['stop_confirmed']
    assert result['termination_method'] == 'pause_move'
    assert result['pause_entries'] == result['pvat_entries'] == 1
    assert result['stop_entries'] == 0
    assert len(client.ik_calls) == 1 and len(client.fk_calls) == 4
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    assert client.write_calls[0][4] <= .08
    assert client.write_calls[0][2] == client.write_calls[0][3] == [0.] * 6
    assert not result['hardware_verified'] and not result['motion_authorized']
    diagnostics = [e for e in recorder.events if e['kind'] == 'stop_diagnostics']
    assert diagnostics and all(e['stop_policy'] == 'pause_move_only' for e in diagnostics)
    call = diagnostics[0]['rpc_calls'][0]
    assert call['method'] == 'pause_move' and call['timeout_ms'] == 200
    samples = diagnostics[0]['samples']
    assert samples[-1]['observed_ns'] - samples[0]['observed_ns'] >= 300_000_000
    tail = [e for e in recorder.events if e['kind'] == 'stop_observation_sample']
    assert len(tail) == 10
    assert all(e['state']['raw_robot_state'] == 'PAUSED' for e in tail)
    assert all(e['state']['motion_state'] == 'FINISHED' for e in tail)
    assert result['local_resources_released']


async def test_default_still_uses_stop_move_when_pause_is_available():
    result, client, _ = await exercise(enabled=False)
    assert result['complete'] and result['stop_entries'] == 1
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']


@pytest.mark.parametrize('value', [None, 1, 0, 'true', [], {}])
async def test_pause_selector_is_strict_boolean_before_factory(value):
    async def forbidden(_ip):
        pytest.fail('factory must not be called')
    with pytest.raises(TypeError, match='pause.*boolean'):
        await run_window(control_settings(), Recorder(), forbidden, lambda: None,
                         lambda: None, offline_pause_comparison=value)


@pytest.mark.parametrize('ip', ['172.16.2.162', '192.168.10.20', 'localhost', '::1', '127.0.0.2'])
async def test_pause_requires_literal_ipv4_loopback_before_factory(ip):
    async def forbidden(_ip):
        pytest.fail('factory must not be called')
    with pytest.raises(ValueError, match='pause.*loopback'):
        await run_window(replace(control_settings(), ip=ip), Recorder(), forbidden,
                         lambda: None, lambda: None, offline_pause_comparison=True)


async def test_missing_pause_capability_cannot_send_motion_or_fallback():
    result, client, _ = await exercise(client=ZClient())
    assert not result['complete']
    assert result['pvat_entries'] == result['pause_entries'] == result['stop_entries'] == 0
    assert client.write_calls == []


@pytest.mark.parametrize('mode', ['running', 'late_finish', 'drift', 'late_effect', 'error'])
async def test_pause_failure_never_becomes_success_or_sends_fallback(mode):
    result, client, recorder = await exercise(mode=mode)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] in ('stop_unverified', 'stop_incomplete')
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert not any(e['kind'] == 'stop_confirmed' for e in recorder.events)


async def test_drift_in_readonly_tail_cannot_report_complete():
    result, client, _ = await exercise(mode='tail_drift')
    assert not result['complete'] and not result['tail_complete']
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


async def test_paused_without_finished_task_evidence_cannot_complete():
    clock = FakeClock()
    client = PauseZClient(clock)
    original = client.pause_move
    async def pause_without_task():
        await original()
        client.running_motion = None
    client.pause_move = pause_without_task
    result, _, recorder = await exercise(client=client, clock=clock)
    assert not result['complete'] and not result['tail_complete']
    assert result['displacement_samples'] == 0
    assert any(e['kind'] == 'stop_observation_read_failed' and
               e['error'] == 'paused_cleanup_finished_motion_evidence_missing'
               for e in recorder.events)


async def test_unknown_pvat_result_still_owns_one_pause_and_keeps_fault():
    clock = FakeClock()
    client = PauseZClient(clock)
    original = client.move_pvat
    async def uncertain(*args):
        await original(*args)
        raise RuntimeError('injected unknown pvat result')
    client.move_pvat = uncertain
    result, _, _ = await exercise(client=client, clock=clock)
    assert result['pvat_remote_result_unknown']
    assert result['latched_fault'] == 'stop_unverified'
    assert not result['complete'] and not result['stop_confirmed']
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


async def test_motion_revocation_preserves_only_reserved_pause():
    clock = FakeClock()
    client = PauseZClient(clock)
    def hot():
        if client.write_calls:
            from app.commissioning.idle_stop_baseline import AdmissionRejected
            raise AdmissionRejected('finite_scope_test_revoked')
    result, _, _ = await exercise(client=client, clock=clock, hot=hot)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


async def test_owner_cancellation_preserves_one_pause_without_fallback():
    entered, release = asyncio.Event(), asyncio.Event()
    class BlockingClock(FakeClock):
        async def sleep(self, seconds):
            if seconds >= .08 and not entered.is_set():
                entered.set()
                await release.wait()
            await super().sleep(seconds)
    clock, report = BlockingClock(), {}
    client = PauseZClient(clock)
    task = asyncio.create_task(exercise(client=client, clock=clock, report=report))
    try:
        await asyncio.wait_for(entered.wait(), 1.)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 1.)
        assert report['interrupted'] and not report['complete']
        assert report['pause_entries'] == 1 and report['stop_entries'] == 0
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    finally:
        release.set()
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


async def test_pause_proxy_denies_duplicate_motion_pause_and_other_writes():
    from app.commissioning.finite_window_run import _FinitePauseScopeClient
    from app.commissioning.idle_stop_baseline import AdmissionRejected
    clock, calls = FakeClock(), []
    client = PauseZClient(clock)
    async def resume():
        pytest.fail('resume must never reach the native client')
    client.resume_move = resume
    proxy = _FinitePauseScopeClient(client, lambda *args: None,
        lambda: calls.append('cleanup'), lambda: None,
        asyncio.current_task(), clock.now_ns)
    await proxy.move_pvat([.00005, -1., 1., 0., 1.57, 0.], [0.] * 6, [0.] * 6, .08)
    await proxy.pause_move()
    with pytest.raises(AdmissionRejected, match='pause_already_used'):
        await proxy.pause_move()
    with pytest.raises(AdmissionRejected, match='pvat_already_used'):
        await proxy.move_pvat([0.] * 6, [0.] * 6, [0.] * 6, .08)
    for method in ('stop_move', 'resume_move', 'stop_sys'):
        with pytest.raises(AdmissionRejected, match='forbidden'):
            await getattr(proxy, method)()
    assert calls == ['cleanup']
    assert proxy.sdk_stop_entries == 0 and proxy.termination_entries == 1
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


async def test_pause_reply_later_than_200ms_stays_unknown_after_late_finish():
    clock, release = FakeClock(), asyncio.Event()
    client = PauseZClient(clock)
    native_pause = client.pause_move
    async def delayed_pause():
        await native_pause()
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                pass
    client.pause_move = delayed_pause
    try:
        result, _, recorder = await exercise(client=client, clock=clock)
        assert result['latched_fault'] == 'stop_unverified'
        assert not result['complete'] and not result['stop_confirmed']
        assert result['pause_entries'] == 1 and result['stop_entries'] == 0
        release.set()
        for _ in range(10):
            await asyncio.sleep(0)
        assert result['latched_fault'] == 'stop_unverified'
        assert not any(e['kind'] == 'stop_confirmed' for e in recorder.events)
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    finally:
        release.set()
        for _ in range(10):
            await asyncio.sleep(0)


@pytest.mark.parametrize('elapsed_ns,accepted', [
    (199_999_999, True), (200_000_000, False), (200_000_001, False),
])
async def test_pause_rpc_deadline_boundary_never_accepts_exactly_200ms(elapsed_ns, accepted):
    clock = FakeClock()
    client = PauseZClient(clock)
    original = client.pause_move
    async def pause():
        await original()
        clock.value += elapsed_ns
    client.pause_move = pause
    result, _, recorder = await exercise(client=client, clock=clock)
    assert result['complete'] is accepted
    assert result['stop_confirmed'] is accepted
    assert result['latched_fault'] == (None if accepted else 'stop_unverified')
    lifecycle = [e for e in recorder.events if e['kind'] == 'stop_rpc_lifecycle']
    assert len(lifecycle) == 1 and lifecycle[0]['method'] == 'pause_move'
    assert lifecycle[0]['outcome'] == ('returned' if accepted else 'returned_late')
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


@pytest.mark.parametrize('finished_after_ns,accepted', [
    (0, True), (180_000_000, True), (199_000_000, False),
    (200_000_000, False), (201_000_000, False), (281_000_000, False),
])
async def test_pause_needs_full_stability_before_original_verification_deadline(finished_after_ns, accepted):
    # Synthetic 20 ms cadence, not a replay of field feedback or physical braking.
    clock = FakeClock()
    client = PauseZClient(clock, mode='running')
    original = client.get_kin_data
    async def kin():
        if client.pause_ns is not None:
            elapsed = clock.now_ns() - client.pause_ns
            client.motion_state = 'FINISHED' if elapsed >= finished_after_ns else 'RUNNING'
        return await original()
    client.get_kin_data = kin
    result, _, recorder = await exercise(client=client, clock=clock)
    assert result['complete'] is accepted
    assert result['stop_confirmed'] is accepted
    assert result['latched_fault'] == (None if accepted else 'stop_incomplete')
    diagnostics = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')
    samples = diagnostics['samples']
    first_eligible = next(s for s in samples if s['stationary'])
    span_ns = samples[-1]['observed_ns'] - first_eligible['observed_ns']
    verification_elapsed_ns = samples[-1]['observed_ns'] - samples[0]['observed_ns']
    if accepted:
        assert span_ns >= 300_000_000 and verification_elapsed_ns < 500_000_000
    else:
        assert verification_elapsed_ns >= 500_000_000
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


@pytest.mark.parametrize('raw', ['IDLE', 'PAUSED', 5, 6])
async def test_settled_raw_state_cannot_hide_unfinished_task(raw):
    clock = FakeClock()
    client = PauseZClient(clock, mode='running')
    original = client.pause_move
    async def pause():
        await original()
        client.robot_state = raw
    client.pause_move = pause
    result, _, _ = await exercise(client=client, clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == 'stop_incomplete'
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
