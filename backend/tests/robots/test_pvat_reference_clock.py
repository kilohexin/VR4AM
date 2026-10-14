"""Local reference lifetime checks with independent fake clocks and SDK only."""
import asyncio

import pytest

from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.command_admission import CommandRef
from app.robots.lebai_ik_policy import evaluate_ik_candidate
from app.robots.lebai_pump import PvatRequest
from app.robots.lebai_pvat import build_pvat_point
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target
from tests.robots.test_lebai_backpressure_experiments import wait_until


PREVIOUS = (.12, 0., 0., 0., 0., 0.)


def _seed(adapter, mono=0, reference=1_000_000_000):
    adapter._previous_sent_qd = PREVIOUS
    adapter._last_pvat_started_ns = mono
    adapter._last_pvat_clock_started_ns = reference


@pytest.mark.parametrize("mono_ms,reference_ms,previous_used", [
    (78, 79.99, True), (78, 80.058, False), (78, 80.3866, False),
    (78, 82.1847, False), (78, 87.4124, False), (80, 79, False),
    (-1, 40, False), (79, -1, False), (0, 0, True),
])
def test_both_independent_intervals_must_be_inside_horizon(
    mono_ms, reference_ms, previous_used,
):
    adapter = RealLebaiAdapter(control_settings(), clock=lambda: int(mono_ms * 1e6))
    _seed(adapter)
    adapter._pvat_clock = lambda: 1_000_000_000 + int(reference_ms * 1e6)
    assert adapter._continuous_pvat_velocity() == (PREVIOUS if previous_used else None)


@pytest.mark.parametrize("bad_time", [None, False, 1.5, -1])
def test_unknown_or_invalid_reference_clock_uses_observed_velocity(bad_time):
    adapter = RealLebaiAdapter(control_settings(), clock=lambda: 40_000_000)
    _seed(adapter)
    adapter._pvat_clock = lambda: bad_time
    assert adapter._continuous_pvat_velocity() is None


def test_reference_clock_exception_does_not_keep_old_velocity():
    adapter = RealLebaiAdapter(control_settings(), clock=lambda: 40_000_000)
    _seed(adapter)

    def failed_clock():
        raise RuntimeError("offline clock failure")

    adapter._pvat_clock = failed_clock
    assert adapter._continuous_pvat_velocity() is None


def test_missing_timestamp_does_not_infer_reference_lifetime():
    adapter = RealLebaiAdapter(control_settings(), clock=lambda: 40_000_000)
    _seed(adapter, reference=None)
    assert adapter._continuous_pvat_velocity() is None


def test_reset_clears_both_reference_clock_histories():
    adapter = RealLebaiAdapter(control_settings())
    _seed(adapter)
    adapter._reset_pvat_history()
    assert adapter._last_pvat_started_ns is None
    assert adapter._last_pvat_clock_started_ns is None


@pytest.mark.asyncio
@pytest.mark.parametrize("expiry_boundary", ["candidate_return", "sdk_task", "clock_unknown"])
@pytest.mark.parametrize("request_kind", ["target", "catch_up"])
async def test_expired_reference_is_rebuilt_at_sdk_entry_and_recorded_exactly(
    monkeypatch, expiry_boundary, request_kind,
):
    adapter, client, mono = await _connected_control_adapter()
    reference = [1_000_000_000]
    adapter._pvat_clock = lambda: reference[0]
    _seed(adapter, mono=mono.now_ns(), reference=reference[0])
    mono.advance_ms(40)
    reference[0] += 40_000_000
    events = []
    selected_points = []

    async def record(event, _time):
        events.append(event)

    def after_selection(selected, snapshot):
        assert selected.point.reference_source == "previous_command_velocity"
        selected_points.append((selected, snapshot))
        if expiry_boundary == "candidate_return":
            reference[0] += 50_000_000
        elif expiry_boundary == "clock_unknown":
            reference[0] = None
        return selected

    async def select(_client, snapshot, request, *, diagnostic_events):
        solution = (.024, *snapshot.actual_q[1:])
        metrics = evaluate_ik_candidate(solution, snapshot.actual_q,
                                        previous_solution_q=solution)
        return after_selection(adapter._build_advancing_candidate(
            snapshot, request.target, metrics, 1.), snapshot)

    original_catch_up = adapter._build_catch_up_candidate

    def select_catch_up(snapshot, request):
        return after_selection(original_catch_up(snapshot, request), snapshot)

    original_wait = adapter._await_motion_call

    async def advancing_wait(method, operation, timeout_s, **kwargs):
        if (expiry_boundary == "sdk_task"
                and method == 'move_pvat'):
            reference[0] += 50_000_000
        return await original_wait(method, operation, timeout_s, **kwargs)

    adapter._event_callback = record
    monkeypatch.setattr(adapter, "_select_advancing_candidate", select)
    monkeypatch.setattr(adapter, "_build_catch_up_candidate", select_catch_up)
    monkeypatch.setattr(adapter, "_await_motion_call", advancing_wait)
    client.read_calls.clear()
    await adapter._pump.start()
    try:
        accepted = None
        if request_kind == 'catch_up':
            accepted = CommandRef(70, adapter._pump._generation, 1)
            adapter._accepted_target_ref = accepted
            adapter._last_sent_tcp = _target(.301)
            adapter._last_accepted_solution_q = (.024, *adapter._snapshot.actual_q[1:])
        await adapter._send_target(PvatRequest(_target(.301), 71, adapter._pump._generation,
                                              catch_up_from=accepted))
        await wait_until(lambda: any(e['kind'] == 'sdk_request_lifecycle' for e in events))
        chosen, snapshot = selected_points[0]
        expected = build_pvat_point(chosen.metrics.solution_q, snapshot.actual_q,
                                    snapshot.actual_qd, None, adapter._pvat_limits)
        assert client.write_calls == [('move_pvat', list(expected.q), list(expected.qd),
                                       list(expected.qdd), expected.horizon_s)]
        for kind in ('sdk_request_lifecycle', 'pvat_sent'):
            entry = next(e for e in events if e['kind'] == kind)
            assert entry['p'] == list(expected.q)
            assert entry['v'] == list(expected.qd)
            assert entry['a'] == list(expected.qdd)
            assert entry['point_inputs']['velocity_reference_source'] == 'observed_velocity'
            assert entry['point_inputs']['actual_q'] == list(snapshot.actual_q)
            assert entry['pvat_reference_timing']['rebuilt_at_sdk_entry'] is True
        sent = next(e for e in events if e['kind'] == 'pvat_sent')
        assert sent['pvat_mode'] == ('catch_up' if accepted else 'advance')
        if accepted:
            assert adapter._accepted_target_ref == accepted
        assert adapter._last_pvat_clock_started_ns == reference[0]
        assert client.read_calls.count('get_kin_data') == 1
        assert client.read_calls.count('kinematics_inverse') == 0
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('revocation', ['pump', 'permission'])
async def test_rebuilding_reference_cannot_bypass_motion_revocation(monkeypatch, revocation):
    from app.robots import lebai_adapter as module

    adapter, client, mono = await _connected_control_adapter()
    reference = [1_000_000_000]
    adapter._pvat_clock = lambda: reference[0]
    _seed(adapter, mono=mono.now_ns(), reference=reference[0])
    mono.advance_ms(40)
    reference[0] += 40_000_000
    allowed = [True]
    adapter.bind_motion_authority(lambda: allowed[0])
    native_rebuild = [False]
    events = []

    async def record(event, _time):
        events.append(event)

    async def select(_client, snapshot, request, *, diagnostic_events):
        solution = (.024, *snapshot.actual_q[1:])
        metrics = evaluate_ik_candidate(solution, snapshot.actual_q,
                                        previous_solution_q=solution)
        selected = adapter._build_advancing_candidate(snapshot, request.target, metrics, 1.)
        reference[0] += 50_000_000
        native_rebuild[0] = True
        return selected

    original_build = module.build_pvat_point

    def revoking_build(*args, **kwargs):
        point = original_build(*args, **kwargs)
        if native_rebuild[0]:
            if revocation == 'pump':
                adapter._pump.invalidate()
            else:
                allowed[0] = False
        return point

    adapter._event_callback = record
    monkeypatch.setattr(adapter, '_select_advancing_candidate', select)
    monkeypatch.setattr(module, 'build_pvat_point', revoking_build)
    await adapter._pump.start()
    try:
        await adapter._send_target(PvatRequest(_target(.301), 71, adapter._pump._generation))
        await wait_until(lambda: any(e['kind'] == 'sdk_request_lifecycle' for e in events))
        assert client.write_calls == []
        assert not any(e['kind'] == 'pvat_sent' for e in events)
        assert next(e for e in events if e['kind'] == 'sdk_request_lifecycle')['rpc_outcome'] == 'not_started'
        assert adapter._last_pvat_clock_started_ns == 1_000_000_000
    finally:
        allowed[0] = True
        await adapter.disconnect()
