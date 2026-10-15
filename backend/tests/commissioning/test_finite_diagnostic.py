"""Fake SDK only. Audit samples are not a model of physical motion."""
import asyncio
import copy
from dataclasses import replace
from unittest.mock import AsyncMock

import pytest

from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter
from tests.robots.fake_lebai import ACTUAL_TCP, FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock, _target, _wait_until


class AuditClient(FakeLebaiClient):
    def __init__(self):
        super().__init__()
        self.fk_calls = []
        self.fk_hook = None

    async def kinematics_inverse(self, pose, joints):
        self.ik_calls.append((pose.copy(), list(joints)))
        return [pose['x'] - ACTUAL_TCP['x'], *joints[1:]]

    async def kinematics_forward(self, joints):
        self.fk_calls.append(tuple(joints))
        result = {**ACTUAL_TCP, 'x': ACTUAL_TCP['x'] + joints[0]}
        if self.fk_hook:
            return await self.fk_hook(len(self.fk_calls), result)
        return result


async def setup(client=None, normal=False, parallel=False):
    client = client or AuditClient()
    clock = FakeClock()
    settings = control_settings()
    settings = replace(settings, control=replace(settings.control,
        max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2))
    events = []

    async def record(event, _ns):
        events.append(event)

    adapter = (RealLebaiAdapter if normal else FiniteWindowDiagnosticAdapter)(
        settings, client_factory=AsyncMock(return_value=client), clock=clock.now_ns,
        pvat_clock=clock.now_ns, sleep=clock.sleep, event_callback=record,
        **({} if normal else {'parallel_fk': parallel}),
    )
    await adapter.connect()
    assert (await adapter.preflight()).ready
    client.read_calls.clear()
    return adapter, client, clock, events


async def test_opt_in_parallel_triplet_keeps_feedback_first_and_blocks_write_until_all_return():
    client=AuditClient();ready=asyncio.Event();release=asyncio.Event();active=0
    async def block(count,pose):
        nonlocal active
        if count==1:return pose
        active+=1
        if active==3:ready.set()
        await release.wait()
        return pose
    client.fk_hook=block
    adapter,_,_,_=await setup(client,parallel=True)
    task=asyncio.create_task(submit(adapter))
    try:
        await asyncio.wait_for(ready.wait(),1)
        assert len(client.fk_calls)==4 and client.write_calls==[]
        release.set();result=await task
        assert result.sdk_outcome=='returned' and len(client.write_calls)==1
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True);await adapter.disconnect()


@pytest.mark.parametrize('wrong',[2,3,4])
async def test_parallel_triplet_checks_every_returned_pose_before_write(wrong):
    client=AuditClient()
    async def change(count,pose):
        await asyncio.sleep(0)
        return {**pose,'y':pose['y']+.01} if count==wrong else pose
    client.fk_hook=change
    adapter,_,_,_=await setup(client,parallel=True)
    try:
        result=await submit(adapter)
        assert result.sdk_outcome=='not_started'
        assert not any(c[0]=='move_pvat' for c in client.write_calls)
    finally:await adapter.disconnect()


async def test_parallel_unknown_read_stays_owned_and_prevents_reconnect(monkeypatch):
    import app.commissioning.finite_diagnostic as module
    monkeypatch.setattr(module,'_FK_TIMEOUT_S',.02)
    client=AuditClient();started=asyncio.Event();release=asyncio.Event()
    async def stubborn(count,pose):
        if count!=3:return pose
        started.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return pose
    client.fk_hook=stubborn
    adapter,_,_,_=await setup(client,parallel=True)
    try:
        result=await submit(adapter)
        assert started.is_set() and result.sdk_outcome=='not_started'
        assert any(not t.done() for t in adapter._finite_pending_fks)
        with pytest.raises(BackendCommandError,match='finite_fk_read_pending'):await adapter.connect()
        assert not any(c[0]=='move_pvat' for c in client.write_calls)
    finally:
        release.set()
        await _wait_until(lambda:not adapter._finite_pending_fks)
        await adapter.disconnect()


@pytest.mark.parametrize('role',[2,3,4])
@pytest.mark.parametrize('kind',['malformed','error','expired','revoked'])
async def test_each_parallel_role_failure_prevents_motion(role,kind):
    client=AuditClient();adapter,_,clock,_=await setup(client,parallel=True)
    allowed=True
    adapter.bind_motion_authority(lambda:allowed)
    async def fail(count,pose):
        nonlocal allowed
        if count==role:
            if kind=='malformed':return {'bad':'payload'}
            if kind=='error':raise RuntimeError('injected')
            if kind=='expired':clock.value=100_000_000
            if kind=='revoked':allowed=False
        await asyncio.sleep(0)
        return pose
    client.fk_hook=fail
    try:
        result=await submit(adapter)
        assert result.sdk_outcome=='not_started'
        assert not any(c[0]=='move_pvat' for c in client.write_calls)
    finally:await adapter.disconnect()


async def submit(adapter):
    receipt = await adapter.submit_tcp(_target(.3002), 91)
    result = await asyncio.wait_for(receipt.completion.wait(), 2)
    return result


async def test_valid_audit_dispatches_one_zero_terminal_window_and_keeps_original_stop():
    adapter, client, _, events = await setup()
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'returned'
        assert len(client.fk_calls) == 4
        assert len(client.write_calls) == 1
        call, = client.write_calls
        assert call[0] == 'move_pvat' and call[2] == call[3] == [0.] * 6 and call[4] == .08
        await _wait_until(lambda: any(e['kind'] == 'finite_candidate_audit' for e in events))
        assert next(e for e in events if e['kind'] == 'finite_candidate_audit')['outcome'] == 'accepted'
        with pytest.raises(BackendCommandError, match='finite_single_attempt_consumed'):
            await adapter.submit_tcp(_target(.3001), 92)
    finally:
        await adapter.disconnect()
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']


async def test_normal_backend_does_not_use_fk_or_zero_terminal_policy():
    adapter, client, _, _ = await setup(normal=True)
    try:
        await submit(adapter)
        assert not client.fk_calls
        assert any(v != 0 for v in client.write_calls[0][2])
    finally:
        await adapter.disconnect()


async def test_missing_fk_capability_blocks_admission_after_connect():
    client = FakeLebaiClient()
    adapter = FiniteWindowDiagnosticAdapter(control_settings(), client_factory=AsyncMock(return_value=client))
    await adapter.connect()
    try:
        assert not adapter._preflight_ready
        preflight = await adapter.preflight()
        assert not preflight.ready and preflight.reason == 'kinematics_forward_unavailable'
        with pytest.raises(BackendCommandError):
            await adapter.submit_tcp(_target(.3002), 91)
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('call_number,kind', [(1, 'feedback'), (2, 'goal'), (3, 'midpoint'), (4, 'endpoint')])
async def test_cartesian_mismatch_calls_owned_stop_outside_motion_lock(call_number, kind):
    adapter, client, _, events = await setup()

    async def bad(index, result):
        if index == call_number:
            result['y'] += .001
        return result

    client.fk_hook = bad
    try:
        result = await submit(adapter)
        assert result.outcome == 'failed' and result.sdk_outcome == 'not_started'
        assert client.write_calls == [('stop_move',)]
        assert len(client.fk_calls) == call_number
        assert adapter._latched_fault is not None
        await _wait_until(lambda: any(e['kind'] == 'stop_diagnostics' for e in events))
        stop = next(e for e in events if e['kind'] == 'stop_diagnostics')
        assert stop['stop_entry']['sdk_lock_held'] is False
        assert stop['stop_policy'] == 'stop_move_only'
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('payload', [None, [], {'x': 0}, {**ACTUAL_TCP, 'z': float('nan')}])
async def test_invalid_fk_payload_does_not_dispatch(payload):
    adapter, client, _, _ = await setup()
    client.fk_hook = AsyncMock(return_value=payload)
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'not_started'
        assert client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('stage', [1, 2, 3, 4])
@pytest.mark.parametrize('age_ns', [99_999_999, 100_000_000])
async def test_lease_expiry_is_checked_after_each_fk_await(stage, age_ns):
    adapter, client, clock, _ = await setup()

    async def slow(index, result):
        if index == stage:
            clock.value = age_ns
        return result

    client.fk_hook = slow
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == ('returned' if age_ns < 100_000_000 else 'not_started')
        assert len(client.fk_calls) == (4 if age_ns < 100_000_000 else stage)
        assert [c[0] for c in client.write_calls] == (['move_pvat'] if age_ns < 100_000_000 else ['stop_move'])
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('change', ['authority', 'generation'])
@pytest.mark.parametrize('stage', [1, 4])
async def test_revocation_during_fk_invalidates_receipt_and_does_not_query_again(change, stage):
    adapter, client, _, _ = await setup()
    allowed = [True]
    adapter.bind_motion_authority(lambda: allowed[0])

    async def revoke(index, result):
        if index == stage:
            if change == 'authority':
                allowed[0] = False
            else:
                adapter._pump.invalidate()
        return result

    client.fk_hook = revoke
    try:
        result = await submit(adapter)
        assert result.outcome == 'invalidated' and result.sdk_outcome == 'not_started'
        assert len(client.fk_calls) == stage and client.write_calls == []
        assert adapter._accepted_target_ref is None
    finally:
        await adapter.disconnect()


async def test_sdk_child_entry_rechecks_audit_lease(monkeypatch):
    adapter, client, clock, _ = await setup()
    original = adapter._await_motion_call

    async def delayed(method, operation, timeout_s, **kwargs):
        if method == 'move_pvat':
            clock.value = 100_000_000
        return await original(method, operation, timeout_s, **kwargs)

    monkeypatch.setattr(adapter, '_await_motion_call', delayed)
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'not_started'
        assert len(client.fk_calls) == 4 and client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('action', ['set_gripper', 'home', 'prepare', 'submit_catch_up'])
async def test_other_actions_are_not_exposed_by_diagnostic(action):
    adapter, client, _, _ = await setup()
    try:
        arguments = {'set_gripper': (0.,), 'home': (None, None), 'prepare': (None, None),
                     'submit_catch_up': (None, 92)}[action]
        with pytest.raises(BackendCommandError, match='finite_diagnostic_action_not_supported'):
            await getattr(adapter, action)(*arguments)
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_fk_timeout_does_not_wait_for_a_cancellation_ignoring_read():
    adapter, client, _, events = await setup()
    release = asyncio.Event()

    async def ignores_cancel(index, result):
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        return result

    client.fk_hook = ignores_cancel
    try:
        receipt = await adapter.submit_tcp(_target(.3002), 91)
        result = await asyncio.wait_for(receipt.completion.wait(), .4)
        assert result.sdk_outcome == 'not_started' and result.error == 'finite_fk_timeout'
        assert client.write_calls == [('stop_move',)] and len(client.fk_calls) == 1
        assert adapter._finite_pending_fk is not None and not adapter._finite_pending_fk.done()
        with pytest.raises(BackendCommandError, match='finite_fk_read_pending'):
            await adapter.connect()
        await _wait_until(lambda: any(e['kind'] == 'finite_candidate_audit' for e in events))
        call = next(e for e in events if e['kind'] == 'finite_candidate_audit')['calls'][0]
        assert call['outcome'] == 'timeout' and call['remote_result_unknown'] is True
        frozen_calls = copy.deepcopy(next(e for e in events if e['kind'] == 'finite_candidate_audit')['calls'])
        await asyncio.wait_for(adapter.disconnect(), .3)
        assert adapter._finite_ticket is None
        release.set()
        await _wait_until(lambda: adapter._finite_pending_fk is None)
        assert next(e for e in events if e['kind'] == 'finite_candidate_audit')['calls'] == frozen_calls
        assert adapter._accepted_target_ref is None
    finally:
        release.set()
        await adapter.disconnect()


async def test_mutable_snapshot_change_during_fk_cannot_rebind_input_evidence():
    adapter, client, _, _ = await setup()

    async def mutate(index, result):
        if index == 4:
            pose = adapter._snapshot.actual_tcp
            pose.p = (pose.p[0] + .00001, *pose.p[1:])
        return result

    client.fk_hook = mutate
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'not_started' and result.error == 'finite_audit_input_changed'
        assert client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


async def test_initial_acceleration_is_rejected_and_routes_to_original_stop():
    adapter, client, _, _ = await setup()
    client.kin_data['actual_joint_acc'][0] = .2876213977285576
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'not_started' and result.error == 'finite_pvat_initial_acceleration_limit'
        assert not client.fk_calls and client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('failure', ['error', 'timeout'])
async def test_fk_failure_and_failed_stop_keep_original_unverified_latch(failure):
    adapter, client, _, _ = await setup()

    async def fails(index, result):
        if failure == 'timeout':
            await asyncio.Event().wait()
        raise RuntimeError('injected_fk_error')

    client.fk_hook = fails
    original = client.stop_move

    async def late_stop():
        await original()
        await asyncio.sleep(.23)

    client.stop_move = late_stop
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'not_started' and result.error == 'stop_unverified'
        assert adapter._latched_fault == 'stop_unverified'
        assert client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


async def test_queued_request_expiry_never_restarts_lease_at_handler_entry():
    adapter, client, clock, _ = await setup()
    try:
        receipt = await adapter.submit_tcp(_target(.3002), 91)
        clock.value = 100_000_000
        result = await asyncio.wait_for(receipt.completion.wait(), 2)
        assert result.sdk_outcome == 'not_started' and result.error == 'finite_diagnostic_expired'
        assert not client.fk_calls and client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


async def test_fk_method_returning_future_is_supported_without_native_import():
    adapter, client, _, _ = await setup()

    def future_result(joints):
        client.fk_calls.append(tuple(joints))
        future = asyncio.get_running_loop().create_future()
        future.set_result({**ACTUAL_TCP, 'x': ACTUAL_TCP['x'] + joints[0]})
        return future

    client.kinematics_forward = future_result
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'returned' and len(client.fk_calls) == 4
        assert [c[0] for c in client.write_calls] == ['move_pvat']
    finally:
        await adapter.disconnect()


async def test_fk_child_cannot_enter_sdk_after_lease_expires(monkeypatch):
    adapter, client, clock, events = await setup()
    original = asyncio.create_task

    def delayed(coroutine, **kwargs):
        if kwargs.get('name') == 'finite-forward-kinematics':
            clock.value = 100_000_000
        return original(coroutine, **kwargs)

    monkeypatch.setattr(asyncio, 'create_task', delayed)
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'not_started' and result.error == 'finite_diagnostic_expired'
        assert not client.fk_calls and client.write_calls == [('stop_move',)]
        await _wait_until(lambda: any(e['kind'] == 'finite_candidate_audit' for e in events))
        call = next(e for e in events if e['kind'] == 'finite_candidate_audit')['calls'][0]
        assert call['outcome'] == 'not_started'
    finally:
        await adapter.disconnect()


async def test_revocation_on_pvat_return_still_requires_shutdown_stop():
    adapter, client, _, _ = await setup()
    allowed = [True]
    adapter.bind_motion_authority(lambda: allowed[0])
    original = client.move_pvat

    async def revoke_on_return(*args):
        value = await original(*args)
        allowed[0] = False
        return value

    client.move_pvat = revoke_on_return
    try:
        result = await submit(adapter)
        assert result.sdk_outcome == 'returned' and result.invalidated
        assert adapter._accepted_target_ref is None
        await adapter.disconnect()
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
        await adapter.disconnect()
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
    finally:
        await adapter.disconnect()
