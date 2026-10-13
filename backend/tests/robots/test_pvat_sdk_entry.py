"""Actual adapter with fake client/clocks; no real execution or stopping proof."""
import asyncio

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_ik_policy import evaluate_ik_candidate
from app.robots.lebai_pump import PvatRequest
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target
from tests.robots.test_lebai_backpressure_experiments import wait_until


async def setup(monkeypatch, hook):
    adapter,client,clock=await _connected_control_adapter()
    events=[]
    async def record(event,_time):events.append(event)
    async def select(_client,snapshot,request,*,diagnostic_events):
        solution=(.024,*snapshot.actual_q[1:])
        metrics=evaluate_ik_candidate(solution,snapshot.actual_q,previous_solution_q=solution)
        return adapter._build_advancing_candidate(snapshot,request.target,metrics,1.)
    original=asyncio.wait_for
    async def delayed(awaitable,timeout):
        if getattr(getattr(awaitable,'cr_code',None),'co_name',None)=='send_if_authorized':
            hook(adapter,client,clock)
        return await original(awaitable,timeout)
    adapter._event_callback=record
    monkeypatch.setattr(adapter,'_select_advancing_candidate',select)
    monkeypatch.setattr(asyncio,'wait_for',delayed)
    await adapter._pump.start()
    client.write_calls.clear();client.read_calls.clear()
    return adapter,client,clock,events


@pytest.mark.asyncio
@pytest.mark.parametrize('excess_ns',[1,1_000_000])
async def test_snapshot_expiring_when_sdk_task_starts_prevents_native_write(monkeypatch,excess_ns):
    def hook(adapter,client,clock):clock.value+=adapter._command_snapshot_max_age_ns()+excess_ns
    adapter,client,clock,events=await setup(monkeypatch,hook)
    before=(adapter._last_pvat_started_ns,adapter._last_pvat_clock_started_ns,adapter._previous_sent_qd,adapter._accepted_target_ref)
    try:
        with pytest.raises(BackendCommandError,match='^robot_state_stale$'):
            await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        await wait_until(lambda:any(e['kind']=='sdk_request_lifecycle' for e in events))
        assert not client.write_calls
        assert not any(e['kind']=='pvat_sent' for e in events)
        assert before==(adapter._last_pvat_started_ns,adapter._last_pvat_clock_started_ns,adapter._previous_sent_qd,adapter._accepted_target_ref)
        entry=next(e for e in events if e['kind']=='sdk_request_lifecycle')
        assert entry['rpc_outcome']=='not_started' and entry['revocation_reason']=='robot_state_stale'
        assert client.read_calls.count('get_kin_data')==1
    finally:await adapter.disconnect()


@pytest.mark.asyncio
async def test_snapshot_age_boundary_remains_inclusive_without_extra_reads(monkeypatch):
    def hook(adapter,client,clock):clock.value+=adapter._command_snapshot_max_age_ns()
    adapter,client,clock,events=await setup(monkeypatch,hook)
    try:
        await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        assert [x[0] for x in client.write_calls]==['move_pvat']
        assert client.read_calls.count('get_kin_data')==1
    finally:await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('age_ns',[99_999_999,100_000_000,100_000_001])
async def test_original_vr_authority_is_rechecked_after_task_scheduling(monkeypatch,age_ns):
    def hook(adapter,client,clock):clock.value+=age_ns
    adapter,client,clock,events=await setup(monkeypatch,hook)
    frame_ns=clock.now_ns()
    allowed=[True]
    adapter.bind_motion_authority(lambda:allowed[0] and 0<=clock.now_ns()-frame_ns<100_000_000)
    try:
        await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        await wait_until(lambda:any(e['kind']=='sdk_request_lifecycle' for e in events))
        if age_ns<100_000_000:
            assert [x[0] for x in client.write_calls]==['move_pvat']
        else:
            assert not client.write_calls and adapter._last_pvat_started_ns is None
            assert next(e for e in events if e['kind']=='sdk_request_lifecycle')['rpc_outcome']=='not_started'
    finally:
        allowed[0]=True
        # Retain the same authority binding; cleanup doesn't dispatch motion.
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_expiry_during_reference_rebuild_still_prevents_sdk_entry(monkeypatch):
    from app.robots import lebai_adapter as module
    reference=[0]
    def hook(adapter,client,clock):reference[0]=90_000_000
    adapter,client,clock,events=await setup(monkeypatch,hook)
    adapter._pvat_clock=lambda:reference[0]
    adapter._last_pvat_started_ns=clock.now_ns()
    adapter._last_pvat_clock_started_ns=0
    adapter._previous_sent_qd=(.12,0.,0.,0.,0.,0.)
    original=module.build_pvat_point
    rebuilding=[False]
    def slow_build(*args,**kwargs):
        point=original(*args,**kwargs)
        if reference[0]>=90_000_000:
            rebuilding[0]=True
            clock.value+=adapter._command_snapshot_max_age_ns()+1
        return point
    monkeypatch.setattr(module,'build_pvat_point',slow_build)
    before=(adapter._last_pvat_started_ns,adapter._last_pvat_clock_started_ns,adapter._previous_sent_qd)
    try:
        with pytest.raises(BackendCommandError,match='^robot_state_stale$'):
            await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        assert rebuilding[0] and not client.write_calls
        assert before==(adapter._last_pvat_started_ns,adapter._last_pvat_clock_started_ns,adapter._previous_sent_qd)
    finally:await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('request_kind',['target','catch_up'])
async def test_authority_expiring_during_sdk_reply_does_not_publish_new_reference_history(monkeypatch,request_kind):
    from app.robots.command_admission import CommandCompletion,CommandRef
    adapter,client,clock,events=await setup(monkeypatch,lambda *_:None)
    frame_ns=clock.now_ns()
    adapter.bind_motion_authority(lambda:0<=clock.now_ns()-frame_ns<100_000_000)
    original=client.move_pvat
    async def send(*args):
        result=await original(*args)
        clock.advance_ms(100)
        return result
    monkeypatch.setattr(client,'move_pvat',send)
    accepted=None
    if request_kind=='catch_up':
        accepted=CommandRef(70,adapter._pump.generation,1)
        adapter._accepted_target_ref=accepted
        adapter._accepted_target_command_id=70
        adapter._last_sent_tcp=_target(.301)
        adapter._last_accepted_solution_q=(.024,*adapter._snapshot.actual_q[1:])
    before=(adapter._last_pvat_started_ns,adapter._last_pvat_clock_started_ns,adapter._previous_sent_qd,adapter._accepted_target_ref)
    completion=CommandCompletion(CommandRef(71,adapter._pump.generation,2))
    try:
        await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation,submission_id=2,completion=completion,catch_up_from=accepted))
        await wait_until(lambda:any(e['kind']=='sdk_request_lifecycle' for e in events))
        assert [x[0] for x in client.write_calls]==['move_pvat']
        assert before==(adapter._last_pvat_started_ns,adapter._last_pvat_clock_started_ns,adapter._previous_sent_qd,adapter._accepted_target_ref)
        assert not any(e['kind']=='pvat_sent' for e in events)
        entry=next(e for e in events if e['kind']=='sdk_request_lifecycle')
        assert entry['rpc_outcome']=='returned' and entry['invalidated'] is True
        assert completion.result.outcome=='invalidated' and completion.result.invalidated is True
        assert completion.result.sdk_outcome=='returned'
        assert completion.result.sdk_request_id==entry['request_id']
        assert completion.result.error=='teleop_motion_authority_revoked'
    finally:await adapter.disconnect()


@pytest.mark.asyncio
async def test_cached_fault_arriving_at_sdk_task_uses_original_stop_after_lock_release(monkeypatch):
    def hook(adapter,client,clock):
        adapter._remember_snapshot_fault(adapter._latest_snapshot_read,'robot_state_fault')
    adapter,client,clock,events=await setup(monkeypatch,hook)
    lock_states=[]
    original=adapter._stop_for_motion_fault
    async def stop(fault):
        lock_states.append(adapter._sdk_lock.locked())
        return await original(fault)
    monkeypatch.setattr(adapter,'_stop_for_motion_fault',stop)
    try:
        with pytest.raises(BackendCommandError,match='^robot_state_fault$'):
            await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        await wait_until(lambda:any(e['kind']=='sdk_request_lifecycle' for e in events))
        assert lock_states==[False]
        assert [x[0] for x in client.write_calls]==['stop_move']
        entry=next(e for e in events if e['kind']=='sdk_request_lifecycle')
        assert entry['rpc_outcome']=='not_started' and entry['revocation_reason']=='robot_state_fault'
        assert not any(e['kind']=='pvat_sent' for e in events)
    finally:await adapter.disconnect()


@pytest.mark.asyncio
async def test_client_replacement_before_sdk_is_not_reported_as_native_failure(monkeypatch):
    def hook(adapter,client,clock):adapter._client=None
    adapter,client,clock,events=await setup(monkeypatch,hook)
    try:
        with pytest.raises(BackendCommandError,match='^robot_disconnected$'):
            await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        await wait_until(lambda:any(e['kind']=='sdk_request_lifecycle' for e in events))
        assert not client.write_calls and adapter._last_pvat_started_ns is None
        assert next(e for e in events if e['kind']=='sdk_request_lifecycle')['rpc_outcome']=='not_started'
    finally:
        adapter._client=client
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_backend_command_error_from_actual_sdk_call_keeps_original_failure_code(monkeypatch):
    adapter,client,clock,events=await setup(monkeypatch,lambda *_:None)
    original=client.move_pvat
    async def failed(*args):
        await original(*args)
        raise BackendCommandError('fake_native_failure')
    monkeypatch.setattr(client,'move_pvat',failed)
    try:
        with pytest.raises(BackendCommandError,match='^sdk_call_failed:move_pvat$'):
            await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        await wait_until(lambda:any(e['kind']=='sdk_request_lifecycle' for e in events))
        assert [x[0] for x in client.write_calls]==['move_pvat']
        assert adapter._last_pvat_started_ns is None
        assert next(e for e in events if e['kind']=='sdk_request_lifecycle')['rpc_outcome']=='error'
    finally:await adapter.disconnect()


@pytest.mark.asyncio
async def test_new_cached_sample_cannot_extend_original_motion_read_at_sdk_task(monkeypatch):
    from dataclasses import replace
    def hook(adapter,client,clock):
        clock.value+=adapter._command_snapshot_max_age_ns()+1
        adapter._snapshot=replace(adapter._snapshot,captured_ns=clock.now_ns(),read_completed_ns=clock.now_ns())
        adapter._latest_snapshot_read.snapshot=adapter._snapshot
    adapter,client,clock,events=await setup(monkeypatch,hook)
    try:
        with pytest.raises(BackendCommandError,match='^robot_state_stale$'):
            await adapter._send_target(PvatRequest(_target(.301),71,adapter._pump.generation))
        assert not client.write_calls and adapter._last_pvat_started_ns is None
    finally:await adapter.disconnect()
