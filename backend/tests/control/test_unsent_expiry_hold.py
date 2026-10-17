"""Offline candidate: an unsent expired segment cannot turn into a retry."""
import asyncio
from dataclasses import replace
import pytest
from app.schemas.messages import TeleopMode
from app.control.target_strategy import TargetUpdate
from tests.control.test_target_lifecycle import setup
from tests.control.test_robot_control import frame
from tests.control.test_segmented_control import rig,send,observe
from tests.robots.test_lebai_adapter_control import _target


async def test_owner_holds_stops_and_requires_release_before_reactivation():
    control,life,backend,clock,latest=await setup()
    life.update=TargetUpdate(control.control_generation,hold=True)
    await control.tick()
    assert control.mode is TeleopMode.HOLD and control._fault is None
    assert len(backend.stops)==1 and backend.targets==[]
    life.update=None
    for seq in range(3,6):
        latest.publish(frame(seq,True),clock.now_ns());await control.tick()
        assert control.mode is TeleopMode.HOLD
    latest.publish(frame(6,False),clock.now_ns());await control.tick()
    latest.publish(frame(7,True),clock.now_ns());await control.tick()
    assert control.mode is TeleopMode.ACTIVE and len(life.anchors)==2


async def test_hold_stop_failure_preserves_stop_unverified():
    from app.robots.base import BackendCommandError
    control,life,backend,_,_=await setup()
    async def fail(reason):raise BackendCommandError('sdk_timeout:stop_move')
    backend.stop=fail;life.update=TargetUpdate(control.control_generation,hold=True)
    await control.tick()
    assert control._fault=='stop_unverified' and control.mode is TeleopMode.FAULT
    assert not (await control.reset_fault()).accepted


async def test_cancelled_hold_recording_cannot_leave_an_unlatched_hold():
    control,life,backend,_,_=await setup();entered=asyncio.Event()
    original=control._record_critical
    async def record(kind,payload):
        if kind=='target_hold':entered.set();await asyncio.Event().wait()
        return await original(kind,payload)
    control._record_critical=record
    life.update=TargetUpdate(control.control_generation,hold=True)
    task=asyncio.create_task(control.tick());await asyncio.wait_for(entered.wait(),1)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):await task
    assert control._fault=='stop_unverified' and control.mode is TeleopMode.FAULT
    assert control._pending_stop_completion and backend.targets==[]


@pytest.mark.parametrize('hold', [False,True])
async def test_expiry_after_settled_segment_has_no_new_write_and_no_automatic_retry(hold):
    adapter,client,clock,box,strategy=await rig(allow_unsent_expiry_hold=hold)
    try:
        await send(strategy,adapter,_target(.31),1);strategy.collect(box[0])
        clock.advance_ms(80)
        for _ in range(3):await observe(adapter,client,clock,box,strategy)
        assert adapter.read_segment_status().ready
        before=list(client.write_calls)
        async def expire(count,pose):clock.advance_ms(101);return pose
        client.fk_hook=expire
        done=await send(strategy,adapter,_target(.32),2)
        assert done.sdk_outcome=='not_started'
        update=strategy.collect(box[0])
        if hold:
            assert update.hold and update.fault is None
            assert client.write_calls==before
            assert adapter._latched_fault is None
            for _ in range(5):
                plan=strategy.prepare(_target(.3),_target(.32),.02,limiter=strategy.limits)
                assert plan.kind=='wait'
                await strategy.dispatch(plan,3,backend=adapter)
            assert client.write_calls==before
            from app.robots.base import StopReason
            await adapter.stop(StopReason.GRIP_RELEASED)
            assert (await adapter.preflight()).ready
            client.fk_hook=None
            box[0]=replace(box[0],generation=box[0].generation+1,now_ns=clock.now_ns())
            strategy.on_anchor((await adapter.get_state()).actual_tcp,box[0])
            resumed=await send(strategy,adapter,_target(.32),4)
            assert resumed.sdk_outcome=='returned' and resumed.error is None
            assert [c[0] for c in client.write_calls[len(before):]]==['stop_move','move_pvat']
        else:
            assert update.fault and not update.hold
            assert client.write_calls[-1][0]=='stop_move'
    finally:await adapter.disconnect()


@pytest.mark.parametrize('boundary',['first_segment','entered','unknown_motion','unknown_read','known_read_fault','pending_fk','pending_read','record_changed','other_failure'])
async def test_incomplete_proof_keeps_the_original_fault_path(boundary):
    from types import SimpleNamespace
    adapter,client,clock,box,strategy=await rig(allow_unsent_expiry_hold=True)
    pending=None
    try:
        if boundary!='first_segment':
            await send(strategy,adapter,_target(.31),1);strategy.collect(box[0]);clock.advance_ms(80)
            for _ in range(3):await observe(adapter,client,clock,box,strategy)
        async def expire(count,pose):
            nonlocal pending
            if boundary=='entered':adapter._finite_entered=True
            elif boundary=='unknown_motion':adapter._motion_calls.unknown_methods.add('move_pvat')
            elif boundary=='unknown_read':adapter._snapshot_calls.unknown_methods.add('get_kin_data')
            elif boundary=='known_read_fault':
                adapter._snapshot_read_fault=(SimpleNamespace(client=client,generation=adapter._pump.generation),'tcp_mismatch')
            elif boundary=='pending_fk':
                pending=asyncio.create_task(asyncio.Event().wait());adapter._finite_fk_wrappers.add(pending)
            elif boundary=='pending_read':adapter._snapshot_calls.active+=1
            elif boundary=='record_changed':adapter._motion_calls.last['move_pvat']={}
            if boundary=='other_failure':return {**pose,'y':pose['y']+.1}
            clock.advance_ms(101);return pose
        client.fk_hook=expire
        await send(strategy,adapter,_target(.32),2)
        update=strategy.collect(box[0])
        assert update.fault is not None and not update.hold
        assert not adapter._segment['unsent_expired']
    finally:
        if boundary=='pending_read':adapter._snapshot_calls.active=0
        if pending is not None:
            pending.cancel();await asyncio.gather(pending,return_exceptions=True)
        await adapter.disconnect()


def test_hold_candidate_cannot_be_enabled_for_a_real_backend():
    from app.robots.segmented_adapter import SegmentedLebaiAdapter
    from tests.control.test_segmented_control import OPTIONS
    from tests.robots.real_settings import control_settings
    with pytest.raises(ValueError,match='unsent_expiry_hold_offline_only'):
        SegmentedLebaiAdapter(control_settings(),settling=OPTIONS,client_factory=None,allow_unsent_expiry_hold=True)


@pytest.mark.parametrize('runtime,flag',[('LEBAI',True),('LEBAI_FAKE',1),('SIMULATOR',True)])
def test_application_selection_rejects_invalid_or_nonoffline_candidate(runtime,flag):
    from app.main import build_backend
    from app.digital_twin.segmented_demo import demo_settings
    from app.recording.noop import NoopRecorder
    with pytest.raises(ValueError,match='unsent_expiry_hold'):
        build_backend(demo_settings(),NoopRecorder(),None,runtime,offline_unsent_expiry_hold=flag)


@pytest.mark.parametrize('other',[dict(stop=True),dict(fault='bad'),dict(committed=_target(.3))])
def test_hold_cannot_be_combined_with_other_lifecycle_decisions(other):
    with pytest.raises(ValueError,match='conflicting_target_update'):TargetUpdate(1,hold=True,**other)
