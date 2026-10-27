"""Bounded math reads use the same trusted entry evidence as state reads."""
import asyncio
import pytest
from app.control.finite_position_types import PositionIntent
from app.robots.base import BackendCommandError
from app.robots.finite_position_adapter import FinitePositionAdapter
from tests.robots.finite_position_fixture import fixture


async def rig():
    settings,client,clock=fixture()
    async def factory(ip):assert ip=='digital-twin';return client
    adapter=FinitePositionAdapter(settings,client_factory=factory,backend_label='LEBAI_FAKE',
        position_model=client.model,clock=clock.now_ns,sleep=clock.sleep)
    await adapter.connect();adapter.bind_position_generation(1)
    return adapter,client,clock


@pytest.mark.parametrize('method',['kinematics_inverse','kinematics_forward'])
@pytest.mark.parametrize('evidence',['local','native_unknown','missing'])
async def test_cancelled_math_only_quarantines_possible_native_entry(method,evidence):
    adapter,client,clock=await rig();entered=asyncio.Event();release=asyncio.Event();count=0
    if evidence!='missing':
        client._native_snapshot_entry_count=lambda name:count if name==method else 0
        client._native_snapshot_result_unknown=lambda name:evidence=='native_unknown'
    original=getattr(client,method)
    async def delayed(*args):
        nonlocal count
        if evidence=='native_unknown':count+=1
        entered.set();await release.wait();return await original(*args)
    setattr(client,method,delayed)
    intent=PositionIntent('cancel-math',1,1,clock.now_ns(),'held',(1.,0.,0.))
    task=asyncio.create_task(adapter.plan_step(intent,authorize=lambda:None))
    try:
        await asyncio.wait_for(entered.wait(),1);task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        await asyncio.sleep(0)
        assert client.write_calls==[] and adapter._position_plan is None
        if evidence=='local':
            assert adapter._snapshot_calls.unknown_methods==set()
            state=await adapter.get_state();assert state.fault is None
        else:
            assert method in adapter._snapshot_calls.unknown_methods
            with pytest.raises(BackendCommandError,match='snapshot_rpc_result_unknown'):
                await adapter.get_state()
        record=adapter._snapshot_calls.last[method]
        assert record['remote_result_unknown'] is (evidence!='local')
    finally:
        release.set();task.cancel();await asyncio.gather(task,return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.parametrize('backend_flag,client_flag,method',[
    (False,True,'kinematics_inverse'),('true',True,'kinematics_inverse'),
    (True,False,'kinematics_inverse'),(True,'true','kinematics_inverse'),
    (True,True,'get_motion_state')])
async def test_nonlocal_math_and_missing_capabilities_keep_original_read_serialization(backend_flag,client_flag,method):
    adapter,client,_clock=await rig();adapter.position_local_math_parallel=backend_flag
    client._position_pure_local_kinematics=client_flag
    entered=asyncio.Event();release=asyncio.Event();read_entered=asyncio.Event()
    original=client.is_connected
    async def connected():read_entered.set();return await original()
    async def math():entered.set();await release.wait();return 19
    client.is_connected=connected
    task=asyncio.create_task(adapter._position_math(method,math,lambda:None));reader=None
    try:
        await entered.wait();reader=asyncio.create_task(adapter.get_state())
        for _ in range(12):await asyncio.sleep(0)
        assert not read_entered.is_set()
        release.set();assert await task==19
        assert (await reader).fault is None
    finally:
        release.set();await asyncio.gather(task,*([] if reader is None else [reader]),return_exceptions=True)
        await adapter.disconnect()


async def test_pure_local_math_still_serializes_math_with_math():
    adapter,client,_clock=await rig();client._position_pure_local_kinematics=True
    first_entered=asyncio.Event();release=asyncio.Event();second_entered=asyncio.Event()
    async def first():first_entered.set();await release.wait();return 1
    async def second():second_entered.set();return 2
    task=asyncio.create_task(adapter._position_math('kinematics_inverse',first,lambda:None));other=None
    try:
        await first_entered.wait();other=asyncio.create_task(adapter._position_math('kinematics_forward',second,lambda:None))
        for _ in range(12):await asyncio.sleep(0)
        assert not second_entered.is_set()
        release.set();assert await task==1 and await other==2
    finally:
        release.set();await asyncio.gather(task,*([] if other is None else [other]),return_exceptions=True)
        await adapter.disconnect()


async def test_late_native_math_reply_does_not_clear_unknown_ownership():
    adapter,client,_clock=await rig();entered=asyncio.Event();release=asyncio.Event();count=0
    client._native_snapshot_entry_count=lambda _name:count
    client._native_snapshot_result_unknown=lambda _name:True
    async def late():
        nonlocal count
        count+=1;entered.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return 23
    task=asyncio.create_task(adapter._position_math('kinematics_inverse',late,lambda:None))
    try:
        await entered.wait();task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        release.set();await asyncio.gather(*tuple(adapter._snapshot_calls.tasks),return_exceptions=True)
        assert adapter._snapshot_calls.unknown_methods=={'kinematics_inverse'}
        with pytest.raises(BackendCommandError,match='snapshot_rpc_result_unknown'):await adapter.get_state()
        assert client.write_calls==[]
    finally:release.set();await adapter.disconnect()


@pytest.mark.parametrize('damage',['client','owner','generation'])
async def test_math_result_cannot_publish_after_connection_ownership_changes(damage):
    from app.robots.lebai_snapshot_calls import OwnedSnapshotCalls
    adapter,client,_clock=await rig();owner=adapter._snapshot_calls
    async def obsolete():
        if damage=='client':adapter._client=object()
        elif damage=='owner':adapter._snapshot_calls=OwnedSnapshotCalls()
        else:adapter._pump.invalidate()
        return 29
    try:
        with pytest.raises((BackendCommandError,TimeoutError)):
            await adapter._position_math('kinematics_inverse',obsolete,lambda:None)
        assert client.write_calls==[]
    finally:
        adapter._client=client;adapter._snapshot_calls=owner
        await adapter.disconnect()


@pytest.mark.parametrize('baseline',[True,-1,'0',None])
async def test_invalid_math_entry_evidence_rejects_before_call(baseline):
    adapter,client,_clock=await rig();calls=[]
    client._native_snapshot_entry_count=lambda _name:baseline
    async def operation():calls.append(1)
    try:
        with pytest.raises(BackendCommandError,match='invalid_snapshot_entry_evidence'):
            await adapter._position_math('kinematics_inverse',operation,lambda:None)
        assert calls==[] and client.write_calls==[]
    finally:
        del client._native_snapshot_entry_count
        await adapter.disconnect()


@pytest.mark.parametrize('method',['kinematics_inverse','kinematics_forward','get_motion_state'])
async def test_math_timeout_with_local_entry_evidence_does_not_poison_next_snapshot(method):
    adapter,client,_clock=await rig();entered=asyncio.Event();release=asyncio.Event()
    client._native_snapshot_entry_count=lambda _name:0
    client._native_snapshot_result_unknown=lambda _name:False
    async def blocked():entered.set();await release.wait()
    owner=adapter._snapshot_calls
    try:
        with pytest.raises(TimeoutError):
            await adapter._position_math(method,blocked,lambda:None)
        await asyncio.sleep(0)
        assert entered.is_set() and owner.unknown_methods==set()
        assert (await adapter.get_state()).fault is None
        assert client.write_calls==[]
    finally:release.set();await adapter.disconnect()


@pytest.mark.parametrize('method',['kinematics_inverse','kinematics_forward'])
async def test_explicit_pure_local_math_keeps_feedback_readable_while_it_waits(method):
    adapter,client,_clock=await rig()
    client._position_pure_local_kinematics=True
    entered=asyncio.Event();release=asyncio.Event();read_entered=asyncio.Event()
    original=client.is_connected
    async def connected():read_entered.set();return await original()
    async def math():entered.set();await release.wait();return 17
    client.is_connected=connected
    task=asyncio.create_task(adapter._position_math(method,math,lambda:None))
    reader=None
    try:
        await entered.wait();reader=asyncio.create_task(adapter.get_state())
        for _ in range(12):await asyncio.sleep(0)
        assert read_entered.is_set(),'pure local math must not own the SDK feedback lock'
        release.set();assert await task==17
        assert (await reader).fault is None and client.write_calls==[]
    finally:
        release.set();await asyncio.gather(task,*([] if reader is None else [reader]),return_exceptions=True)
        await adapter.disconnect()
