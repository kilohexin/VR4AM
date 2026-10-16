"""Only synthetic control objects; no robot or SDK imports."""
import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock
import pytest

async def test_release_can_revoke_claw_while_preflight_is_awaiting():
    from app.api.operator_jobs import OperatorJobs
    entered,finish=asyncio.Event(),asyncio.Event()
    writes=[]
    async def claw(value,*,is_current):
        entered.set();await finish.wait()
        if not is_current():raise RuntimeError('revoked')
        writes.append(value)
    control=SimpleNamespace(command_gripper=claw,_enter_fault=AsyncMock())
    send=AsyncMock();jobs=OperatorJobs(control,send)
    message=SimpleNamespace(request_id='a',value=.51)
    await jobs.gripper(message);await entered.wait()
    jobs.cancel_gripper('wrong');assert jobs.gripper_current
    jobs.cancel_gripper('a');finish.set();await jobs.join()
    assert writes==[] and send.await_args.args[0]['accepted'] is False
    await jobs.close()

async def test_claw_has_no_pending_queue_or_request_id_replay():
    from app.api.operator_jobs import OperatorJobs
    entered,finish=asyncio.Event(),asyncio.Event()
    async def claw(value,*,is_current):entered.set();await finish.wait()
    control=SimpleNamespace(command_gripper=claw,_enter_fault=AsyncMock())
    send=AsyncMock();jobs=OperatorJobs(control,send)
    a=SimpleNamespace(request_id='a',value=.51);b=SimpleNamespace(request_id='b',value=.52)
    await jobs.gripper(a);await entered.wait();await jobs.gripper(b)
    assert send.await_args.args[0]['message']=='request_busy'
    finish.set();await jobs.join();await jobs.gripper(a)
    assert send.await_args.args[0]['message']=='duplicate_request'
    await jobs.close()

async def test_disconnect_revokes_an_unentered_claw_write():
    from app.api.operator_jobs import OperatorJobs
    entered,finish=asyncio.Event(),asyncio.Event();writes=[]
    async def claw(value,*,is_current):
        entered.set();await finish.wait()
        if is_current():writes.append(value)
    jobs=OperatorJobs(SimpleNamespace(command_gripper=claw,_enter_fault=AsyncMock()),AsyncMock())
    await jobs.gripper(SimpleNamespace(request_id='a',value=.51));await entered.wait()
    closing=asyncio.create_task(jobs.close());await asyncio.sleep(0);finish.set();await closing
    assert writes==[]

async def test_preparing_does_not_block_receiver_side_stop_or_accept_duplicate_arm():
    from app.api.operator_jobs import OperatorJobs
    entered,finish=asyncio.Event(),asyncio.Event()
    async def arm():entered.set();await finish.wait()
    control=SimpleNamespace(arm=arm,_enter_fault=AsyncMock())
    send=AsyncMock();jobs=OperatorJobs(control,send)
    await jobs.arm(SimpleNamespace(request_id='a'));await entered.wait()
    await jobs.arm(SimpleNamespace(request_id='b'))
    assert send.await_args.args[0]['type']=='arm_rejected'
    finish.set();await jobs.join();assert send.await_args.args[0]['type']=='arm_ack'
    await jobs.close()

async def test_explicit_prepare_is_a_distinct_owned_job_and_rejects_replay():
    from app.api.operator_jobs import OperatorJobs
    entered,finish=asyncio.Event(),asyncio.Event()
    async def prepare():entered.set();await finish.wait()
    control=SimpleNamespace(prepare=prepare,arm=AsyncMock())
    send=AsyncMock();jobs=OperatorJobs(control,send)
    await jobs.prepare(SimpleNamespace(request_id='p'));await entered.wait()
    await jobs.arm(SimpleNamespace(request_id='unlock'))
    assert send.await_args.args[0]['type']=='arm_rejected'
    finish.set();await jobs.join()
    assert send.await_args.args[0]['type']=='prepare_ack'
    await jobs.prepare(SimpleNamespace(request_id='p'))
    assert send.await_args.args[0]['type']=='prepare_rejected'
    control.arm.assert_not_awaited();await jobs.close()

async def test_websocket_receives_cancel_while_claw_read_is_pending():
    from app.api.teleop_ws import _receive_messages
    from fastapi import WebSocketDisconnect
    entered,finish,pong=asyncio.Event(),asyncio.Event(),asyncio.Event()
    queue=asyncio.Queue();writes=[];sent=[]
    async def claw(value,*,is_current):
        entered.set();await finish.wait()
        if not is_current():raise RuntimeError('revoked')
        writes.append(value)
    async def receive():
        value=await queue.get()
        if value is None:raise WebSocketDisconnect()
        return value
    async def send(value):
        sent.append(value)
        if value['type']=='pong':pong.set()
    control=SimpleNamespace(command_gripper=claw,_enter_fault=AsyncMock())
    socket=SimpleNamespace(app=SimpleNamespace(state=object()),receive_json=receive,send_json=send)
    task=asyncio.create_task(_receive_messages(socket,control,asyncio.Event(),asyncio.Lock()))
    try:
        await queue.put(dict(v=1,type='gripper_request',request_id='c',value=.51))
        await asyncio.wait_for(entered.wait(),1)
        await queue.put(dict(v=1,type='gripper_cancel',request_id='c'))
        await queue.put(dict(v=1,type='ping',request_id='p'))
        await asyncio.wait_for(pong.wait(),1)
        finish.set();await queue.put(None);await asyncio.wait_for(task,1)
        assert writes==[]
    finally:
        finish.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True)

async def test_already_finished_job_is_joined_without_waiting_for_a_callback():
    from app.api.operator_jobs import OperatorJobs
    jobs=OperatorJobs(SimpleNamespace(),AsyncMock())
    done=asyncio.create_task(asyncio.sleep(0));await done
    jobs._tasks.add(done)
    await jobs.close()
    assert jobs._tasks==set()

async def test_automation_keeps_its_exclusive_claw_authority():
    from app.api.operator_jobs import OperatorJobs
    control=SimpleNamespace(command_gripper=AsyncMock(),_enter_fault=AsyncMock())
    send=AsyncMock();jobs=OperatorJobs(control,send,allowed=lambda:False)
    await jobs.gripper(SimpleNamespace(request_id='a',value=.51))
    control.command_gripper.assert_not_awaited()
    assert send.await_args.args[0]['message']=='automation_active'
    await jobs.close()

async def test_removed_rehearsal_cannot_start_through_an_old_client():
    from app.api.teleop_ws import _handle_rehearsal_message
    from app.schemas.messages import OfflineRehearsalBeginMessage
    socket=SimpleNamespace(send_json=AsyncMock(),app=SimpleNamespace(state=object()))
    message=OfflineRehearsalBeginMessage(v=1,type='offline_rehearsal_begin',request_id='old',plan_version=1)
    await _handle_rehearsal_message(socket,SimpleNamespace(),object(),message,asyncio.Lock())
    assert socket.send_json.await_args.args[0]['reason']=='feature_removed'

async def test_websocket_receives_stop_while_explicit_preparation_is_pending():
    from app.api.teleop_ws import _receive_messages
    from fastapi import WebSocketDisconnect
    entered,finish,pong=asyncio.Event(),asyncio.Event(),asyncio.Event()
    queue=asyncio.Queue();sent=[]
    async def prepare():entered.set();await finish.wait()
    async def receive():
        value=await queue.get()
        if value is None:raise WebSocketDisconnect()
        return value
    async def send(value):
        sent.append(value)
        if value['type']=='pong':pong.set()
    control=SimpleNamespace(prepare=prepare,disarm=AsyncMock())
    socket=SimpleNamespace(app=SimpleNamespace(state=object()),receive_json=receive,send_json=send)
    task=asyncio.create_task(_receive_messages(socket,control,asyncio.Event(),asyncio.Lock()))
    try:
        await queue.put(dict(v=1,type='prepare_request',request_id='prepare'))
        await asyncio.wait_for(entered.wait(),1)
        await queue.put(dict(v=1,type='disarm',request_id='stop'))
        await queue.put(dict(v=1,type='ping',request_id='ping'))
        await asyncio.wait_for(pong.wait(),1)
        control.disarm.assert_awaited_once()
        finish.set();await queue.put(None);await asyncio.wait_for(task,1)
        assert any(x['type']=='disarm_ack' for x in sent)
    finally:
        finish.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True)
