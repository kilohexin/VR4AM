"""Deterministic sender timing; no WebSocket network or robot imports."""
import asyncio
from types import SimpleNamespace
import pytest
from app.api import teleop_ws

@pytest.mark.parametrize('source',['read','send'])
@pytest.mark.parametrize('work,expected',[(.015,[0.,.04,.08]),(.07,[0.,.11,.22])])
async def test_read_work_is_in_the_period_and_missed_slots_are_not_burst(monkeypatch,work,expected,source):
    now=0.;starts=[];delays=[]
    async def state():
        nonlocal now
        starts.append(now)
        if source=='read':now+=work
        return SimpleNamespace(model_dump=lambda **kw:{'type':'robot_state'})
    async def send(value):
        nonlocal now
        if source=='send':now+=work
    async def sleep(delay):
        nonlocal now
        assert delay>0
        delays.append(delay);now+=delay
        if len(starts)==3:raise asyncio.CancelledError()
    monkeypatch.setattr(teleop_ws,'asyncio',SimpleNamespace(
        get_running_loop=lambda:SimpleNamespace(time=lambda:now),sleep=sleep))
    socket=SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace(settings=SimpleNamespace(state_hz=25))),send_json=send)
    with pytest.raises(asyncio.CancelledError):
        await teleop_ws.state_sender(socket,SimpleNamespace(state_message=state),asyncio.Lock())
    assert starts==pytest.approx(expected)
    assert min(b-a for a,b in zip(starts,starts[1:]))>=.04-1e-12

async def test_early_sleep_wakeup_cannot_start_an_early_poll(monkeypatch):
    now=0.;starts=[];wakes=0
    async def state():
        nonlocal now
        starts.append(now);now+=.01
        return SimpleNamespace(model_dump=lambda **kw:{})
    async def send(value):pass
    async def sleep(delay):
        nonlocal now,wakes
        wakes+=1
        now+=delay/2 if wakes%2 else delay
        if len(starts)==3:raise asyncio.CancelledError()
    monkeypatch.setattr(teleop_ws,'asyncio',SimpleNamespace(
        get_running_loop=lambda:SimpleNamespace(time=lambda:now),sleep=sleep))
    socket=SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace(settings=SimpleNamespace(state_hz=25))),send_json=send)
    with pytest.raises(asyncio.CancelledError):
        await teleop_ws.state_sender(socket,SimpleNamespace(state_message=state),asyncio.Lock())
    assert starts==pytest.approx([0.,.04,.08])


async def test_cancellation_while_waiting_for_send_lock_never_reads():
    from unittest.mock import AsyncMock
    lock=asyncio.Lock();await lock.acquire()
    control=SimpleNamespace(state_message=AsyncMock())
    task=asyncio.create_task(teleop_ws.state_sender(SimpleNamespace(),control,lock))
    await asyncio.sleep(0)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):await task
    control.state_message.assert_not_awaited()
    assert lock.locked()
    lock.release()
