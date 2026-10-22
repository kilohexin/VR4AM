"""Memory wire / explicit Fake client: closing a page before any actuator write."""
import asyncio
from types import SimpleNamespace
import pytest
from app.api.teleop_ws import teleop_websocket
from app.control.robot_control import RobotControl,LatestVRFrame
from app.recording.noop import NoopRecorder
from app.diagnostics.store import DiagnosticsStore
from tests.robots.test_finite_position_web import make
from tests.api.test_background_teleop import MemoryWire


async def test_idle_page_close_finishes_started_read_without_poisoning_connection():
    a,c,clock=await make();await a.connect()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    await control.connect()
    state=SimpleNamespace(control=control,latest=control.latest,teleop_sender_tasks=set(),
        teleop_owner=None,teleop_owner_lock=asyncio.Lock())
    wire=MemoryWire(state);entered=asyncio.Event();release=asyncio.Event();cancelled=[]
    original=c.get_estop_reason
    async def blocked():
        entered.set()
        try:await release.wait()
        except asyncio.CancelledError:cancelled.append(True);raise
        return await original()
    c.get_estop_reason=blocked
    task=asyncio.create_task(teleop_websocket(wire))
    wire.send(dict(v=1,type='hello',request_id='hello'))
    try:
        await asyncio.wait_for(entered.wait(),1)
        wire.disconnect()
        await asyncio.sleep(.02)
        assert not cancelled and not task.done()
        intruder=MemoryWire(state)
        await teleop_websocket(intruder)
        assert intruder.messages[0]['reason']=='controller_occupied'
        release.set()
        await asyncio.wait_for(task,2)
        assert not a._snapshot_calls.unknown_methods and not control._stop_unverified
        assert [r[0] for r in c.write_calls]==['stop_move']
        assert state.teleop_owner is None
    finally:
        release.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True)
        await asyncio.gather(control.stop(),return_exceptions=True)
        await a.disconnect()


async def test_idle_proof_never_applies_after_any_motion_or_claw_entry():
    a,c,clock=await make()
    try:
        await a.connect()
        assert a.web_idle_close_safe()
        a._client.move_entries=1
        assert not a.web_idle_close_safe()
        a._client.move_entries=0;a._client.claw_entries=1
        assert not a.web_idle_close_safe()
        a._client.claw_entries=0;a._snapshot_calls.unknown_methods.add('get_tcp')
        assert not a.web_idle_close_safe()
    finally:await a.disconnect()


async def test_disconnect_during_idle_stop_verification_does_not_cancel_stop(monkeypatch):
    from app.api import teleop_ws
    from fastapi import WebSocketDisconnect
    from unittest.mock import AsyncMock
    a,c,clock=await make();await a.connect()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    await control.connect()
    state=SimpleNamespace(control=control,latest=control.latest,teleop_sender_tasks=set(),
        teleop_owner=None,teleop_owner_lock=asyncio.Lock(),diagnostics=DiagnosticsStore(),runtime_backend='LEBAI')
    wire=MemoryWire(state);entered=asyncio.Event();release=asyncio.Event();cancelled=[]
    control.arm=AsyncMock();control.prepare=AsyncMock()
    original=c.get_estop_reason
    async def blocked():
        if a._stop_lock.locked() and not release.is_set():
            entered.set()
            try:await release.wait()
            except asyncio.CancelledError:cancelled.append(True);raise
        return await original()
    c.get_estop_reason=blocked
    async def diagnostics(*args,**kwargs):
        await entered.wait()
        raise WebSocketDisconnect()
    monkeypatch.setattr(teleop_ws,'_delayed_diagnostics_sender',diagnostics)
    task=asyncio.create_task(teleop_websocket(wire))
    wire.send(dict(v=1,type='hello',request_id='hello'))
    wire.send(dict(v=1,type='disarm',request_id='idle-stop'))
    try:
        await asyncio.wait_for(entered.wait(),1)
        await asyncio.sleep(.02)
        assert not cancelled and not task.done()
        wire.send(dict(v=1,type='arm_request',request_id='buffered-arm'))
        wire.send(dict(v=1,type='prepare_request',request_id='buffered-prepare'))
        release.set();await asyncio.wait_for(task,2)
        control.arm.assert_not_awaited();control.prepare.assert_not_awaited()
        assert not control._stop_unverified and not a._snapshot_calls.unknown_methods
        assert [r[0] for r in c.write_calls]==['stop_move']
    finally:
        release.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True)
        await asyncio.gather(control.stop(),return_exceptions=True);await a.disconnect()


async def test_real_read_timeout_still_latches_unknown_in_idle_drain():
    a,c,clock=await make();await a.connect()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    await control.connect()
    state=SimpleNamespace(control=control,latest=control.latest,teleop_sender_tasks=set(),
        teleop_owner=None,teleop_owner_lock=asyncio.Lock())
    wire=MemoryWire(state);entered=asyncio.Event();release=asyncio.Event()
    async def blocked():entered.set();await release.wait();return 0
    c.get_estop_reason=blocked
    task=asyncio.create_task(teleop_websocket(wire));wire.send(dict(v=1,type='hello',request_id='hello'))
    try:
        await asyncio.wait_for(entered.wait(),1);wire.disconnect()
        await asyncio.wait_for(asyncio.gather(task,return_exceptions=True),2)
        assert 'get_estop_reason' in a._snapshot_calls.unknown_methods
        assert control._stop_unverified
        assert not a.web_idle_close_safe()
    finally:
        release.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True)
        await asyncio.gather(control.stop(),return_exceptions=True);await a.disconnect()


async def test_external_cancel_during_idle_drain_joins_children(monkeypatch):
    from app.api import teleop_ws
    a,c,clock=await make();await a.connect()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    await control.connect()
    state=SimpleNamespace(control=control,latest=control.latest)
    wire=MemoryWire(state);entered=asyncio.Event();release=asyncio.Event();draining=asyncio.Event()
    children=[];registered=set()
    async def blocked():entered.set();await release.wait();return 0
    c.get_estop_reason=blocked
    original=teleop_ws._drain_idle_web_close
    async def drain(control,tasks,*args):
        children.extend(tasks);draining.set()
        await original(control,tasks,*args)
    monkeypatch.setattr(teleop_ws,'_drain_idle_web_close',drain)
    task=asyncio.create_task(teleop_ws._run_coupled_session(wire,control,registered))
    wire.send(dict(v=1,type='hello',request_id='hello'))
    try:
        await asyncio.wait_for(entered.wait(),1);wire.disconnect()
        await asyncio.wait_for(draining.wait(),1);await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        assert children and all(child.done() for child in children)
        assert not registered
    finally:
        release.set()
        for child in children:
            if not child.done():child.cancel()
        await asyncio.gather(*children,return_exceptions=True)
        await asyncio.gather(control.stop(),return_exceptions=True);await a.disconnect()
