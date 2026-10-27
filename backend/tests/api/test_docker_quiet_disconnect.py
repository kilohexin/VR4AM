import asyncio
from types import SimpleNamespace
import pytest
from app.api.teleop_ws import teleop_websocket
from app.schemas.messages import TeleopMode
from tests.control.test_docker_idle_input import rig
from tests.api.test_background_teleop import MemoryWire


@pytest.mark.parametrize('mode',[TeleopMode.ARMED,TeleopMode.HOLD])
async def test_completed_docker_actions_drain_started_read_before_disconnect_stop(mode):
    control,a,c,_clock,_latest=await rig()
    a._client.move_entries=1;control.machine.mode=mode
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
    task=asyncio.create_task(teleop_websocket(wire));wire.send(dict(v=1,type='hello',request_id='hello'))
    try:
        await asyncio.wait_for(entered.wait(),1);wire.disconnect();await asyncio.sleep(.02)
        assert not cancelled and not task.done()
        release.set();await asyncio.wait_for(task,2)
        assert not a._snapshot_calls.unknown_methods and not control._stop_unverified
        assert [r[0] for r in c.write_calls]==['stop_move']
        assert state.teleop_owner is None and control.mode is TeleopMode.DISCONNECTED
    finally:
        release.set()
        if not task.done():task.cancel()
        await asyncio.gather(task,return_exceptions=True);await a.disconnect()
