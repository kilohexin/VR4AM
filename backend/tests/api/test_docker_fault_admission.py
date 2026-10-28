import asyncio
import pytest
from types import SimpleNamespace
from unittest.mock import AsyncMock
from app.schemas.messages import TeleopMode
from app.api.teleop_ws import teleop_websocket
from tests.api.test_background_teleop import MemoryWire


@pytest.mark.parametrize('policy',['finite_position_docker','finite_position_web'])
@pytest.mark.parametrize('source',['control','backend','read_unknown','write_unknown'])
async def test_quarantined_finite_connection_is_rejected_without_reads_commands_or_owner_claim(policy,source):
    backend=SimpleNamespace(position_policy=policy,_latched_fault='stop_unverified' if source=='backend' else None,
        _snapshot_calls=SimpleNamespace(unknown_methods={'get_tcp'} if source=='read_unknown' else set()),
        _motion_calls=SimpleNamespace(unknown_methods={'movej'} if source=='write_unknown' else set()))
    control=SimpleNamespace(backend=backend,_fault='stop_unverified' if source=='control' else None,
        mode=TeleopMode.DISCONNECTED,connect=AsyncMock(side_effect=AssertionError('quarantined client reached connect')),
        on_disconnect=AsyncMock())
    state=SimpleNamespace(control=control,teleop_owner=None,teleop_owner_lock=asyncio.Lock())
    wire=MemoryWire(state)
    await teleop_websocket(wire)
    assert wire.messages[0]['reason']=='backend_fault' and state.teleop_owner is None
    assert backend._latched_fault==('stop_unverified' if source=='backend' else None)
    assert backend._snapshot_calls.unknown_methods==({'get_tcp'} if source=='read_unknown' else set())
    assert backend._motion_calls.unknown_methods==({'movej'} if source=='write_unknown' else set())
    control.connect.assert_not_awaited()
    control.on_disconnect.assert_not_awaited()
