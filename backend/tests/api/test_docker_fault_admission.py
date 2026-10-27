import asyncio
from types import SimpleNamespace
from app.api.teleop_ws import teleop_websocket
from tests.api.test_background_teleop import MemoryWire


async def test_quarantined_docker_connection_is_rejected_without_reads_commands_or_owner_claim():
    backend=SimpleNamespace(position_policy='finite_position_docker',_latched_fault='stop_unverified',
        _snapshot_calls=SimpleNamespace(unknown_methods={'kinematics_forward'}),
        _motion_calls=SimpleNamespace(unknown_methods=set()))
    control=SimpleNamespace(backend=backend,_fault='stop_unverified')
    state=SimpleNamespace(control=control,teleop_owner=None,teleop_owner_lock=asyncio.Lock())
    wire=MemoryWire(state)
    await teleop_websocket(wire)
    assert wire.messages[0]['reason']=='backend_fault' and state.teleop_owner is None
