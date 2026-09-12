"""Offline integration evidence: real stop and recorder, hardware boundary fake."""
import asyncio
import json

import pytest

from app.recording.commissioning import CommissioningRecorder
from app.robots.base import StopReason
from app.robots.lebai_pump import PvatRequest
from tests.robots.test_lebai_adapter_control import (
    _connected_control_adapter,
    _target,
    _wait_until,
)


@pytest.mark.asyncio
async def test_stop_invalidated_inflight_pvat_survives_recording_to_disk(tmp_path):
    # Removing lifecycle scheduling must lose the otherwise unlogged SDK call.
    adapter, client, clock = await _connected_control_adapter()
    recorder = CommissioningRecorder(root=tmp_path, metadata={'offline': True})
    await recorder.start()
    adapter._event_callback = recorder.write_event
    entered = asyncio.Event()
    release = asyncio.Event()
    original_send = client.move_pvat

    async def inflight_send(*args):
        result = await original_send(*args)
        entered.set()
        await release.wait()
        return result

    client.move_pvat = inflight_send
    await adapter._pump.start()
    generation = adapter._pump._generation
    sender = asyncio.create_task(adapter._send_target(PvatRequest(_target(), 71, generation)))
    stopper = None
    try:
        await asyncio.wait_for(entered.wait(), 1)
        stopper = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
        await _wait_until(lambda: not adapter._pump.is_current(generation))
        release.set()
        await asyncio.wait_for(asyncio.gather(sender, stopper), 2)
    finally:
        release.set()
        await asyncio.gather(sender, *([stopper] if stopper else []), return_exceptions=True)
        await adapter.disconnect()
        await recorder.close()

    events = [json.loads(line) for line in recorder.jsonl_path.read_text(encoding='utf-8').splitlines()]
    calls = [e for e in events if e['kind'] == 'sdk_request_lifecycle']
    assert len(calls) == 1
    call = calls[0]
    assert call['method'] == 'move_pvat'
    assert call['command_id'] == 71
    assert call['rpc_outcome'] == 'returned'
    assert call['invalidated'] is True
    assert call['p'] == client.write_calls[0][1]
    assert call['v'] == client.write_calls[0][2]
    assert call['a'] == client.write_calls[0][3]
    assert not any(e['kind'] == 'pvat_sent' for e in events)
    stops = [e for e in events if e['kind'] == 'stop_diagnostics']
    assert len(stops) == 1
    assert stops[0]['outcome'] == 'confirmed'
    assert stops[0]['reason'] == 'grip_released'
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
    summary = json.loads((recorder.session_dir / 'summary.json').read_text(encoding='utf-8'))
    assert summary['dropped_normal_events'] == 0
    # Critical and normal queues may be flushed in different file order.
    assert sum(e['kind'] == 'session_ended' for e in events) == 1
