"""Deterministic fake SDK probes, not robot performance measurements."""
import asyncio

import pytest

from app.recording.commissioning import CommissioningRecorder
from app.robots.lebai_pump import PvatRequest
from app.robots.lebai_sdk_lock import SdkLock
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target


DELAY_NS = 15_000_000


@pytest.mark.asyncio
@pytest.mark.parametrize('phase', ['snapshot', 'command_lock', 'ik', 'logging'])
async def test_injected_delay_is_attributed_to_its_software_phase(monkeypatch, phase):
    adapter, client, clock = await _connected_control_adapter()
    performance = [0]
    events = []
    monkeypatch.setattr('app.robots.lebai_adapter.sample_performance_ns', lambda: performance[0])
    original_read, original_ik = client.get_kin_data, client.kinematics_inverse

    async def read():
        if phase == 'snapshot':
            performance[0] += DELAY_NS
        return await original_read()

    async def ik(*args):
        if phase == 'ik':
            performance[0] += DELAY_NS
        return await original_ik(*args)

    class InjectedLock(SdkLock):
        count = 0

        async def __aenter__(self):
            self.count += 1
            if phase == 'command_lock' and self.count == 2:
                performance[0] += DELAY_NS
            return await super().__aenter__()

    async def record(event, timestamp):
        if phase == 'logging' and event['kind'] == 'robot_kinematics':
            performance[0] += DELAY_NS
        events.append(event)

    client.get_kin_data, client.kinematics_inverse = read, ik
    adapter._sdk_lock = InjectedLock()
    adapter._event_callback = record
    await adapter._pump.start()
    try:
        await adapter._send_target(PvatRequest(_target(), 1, adapter._pump.generation))
        sent = next(x for x in events if x['kind'] == 'pvat_sent')
        d = sent['diagnostic_timing']
        s = d['snapshot_timing']
        measured = {
            'snapshot': s['completed_ns'] - s['acquired_ns'],
            'command_lock': d['command_lock_acquired_ns'] - d['command_lock_requested_ns'],
            'ik': d['candidate_finished_ns'] - d['command_lock_acquired_ns'],
            'logging': d['snapshot_finished_ns'] - s['completed_ns'],
        }
        assert measured == {key: DELAY_NS if key == phase else 0 for key in measured}
        assert len(client.ik_calls) == 1
        assert [x[0] for x in client.write_calls] == ['move_pvat']
        assert clock.now_ns() == 0  # Diagnostic time never advances safety time.
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('concurrent_observer', [True, False])
async def test_full_observer_read_can_queue_between_snapshot_and_send(
    monkeypatch, concurrent_observer,
):
    adapter, client, clock = await _connected_control_adapter()
    performance = [0]
    events = []
    motion_read_entered, release_motion = asyncio.Event(), asyncio.Event()
    observer_entered, release_observer = asyncio.Event(), asyncio.Event()
    monkeypatch.setattr('app.robots.lebai_adapter.sample_performance_ns', lambda: performance[0])
    original_read = client.get_kin_data
    read_count = 0

    async def read():
        nonlocal read_count
        read_count += 1
        if read_count == 1:
            motion_read_entered.set()
            await release_motion.wait()
        elif read_count == 2:
            performance[0] += DELAY_NS
            observer_entered.set()
            await release_observer.wait()
        return await original_read()

    async def record(event, timestamp):
        events.append(event)

    client.get_kin_data = read
    client.read_calls.clear()
    adapter._event_callback = record
    await adapter._pump.start()
    sending = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    observer = None
    try:
        await asyncio.wait_for(motion_read_entered.wait(), 1)
        if concurrent_observer:
            observer = asyncio.create_task(adapter.get_state())
            await asyncio.sleep(0)  # Queue observer behind the motion snapshot.
        release_motion.set()
        if concurrent_observer:
            await asyncio.wait_for(observer_entered.wait(), 1)
            assert client.ik_calls == []
            assert client.write_calls == []
            release_observer.set()
        await asyncio.wait_for(sending, 1)
        if observer is None:
            release_observer.set()
            observer = asyncio.create_task(adapter.get_state())
        await asyncio.wait_for(observer, 1)
        d = next(x for x in events if x['kind'] == 'pvat_sent')['diagnostic_timing']
        assert d['command_lock_acquired_ns'] - d['command_lock_requested_ns'] == (
            DELAY_NS if concurrent_observer else 0)
        # Both alternatives perform every full read, including TCP and E-stop.
        for name in ['get_robot_state', 'get_estop_reason', 'get_kin_data',
                     'get_tcp', 'get_claw', 'get_running_motion']:
            assert client.read_calls.count(name) == 2
        assert len(client.ik_calls) == 1
        assert [x[0] for x in client.write_calls] == ['move_pvat']
    finally:
        release_motion.set()
        release_observer.set()
        for task in [sending, observer]:
            if task is not None and not task.done():
                task.cancel()
        await asyncio.gather(*[x for x in [sending, observer] if x is not None],
                             return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('started', [True, False])
async def test_commissioning_normal_record_enqueue_does_not_yield_or_write_disk(tmp_path, started):
    recorder = CommissioningRecorder(tmp_path, {})
    if started:
        await recorder.start()
    try:
        before = recorder.jsonl_path.read_bytes() if started else None
        ran = []
        asyncio.get_running_loop().call_soon(lambda: ran.append(True))
        await recorder.write_event({'kind': 'robot_kinematics'}, 0)
        assert ran == []
        assert len(recorder._normal) == 1
        if started:
            assert recorder.jsonl_path.read_bytes() == before
        else:
            assert not recorder.jsonl_path.exists()
        await asyncio.sleep(0)
        assert ran == [True]
    finally:
        await recorder.close()
