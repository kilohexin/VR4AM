"""Server receipt time belongs to the control clock; future data is not fresh."""
import asyncio
import json
from contextlib import suppress

import pytest

from app.api import teleop_ws
from app.robots.base import BackendCommandError, StopReason
from app.schemas.messages import TeleopMode
from tests.api.test_xr_wire_replay import MemoryWebSocket
from tests.control.test_robot_control import RecordingRecorder, activate, connect_release_arm, frame, make_control


async def test_receiver_uses_control_clock_instead_of_an_independent_source(monkeypatch):
    control, latest, _backend, clock = make_control()
    wire = MemoryWebSocket(latest)
    # Deliberately different epoch; client time is independently unrelated too.
    monkeypatch.setattr(teleop_ws, 'monotonic_ns', lambda: 9_000_000_000_000, raising=False)
    receiver = asyncio.create_task(teleop_ws._receive_messages(wire, control, asyncio.Event(), asyncio.Lock()))
    try:
        payload = frame(1, False, client_mono_ms=999_999_999).model_dump(mode='json')
        wire.incoming.put_nowait(json.dumps(payload))
        wire.incoming.put_nowait(json.dumps({'v': 1, 'type': 'ping', 'request_id': 'clock-barrier'}))
        await wire.receive_type('pong', 'clock-barrier')
        assert latest.snapshot().received_ns == clock.now_ns()
        assert latest.snapshot().frame.client_mono_ms == 999_999_999
    finally:
        receiver.cancel()
        with suppress(asyncio.CancelledError):
            await receiver


@pytest.mark.parametrize('active', [False, True])
@pytest.mark.parametrize('future_ns', [1, 60_000_000_000])
async def test_future_receipt_cannot_activate_or_send_claw_or_target(active, future_ns):
    control, latest, backend, clock = make_control()
    if active:
        await activate(control, latest, clock)
    else:
        await connect_release_arm(control, latest, clock)
    before = list(backend.targets), list(backend.gripper_commands)
    clock.advance_ms(100)
    latest.publish(frame(3, True, trigger=0.8), clock.now_ns() + future_ns)
    await control.tick()
    assert (backend.targets, backend.gripper_commands) == before
    assert control.mode is TeleopMode.STALE
    assert backend.stops == [StopReason.STALE]
    assert not control.mapper.has_anchor


async def test_future_receipt_is_not_a_current_frame_during_async_work():
    control, latest, _backend, clock = make_control()
    await activate(control, latest, clock)
    latest.publish(frame(3, True), clock.now_ns() + 1)
    received = latest.snapshot()
    assert not control._frame_is_current(received, control.control_generation)


async def test_future_receipt_state_reports_unknown_age_instead_of_zero():
    control, latest, _backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns() + 1)
    state = await control.state_message()
    assert state.sample_age_ms is None


@pytest.mark.parametrize('age_ms, expected', [(0, TeleopMode.ACTIVE), (99, TeleopMode.ACTIVE), (100, TeleopMode.STALE)])
async def test_valid_receipt_age_keeps_original_100ms_boundary(age_ms, expected):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    clock.advance_ms(age_ms)
    await control.tick()
    assert control.mode is expected
    assert backend.stops == ([StopReason.STALE] if expected is TeleopMode.STALE else [])


@pytest.mark.parametrize('coalesced', [False, True])
@pytest.mark.parametrize('future_ns', [1, 60_000_000_000])
async def test_future_release_uses_stale_stop_instead_of_regrip_hold(coalesced, future_ns):
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)
    latest.publish(frame(3, False), clock.now_ns() + future_ns)
    if coalesced:
        latest.publish(frame(4, True), clock.now_ns() + future_ns)
    await control.tick()
    assert control.mode is TeleopMode.STALE
    assert backend.stops == [StopReason.STALE]
    assert not control.mapper.has_anchor
    await control.state_message()  # Complete the fake verified stop, still locked.
    latest.publish(frame(5, True), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    assert backend.targets == [] and backend.gripper_commands == []


@pytest.mark.parametrize('error, fault', [('sdk_timeout:stop_move', 'stop_unverified'), ('stop_incomplete', 'stop_incomplete')])
async def test_future_receipt_stop_failure_keeps_original_latch(error, fault):
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)

    async def failed_stop(reason):
        backend.stops.append(reason)
        raise BackendCommandError(error)

    backend.stop = failed_stop
    latest.publish(frame(3, True, trigger=0.8), clock.now_ns() + 1)
    with pytest.raises(BackendCommandError, match=f'^{error}$'):
        await control.tick()
    state = await control.state_message()
    assert state.mode is TeleopMode.FAULT and state.fault == fault
    assert backend.gripper_commands == [] and backend.targets == []


async def test_future_stop_completion_does_not_repeat_or_auto_rearm_when_clock_catches_up():
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)
    latest.publish(frame(3, True), clock.now_ns() + 1)
    await control.tick()
    assert (await control.state_message()).mode is TeleopMode.STALE
    assert control.mode is TeleopMode.DISARMED
    await control.tick()
    clock.advance_ms(0.000001)
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    assert backend.stops == [StopReason.STALE]
    assert backend.gripper_commands == [] and backend.targets == []


@pytest.mark.parametrize('mode', [TeleopMode.READY, TeleopMode.DISARMED])
@pytest.mark.parametrize('future_ns', [1, 60_000_000_000])
async def test_future_released_frame_cannot_authorize_manual_home(mode, future_ns):
    control, latest, backend, clock = make_control()
    await control.connect()
    control.machine.mode = mode
    latest.publish(frame(1, False), clock.now_ns() + future_ns)
    # Call before tick: Home must perform its own receipt qualification.
    result = await control.home()
    assert not result.accepted and result.reason == 'grip_pressed'
    assert backend.stops == [] and backend.home_phases == []


async def test_overrun_diagnostic_does_not_report_future_receipt_as_zero_age(monkeypatch):
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns() + 60_000_000_000)
    tick_count = 0

    async def idle_tick():
        nonlocal tick_count
        tick_count += 1
        if tick_count == 3:
            control._running = False

    async def delayed_wakeup(_delay):
        clock.advance_ms(70)

    monkeypatch.setattr('app.control.robot_control.asyncio.sleep', delayed_wakeup)
    control.tick = idle_tick
    control._running = True
    await control.run()
    faults = [event for event in recorder.events if event['kind'] == 'robot_fault']
    assert len(faults) == 1
    assert faults[0]['payload']['reason'] == 'control_overrun'
    assert faults[0]['payload']['frame_age_ms'] is None
    assert backend.stops == [StopReason.FAULT]
