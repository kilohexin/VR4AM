"""State-message ordering contracts with an in-memory backend only."""

import asyncio

import pytest

from app.control.coordinate_mapper import CoordinateMapper
from app.recording.commissioning import RecorderUnavailable
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from app.schemas.messages import BackendState, TeleopMode
from tests.control.test_robot_control import frame, make_control


class StateRecorder(NoopRecorder):
    def __init__(self):
        self.rows = []
        self.on_write = None

    async def write_robot_state(self, message, server_mono_ns):
        self.rows.append((message.model_copy(deep=True), server_mono_ns))
        if self.on_write is not None:
            await self.on_write()


@pytest.mark.parametrize(
    "mode,robot_state,pending,hard,unverified,completed",
    [
        (TeleopMode.STALE, BackendState.IDLE, True, False, False, True),
        (TeleopMode.FAULT, BackendState.IDLE, True, False, False, True),
        (TeleopMode.STALE, BackendState.MOVING, True, False, False, False),
        (TeleopMode.FAULT, BackendState.FAULT, True, False, False, False),
        (TeleopMode.STALE, BackendState.MOVING, True, True, False, True),
        (TeleopMode.FAULT, BackendState.FAULT, True, True, False, True),
        (TeleopMode.STALE, BackendState.IDLE, True, False, True, False),
        (TeleopMode.FAULT, BackendState.IDLE, True, True, True, False),
        (TeleopMode.READY, BackendState.IDLE, True, True, False, False),
        (TeleopMode.STALE, BackendState.IDLE, False, True, False, False),
    ],
)
async def test_message_and_record_precede_stop_transition(
    mode, robot_state, pending, hard, unverified, completed,
):
    recorder = StateRecorder()
    control, _, backend, _ = make_control(recorder=recorder)
    control.machine.mode = mode
    control._pending_stop_completion = pending
    control._hard_stop_completion = hard
    control._stop_unverified = unverified
    backend.robot_state = robot_state
    generation = control.control_generation

    async def observe_before_transition():
        assert control.mode is mode
        assert control.control_generation == generation
        assert control._pending_stop_completion is pending

    recorder.on_write = observe_before_transition
    result = await control.state_message()
    assert result.mode is mode
    assert result.robot_state is robot_state
    assert recorder.rows == [(result, result.server_mono_ns)]
    assert control.mode is (TeleopMode.DISARMED if completed else mode)
    assert control._pending_stop_completion is (False if completed else pending)
    assert control._hard_stop_completion is (False if completed else hard)
    assert control._completed_stale_stop is (completed and mode is TeleopMode.STALE)
    assert control.control_generation == generation + int(completed)
    assert backend.stops == []


@pytest.mark.parametrize("input_offset_ms,expected_age", [(None, None), (-10, 10), (10, 0)])
async def test_composition_alone_does_not_observe_fault_record_or_complete_stop(input_offset_ms, expected_age):
    recorder = StateRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    state = (await backend.get_state()).model_copy(update={"fault": "backend_fault"})
    original = state.model_copy(deep=True)
    control.machine.stale()
    control._pending_stop_completion = True
    control._constraint = "tracking_lag"
    control.last_seq = 7
    control.mapper = CoordinateMapper(translation_scale=0.2)
    if input_offset_ms is not None:
        latest.publish(frame(7, False), clock.now_ns() + input_offset_ms * 1_000_000)

    async def forbidden_read():
        pytest.fail("composition must not read the backend")

    backend.get_state = forbidden_read
    result = control._compose_state_message(state)
    assert result.mode is TeleopMode.STALE
    assert result.fault == "backend_fault"
    assert result.ack_seq == 7
    assert result.sample_age_ms == expected_age
    assert result.constraint == "tracking_lag"
    assert result.translation_scale == 0.2
    assert result.actual_tcp == state.actual_tcp
    assert state == original
    assert control._fault is None
    assert control._stop_unverified is False
    assert control._pending_stop_completion is True
    assert control.control_generation == 0
    assert control.mode is TeleopMode.STALE
    assert recorder.rows == []
    assert backend.stops == []


@pytest.mark.parametrize("existing_fault", [None, "ik_unreachable"])
async def test_backend_fault_observed_before_composition_without_replacing_existing_fault(existing_fault):
    recorder = StateRecorder()
    control, _, backend, _ = make_control(recorder=recorder)
    control.machine.mode = TeleopMode.READY
    control._fault = existing_fault
    original_read = backend.get_state
    reads = 0

    async def fault_read():
        nonlocal reads
        reads += 1
        return (await original_read()).model_copy(update={"fault": "stop_unverified"})

    backend.get_state = fault_read
    result = await control.state_message()
    assert reads == 1
    assert result.fault == (existing_fault or "stop_unverified")
    assert result.mode is (TeleopMode.READY if existing_fault else TeleopMode.FAULT)
    assert control._stop_unverified is (existing_fault is None)
    assert recorder.rows[0][0] == result
    assert backend.stops == []


async def test_metadata_is_sampled_after_read_and_before_recording_await():
    recorder = StateRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    control.machine.connect()
    original_read = backend.get_state
    reads = 0

    async def delayed_read():
        nonlocal reads
        reads += 1
        state = await original_read()
        latest.publish(frame(42, False), clock.now_ns())
        clock.advance_ms(12)
        control.last_seq = 42
        control._constraint = "tracking_lag"
        return state

    async def delayed_record():
        clock.advance_ms(30)
        control.last_seq = 43
        latest.publish(frame(43, False), clock.now_ns())

    backend.get_state = delayed_read
    recorder.on_write = delayed_record
    result = await control.state_message()
    assert reads == 1
    assert result.server_mono_ns == 1_012_000_000
    assert result.ack_seq == 42
    assert result.sample_age_ms == 12
    assert result.constraint == "tracking_lag"
    assert recorder.rows == [(result, 1_012_000_000)]
    assert control.last_seq == 43


async def test_recorder_unavailable_runs_existing_fault_handler_before_stop_completion():
    recorder = StateRecorder()
    control, _, backend, _ = make_control(recorder=recorder)
    control.machine.mode = TeleopMode.ACTIVE

    async def fail_record():
        raise RecorderUnavailable("offline recorder failure")

    recorder.on_write = fail_record
    result = await control.state_message()
    assert result.mode is TeleopMode.ACTIVE
    assert result.fault is None
    assert backend.stops == [StopReason.FAULT]
    assert control._fault == "recording_unavailable"
    assert control.mode is TeleopMode.DISARMED
    assert control._pending_stop_completion is False
    # Stop request, recording-fault latch, then pending-stop completion.
    assert control.control_generation == 3


@pytest.mark.parametrize("phase", ["read", "record"])
@pytest.mark.parametrize("error_type", [RuntimeError, asyncio.CancelledError])
async def test_read_or_record_failure_propagates_without_completing_stop(phase, error_type):
    recorder = StateRecorder()
    control, _, backend, _ = make_control(recorder=recorder)
    control.machine.stale()
    control._pending_stop_completion = True
    generation = control.control_generation

    async def fail():
        raise error_type("offline failure")

    if phase == "read":
        backend.get_state = fail
    else:
        recorder.on_write = fail
    with pytest.raises(error_type, match="offline failure"):
        await control.state_message()
    assert len(recorder.rows) == (0 if phase == "read" else 1)
    assert control.mode is TeleopMode.STALE
    assert control._pending_stop_completion is True
    assert control.control_generation == generation
    assert backend.stops == []


async def test_stop_completion_rechecks_live_latch_after_recording_await():
    recorder = StateRecorder()
    control, _, backend, _ = make_control(recorder=recorder)
    control.machine.stale()
    control._pending_stop_completion = True

    async def latch_during_record():
        control._latch_stop_unverified()

    recorder.on_write = latch_during_record
    result = await control.state_message()
    assert result.mode is TeleopMode.STALE
    assert control.mode is TeleopMode.FAULT
    assert control._pending_stop_completion is True
    assert control._stop_unverified is True
    assert control.control_generation == 1
    assert backend.stops == []


@pytest.mark.parametrize("mode_after_record", [TeleopMode.STALE, TeleopMode.DISCONNECTED])
async def test_completion_uses_original_read_but_current_mode_after_recording(mode_after_record):
    recorder = StateRecorder()
    control, _, backend, _ = make_control(recorder=recorder)
    control.machine.stale()
    control._pending_stop_completion = True
    original_read = backend.get_state
    reads = 0

    async def read_once():
        nonlocal reads
        reads += 1
        return await original_read()

    async def change_during_record():
        backend.robot_state = BackendState.MOVING
        control.machine.mode = mode_after_record

    backend.get_state = read_once
    recorder.on_write = change_during_record
    result = await control.state_message()
    assert reads == 1
    assert result.robot_state is BackendState.IDLE
    assert result.mode is TeleopMode.STALE
    if mode_after_record is TeleopMode.STALE:
        assert control.mode is TeleopMode.DISARMED
        assert control._pending_stop_completion is False
        assert control.control_generation == 1
    else:
        assert control.mode is TeleopMode.DISCONNECTED
        assert control._pending_stop_completion is True
        assert control.control_generation == 0
    assert backend.stops == []
