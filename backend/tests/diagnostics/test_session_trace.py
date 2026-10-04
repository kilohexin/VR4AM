import asyncio
import json

import pytest

from app.diagnostics.recorder import DiagnosticsRecorder
from app.diagnostics.store import DiagnosticsStore
from app.recording.commissioning import CommissioningRecorder, RecorderUnavailable
from tests.control.test_robot_control import make_control


async def test_nowait_event_uses_normal_capacity_without_occupying_stop_queue(tmp_path):
    recorder = CommissioningRecorder(tmp_path, {}, session_id="trace", capacity=1)
    assert recorder.try_write_event({"kind": "teleop_session_lifecycle", "phase": "accepted"}, 10)
    assert not recorder.try_write_event({"kind": "teleop_session_lifecycle", "phase": "cancel_requested"}, 20)
    await recorder.write_critical_event("stop_requested", {}, 30)
    assert len(recorder._normal) == len(recorder._critical) == 1
    assert recorder.dropped_normal_events == 1
    assert recorder.fatal_error is None
    assert not recorder.session_dir.exists()  # Enqueue does not perform file I/O.
    await recorder.start()
    await recorder.close()
    rows = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    assert any(row["kind"] == "stop_requested" for row in rows)
    trace = next(row for row in rows if row["kind"] == "teleop_session_lifecycle")
    assert trace["server_mono_ns"] == 10 and not trace["critical"]


@pytest.mark.parametrize("unavailable", ["fatal", "closed"])
async def test_nowait_unavailable_is_dropped_but_normal_async_contract_still_raises(tmp_path, unavailable):
    recorder = CommissioningRecorder(tmp_path, {})
    if unavailable == "fatal":
        recorder.fatal_error = "offline_disk_failure"
    else:
        recorder._closing = True
    assert not recorder.try_write_event({"kind": "teleop_session_lifecycle"}, 1)
    assert recorder.dropped_normal_events == 1
    with pytest.raises(RecorderUnavailable):
        await recorder.write_event({"kind": "ordinary_event"}, 2)


@pytest.mark.parametrize("primary_kind", ["full", "fatal", "unsupported", "raises", "async_only"])
async def test_diagnostics_retains_memory_trace_when_primary_cannot_enqueue(tmp_path, primary_kind):
    primary = CommissioningRecorder(tmp_path, {}, capacity=1)
    if primary_kind == "full":
        await primary.write_event({"kind": "existing"}, 0)
    elif primary_kind == "fatal":
        primary.fatal_error = "offline_disk_failure"
    else:
        class Primary:
            async def write_event(self, *args):
                pytest.fail("must not fall back to asynchronous write")
        primary = Primary()
        if primary_kind == "raises":
            def fail(*args):
                raise RuntimeError("offline_failure")
            primary.try_write_event = fail
        elif primary_kind == "async_only":
            primary.try_write_event = primary.write_event
    store = DiagnosticsStore()
    recorder = DiagnosticsRecorder(primary, store)
    assert not recorder.try_write_event({"kind": "teleop_session_lifecycle", "phase": "tasks_joined"}, 10)
    message = store.message(runtime="SIMULATOR", hardware_verified=False, server_mono_ns=10,
                            control_generation=0, log_session_dir=None)
    assert message.recent_events[-1].payload["phase"] == "tasks_joined"
    assert recorder.dropped_nonblocking_events == 1


async def test_trace_cumulative_timestamps_survive_store_coalescing_and_task_errors():
    from app.diagnostics.session_trace import SessionTrace

    control, _, _, clock = make_control()
    store = DiagnosticsStore()
    control.recorder = DiagnosticsRecorder(control.recorder, store)
    trace = SessionTrace(control)
    trace.mark("accepted")

    async def fail():
        raise RuntimeError("do not put arbitrary exception contents in telemetry")

    task = asyncio.create_task(fail(), name="teleop-state-50hz")
    await asyncio.gather(task, return_exceptions=True)
    clock.advance_ms(7)
    trace.task_done(task)
    clock.advance_ms(3)
    trace.mark("cancel_requested", cancel_reason="peer_completed")
    clock.advance_ms(11)
    trace.mark("tasks_joined")
    trace.mark("disconnect_started")
    clock.advance_ms(9)
    trace.mark("disconnect_completed", disconnect_outcome="returned")
    trace.mark("owner_released")
    message = store.message(runtime="SIMULATOR", hardware_verified=False, server_mono_ns=clock.now_ns(),
                            control_generation=0, log_session_dir=None)
    event = message.recent_events[-1]
    assert event.kind == "teleop_session_lifecycle"
    payload = event.payload
    assert payload["accepted_ns"] == 1_000_000_000
    assert payload["cancel_requested_ns"] == 1_010_000_000
    assert payload["tasks_joined_ns"] == payload["disconnect_started_ns"] == 1_021_000_000
    assert payload["disconnect_completed_ns"] == payload["owner_released_ns"] == 1_030_000_000
    assert payload["task_exits"] == [{"name": "teleop-state-50hz", "observed_ns": 1_007_000_000,
                                      "outcome": "error", "error_type": "RuntimeError"}]
    assert "do not put" not in json.dumps(payload)
    assert len(payload["session_trace_id"]) == 32
    assert len(message.recent_events) == 1


async def test_trace_sink_failure_never_escapes_or_falls_back_to_async():
    from app.diagnostics.session_trace import SessionTrace

    control, _, _, _ = make_control()

    class FailedSink:
        def try_write_event(self, *args):
            raise RuntimeError("offline_failure")

        async def write_event(self, *args):
            pytest.fail("no async fallback")

    control.recorder = FailedSink()
    trace = SessionTrace(control)
    trace.mark("cancel_requested")
    trace.mark("tasks_joined")
    assert trace.dropped_events == 2
    assert control._fault is None
    assert control.control_generation == 0


async def test_broken_memory_store_does_not_prevent_normal_queue_enqueue(tmp_path):
    class FailedStore:
        def observe_event(self, *args, **kwargs):
            raise RuntimeError("offline_store_failed")

    primary = CommissioningRecorder(tmp_path, {})
    recorder = DiagnosticsRecorder(primary, FailedStore())
    assert not recorder.try_write_event({"kind": "teleop_session_lifecycle"}, 1)
    assert len(primary._normal) == 1
    assert recorder.dropped_nonblocking_events == 1


async def test_cancelled_task_callback_and_task_detail_capacity_are_bounded():
    from app.diagnostics.session_trace import SessionTrace

    control, _, _, clock = make_control()
    store = DiagnosticsStore(capacity=2)
    control.recorder = DiagnosticsRecorder(control.recorder, store)
    trace = SessionTrace(control)
    tasks = [asyncio.create_task(asyncio.sleep(0), name=f"probe-{index}") for index in range(10)]
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
    for task in tasks:
        trace.task_done(task)
    message = store.message(runtime="SIMULATOR", hardware_verified=False, server_mono_ns=clock.now_ns(),
                            control_generation=0, log_session_dir=None)
    exits = message.recent_events[-1].payload["task_exits"]
    assert len(exits) == 8
    assert all(item["outcome"] == "cancelled" and item["error_type"] is None for item in exits)
    assert len(message.recent_events) <= 2
