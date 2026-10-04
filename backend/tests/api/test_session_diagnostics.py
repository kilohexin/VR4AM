import asyncio
from types import SimpleNamespace

import pytest
from fastapi import WebSocketDisconnect

from app.api.teleop_ws import teleop_websocket
from app.diagnostics.recorder import DiagnosticsRecorder
from app.diagnostics.store import DiagnosticsStore
from app.recording.commissioning import CommissioningRecorder
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from tests.api.test_observer_session_lifecycle import ProbeSocket
from tests.control.test_robot_control import make_control


class RuntimeSocket(ProbeSocket):
    async def send_json(self, payload):
        if payload["type"] == "robot_state":
            self.sent.append(payload)
            self.sending.set()
            await self.release_send.wait()
            if self.send_error is not None:
                raise self.send_error
        else:
            await super().send_json(payload)


def runtime(primary):
    control, latest, backend, clock = make_control()
    store = DiagnosticsStore()
    control.recorder = DiagnosticsRecorder(primary, store)
    state = SimpleNamespace(control=control, latest=latest, teleop_sender_tasks=set(),
                            teleop_owner=None, teleop_owner_lock=asyncio.Lock())
    return control, backend, clock, store, RuntimeSocket(state)


def lifecycle(store, clock):
    rows = store.message(runtime="SIMULATOR", hardware_verified=False, server_mono_ns=clock.now_ns(),
                         control_generation=0, log_session_dir=None).recent_events
    return [row.payload for row in rows if row.kind == "teleop_session_lifecycle"]


@pytest.mark.parametrize("sink", ["normal", "full", "fatal", "broken_nowait", "unsupported", "async_only"])
async def test_real_sender_cleanup_trace_does_not_wait_for_diagnostic_sink(tmp_path, sink):
    if sink in {"normal", "full", "fatal"}:
        primary = CommissioningRecorder(tmp_path, {}, capacity=1 if sink == "full" else 100)
    else:
        primary = NoopRecorder()
        if sink == "unsupported":
            primary.try_write_event = None
        elif sink == "async_only":
            async def bad_async(*args):
                pytest.fail("must not await optional diagnostic method")
            primary.try_write_event = bad_async
        else:
            def broken(*args):
                raise RuntimeError("offline_best_effort_failure")
            primary.try_write_event = broken
    control, backend, clock, store, socket = runtime(primary)
    previous_latest = control.latest
    stop_starts = []
    original_stop = backend.stop

    async def stop(reason):
        stop_starts.append(clock.now_ns())
        clock.advance_ms(16)
        await original_stop(reason)

    backend.stop = stop
    task = asyncio.create_task(teleop_websocket(socket))
    try:
        await asyncio.wait_for(socket.sending.wait(), 1)
        if sink == "fatal":
            primary.fatal_error = "offline_disk_failure"
        clock.advance_ms(7)
        socket.incoming.put_nowait(WebSocketDisconnect())
        await asyncio.wait_for(task, 1)
        rows = lifecycle(store, clock)
        assert rows, "session lifecycle events are missing"
        final = rows[-1]
        assert final["phase"] == "owner_released"
        assert final["accepted_ns"] == 1_000_000_000
        assert final["cancel_requested_ns"] == final["tasks_joined_ns"] == 1_007_000_000
        assert final["disconnect_started_ns"] == stop_starts[0] == 1_007_000_000
        assert final["disconnect_completed_ns"] == final["owner_released_ns"] == 1_023_000_000
        assert final["disconnect_outcome"] == "returned"
        assert {item["name"] for item in final["task_exits"]} == {
            "teleop-receiver", "teleop-state-50hz", "teleop-rehearsal-worker",
        }
        assert backend.stops == [StopReason.DISCONNECT]
        # Existing critical-record failure handling still owns control faults.
        assert control._fault == ("recording_unavailable" if sink == "fatal" else None)
        assert socket.app.state.teleop_owner is None
        assert socket.app.state.teleop_sender_tasks == set()
        assert control.latest is not previous_latest
        if sink != "normal":
            assert control.recorder.dropped_nonblocking_events > 0
        if sink in {"normal", "full", "fatal"}:
            assert not primary.session_dir.exists()
            if sink == "full":
                assert primary.dropped_normal_events > 0
                assert len(primary._normal) == 1
                assert primary.fatal_error is None
    finally:
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize("cause", ["external_cancel", "send_error", "cleanup_error"])
async def test_session_trace_preserves_error_and_cancel_outcomes(cause):
    control, backend, clock, store, socket = runtime(NoopRecorder())
    if cause == "cleanup_error":
        async def fail_stop(reason):
            raise RuntimeError("offline_stop_failed")
        backend.stop = fail_stop
    task = asyncio.create_task(teleop_websocket(socket))
    try:
        await asyncio.wait_for(socket.sending.wait(), 1)
        if cause == "external_cancel":
            task.cancel()
            expected = asyncio.CancelledError
        elif cause == "send_error":
            socket.send_error = RuntimeError("offline_send_failed")
            socket.release_send.set()
            expected = RuntimeError
        else:
            socket.incoming.put_nowait(WebSocketDisconnect())
            expected = RuntimeError
        with pytest.raises(expected):
            await asyncio.wait_for(task, 1)
        final = lifecycle(store, clock)[-1]
        assert final["cancel_reason"] == ("external_cancel" if cause == "external_cancel" else "peer_completed")
        assert final["disconnect_outcome"] == ("error" if cause == "cleanup_error" else "returned")
        assert final["phase"] == "owner_released"
        assert socket.app.state.teleop_owner is None
        if cause == "cleanup_error":
            assert control._fault == "stop_unverified"
            assert final["disconnect_error_type"] == "RuntimeError"
    finally:
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)
