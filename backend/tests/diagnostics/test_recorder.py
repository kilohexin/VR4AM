import asyncio

import pytest

from app.diagnostics.recorder import DiagnosticsRecorder
from app.diagnostics.store import DiagnosticsStore
from app.recording.commissioning import RecorderUnavailable
from app.schemas.messages import BackendState, Pose, RobotStateMessage, TeleopMode


class RecordingPrimary:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.event_started = asyncio.Event()
        self.release_event = asyncio.Event()
        self.session_dir = "logs/commissioning/session-a"

    async def start(self) -> None:
        self.calls.append(("start",))

    async def write_vr_frame(self, frame: object, server_mono_ns: int) -> None:
        self.calls.append(("vr", frame, server_mono_ns))

    async def write_robot_state(self, state: object, server_mono_ns: int) -> None:
        self.calls.append(("state", state, server_mono_ns))

    async def write_event(self, event: object, server_mono_ns: int) -> None:
        self.calls.append(("event", event, server_mono_ns))
        self.event_started.set()
        await self.release_event.wait()

    async def write_critical_event(
        self,
        kind: str,
        payload: object,
        server_mono_ns: int,
    ) -> None:
        self.calls.append(("critical", kind, payload, server_mono_ns))

    async def write_camera_frame(self, frame: object) -> None:
        self.calls.append(("camera", frame))

    async def close(self) -> None:
        self.calls.append(("close",))


def _robot_state() -> RobotStateMessage:
    return RobotStateMessage(
        server_mono_ns=2,
        mode=TeleopMode.READY,
        robot_state=BackendState.IDLE,
        actual_tcp=Pose(p=(0.3, 0.0, 0.4), q=(0.0, 0.0, 0.0, 1.0)),
        actual_q=(0.0,) * 6,
        gripper=0.0,
    )


@pytest.mark.asyncio
async def test_recorder_observes_event_only_after_primary_write_finishes() -> None:
    primary = RecordingPrimary()
    store = DiagnosticsStore()
    recorder = DiagnosticsRecorder(primary, store)

    write = asyncio.create_task(
        recorder.write_event({"kind": "pvat_sent", "command_id": 17}, 10)
    )
    await primary.event_started.wait()
    assert not store.message(
        runtime="SIMULATOR",
        hardware_verified=False,
        server_mono_ns=10,
        control_generation=0,
        log_session_dir=None,
    ).recent_events

    primary.release_event.set()
    await write

    assert primary.calls == [("event", {"kind": "pvat_sent", "command_id": 17}, 10)]
    assert store.message(
        runtime="SIMULATOR",
        hardware_verified=False,
        server_mono_ns=10,
        control_generation=0,
        log_session_dir=None,
    ).recent_events[0].payload == {"command_id": 17}


@pytest.mark.asyncio
async def test_recorder_propagates_primary_recorder_unavailable_unchanged() -> None:
    failure = RecorderUnavailable("primary_failure")

    class UnavailablePrimary(RecordingPrimary):
        async def write_critical_event(
            self,
            kind: str,
            payload: object,
            server_mono_ns: int,
        ) -> None:
            raise failure

    store = DiagnosticsStore()
    recorder = DiagnosticsRecorder(UnavailablePrimary(), store)

    with pytest.raises(RecorderUnavailable) as caught:
        await recorder.write_critical_event("stop_requested", {"reason": "fault"}, 11)

    assert caught.value is failure
    assert not store.message(
        runtime="LEBAI",
        hardware_verified=False,
        server_mono_ns=11,
        control_generation=0,
        log_session_dir=None,
    ).recent_events


@pytest.mark.asyncio
async def test_recorder_delegates_passthrough_calls_and_session_directory() -> None:
    primary = RecordingPrimary()
    primary.release_event.set()
    recorder = DiagnosticsRecorder(primary, DiagnosticsStore())

    await recorder.start()
    await recorder.write_vr_frame("frame", 1)
    state = _robot_state()
    await recorder.write_robot_state(state, 2)
    await recorder.write_camera_frame("camera")
    await recorder.close()

    assert recorder.log_session_dir == "logs/commissioning/session-a"
    assert primary.calls == [
        ("start",),
        ("vr", "frame", 1),
        ("state", state, 2),
        ("camera", "camera"),
        ("close",),
    ]
