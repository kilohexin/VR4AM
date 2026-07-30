from __future__ import annotations

from app.diagnostics.store import DiagnosticsStore
from app.recording.base import RecorderSink
from app.schemas.messages import RobotStateMessage, VRFrame


class DiagnosticsRecorder:
    def __init__(self, primary: RecorderSink, store: DiagnosticsStore) -> None:
        self.primary = primary
        self.store = store

    @property
    def log_session_dir(self) -> str | None:
        session_dir = getattr(self.primary, "session_dir", None)
        return None if session_dir is None else str(session_dir)

    async def start(self) -> None:
        await self.primary.start()

    async def write_vr_frame(self, frame: VRFrame, server_mono_ns: int) -> None:
        await self.primary.write_vr_frame(frame, server_mono_ns)

    async def write_robot_state(
        self,
        state: RobotStateMessage,
        server_mono_ns: int,
    ) -> None:
        await self.primary.write_robot_state(state, server_mono_ns)
        self.store.observe_robot_state(state, server_mono_ns)

    async def write_event(self, event: object, server_mono_ns: int) -> None:
        await self.primary.write_event(event, server_mono_ns)
        self.store.observe_event(event, server_mono_ns, critical=False)

    async def write_critical_event(
        self,
        kind: str,
        payload: object,
        server_mono_ns: int,
    ) -> None:
        await self.primary.write_critical_event(kind, payload, server_mono_ns)
        self.store.observe_event(
            {"kind": kind, "payload": payload},
            server_mono_ns,
            critical=True,
        )

    async def write_camera_frame(self, frame: object) -> None:
        await self.primary.write_camera_frame(frame)

    async def close(self) -> None:
        await self.primary.close()
