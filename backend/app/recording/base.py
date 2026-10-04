from inspect import iscoroutinefunction
from typing import Protocol

from app.schemas.messages import RobotStateMessage, VRFrame


def try_record_event_nowait(recorder: object, event: object, server_mono_ns: int) -> bool:
    """Optional synchronous enqueue only; never fall back to async recording.

    Implementations of try_write_event must do bounded in-memory work and return
    a bool. This best-effort channel is not for critical control records.
    """
    try:
        write = getattr(recorder, "try_write_event", None)
        if not callable(write) or iscoroutinefunction(write):
            return False
        return write(event, server_mono_ns) is True
    except Exception:
        return False


class RecorderSink(Protocol):
    async def start(self) -> None: ...

    async def write_vr_frame(self, frame: VRFrame, server_mono_ns: int) -> None: ...

    async def write_robot_state(self, state: RobotStateMessage, server_mono_ns: int) -> None: ...

    async def write_event(self, event: object, server_mono_ns: int) -> None: ...

    async def write_critical_event(
        self,
        kind: str,
        payload: object,
        server_mono_ns: int,
    ) -> None: ...

    async def write_camera_frame(self, frame: object) -> None: ...

    async def close(self) -> None: ...
