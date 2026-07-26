import inspect

import pytest

from app.recording.noop import NoopRecorder


@pytest.mark.asyncio
async def test_noop_recorder_preserves_all_time_stamped_interfaces() -> None:
    recorder = NoopRecorder()

    assert await recorder.start() is None
    assert await recorder.write_vr_frame(object(), 1) is None
    assert await recorder.write_robot_state(object(), 2) is None
    assert await recorder.write_event(object(), 3) is None
    assert await recorder.write_critical_event("stop", {}, 4) is None
    assert await recorder.write_camera_frame(object()) is None
    assert await recorder.close() is None
    assert inspect.iscoroutinefunction(recorder.write_camera_frame)
