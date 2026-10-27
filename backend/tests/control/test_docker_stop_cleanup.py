import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock
from app.api.teleop_ws import _drain_idle_web_close
from app.control.robot_control import RobotControl,LatestVRFrame
from app.recording.noop import NoopRecorder
from tests.control.test_robot_control import FakeBackend,FakeClock


async def test_shutdown_allows_a_valid_virtual_stop_to_finish_its_bounded_profile():
    backend=FakeBackend();backend._stop_cleanup_grace_s=3.5
    control=RobotControl(backend=backend,latest=LatestVRFrame(),clock=FakeClock(),recorder=NoopRecorder())
    control._stop_backend=AsyncMock(return_value=True)
    task=asyncio.create_task(asyncio.sleep(1.1));control._task=task;control._backend_stop_owners.add(task)
    await control.stop()
    assert task.done() and not task.cancelled()


async def test_web_close_does_not_cancel_virtual_stop_inside_its_allowed_profile():
    task=asyncio.create_task(asyncio.sleep(1.1))
    control=SimpleNamespace(backend=SimpleNamespace(_stop_cleanup_grace_s=3.5),
        _backend_stop_owners={task},_advance_control_generation=lambda:None)
    await _drain_idle_web_close(control,[task],None,asyncio.Event(),asyncio.Event())
    assert task.done() and not task.cancelled()
