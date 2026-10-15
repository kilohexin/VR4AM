"""Unlock preparation, offline ownership and cancellation regressions."""
import asyncio

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError
from app.schemas.messages import TeleopMode
from tests.control.test_robot_control import FakeBackend, FakeClock, frame


class PreparingBackend(FakeBackend):
    supports_prepare_on_arm = True

    def __init__(self):
        super().__init__()
        self.entered, self.finish = asyncio.Event(), asyncio.Event()
        self.prepare_count = 0

    async def prepare(self, options, on_phase):
        self.prepare_count += 1
        on_phase('homing')
        self.entered.set()
        await self.finish.wait()
        on_phase('stabilizing')


async def rig():
    backend, clock, latest = PreparingBackend(), FakeClock(), LatestVRFrame()
    control = RobotControl(backend=backend, latest=latest, clock=clock,
                           recorder=NoopRecorder(), prepare_on_arm=True)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    return control, backend, clock, latest


async def test_unlock_waits_for_prepare_and_stable_completion_before_armed():
    control, backend, clock, latest = await rig()
    assert backend.prepare_count == 0
    task = asyncio.create_task(control.arm())
    await asyncio.wait_for(backend.entered.wait(), .5)
    try:
        state = await control.state_message()
        assert state.mode is TeleopMode.READY
        assert state.recovery_phase == 'homing'
        assert not state.manual_input_ready
        with pytest.raises(RuntimeError, match='gripper_command_requires_armed'):
            await control.command_gripper(1.)
        with pytest.raises(RuntimeError, match='arm_blocked_by_recovery'):
            await control.arm()
        latest.publish(frame(2, False), clock.now_ns())
        backend.finish.set()
        await task
        assert control.mode is TeleopMode.ARMED
        assert backend.prepare_count == 1
        assert backend.targets == backend.gripper_commands == []
        assert (await control.state_message()).recovery_phase is None
    finally:
        backend.finish.set()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('revoke', ['stop', 'disconnect', 'stale', 'hidden', 'grip', 'session'])
async def test_revoked_preparation_never_arms_or_replays(revoke):
    control, backend, clock, latest = await rig()
    task = asyncio.create_task(control.arm())
    await asyncio.wait_for(backend.entered.wait(), .5)
    try:
        if revoke == 'stop':
            await control.disarm()
        elif revoke == 'disconnect':
            await control.on_disconnect()
        else:
            if revoke == 'stale':
                clock.advance_ms(100)
            else:
                latest.publish(frame(2, revoke == 'grip',
                    visibility='hidden' if revoke == 'hidden' else 'visible',
                    session_id='other' if revoke == 'session' else 'test'), clock.now_ns())
            await control.tick()
        assert backend.stops, 'preparation watchdog must stop without waiting for prepare completion'
        backend.finish.set()
        with pytest.raises(RuntimeError, match='arm_'):
            await task
        assert control.mode is not TeleopMode.ARMED
        latest.publish(frame(3, False), clock.now_ns())
        await control.tick()
        assert backend.prepare_count == 1
        assert backend.targets == backend.gripper_commands == []
    finally:
        backend.finish.set()
        await asyncio.gather(task, return_exceptions=True)


async def test_failed_preparation_stops_and_does_not_arm():
    control, backend, _clock, _latest = await rig()
    async def failed(options, on_phase):
        on_phase('homing')
        raise BackendCommandError('prepare_failed')
    backend.prepare = failed
    with pytest.raises(RuntimeError, match='arm_prepare_failed'):
        await control.arm()
    assert backend.stops
    assert control.mode is TeleopMode.DISARMED


async def test_prepare_opt_in_requires_backend_capability():
    with pytest.raises(ValueError, match='prepare_on_arm_not_supported'):
        RobotControl(backend=FakeBackend(), latest=LatestVRFrame(), clock=FakeClock(),
                     recorder=NoopRecorder(), prepare_on_arm=True)

async def test_preparation_keeps_original_deadline_policy_while_initial_stop_waits():
    control,backend,_clock,_latest=await rig()
    entered,finish=asyncio.Event(),asyncio.Event()
    original=backend.stop
    async def stop(reason):entered.set();await finish.wait();await original(reason)
    backend.stop=stop
    control._deadline_rebase_requested=False
    task=asyncio.create_task(control.arm())
    try:
        await asyncio.wait_for(entered.wait(),1)
        assert not control._deadline_rebase_requested
    finally:
        finish.set();backend.finish.set()
        await asyncio.gather(task,return_exceptions=True)
