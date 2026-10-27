"""Owned preparation keeps its input watchdog without a synthetic stream debt."""
import asyncio
import pytest
from app.control.robot_control import LatestVRFrame,RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from app.schemas.messages import TeleopMode
from tests.control.test_prepare_before_arm import PreparingBackend
from tests.control.test_robot_control import FakeClock,frame


class OfflinePositionBackend(PreparingBackend):
    position_candidate=True
    position_offline_prepare_scheduler=True
    def __init__(self):
        super().__init__();self.diagnostics=[]
    def _schedule_sdk_diagnostic(self,event):self.diagnostics.append(event)


@pytest.mark.parametrize('opt_in',[True,False,'true'])
async def test_only_explicit_offline_prepare_avoids_stream_catchup_with_fresh_inputs(monkeypatch,opt_in):
    backend,clock,latest=OfflinePositionBackend(),FakeClock(),LatestVRFrame()
    backend.position_offline_prepare_scheduler=opt_in
    control=RobotControl(backend=backend,latest=latest,clock=clock,recorder=NoopRecorder(),prepare_on_arm=True)
    await control.connect();latest.publish(frame(1,False),clock.now_ns());await control.tick()
    prepare=asyncio.create_task(control.arm());await backend.entered.wait()
    stops_before=list(backend.stops);original_sleep=asyncio.sleep;wakes=0
    async def delayed_wakeup(_delay):
        nonlocal wakes
        wakes+=1;clock.advance_ms(65);latest.publish(frame(wakes+1,False),clock.now_ns())
        if wakes==4:control._running=False
        await original_sleep(0)
    monkeypatch.setattr('app.control.robot_control.asyncio.sleep',delayed_wakeup)
    try:
        control._running=True;await control.run()
        if opt_in is True:
            assert control._fault is None and backend.stops==stops_before
            assert backend.diagnostics and all(e['stream_catchup'] is False for e in backend.diagnostics)
            assert control.mode is TeleopMode.READY
        else:
            assert control._fault=='control_overrun' and StopReason.FAULT in backend.stops
    finally:
        prepare.cancel();await asyncio.gather(prepare,return_exceptions=True)


@pytest.mark.parametrize('revoke',['stale','grip','hidden','session'])
async def test_offline_prepare_scheduler_does_not_preserve_revoked_input(monkeypatch,revoke):
    backend,clock,latest=OfflinePositionBackend(),FakeClock(),LatestVRFrame()
    control=RobotControl(backend=backend,latest=latest,clock=clock,recorder=NoopRecorder(),prepare_on_arm=True)
    await control.connect();latest.publish(frame(1,False),clock.now_ns());await control.tick()
    prepare=asyncio.create_task(control.arm());await backend.entered.wait();before=len(backend.stops)
    original_sleep=asyncio.sleep;wakes=0
    async def revoked_wakeup(_delay):
        nonlocal wakes
        wakes+=1;clock.advance_ms(101 if revoke=='stale' else 65)
        if revoke!='stale':latest.publish(frame(wakes+1,revoke=='grip',
            visibility='hidden' if revoke=='hidden' else 'visible',
            session_id='other' if revoke=='session' else 'test'),clock.now_ns())
        if wakes==2:control._running=False
        await original_sleep(0)
    monkeypatch.setattr('app.control.robot_control.asyncio.sleep',revoked_wakeup)
    try:
        control._running=True;await control.run()
        assert len(backend.stops)>before
        assert control.mode not in (TeleopMode.ARMED,TeleopMode.ACTIVE)
        assert not backend.diagnostics
    finally:
        prepare.cancel();await asyncio.gather(prepare,return_exceptions=True)
