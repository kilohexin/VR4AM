"""Complete synthetic workflow with an explicit clock, never a real SDK."""
import asyncio
import math
import sys
import pytest
from scipy.spatial.transform import Rotation
from app.control.robot_control import RobotControl,LatestVRFrame
from app.digital_twin.segmented_demo import SegmentedDemoClient,demo_settings
from app.main import _build_mapper,_build_limiter
from app.recording.noop import NoopRecorder
from app.robots.segmented_adapter import SegmentedLebaiAdapter
from app.schemas.messages import ControllerState,VRFrame,TeleopMode
from tests.robots.test_lebai_adapter_control import FakeClock

class Workflow:
    def __init__(self):
        self.clock=FakeClock();self.latest=LatestVRFrame();self.seq=0
        self.settings=demo_settings();self.client=SegmentedDemoClient.idle(self.settings.lebai,clock=self.clock.now_ns)
        async def factory(ip):
            assert ip=='digital-twin'
            return self.client
        async def sleep(seconds):
            self.clock.advance_ms(seconds*1000)
            self.publish(False)
            await asyncio.sleep(0)
        self.backend=SegmentedLebaiAdapter(self.settings.lebai,settling=self.settings.segmented_target.settling,
            client_factory=factory,allow_prepare=True,allow_tool_spin=True,backend_label='LEBAI_FAKE',
            clock=self.clock.now_ns,pvat_clock=self.clock.now_ns,sleep=sleep)
        self.control=RobotControl(backend=self.backend,latest=self.latest,clock=self.clock,recorder=NoopRecorder(),
            mapper=_build_mapper(self.settings),limiter=_build_limiter(self.settings,'LEBAI_FAKE'),
            segmented_settings=self.settings.segmented_target,prepare_on_arm=True,explicit_preparation=True)

    def publish(self,grip,p=(.56,.42,.18),q=(0.,0.,0.,1.)):
        self.seq+=1
        self.latest.publish(VRFrame(v=1,type='vr_frame',session_id='owned-workflow',seq=self.seq,
            client_mono_ms=self.clock.now_ns()/1e6,tracking_valid=True,visibility='visible',
            right=ControllerState(p=p,q=q,grip=grip,trigger=0)),self.clock.now_ns())

    async def step(self,grip,p=(.56,.42,.18),q=(0.,0.,0.,1.)):
        self.clock.advance_ms(20);self.publish(grip,p,q)
        await self.backend.get_state()
        await self.control.tick()
        receipt=self.control.target_strategy.receipt
        if receipt and receipt.completion.result is None and not receipt.completion.invalidated:
            await asyncio.wait_for(receipt.completion.wait(),2)
        state=await self.control.state_message()
        assert state.fault is None,state.fault
        return state

    async def prepare(self):
        await self.step(False)
        count=len(self.client._write_calls)
        await self.control.arm()
        assert len(self.client._write_calls)==count,'unlock must not write'
        await self.control.prepare()
        state=await self.control.state_message()
        assert state.mode is TeleopMode.ARMED and state.manual_input_ready
        assert state.actual_q==pytest.approx(self.settings.lebai.teleop_ready_q,abs=.001)

async def test_prepare_hold_translate_spin_claw_stop_and_prepare_again():
    w=Workflow()
    try:
        await w.backend.connect();await w.control.connect();await w.prepare()
        for axis in (0,2):
            before=(await w.control.state_message()).actual_tcp
            await w.step(True)
            for i in range(1,31):
                p=[.56,.42,.18];p[axis]+=.0004*i
                await w.step(True,tuple(p))
            after=await w.step(False)
            assert after.mode is TeleopMode.HOLD and after.manual_input_ready
            assert after.actual_tcp.p[axis]>before.p[axis]+1e-7
            stopped=after.actual_q
            for _ in range(10):assert (await w.step(False)).actual_q==stopped
        before=(await w.control.state_message()).actual_tcp
        tool_axis=Rotation.from_quat(before.q).apply((0,0,1))
        await w.step(True)
        for i in range(1,31):
            q=tuple(Rotation.from_rotvec(tool_axis*math.radians(3)*.02*i).as_quat())
            await w.step(True,q=q)
        after=await w.step(False)
        assert (Rotation.from_quat(before.q).inv()*Rotation.from_quat(after.actual_tcp.q)).magnitude()>1e-6
        await w.control.command_gripper(.35)
        assert (await w.control.state_message()).gripper==pytest.approx(.35)
        for _ in range(10):assert (await w.step(False)).gripper==pytest.approx(.35)
        await w.control.disarm()
        assert not (await w.control.state_message()).manual_input_ready
        await w.prepare()
        assert [x[0] for x in w.client._write_calls].count('movej')==2
        assert 'lebai_sdk' not in sys.modules
    finally:
        await w.backend.disconnect()
