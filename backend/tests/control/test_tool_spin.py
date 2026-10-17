"""Tool-local rotation audit with explicit synthetic SDK only."""
import asyncio
from dataclasses import replace
import math
from unittest.mock import AsyncMock
import pytest
from scipy.spatial.transform import Rotation
from app.schemas.messages import Pose
from app.control.target_strategy import TargetContext
from app.robots.lebai_codec import pose_to_lebai
from tests.control.test_segmented_control import OPTIONS,limiter
from tests.commissioning.test_finite_diagnostic import AuditClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock,_target


@pytest.mark.parametrize('angle_rad,accepted',[(2e-9,True),(5e-9,True),(1e-8,True),
    (math.radians(1e-6)*.99,True),(math.radians(1e-6)*1.01,False)])
async def test_translated_reference_uses_translation_audit_with_orientation_residue(angle_rad,accepted):
    from tests.control.test_segmented_control import rig,send
    adapter,client,_clock,_box,strategy=await rig()
    adapter.supports_tool_spin=True
    try:
        target=Pose(p=_target(.31).p,q=tuple(Rotation.from_rotvec((0,0,angle_rad)).as_quat()))
        # Preserve the tiny requested residue in the planner's reference;
        # the actual feedback still has the preceding numerical IK rotation.
        strategy.pose=strategy.pose.model_copy(update={'q':target.q})
        result=await send(strategy,adapter,target,1)
        if accepted:
            assert result.sdk_outcome=='returned',result.error
            assert len(client.fk_calls)==4
            assert [call[0] for call in client.write_calls]==['move_pvat']
        else:
            assert result.sdk_outcome=='not_started' and result.error=='finite_diagnostic_translation_only'
            assert not any(call[0]=='move_pvat' for call in client.write_calls)
    finally:await adapter.disconnect()

class SpinClient(AuditClient):
    async def kinematics_inverse(self,pose,joints):
        self.ik_calls.append((pose.copy(),list(joints)))
        return [*joints[:5],pose['rz']]
    async def kinematics_forward(self,joints):
        self.fk_calls.append(tuple(joints))
        result=pose_to_lebai(Pose(p=_target(.3).p,q=tuple(Rotation.from_rotvec((0,0,joints[5])).as_quat())))
        return await self.fk_hook(len(self.fk_calls),result) if self.fk_hook else result

async def spin_rig():
    from app.robots.segmented_adapter import SegmentedLebaiAdapter
    from app.control.segmented_strategy import SegmentedTargetStrategy
    clock,client=FakeClock(),SpinClient()
    settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=1.5,
        max_joint_acceleration_radps2=4.))
    adapter=SegmentedLebaiAdapter(settings,settling=OPTIONS,allow_tool_spin=True,
        client_factory=AsyncMock(return_value=client),clock=clock.now_ns,
        pvat_clock=clock.now_ns,sleep=clock.sleep)
    await adapter.connect()
    assert (await adapter.preflight()).ready
    box=[TargetContext(1,clock.now_ns(),True,True)]
    strategy=SegmentedTargetStrategy(adapter,limiter(),context=lambda:box[0])
    strategy.on_anchor(_target(.3),box[0])
    return adapter,client,clock,box,strategy

def spin_pose(angle=.05,axis=(0,0,1)):
    return Pose(p=_target(.3).p,q=tuple(Rotation.from_rotvec([math.radians(angle)*v for v in axis]).as_quat()))

@pytest.mark.parametrize('sign',[-1,1])
async def test_self_spin_uses_four_fk_zero_terminal_and_existing_settling(sign):
    adapter,client,clock,box,strategy=await spin_rig()
    try:
        plan=strategy.prepare(_target(.3),spin_pose(sign*.05),.02,limiter=limiter())
        assert plan.kind=='candidate' and plan.pose.p==_target(.3).p
        await strategy.dispatch(plan,1,backend=adapter)
        done=await asyncio.wait_for(strategy.receipt.completion.wait(),2)
        assert done.sdk_outcome=='returned',done.error
        assert len(client.ik_calls)==1 and len(client.fk_calls)==4
        assert client.write_calls[0][2:5]==([0.]*6,[0.]*6,.08)
        assert strategy.collect(box[0]).committed==plan.pose
        assert not adapter.read_segment_status().ready
        client.kin_data['actual_joint_pose']=list(adapter.read_motion_view().accepted.solution_q)
        client.kin_data['actual_tcp_pose']=pose_to_lebai(plan.pose)
        clock.value+=80_000_000
        for _ in range(3):
            clock.value+=20_000_000
            box[0]=replace(box[0],now_ns=clock.now_ns())
            await adapter.get_state()
            strategy.collect(box[0])
        assert adapter.read_segment_status().ready
    finally:await adapter.disconnect()

@pytest.mark.parametrize('kind',['pitch','mixed','oversize','drift','wrong_axis','wrong_direction'])
async def test_invalid_spin_or_fk_sample_cannot_send_motion(kind):
    adapter,client,_clock,_box,_strategy=await spin_rig()
    try:
        target=spin_pose()
        if kind=='pitch':target=spin_pose(axis=(0,1,0))
        if kind=='oversize':target=spin_pose(.11)
        if kind=='mixed':target=target.model_copy(update={'p':(.30001,0,.4)})
        if kind in ('drift','wrong_axis','wrong_direction'):
            async def bad(index,result):
                if index==3:
                    result=dict(result)
                    if kind=='drift':result['x']+=.00001
                    elif kind=='wrong_axis':result['ry']=.001
                    else:result['rz']=-abs(result['rz'])
                return result
            client.fk_hook=bad
        receipt=await adapter.submit_segment(target,1)
        done=await asyncio.wait_for(receipt.completion.wait(),2)
        assert done.sdk_outcome=='not_started' and done.error is not None
        if kind in ('drift','wrong_axis','wrong_direction'):
            assert done.error=='segment_tool_spin_fk_mismatch'
        assert not any(c[0]=='move_pvat' for c in client.write_calls)
    finally:await adapter.disconnect()

def test_spin_geometry_uses_tool_axis_not_world_z():
    from app.robots.tool_spin import tool_spin_angle
    origin=Pose(p=(.3,0,.4),q=tuple(Rotation.from_euler('y',90,degrees=True).as_quat()))
    target=Pose(p=origin.p,q=tuple((Rotation.from_quat(origin.q)*Rotation.from_rotvec((0,0,.001))).as_quat()))
    assert tool_spin_angle(origin,target)==pytest.approx(.001)

async def test_spin_plan_reserves_room_below_the_snapshot_hard_limit():
    from app.robots.tool_spin import tool_spin_angle
    adapter,client,_clock,_box,strategy=await spin_rig()
    try:
        # A settled IK endpoint can differ slightly from the accepted reference.
        origin=spin_pose(-math.degrees(2e-12))
        strategy.limits.max_angular_speed=1.
        strategy.limits.max_angular_accel=10.
        strategy.limits.max_angular_step_rad=1.
        plan=strategy.prepare(_target(.3),spin_pose(1),.02,limiter=limiter())
        assert plan.kind=='candidate'
        assert abs(tool_spin_angle(origin,plan.pose))<math.radians(.1)
    finally:await adapter.disconnect()

async def test_equivalent_quaternion_roundoff_is_not_a_different_accepted_target():
    adapter,client,_clock,box,strategy=await spin_rig()
    try:
        plan=strategy.prepare(_target(.3),spin_pose(.05),.02,limiter=limiter())
        await strategy.dispatch(plan,1,backend=adapter)
        await asyncio.wait_for(strategy.receipt.completion.wait(),2)
        q=list(adapter._last_sent_tcp.q);q[0]+=1e-16
        adapter._last_sent_tcp=adapter._last_sent_tcp.model_copy(update={'q':tuple(q)})
        assert adapter.read_motion_view().accepted.tcp!=plan.pose
        assert strategy.collect(box[0]).fault is None
    finally:await adapter.disconnect()

@pytest.mark.parametrize('change',['position','rotation'])
async def test_changed_accepted_pose_is_still_rejected(change):
    adapter,client,_clock,box,strategy=await spin_rig()
    try:
        plan=strategy.prepare(_target(.3),spin_pose(.05),.02,limiter=limiter())
        await strategy.dispatch(plan,1,backend=adapter)
        await asyncio.wait_for(strategy.receipt.completion.wait(),2)
        if change=='position':
            p=list(adapter._last_sent_tcp.p);p[0]+=1e-12
            adapter._last_sent_tcp=adapter._last_sent_tcp.model_copy(update={'p':tuple(p)})
        else:
            q=tuple((Rotation.from_quat(adapter._last_sent_tcp.q)*Rotation.from_rotvec((0,0,1e-8))).as_quat())
            adapter._last_sent_tcp=adapter._last_sent_tcp.model_copy(update={'q':q})
        assert strategy.collect(box[0]).fault=='segment_acceptance_failed'
    finally:await adapter.disconnect()

async def test_spin_finite_constructor_failure_keeps_the_outside_lock_stop(monkeypatch):
    import app.robots.segmented_adapter as module
    from app.robots.base import BackendCommandError
    adapter,client,_clock,_box,_strategy=await spin_rig()
    stops=[]
    original=adapter._stop_for_motion_fault
    async def stop(fault):
        stops.append(adapter._sdk_lock.locked())
        await original(fault)
    monkeypatch.setattr(adapter,'_stop_for_motion_fault',stop)
    def failed(*args):raise BackendCommandError('finite_pvat_target_joint_limit')
    monkeypatch.setattr(module,'build_finite_pvat_candidate',failed)
    try:
        receipt=await adapter.submit_segment(spin_pose(),1)
        done=await asyncio.wait_for(receipt.completion.wait(),2)
        assert done.sdk_outcome=='not_started'
        assert stops==[False]
    finally:await adapter.disconnect()
