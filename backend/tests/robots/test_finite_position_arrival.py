from dataclasses import replace
import pytest
from app.robots.finite_position_plan import evaluate_position_arrival
from tests.robots.test_finite_position_adapter import prepared


@pytest.mark.parametrize('kind,expected',[
    ('no_progress','position_no_actual_progress'),('reverse','position_no_actual_progress'),
    ('lateral','position_lateral_deviation'),('error','position_endpoint_error'),
    ('running','position_not_finished'),('paused','position_not_finished'),
    ('foreign','position_foreign_task'),('rotation','position_orientation_error'),
    ('velocity','position_not_stable'),('binding','position_binding_changed')])
async def test_arrival_cannot_be_claimed_from_task_status_alone(kind,expected):
    a,_,_,plan=await prepared()
    try:
        s=replace(plan.start,tcp_p=plan.target_p,actual_q=plan.goal_joints,task_state='FINISHED')
        if kind=='no_progress':s=replace(s,tcp_p=plan.start.tcp_p)
        if kind=='reverse':s=replace(s,tcp_p=(.2995,0.,.4))
        if kind=='lateral':s=replace(s,tcp_p=(.302,.00041,.4))
        if kind=='error':s=replace(s,tcp_p=(.30251,0.,.4))
        if kind=='running':s=replace(s,task_state='RUNNING')
        if kind=='paused':s=replace(s,robot_state='HOLD')
        if kind=='foreign':s=replace(s,running_id=999)
        if kind=='rotation':s=replace(s,tcp_q=(0.,0.,.01,.99994999875))
        if kind=='velocity':s=replace(s,actual_qd=(.01,0.,0.,0.,0.,0.))
        if kind=='binding':s=replace(s,binding=replace(s.binding))
        assert evaluate_position_arrival(plan,s,motion_id=7)==expected
    finally:await a.disconnect()


async def test_nonfinite_feedback_cannot_pass_arrival_contract():
    a,_,_,p=await prepared()
    try:
        with pytest.raises(ValueError):replace(p.start,tcp_p=(float('nan'),0.,.4))
    finally:await a.disconnect()


@pytest.mark.parametrize('fraction,expected',[
    (.1292,'position_no_actual_progress'),
    (.74,'position_endpoint_error'),
    (.75,None),
    (.9,None),
])
async def test_finished_target_at_goal_does_not_replace_actual_tcp_progress(fraction,expected):
    """A finished task and achieved target joints cannot certify actual arrival."""
    a,_,_,plan=await prepared()
    try:
        actual=tuple(start+fraction*(goal-start)
            for start,goal in zip(plan.start.tcp_p,plan.target_p))
        sample=replace(plan.start,tcp_p=actual,actual_q=plan.goal_joints,
            target_q=plan.goal_joints,task_state='FINISHED',robot_state='IDLE')
        assert evaluate_position_arrival(plan,sample,motion_id=7)==expected
    finally:await a.disconnect()


@pytest.mark.parametrize('progress,error',[(.00099,'position_no_actual_progress'),
    (.00149,'position_endpoint_error'),(.0015,None),(.002,None)])
async def test_two_mm_arrival_requires_real_progress_and_half_mm_endpoint(progress,error):
    a,_,_,p=await prepared()
    try:
        start=p.start.tcp_p
        p=replace(p,target_p=(start[0]+.002,*start[1:]))
        s=replace(p.start,tcp_p=(start[0]+progress,*start[1:]),
            actual_q=p.goal_joints,target_q=p.goal_joints,task_state='FINISHED')
        assert evaluate_position_arrival(p,s,motion_id=7)==error
    finally:await a.disconnect()


@pytest.mark.parametrize('joint_error,expected',[(.0015,None),(.0018,'position_joint_endpoint')])
async def test_arrival_joint_tolerance_is_one_tenth_degree(joint_error,expected):
    a,_,_,p=await prepared()
    try:
        actual=list(p.goal_joints);actual[0]+=joint_error
        s=replace(p.start,tcp_p=p.target_p,actual_q=tuple(actual),task_state='FINISHED')
        assert evaluate_position_arrival(p,s,motion_id=7)==expected
    finally:await a.disconnect()
