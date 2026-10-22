"""Explicit coarse-grasp tolerance candidate, injected Fake clients only."""
import math
import pytest
from app.control.finite_position_types import PositionIntent
from tests.robots.finite_position_fixture import fixture
from tests.robots.test_finite_position_adapter import api,execute


async def prepared(policy):
    s,c,clock=fixture()
    async def factory(ip):assert ip=='digital-twin';return c
    a=api().FinitePositionAdapter(s,backend_label='LEBAI_FAKE',client_factory=factory,
        position_model=c.model,arrival_policy=policy,clock=clock.now_ns,sleep=clock.sleep)
    await a.connect();a.bind_position_generation(1)
    plan=await a.plan_step(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)),authorize=lambda:None)
    return a,c,clock,plan


@pytest.mark.parametrize('policy,complete',[('strict',False),('coarse_grasp',True)])
async def test_small_position_and_rotation_errors_are_explicitly_selectable(policy,complete):
    a,c,clock,p=await prepared(policy);original=c.advance
    def residual():
        original()
        if c.start_ns is None or clock.now_ns()-c.start_ns<200_000_000:return
        c.kin_data['actual_joint_pose'][0]-=.0006
        c.kin_data['actual_tcp_pose']['x']-=.0006
        c.kin_data['actual_tcp_pose']['rz']=math.radians(.15)
    c.advance=residual
    try:
        r=await execute(a,p)
        assert r.complete==complete,r
        assert [x[0] for x in c.write_calls]==(['movej'] if complete else ['movej','stop_move'])
    finally:await a.disconnect()


async def test_coarse_allows_settling_after_two_seconds_without_another_target():
    a,c,clock,p=await prepared('coarse_grasp');original=c.advance
    def settling():
        original()
        if c.start_ns is not None and clock.now_ns()-c.start_ns<2_600_000_000:
            c.kin_data['actual_joint_pose'][3]+=.002
    c.advance=settling
    try:
        r=await execute(a,p)
        assert r.complete,r
        assert 2_800_000_000<=clock.now_ns()<5_000_000_000
        assert [x[0] for x in c.write_calls]==['movej']
    finally:await a.disconnect()


async def test_coarse_timeout_is_fixed_and_still_stops_once():
    a,c,clock,p=await prepared('coarse_grasp');original=c.advance
    def never_settle():
        original()
        if c.start_ns is not None:c.kin_data['actual_joint_pose'][3]+=.002
    c.advance=never_settle
    try:
        r=await execute(a,p)
        assert not r.complete
        assert 5_000_000_000<=clock.now_ns()<6_000_000_000
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()


@pytest.mark.parametrize('damage',['orientation','endpoint','lateral','speed'])
async def test_coarse_still_rejects_outside_its_bounds(damage):
    a,c,clock,p=await prepared('coarse_grasp');original=c.advance
    def damaged():
        original()
        if c.start_ns is None or clock.now_ns()-c.start_ns<200_000_000:return
        if damage=='orientation':c.kin_data['actual_tcp_pose']['rz']=math.radians(.2001)
        elif damage=='endpoint':c.kin_data['actual_tcp_pose']['x']+=.00101
        elif damage=='lateral':c.kin_data['actual_tcp_pose']['y']+=.000401
        else:c.kin_data['actual_joint_speed'][0]=.051
    c.advance=damaged
    try:
        r=await execute(a,p);assert not r.complete
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()


async def test_policy_change_after_planning_cannot_send():
    a,c,clock,p=await prepared('coarse_grasp')
    from app.control.finite_position_types import position_arrival_policy
    a.position_arrival_policy=position_arrival_policy('strict')
    try:
        r=await execute(a,p);assert not r.entered and not c.write_calls
    finally:await a.disconnect()


@pytest.mark.parametrize('kind,inside',[('tcp',True),('tcp',False),('joint',True),('joint',False)])
async def test_coarse_stability_uses_fixed_anchors_and_two_hundred_ms(kind,inside):
    a,c,clock,p=await prepared('coarse_grasp');original=c.advance;count=0
    def jitter():
        nonlocal count
        original()
        if c.start_ns is None or clock.now_ns()-c.start_ns<200_000_000:return
        count+=1;sign=1 if count%2 else -1
        if kind=='tcp':c.kin_data['actual_tcp_pose']['x']+=sign*(.00004 if inside else .00009)
        else:c.kin_data['actual_joint_pose'][3]+=sign*math.radians(.005 if inside else .015)
    c.advance=jitter
    try:
        r=await execute(a,p);assert r.complete==inside
        assert clock.now_ns()>=(400_000_000 if inside else 5_000_000_000)
        assert [x[0] for x in c.write_calls]==(['movej'] if inside else ['movej','stop_move'])
    finally:await a.disconnect()
