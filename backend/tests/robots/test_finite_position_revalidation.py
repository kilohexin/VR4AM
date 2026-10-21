"""Planning is bounded separately; only a newly verified start may be sent."""
import pytest
from app.robots.base import BackendCommandError
from app.control.finite_position_types import PositionIntent
from tests.robots.test_finite_position_adapter import prepared,execute


async def replan(a,c,clock,*,delay=.025,damage=None):
    forward=c.kinematics_forward;calls=0
    async def delayed(q):
        nonlocal calls
        await clock.sleep(delay);result=await forward(q);calls+=1
        if calls==5 and damage is not None:damage()
        return result
    c.kinematics_forward=delayed
    return await a.plan_step(PositionIntent('web',1,2,clock.now_ns(),'held',(1.,0.,0.)),authorize=lambda:None)


async def test_slow_planning_revalidates_same_start_then_naturally_finishes():
    a,c,clock,_=await prepared();began=clock.now_ns()
    try:
        p=await replan(a,c,clock)
        assert clock.now_ns()-began>=125_000_000
        assert 0<=clock.now_ns()-p.start.captured_ns<100_000_000
        r=await execute(a,p)
        assert r.complete and [x[0] for x in c.write_calls]==['movej']
    finally:await a.disconnect()


@pytest.mark.parametrize('damage',['joint','target_joint','tcp','target_tcp','tool','state','task'])
async def test_changed_start_after_math_is_zero_write(damage):
    a,c,clock,_=await prepared()
    def change():
        if damage=='joint':c.kin_data['actual_joint_pose'][0]+=.000001
        if damage=='target_joint':c.kin_data['target_joint_pose'][0]+=.000001
        if damage=='tcp':c.kin_data['actual_tcp_pose']['x']+=.000001
        if damage=='target_tcp':c.kin_data['target_tcp_pose']['x']+=.000001
        if damage=='tool':c.tcp['z']+=.000001  # Within normal TCP tolerance; still a changed input.
        if damage=='state':c.robot_state='PAUSED'
        if damage=='task':c.running_motion=88
    try:
        with pytest.raises((ValueError,BackendCommandError),match='position_(observer_changed|preflight)|robot_state_hold'):
            await replan(a,c,clock,damage=change)
        assert not c.write_calls and a._position_plan is None
    finally:await a.disconnect()


async def test_final_revalidation_itself_must_be_fresh():
    a,c,clock,_=await prepared();read=a._read_snapshot
    async def slow(**kwargs):
        s=await read(**kwargs)
        if kwargs.get('read_purpose')=='position_revalidate':await clock.sleep(.1)
        return s
    a._read_snapshot=slow
    try:
        with pytest.raises(ValueError,match='position_snapshot_expired'):await replan(a,c,clock)
        assert not c.write_calls and a._position_plan is None
    finally:await a.disconnect()


async def test_math_cannot_keep_a_plan_alive_indefinitely():
    a,c,clock,_=await prepared()
    try:
        with pytest.raises(ValueError,match='position_planning_expired'):
            await replan(a,c,clock,delay=.2)
        assert not c.write_calls and a._position_plan is None
    finally:await a.disconnect()


async def test_release_during_final_read_cannot_reach_native_entry():
    a,c,clock,_=await prepared();read=a._read_snapshot;allowed=True
    async def released(**kwargs):
        nonlocal allowed
        result=await read(**kwargs)
        if kwargs.get('read_purpose')=='position_revalidate':allowed=False
        return result
    def authorize():
        if not allowed:raise ValueError('position_entry_revoked')
    a._read_snapshot=released
    try:
        with pytest.raises(ValueError,match='position_entry_revoked'):
            await a.plan_step(PositionIntent('web',1,2,clock.now_ns(),'held',(1.,0.,0.)),authorize=authorize)
        assert not c.write_calls and a._position_plan is None
    finally:await a.disconnect()


async def test_wall_clock_timeout_cancels_blocked_pure_audit_without_late_plan(monkeypatch):
    import asyncio
    import threading
    from app.robots import finite_position_adapter as module
    a,c,clock,_=await prepared();release=threading.Event();finished=threading.Event()
    audit=module.audit_position_path
    def blocked(*args,**kwargs):
        release.wait(2.)
        try:return audit(*args,**kwargs)
        finally:finished.set()
    monkeypatch.setattr(module,'audit_position_path',blocked)
    try:
        with pytest.raises(ValueError,match='position_planning_expired'):
            await a.plan_step(PositionIntent('web',1,2,clock.now_ns(),'held',(1.,0.,0.)),authorize=lambda:None)
        assert a._position_plan is None and not c.write_calls
        release.set()
        await asyncio.to_thread(finished.wait,1.)
        assert finished.is_set() and a._position_plan is None and not c.write_calls
    finally:release.set();await a.disconnect()
