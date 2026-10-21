import asyncio
import importlib
import pytest
from app.control.finite_position_types import PositionIntent
from app.robots.base import StopReason
from tests.robots.test_finite_position_adapter import prepared
from tests.commissioning.test_idle_stop_baseline import Recorder


def api():
    assert importlib.util.find_spec('app.control.finite_position_session'), 'finite session missing'
    return importlib.import_module('app.control.finite_position_session')


async def fixture():
    a,c,clock,_=await prepared()
    s=api().FinitePositionSession(a,a.position_binding,clock=clock.now_ns,recorder=Recorder(),authorize=lambda:None)
    return s,a,c,clock


async def drive(s,clock,*,hold=False,until=lambda:False):
    for i in range(3000):
        if hold:s.offer(PositionIntent('web',1,s._last_seq+1,clock.now_ns(),'held',(1.,0.,0.)))
        await s.advance();await clock.sleep(.001);await asyncio.sleep(.001)
        if until():return
    assert False,'session did not settle'


async def test_held_release_drains_without_second_move():
    s,a,c,clock=await fixture()
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await drive(s,clock,until=lambda:bool(c.write_calls))
        s.offer(PositionIntent('web',1,2,clock.now_ns(),'release',(0.,0.,0.)))
        await drive(s,clock,until=lambda:s.view().phase=='ready')
        assert [x[0] for x in c.write_calls]==['movej']
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_fresh_held_updates_during_slow_planning_continue_then_release_without_stop():
    s,a,c,clock=await fixture();forward=c.kinematics_forward
    async def slow(q):await clock.sleep(.025);return await forward(q)
    c.kinematics_forward=slow
    try:
        await drive(s,clock,hold=True,until=lambda:len(c.write_calls)>=2)
        assert [x[0] for x in c.write_calls]==['movej','movej']
        s.offer(PositionIntent('web',1,s._last_seq+1,clock.now_ns(),'release',(0.,)*3))
        await drive(s,clock,until=lambda:s.phase=='ready' and not s.busy)
        assert [x[0] for x in c.write_calls]==['movej','movej'] and not a._latched_fault
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_held_continues_from_new_feedback_with_single_inflight():
    s,a,c,clock=await fixture()
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await drive(s,clock,hold=True,until=lambda:len(c.write_calls)>=2)
        goals=[x[1][0] for x in c.write_calls if x[0]=='movej']
        assert len(goals)==2 and abs(goals[0]-.002)<1e-9 and abs(goals[1]-.004)<1e-9
        assert s.view().entered_steps==2
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_input_expiry_is_fault_not_normal_release():
    s,a,c,clock=await fixture()
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await drive(s,clock,until=lambda:s.view().phase=='fault')
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_repress_and_reverse_never_queue_a_second_inflight_goal():
    s,a,c,clock=await fixture()
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await drive(s,clock,until=lambda:bool(c.write_calls))
        s.offer(PositionIntent('web',1,2,clock.now_ns(),'release',(0.,)*3))
        s.offer(PositionIntent('web',1,3,clock.now_ns(),'held',(-1.,0.,0.)))
        assert len(c.write_calls)==1
        for _ in range(300):
            s.offer(PositionIntent('web',1,s._last_seq+1,clock.now_ns(),'held',(-1.,0.,0.)))
            await s.advance();await clock.sleep(.001);await asyncio.sleep(.001)
            if len(c.write_calls)==2:break
        assert [x[0] for x in c.write_calls]==['movej','movej']
        assert abs(c.write_calls[1][1][0])<1e-9
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_ten_step_limit_requires_release_before_new_session():
    s,a,c,clock=await fixture()
    try:
        await drive(s,clock,hold=True,until=lambda:s.view().phase=='hold')
        assert len(c.write_calls)==10 and s.view().entered_steps==10
        for _ in range(5):
            s.offer(PositionIntent('web',1,s._last_seq+1,clock.now_ns(),'held',(1.,0.,0.)))
            await s.advance();await clock.sleep(.001)
        assert len(c.write_calls)==10
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_completed_released_session_can_idle_then_accept_a_new_gesture():
    s,a,c,clock=await fixture()
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await drive(s,clock,until=lambda:bool(c.write_calls))
        s.offer(PositionIntent('web',1,2,clock.now_ns(),'release',(0.,)*3))
        await drive(s,clock,until=lambda:s.view().phase=='ready' and not s.busy)
        await clock.sleep(11);await s.advance()
        assert s.phase=='hold' and not s.closed and [x[0] for x in c.write_calls]==['movej']
        s.offer(PositionIntent('web',1,3,clock.now_ns(),'held',(-1.,0.,0.)))
        await drive(s,clock,hold=True,until=lambda:len(c.write_calls)==2)
        assert len(c.write_calls)==2
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_revoked_external_authority_between_plan_and_send_is_zero_write():
    s,a,c,clock=await fixture();allowed=True;original=a.plan_step
    def authorize():
        if not allowed:raise ValueError('position_authority_revoked')
    async def revoke_after_plan(*args,**kwargs):
        nonlocal allowed
        plan=await original(*args,**kwargs);allowed=False;return plan
    s.authorize=authorize;a.plan_step=revoke_after_plan
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await s.advance();await s.worker
        assert c.write_calls==[] and s.phase=='fault'
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


@pytest.mark.parametrize('stage',['planning','entry','executing'])
@pytest.mark.parametrize('damage',['fatal_error','dropped_normal_events'])
async def test_recorder_failure_revokes_each_stage(stage,damage):
    s,a,c,clock=await fixture();original=a.plan_step;inverse=c.kinematics_inverse
    def fail_recording():setattr(s.recorder,damage,1)
    async def bad_math(*args):
        fail_recording();return await inverse(*args)
    async def bad_entry(*args,**kwargs):
        p=await original(*args,**kwargs);fail_recording();return p
    if stage=='planning':c.kinematics_inverse=bad_math
    if stage=='entry':a.plan_step=bad_entry
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        if stage=='executing':
            await drive(s,clock,until=lambda:bool(c.write_calls));fail_recording()
            await drive(s,clock,until=lambda:s.phase=='fault')
        else:await s.advance();await s.worker
        assert s.phase=='fault'
        assert [x[0] for x in c.write_calls]==(['movej','stop_move'] if stage=='executing' else [])
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()


async def test_fixed_session_deadline_aborts_even_after_explicit_release():
    s,a,c,clock=await fixture()
    try:
        s.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.)))
        await drive(s,clock,until=lambda:bool(c.write_calls))
        s.offer(PositionIntent('web',1,2,clock.now_ns(),'release',(0.,)*3))
        s.gate.deadline_ns=clock.now_ns()+50_000_000
        await drive(s,clock,until=lambda:s.phase=='fault')
        assert s.reason=='position_session_expired'
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await s.abort(StopReason.SHUTDOWN);await a.disconnect()
