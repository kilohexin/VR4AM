import asyncio
import importlib
from dataclasses import replace
import pytest
from app.control.finite_position_types import PositionIntent
from app.robots.base import StopReason
from tests.robots.finite_position_fixture import fixture


def api():
    assert importlib.util.find_spec('app.robots.finite_position_adapter'), 'position executor missing'
    return importlib.import_module('app.robots.finite_position_adapter')


async def prepared():
    s,c,clock=fixture()
    async def factory(ip):assert ip=='digital-twin';return c
    a=api().FinitePositionAdapter(s,backend_label='LEBAI_FAKE',client_factory=factory,
        position_model=c.model,clock=clock.now_ns,sleep=clock.sleep)
    await a.connect();a.bind_position_generation(1)
    intent=PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.))
    plan=await a.plan_step(intent,authorize=lambda:None)
    return a,c,clock,plan


async def execute(a,plan,**kw):
    return await a.execute_step(plan,before_entry=kw.get('before_entry',lambda:None),
        monitor_authorize=kw.get('monitor_authorize',lambda:None),on_entered=kw.get('on_entered',lambda:None))


async def test_one_move_reaches_goal_without_stop():
    a,c,clock,p=await prepared();r=await execute(a,p)
    assert r.complete and r.entered and not r.remote_unknown
    assert [x[0] for x in c.write_calls]==['movej']
    assert abs(r.final_sample.tcp_p[0]-.302)<1e-9
    assert clock.now_ns()>=300_000_000
    await a.disconnect()


async def test_finished_without_actual_progress_is_rejected():
    a,c,_,p=await prepared();c.no_progress=True;r=await execute(a,p)
    assert not r.complete and 'progress' in r.reason
    assert [x[0] for x in c.write_calls]==['movej','stop_move']
    await a.disconnect()


async def test_finished_task_with_lagging_endpoint_waits_for_actual_settling():
    a,c,clock,p=await prepared();original=c.advance
    def delayed_actual():
        original()
        if c.start_ns is None:return
        elapsed=clock.now_ns()-c.start_ns
        if 200_000_000<=elapsed<500_000_000:
            actual=tuple(start+.7*(goal-start) for start,goal in zip(c.start,c.goal))
            point,_=c.model.forward(actual)
            c.kin_data['actual_joint_pose']=list(actual)
            c.kin_data['actual_tcp_pose']=dict(x=point[0],y=point[1],z=point[2],rz=0.,ry=0.,rx=0.)
    c.advance=delayed_actual
    try:
        result=await execute(a,p)
        assert result.complete,result
        assert [x[0] for x in c.write_calls]==['movej']
        assert clock.now_ns()>=600_000_000
    finally:await a.disconnect()


async def test_finished_task_endpoint_lag_still_fails_at_original_deadline():
    a,c,clock,p=await prepared();original=c.advance
    def lagging():
        original()
        if c.start_ns is None:return
        actual=tuple(start+.7*(goal-start) for start,goal in zip(c.start,c.goal))
        point,_=c.model.forward(actual)
        c.kin_data['actual_joint_pose']=list(actual)
        c.kin_data['actual_tcp_pose']=dict(x=point[0],y=point[1],z=point[2],rz=0.,ry=0.,rx=0.)
    c.advance=lagging
    try:
        result=await execute(a,p)
        assert not result.complete
        assert clock.now_ns()>=2_000_000_000
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()


@pytest.mark.parametrize('kind',['joint','speed'])
async def test_finished_task_waits_for_joint_and_speed_settling(kind):
    a,c,clock,p=await prepared();original=c.advance
    def settling():
        original()
        if c.start_ns is None:return
        waiting=200_000_000<=clock.now_ns()-c.start_ns<500_000_000
        if kind=='joint' and waiting:c.kin_data['actual_joint_pose'][3]+=.003
        c.kin_data['actual_joint_speed']=[.005 if kind=='speed' and waiting else 0.,0.,0.,0.,0.,0.]
    c.advance=settling
    try:
        result=await execute(a,p)
        assert result.complete,result
        assert [x[0] for x in c.write_calls]==['movej']
        assert clock.now_ns()>=600_000_000
    finally:await a.disconnect()


@pytest.mark.parametrize('kind',['overshoot','orientation','lateral'])
async def test_waiting_for_endpoint_does_not_delay_geometric_violation(kind):
    import math
    a,c,clock,p=await prepared();original=c.advance
    def unsafe():
        original()
        if c.start_ns is None or clock.now_ns()-c.start_ns<200_000_000:return
        fraction=1.3 if kind=='overshoot' else .7
        actual=tuple(start+fraction*(goal-start) for start,goal in zip(c.start,c.goal))
        point,_=c.model.forward(actual)
        c.kin_data['actual_joint_pose']=list(actual)
        c.kin_data['actual_tcp_pose']=dict(x=point[0],y=point[1]+(.0005 if kind=='lateral' else 0.),
            z=point[2],rz=math.radians(.2) if kind=='orientation' else 0.,ry=0.,rx=0.)
    c.advance=unsafe
    expected={'overshoot':'position_endpoint_error','orientation':'position_orientation_error',
        'lateral':'position_lateral_deviation'}[kind]
    try:
        result=await execute(a,p)
        assert not result.complete and result.reason==expected
        assert clock.now_ns()<2_000_000_000
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()


@pytest.mark.parametrize('kind,complete',[('single',True),('sustained',False),('large',False),('target',False)])
async def test_bounded_acceleration_feedback_is_wired_into_step_monitoring(kind,complete):
    a,c,clock,p=await prepared();original=c.get_kin_data;reads=0
    async def data():
        nonlocal reads
        result=await original()
        if c.start_ns is not None:
            reads+=1
            field='target_joint_acc' if kind=='target' else 'actual_joint_acc'
            value=.201 if kind=='target' else .401 if kind=='large' else .3
            if kind=='single' and reads!=1:value=0.
            result[field]=[value,0.,0.,0.,0.,0.]
        return result
    c.get_kin_data=data;await clock.sleep(.001)
    try:
        r=await execute(a,p)
        assert r.complete==complete
        assert [x[0] for x in c.write_calls]==(['movej'] if complete else ['movej','stop_move'])
        if not complete:assert r.reason=='position_reported_rate_limit'
    finally:await a.disconnect()


@pytest.mark.parametrize('fraction,complete',[(.8,True),(.49,False)])
async def test_partial_following_obeys_new_arrival_contract_without_extra_targets(fraction,complete):
    a,c,_,p=await prepared();advance=c.advance
    def partial():
        advance()
        if c.start_ns is None:return
        actual=tuple(start+fraction*(now-start)
            for start,now in zip(c.start,c.kin_data['actual_joint_pose']))
        tcp,_=c.model.forward(actual)
        c.kin_data['actual_joint_pose']=list(actual)
        c.kin_data['actual_tcp_pose']=dict(x=tcp[0],y=tcp[1],z=tcp[2],rz=0.,ry=0.,rx=0.)
    c.advance=partial
    try:
        result=await execute(a,p)
        assert result.complete==complete
        assert [x[0] for x in c.write_calls]==(['movej'] if complete else ['movej','stop_move'])
        assert not result.remote_unknown
    finally:await a.disconnect()


@pytest.mark.parametrize('damage',['stale','revoked','tool','plan_copy'])
async def test_rejection_before_entry_has_no_write(damage):
    a,c,clock,p=await prepared();check=lambda:None
    if damage=='stale':await clock.sleep(.1)
    if damage=='revoked':
        def check():raise ValueError('position_revoked')
    if damage=='tool':a.settings=replace(a.settings,tcp_position_tolerance_m=.002)
    if damage=='plan_copy':p=replace(p)
    r=await execute(a,p,before_entry=check)
    assert not r.complete and not r.entered and c.write_calls==[]
    await a.disconnect()


async def test_post_entry_revoke_stops_once_without_reusing_plan():
    a,c,_,p=await prepared();current=[True]
    def entered():current[0]=False
    def monitor():
        if not current[0]:raise ValueError('position_revoked')
    r=await execute(a,p,on_entered=entered,monitor_authorize=monitor)
    assert not r.complete and len(c.write_calls)==2
    second=await execute(a,p);assert not second.entered and len(c.write_calls)==2
    await a.disconnect()


async def test_final_entry_callback_rejection_has_zero_native_writes():
    a,c,_,p=await prepared()
    def reject():raise ValueError('position_entry_revoked')
    try:
        r=await execute(a,p,on_entered=reject)
        assert not r.entered and not r.complete and c.write_calls==[]
    finally:await a.disconnect()


async def test_real_target_and_missing_injection_rejected_without_connect():
    s,_,_=fixture()
    with pytest.raises(ValueError):api().FinitePositionAdapter(replace(s,ip='172.16.2.162'))


async def test_second_submission_cannot_release_first_step_busy_gate():
    a,c,_,p=await prepared();entered=asyncio.Event();release=asyncio.Event();original=c.movej
    async def blocked(*args):
        entered.set();await release.wait();return await original(*args)
    c.movej=blocked
    first=asyncio.create_task(execute(a,p))
    await entered.wait()
    try:
        second=await execute(a,p)
        assert not second.entered and a.position_busy
    finally:release.set()
    try:assert (await first).complete
    finally:await a.disconnect()


async def test_late_motion_reply_stays_unknown_after_stop():
    a,c,_,p=await prepared();release=asyncio.Event()
    async def late(*args):
        c.write_calls.append(('movej',*args))
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return 77
    c.movej=late
    try:
        result=await asyncio.wait_for(execute(a,p),2)
        assert not result.complete and result.remote_unknown and a._latched_fault=='stop_unverified'
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
        release.set();await asyncio.sleep(0)
        assert a._latched_fault=='stop_unverified'
    finally:release.set();await a.disconnect()


async def test_foreign_task_rejected_without_second_target():
    a,c,_,p=await prepared();c.foreign=True;r=await execute(a,p)
    assert not r.complete and r.reason=='position_foreign_task'
    assert [x[0] for x in c.write_calls]==['movej','stop_move']
    await a.disconnect()


async def test_path_audit_does_not_block_release_processing(monkeypatch):
    import threading
    m=api();a,c,_,_=await prepared();order=[];released=threading.Event();started=threading.Event();original=m.audit_position_path
    def slow_audit(*args,**kwargs):
        started.set();released.wait(.15);order.append('audit_done');return original(*args,**kwargs)
    async def release():
        while not started.is_set():await asyncio.sleep(0)
        order.append('release_processed');released.set()
    monkeypatch.setattr(m,'audit_position_path',slow_audit)
    intent=PositionIntent('web',1,2,a._clock(),'held',(1.,0.,0.))
    task=asyncio.create_task(release())
    try:
        try:await a.plan_step(intent,authorize=lambda:None)
        except ValueError:pass
        await task
        assert order==['release_processed','audit_done']
    finally:released.set();await a.disconnect()


@pytest.mark.parametrize('damage,reason',[
    ('workspace','position_tcp_outside_startup_envelope'),('joint','position_joint_outside_soft_limits'),
    ('speed','position_reported_rate_limit'),('acceleration','position_reported_rate_limit')])
async def test_inflight_feedback_limits_abort_before_natural_finish(damage,reason):
    a,c,clock,p=await prepared();original=c.get_kin_data
    async def damaged():
        value=await original()
        if c.write_calls and c.write_calls[-1][0]=='movej':
            if damage=='workspace':value['actual_tcp_pose']={**value['actual_tcp_pose'],'x':.6}
            if damage=='joint':value['actual_joint_pose']=[2.99,-1.,1.,0.,1.57,0.]
            if damage=='speed':value['actual_joint_speed']=[.3,0.,0.,0.,0.,0.]
            if damage=='acceleration':value['actual_joint_acc']=[1.,0.,0.,0.,0.,0.]
        return value
    c.get_kin_data=damaged
    try:
        r=await execute(a,p)
        assert not r.complete and r.reason==reason
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()


async def test_new_observer_pose_invalidates_previously_planned_step():
    a,c,_,p=await prepared()
    c.kin_data['actual_joint_pose'][0]+=.00001
    await a._read_snapshot(read_purpose='observer')
    try:
        r=await execute(a,p)
        assert not r.entered and r.reason=='position_observer_changed' and c.write_calls==[]
    finally:await a.disconnect()


async def test_zero_reported_speed_does_not_certify_a_wandering_endpoint():
    a,c,_,p=await prepared();original=c.get_kin_data;count=0
    async def wandering():
        nonlocal count
        data=await original()
        if c.start_ns is not None and c.motion_state=='FINISHED':
            count+=1;offset=.00004*(1 if count%2 else -1)
            data['actual_joint_pose']=[p.goal_joints[0]+offset,*p.goal_joints[1:]]
            data['actual_tcp_pose']={**data['actual_tcp_pose'],'x':p.target_p[0]+offset}
        return data
    c.get_kin_data=wandering
    try:
        r=await execute(a,p)
        assert not r.complete and [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()


async def test_prepare_home_and_claw_reject_while_position_step_owns_motion():
    a,c,_,p=await prepared();entered=asyncio.Event();release=asyncio.Event();original=c.movej
    async def blocked(*args):
        entered.set();await release.wait();return await original(*args)
    c.movej=blocked;task=asyncio.create_task(execute(a,p));await entered.wait()
    try:
        for method in (a.prepare,a.home,a.set_gripper):
            with pytest.raises(Exception,match='position_busy'):await method(None)
        assert c.write_calls==[]
    finally:
        release.set();await task;await a.disconnect()


async def test_final_read_crossing_completion_deadline_cannot_certify_arrival():
    a,c,clock,p=await prepared();advance=c.advance;state=c.get_motion_state
    def late_finish():
        advance()
        if c.start_ns is not None and clock.now_ns()-c.start_ns<1_860_000_000:
            c.robot_state='MOVING';c.motion_state='RUNNING';c.running_motion=7
    async def delayed(*args):
        await clock.sleep(.04);return await state(*args)
    c.advance=late_finish;c.get_motion_state=delayed
    try:
        r=await execute(a,p)
        assert not r.complete and [x[0] for x in c.write_calls]==['movej','stop_move']
    finally:await a.disconnect()
