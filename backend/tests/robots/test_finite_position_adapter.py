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
    assert abs(r.final_sample.tcp_p[0]-.3005)<1e-9
    assert clock.now_ns()>=300_000_000
    await a.disconnect()


async def test_finished_without_actual_progress_is_rejected():
    a,c,_,p=await prepared();c.no_progress=True;r=await execute(a,p)
    assert not r.complete and 'progress' in r.reason
    assert [x[0] for x in c.write_calls]==['movej','stop_move']
    await a.disconnect()


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
