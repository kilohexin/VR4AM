"""Self-owned synthetic clients; no SDK import or real network access."""
import asyncio
import time
import pytest
from tests.commissioning.test_finite_position_site import fixture,run,api


async def test_independent_fifteen_ms_fields_fit_freshness_without_raising_ttl():
    data=fixture();c=data[1];clock=data[2]
    clock.now_ns=time.perf_counter_ns;clock.sleep=asyncio.sleep
    for name in ('is_connected','get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion'):
        original=getattr(c,name)
        async def delayed(method=original):await asyncio.sleep(.015);return await method()
        setattr(c,name,delayed)
    r,c,_=await run(data)
    assert r['complete'],r
    assert r['movej_entries']==1 and r['stop_entries']==0 and r['tail_complete']
    assert r['snapshot_read_policy']=='owned_six_fields_parallel'


async def test_each_snapshot_group_is_owned_and_all_six_finish_before_install():
    data=fixture();c=data[1];active=set();maximum=0
    for name in ('get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion'):
        original=getattr(c,name)
        async def observed(method=original,label=name):
            nonlocal maximum
            active.add(label);maximum=max(maximum,len(active))
            try:await asyncio.sleep(0);return await method()
            finally:active.remove(label)
        setattr(c,name,observed)
    r,c,_=await run(data)
    assert r['complete'],r
    assert maximum==6 and r['snapshot_groups']
    assert all(x['outcome']=='returned' and len(x['methods'])==6 for x in r['snapshot_groups'])


async def test_scope_revocation_during_group_cannot_install_feedback_or_move():
    data=fixture();c=data[1];original=c.get_tcp;allowed=True
    async def revoked():
        nonlocal allowed
        result=await original();allowed=False;return result
    def permit():
        if not allowed:raise ValueError('position_site_test_revoked')
    c.get_tcp=revoked
    r,c,_=await run(data,authorize=permit)
    assert not r['complete'] and not c.write_calls
    assert r['movej_entries']==r['stop_entries']==0


async def test_failed_group_keeps_uncancellable_native_read_quarantined():
    data=fixture();c=data[1];release=asyncio.Event();entered=asyncio.Event()
    async def late():
        entered.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return c.kin_data
    async def failed():
        await entered.wait();raise ValueError('test_read_failure')
    c.get_kin_data=late;c.get_tcp=failed
    try:
        r,c,_=await asyncio.wait_for(run(data),3.)
        assert not r['complete'] and r['remote_unknown'] and not c.write_calls
        release.set();await asyncio.sleep(0)
        assert r['remote_unknown'] and r['movej_entries']==r['stop_entries']==0
    finally:release.set();await asyncio.sleep(0)


def proxy():
    data=fixture();c=data[1];clock=data[2]
    return api()._SingleClient(c,lambda:None,lambda:None,lambda:None,asyncio.current_task(),clock.now_ns),c


async def test_duplicate_field_fails_group_and_cannot_start_another_group():
    p,c=proxy()
    with pytest.raises(ValueError,match='position_snapshot_group_duplicate'):
        with p.position_snapshot_scope(lambda:None,'test'):
            await p.get_tcp();await p.get_tcp()
    assert p.read_gate_closed and p.snapshot_groups[-1]['outcome']=='failed'
    with pytest.raises(Exception):
        with p.position_snapshot_scope(lambda:None,'next'):pass
    assert c.read_calls==['get_tcp'] and not c.write_calls


async def test_foreign_read_cannot_join_an_active_group():
    p,c=proxy();go=asyncio.Event()
    async def foreign():await go.wait();return await p.get_tcp()
    task=asyncio.create_task(foreign())  # Created outside the group's ContextVar.
    with pytest.raises(ValueError,match='position_snapshot_foreign_read'):
        with p.position_snapshot_scope(lambda:None,'test'):
            go.set();await task
    assert not c.read_calls and not c.write_calls and p.read_gate_closed


async def test_motion_is_forbidden_until_group_has_closed():
    p,c=proxy();p.fixed_move=((0.,)*6,.02,.01,0.,0.)
    with pytest.raises(ValueError,match='position_snapshot_group_still_active'):
        with p.position_snapshot_scope(lambda:None,'test'):
            await p.movej((0.,)*6,.02,.01,0.,0.)
    assert not c.write_calls and p.movej_entries==0


async def test_pending_child_stays_unknown_after_group_context_exits():
    p,c=proxy();release=asyncio.Event();entered=asyncio.Event()
    await p.is_connected()  # A completed serial parent must not mask this child.
    async def blocked():entered.set();await release.wait();return c.kin_data
    c.get_kin_data=blocked;task=None
    try:
        with pytest.raises(ValueError,match='test_close_group'):
            with p.position_snapshot_scope(lambda:None,'test'):
                task=asyncio.create_task(p.get_kin_data());await entered.wait()
                raise ValueError('test_close_group')
        assert p._native_snapshot_result_unknown('get_kin_data') is True
        assert p.read_gate_closed
    finally:
        release.set()
        if task is not None:await asyncio.gather(task,return_exceptions=True)
