"""Existing prepare API with explicit injected clients; no device access."""
import asyncio
import copy
import importlib
import importlib.util
from dataclasses import replace
from unittest.mock import AsyncMock

import pytest

from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock
from tests.commissioning.test_idle_stop_baseline import Recorder


def module():
    assert importlib.util.find_spec('app.commissioning.ready_return'), 'single ready-return runner missing'
    return importlib.import_module('app.commissioning.ready_return')


def fixture():
    s=control_settings()
    s=replace(s,control=replace(s.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    start=list(s.teleop_ready_q);start[4]+=.0010546117916712117
    c=FakeLebaiClient.idle(q=start);c.running_motion=573;c.motion_state='FINISHED'
    original=c.movej
    async def converge(p,a,v,t,r):
        value=await original(p,a,v,t,r)
        for side in ('actual','target'):c.kin_data[side+'_joint_pose']=list(p)
        return value
    async def factor():return 100
    c.movej=converge;c.get_velocity_factor=factor
    return s,tuple(start),c,FakeClock(),Recorder()


async def run(data=None,**kw):
    s,q,c,clock,recorder=data or fixture();factory=AsyncMock(return_value=c)
    r=await module().run_ready_return(s,q,recorder,factory,kw.pop('authorize',lambda:None),
        clock=clock.now_ns,sleep=clock.sleep,**kw)
    return r,c,factory


async def test_one_exact_prepare_and_audit_without_stop_or_other_motion():
    data=fixture();r,c,factory=await run(data)
    assert r['complete'],r
    assert r['movej_entries']==1 and r['stop_entries']==0 and factory.await_count==1
    assert c.write_calls==[('movej',list(data[0].teleop_ready_q),.2,.05,0.,0.)]
    assert r['tail_complete'] and r['post_preflight_ready']
    assert r['speed_audit']['status']=='within_observed_limit'
    assert not r['hardware_verified'] and not r['motion_time_stop_verified']
    assert r['local_resources_released'] and not r['native_connection_release_verified']


@pytest.mark.parametrize('step_rad',[.00002,.000005])
async def test_preparation_cannot_settle_while_position_drifts_with_zero_reported_speed(monkeypatch,step_rad):
    data=fixture();c=data[2];original=c.get_kin_data;count=0
    monkeypatch.setattr(module(),'OPTIONS',replace(module().OPTIONS,timeout_s=.6))
    async def drifting():
        nonlocal count
        value=await original()
        if c.write_calls and c.write_calls[-1][0]=='movej':
            count+=1
            # Always inside the original 0.001 rad target tolerance, with a
            # quantized zero speed. The measured position is never at rest.
            value['actual_joint_pose'][3]=data[0].teleop_ready_q[3]+(count%20)*step_rad
        return value
    c.get_kin_data=drifting
    result,c,_=await run(data)
    assert not result['complete'],result
    assert not result.get('post_preflight_ready',False)
    assert result['movej_entries']==1 and result['stop_entries']==1


async def test_preparation_waits_until_drift_ends_then_returns_without_stop():
    data=fixture();c=data[2];clock=data[3];original=c.get_kin_data;first=None;count=0
    async def settling():
        nonlocal first,count
        value=await original()
        if c.write_calls:
            if first is None:first=clock.now_ns()
            count+=1
            offset=(count%20)*.00002 if clock.now_ns()-first<600_000_000 else 0.
            value['actual_joint_pose'][3]=data[0].teleop_ready_q[3]+offset
        return value
    c.get_kin_data=settling
    result,c,_=await run(data)
    assert result['complete'],{k:result.get(k) for k in ('error','error_type','movej_entries',
        'stop_entries','speed_audit','duration_s','tail_complete','post_preflight_ready')}
    assert result['duration_s']>=2.9  # 0.6s drift, 0.3s stable, 2s tail.
    assert result['movej_entries']==1 and result['stop_entries']==0
    assert 512<len(result['read_calls'])<=module().MAX_READS


@pytest.mark.parametrize('damage',['paused','start','tool','actual_rate','target_rate','factor','estop','running'])
async def test_bad_initial_reading_has_zero_move_and_zero_stop(damage):
    data=fixture();s,q,c,clock,recorder=data
    if damage=='paused':c.robot_state='PAUSED'
    elif damage=='start':
        c.kin_data['actual_joint_pose'][0]+=.002
    elif damage=='tool':c.tcp['z']+=.01
    elif damage=='actual_rate':c.kin_data['actual_joint_speed'][0]=1e-9
    elif damage=='target_rate':c.kin_data['target_joint_acc'][0]=1e-9
    elif damage=='factor':
        async def factor():return 50
        c.get_velocity_factor=factor
    elif damage=='estop':c.estop_reason='ESTOP'
    else:c.motion_state='RUNNING'
    r,c,_=await run(data)
    assert not r['complete'] and r['movej_entries']==r['stop_entries']==0
    assert not c.write_calls


async def test_expired_before_connect_never_calls_factory():
    def deny():raise ValueError('ready_return_test_expired')
    r,c,factory=await run(authorize=deny)
    assert not r['complete'] and not c.write_calls
    factory.assert_not_called()


async def test_revocation_at_prepare_phase_before_native_entry_has_zero_writes():
    data=fixture();allowed=True;recorder=data[-1];old=recorder.write_critical_event
    async def record(kind,payload,stamp):
        nonlocal allowed
        await old(kind,payload,stamp)
        if kind=='ready_return_admission':allowed=False
    recorder.write_critical_event=record
    def check():
        if not allowed:raise ValueError('ready_return_test_revoked')
    r,c,_=await run(data,authorize=check)
    assert not r['complete'] and not c.write_calls


async def test_reported_overspeed_after_arrival_is_not_success_or_retry():
    data=fixture();c=data[2];read=c.get_kin_data
    async def overspeed():
        value=await read()
        if c.write_calls:value['actual_joint_speed']=[.06,0,0,0,0,0]
        # One audit sample above the request, then settle normally.
        if c.write_calls:c.get_kin_data=read
        return value
    c.get_kin_data=overspeed
    r,c,_=await run(data)
    assert not r['complete'] and r['speed_audit']['status']=='exceeded'
    assert r['movej_entries']==1 and r['stop_entries']==0
    assert [x[0] for x in c.write_calls]==['movej']


async def test_cancelled_native_move_with_late_return_retains_unknown_and_one_stop():
    module()
    data=fixture();c=data[2];entered=asyncio.Event();release=asyncio.Event();report={}
    async def late(*args):
        c.write_calls.append(('movej',*args));entered.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return 77
    c.movej=late
    task=asyncio.create_task(run(data,report=report))
    try:
        await asyncio.wait_for(entered.wait(),1);task.cancel()
        with pytest.raises(asyncio.CancelledError):await asyncio.wait_for(task,2)
        assert report['movej_entries']==report['stop_entries']==1
        assert report['movej_remote_result_unknown'] and report['latched_fault']=='stop_unverified'
        sealed=copy.deepcopy(report);release.set()
        for _ in range(6):await asyncio.sleep(0)
        assert report==sealed and len(c.write_calls)==2
    finally:release.set()


async def test_authority_lost_after_move_uses_original_stop_and_no_second_target():
    data=fixture();c=data[2];old=c.movej;allowed=True
    async def revoked(*args):
        nonlocal allowed
        value=await old(*args);allowed=False;return value
    c.movej=revoked
    def check():
        if not allowed:raise ValueError('ready_return_test_revoked')
    r,c,_=await run(data,authorize=check)
    assert not r['complete'] and r['movej_entries']==r['stop_entries']==1
    assert [x[0] for x in c.write_calls]==['movej','stop_move']


async def test_recorder_close_failure_cannot_leave_complete_true():
    data=fixture()
    async def failed():raise RuntimeError('private recorder failure')
    data[-1].close=failed
    r,c,_=await run(data)
    assert not r['complete'] and 'RuntimeError' in r['cleanup_errors']
    assert 'private recorder failure' not in str(r)
    assert r['movej_entries']==1 and r['stop_entries']==0
    event=next(x for x in data[-1].events if x['kind']=='ready_return_result')
    assert event['complete'] is False and event['recording_closed_verified'] is False


async def test_late_movej_timeout_is_bounded_without_waiting_for_cancel_ack():
    data=fixture();c=data[2];release=asyncio.Event()
    async def late(*args):
        c.write_calls.append(('movej',*args))
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return 91
    c.movej=late
    try:
        r,c,_=await asyncio.wait_for(run(data),1)
        assert not r['complete'] and r['movej_remote_result_unknown']
        assert r['movej_entries']==r['stop_entries']==1 and r['latched_fault']=='stop_unverified'
    finally:
        release.set()
        for _ in range(5):await asyncio.sleep(0)


async def test_zero_native_entry_keeps_the_actual_local_rejection_reason():
    def reject():raise ValueError('ready_return_test_write_rejected')
    r,c,_=await run(write_authorize=reject)
    assert not c.write_calls and r['movej_entries']==r['stop_entries']==0
    assert r['movej_entry_rejection']=='ready_return_test_write_rejected'
    assert r['error']=='ready_return_test_write_rejected'


async def test_one_permission_poll_per_read_guard_when_same_callback_is_supplied(monkeypatch):
    m=module();old=m._PrepareClient.read_guard;count=0;polls=[]
    def permission():
        nonlocal count
        count+=1
    def guard(self):
        before=count;old(self);polls.append(count-before)
    monkeypatch.setattr(m._PrepareClient,'read_guard',guard)
    r,c,_=await run(authorize=permission,read_authorize=permission)
    assert r['complete'] and polls and set(polls)=={1}
