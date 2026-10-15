import asyncio
import time
import pytest
from app.commissioning.native_jog_site_scope import SiteJogScope
from app.commissioning.native_jog_site_client import SiteJogClient
from app.commissioning.native_jog_site_run import run_site_window
from app.robots.lebai_codec import pose_to_lebai
from tests.commissioning.test_native_jog_site_scope import arguments, Q
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.fake_lebai import FakeLebaiClient


def setup(**kw):
    args, pose = arguments(**kw)
    scope = SiteJogScope(**args)
    client = FakeLebaiClient.idle(q=list(Q))
    client.kin_data['actual_tcp_pose'] = pose_to_lebai(pose)
    client.kin_data['target_tcp_pose'] = pose_to_lebai(pose)
    async def fk(q): return pose_to_lebai(pose)
    async def factor(): return 100
    async def speedl(*v): client.write_calls.append(('speedl', *v)); return 7
    client.kinematics_forward, client.get_velocity_factor, client.speedl = fk, factor, speedl
    return scope, client


async def run(scope=None, client=None, report=None):
    if scope is None: scope, client = setup()
    async def factory(ip): assert ip == scope.settings.ip; return client
    result = await run_site_window(scope, Recorder(), factory, clock=time.perf_counter_ns,
        sleep=asyncio.sleep, report=report)
    return result, client


async def test_normal_single_window_and_sealed_report():
    r, c = await run()
    assert r['complete'], r
    assert r['speedl_entries'] == r['stop_entries'] == 1
    assert r['fk_entries'] == r['velocity_factor_entries'] == 1
    assert r['read_entries'] <= 256
    assert [v[0] for v in c.write_calls] == ['speedl', 'stop_move']
    assert r['stop_confirmed'] and r['tail_complete']
    assert not any(r[k] for k in ('hardware_verified','joint_bounds_verified','braking_verified','continuous_control_admitted'))


@pytest.mark.parametrize('kind', ['factor','fk','state','hot','write'])
async def test_prewrite_rejection_has_zero_stop(kind):
    scope, c = setup()
    if kind == 'factor':
        async def factor(): return 50
        c.get_velocity_factor = factor
    elif kind == 'fk':
        async def fk(q): return dict(x=0,y=0,z=0,rx=0,ry=0,rz=0)
        c.kinematics_forward = fk
    elif kind == 'state': c.robot_state = 'PAUSED'
    else:
        def deny(): raise ValueError('native_jog_test_rejected')
        scope, c = setup(**{('hot_check' if kind == 'hot' else 'write_check'):deny})
    r, c = await run(scope,c)
    assert not r['complete'] and r['speedl_entries'] == r['stop_entries'] == 0
    assert not c.write_calls


async def test_global_budget_and_math_factor_caps_include_raw_surface():
    s,c=setup();p=SiteJogClient(c,s,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    await p.get_velocity_factor()
    with pytest.raises(Exception): await p.get_velocity_factor()
    # Fresh proxy: direct read-only surface is still globally metered.
    p=SiteJogClient(c,s,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    for _ in range(256): await p.client.get_robot_state()
    with pytest.raises(Exception): await p.client.get_robot_state()
    assert p.read_entries == 256
    with pytest.raises(Exception): await p.client.stop_move()
    with pytest.raises(Exception): await p.move_pvat([],[],[],.08)


async def test_parallel_group_duplicate_and_foreign_reads_rejected():
    s,c=setup();p=SiteJogClient(c,s,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    await p.get_estop_reason()
    with p._parallel_snapshot_scope(lambda:None):
        await p.get_robot_state()
        with pytest.raises(Exception): await p.get_robot_state()


async def test_changed_q_after_fk_is_not_reused():
    s,c=setup();old=c.kinematics_forward
    async def changed(q):
        result=await old(q)
        c.kin_data['actual_joint_pose'][0]+=.00001
        return result
    c.kinematics_forward=changed
    r,_=await run(s,c)
    assert r['fk_entries']==1 and r['speedl_entries']==r['stop_entries']==0


async def test_raw_speed_parameters_and_reserved_stop_cannot_bypass_scope():
    s,c=setup();p=SiteJogClient(c,s,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    with pytest.raises(Exception):await p.speedl(.01,dict(x=.005,y=0,z=0,rx=0,ry=0,rz=0),.08,{})
    with pytest.raises(Exception):await p.stop_move()
    with pytest.raises(Exception):await p.client.speedl(.01,{},.08,{})
    assert p.speedl_entries==p.sdk_stop_entries==0 and not c.write_calls


async def test_fk_budget_is_one_and_parallel_foreign_read_closes_gate():
    s,c=setup();p=SiteJogClient(c,s,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    await p.kinematics_forward(Q)
    with pytest.raises(Exception):await p.kinematics_forward(Q)
    assert p.special_entries['kinematics_forward']==1


