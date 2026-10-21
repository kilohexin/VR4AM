"""Injected clients only. Never load SDK or connect to a robot."""
import asyncio
import importlib
import math
from dataclasses import replace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import Pose
from tests.robots.finite_position_fixture import PositionClient,fixture as linear_fixture
from tests.commissioning.test_idle_stop_baseline import Recorder


def api():
    assert importlib.util.find_spec('app.commissioning.finite_position_site'),'single finite position site entry missing'
    return importlib.import_module('app.commissioning.finite_position_site')


def fixture():
    m=api();s,_,clock=linear_fixture()
    s=replace(s,ip='127.0.0.1',teleop_ready_q=s.home_q,
        control=replace(s.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    model=m.MdhPositionModel.from_settings(s);p,q=model.forward(s.home_q)
    s=replace(s,startup_tcp_min_m=tuple(x-.01 for x in p),startup_tcp_max_m=tuple(x+.01 for x in p))
    class Client(PositionClient):
        async def get_velocity_factor(self):return 100
        async def kinematics_forward(self,joints):
            p,q=model.forward(joints);return pose_to_lebai(Pose(p=p,q=q))
        async def kinematics_inverse(self,pose,joints):
            from app.robots.lebai_codec import pose_from_lebai
            target=pose_from_lebai(pose);joints=np.array(joints)
            for _ in range(10):
                p,q=model.forward(joints)
                error=np.r_[np.array(target.p)-p,(Rotation.from_quat(target.q)*Rotation.from_quat(q).inv()).as_rotvec()]
                if np.linalg.norm(error)<1e-10:break
                joints+=np.linalg.solve(model.jacobian(joints),error)
            return [float(x) for x in joints]
        def advance(self):
            if self.start_ns is None:return
            u=min(1.,max(0.,(clock.now_ns()-self.start_ns)/200_000_000))
            actual=self.start if self.no_progress else tuple(a+(b-a)*u for a,b in zip(self.start,self.goal))
            p,q=model.forward(actual)
            for side in ('actual','target'):
                self.kin_data[side+'_joint_pose']=list(actual if side=='actual' else self.goal)
                self.kin_data[side+'_tcp_pose']=pose_to_lebai(Pose(p=p,q=q))
            self.robot_state='MOVING' if u<1 else 'IDLE';self.motion_state='RUNNING' if u<1 else 'FINISHED'
            self.running_motion=(999 if self.foreign else 7) if u<1 else 0
    c=Client(clock);c.model=model
    for side in ('actual','target'):c.kin_data[side+'_tcp_pose']=pose_to_lebai(Pose(p=p,q=q))
    return s,c,clock,Recorder(),model


async def run(data=None,**kwargs):
    s,c,clock,recorder,model=data or fixture();connections=[]
    async def factory(ip):connections.append(ip);return c
    result=await api().run_single_position(s,recorder,factory,kwargs.pop('authorize',lambda:None),
        config_sha256='a'*64,model=model,clock=clock.now_ns,sleep=clock.sleep,**kwargs)
    return result,c,connections


async def test_one_fixed_half_mm_z_natural_arrival_without_stop_or_continuation():
    r,c,connections=await run()
    assert r['complete'],r
    assert len(connections)==1 and r['movej_entries']==1 and r['stop_entries']==0
    assert r['ik_entries']==1 and r['fk_entries']==5 and r['tail_complete']
    assert [x[0] for x in c.write_calls]==['movej'] and c.write_calls[0][-2:]==(0.,0.)
    assert r['natural_arrival_confirmed'] and not r['hardware_verified'] and not r['continuous_control_admitted']


@pytest.mark.parametrize('damage',['paused','pose','tool','factor','moving'])
async def test_bad_preflight_never_writes(damage):
    data=fixture();s,c,clock,recorder,model=data
    if damage=='paused':c.robot_state='PAUSED'
    if damage=='pose':c.kin_data['actual_joint_pose'][0]+=.002
    if damage=='tool':c.tcp['z']+=.01
    if damage=='moving':c.kin_data['actual_joint_speed'][0]=1e-6
    if damage=='factor':
        async def factor():return 50
        c.get_velocity_factor=factor
    r,c,_=await run(data)
    assert not r['complete'] and r['movej_entries']==r['stop_entries']==0 and c.write_calls==[]


async def test_slow_planning_refreshes_start_and_naturally_finishes_without_stop():
    data=fixture();s,c,clock,recorder,model=data;original=c.kinematics_forward
    async def slow(q):await clock.sleep(.025);return await original(q)
    c.kinematics_forward=slow;r,c,_=await run(data)
    assert r['complete'],r
    assert r['movej_entries']==1 and r['stop_entries']==0 and r['fk_entries']==5
    assert [x[0] for x in c.write_calls]==['movej']


@pytest.mark.parametrize('damage',['no_progress','foreign'])
async def test_inflight_failure_uses_only_one_original_stop(damage):
    data=fixture();setattr(data[1],damage,True);r,c,_=await run(data)
    assert not r['complete'] and r['movej_entries']==r['stop_entries']==1
    assert [x[0] for x in c.write_calls]==['movej','stop_move']


async def test_revocation_before_connection_is_zero_access():
    def deny():raise ValueError('position_site_revoked')
    r,c,connections=await run(authorize=deny)
    assert not r['complete'] and connections==[] and c.write_calls==[]


async def test_standalone_connection_does_not_start_background_stream():
    m=api();s,c,clock,_,model=fixture()
    async def factory(ip):return c
    a=m._SiteAdapter(s,client_factory=factory,backend_label='LEBAI',position_model=model,clock=clock.now_ns,sleep=clock.sleep)
    try:
        await a.connect()
        assert not a._pump.running and not a.position_candidate
    finally:await a.disconnect()


async def test_revocation_at_proxy_boundary_has_proof_of_zero_native_writes(monkeypatch):
    m=api();data=fixture();allowed=True;original=m._SingleClient.movej
    def authorize():
        if not allowed:raise ValueError('position_site_test_revoked')
    async def revoke(self,*args):
        nonlocal allowed
        allowed=False;return await original(self,*args)
    monkeypatch.setattr(m._SingleClient,'movej',revoke)
    r,c,_=await run(data,authorize=authorize)
    assert not r['complete'] and r['movej_entries']==r['stop_entries']==0
    assert not r['remote_unknown'] and not r['latched_fault'] and not c.write_calls


async def test_late_native_move_cannot_clear_unknown_or_cause_a_second_motion():
    data=fixture();c=data[1];release=asyncio.Event()
    async def late(*args):
        c.write_calls.append(('movej',*args))
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return 7
    c.movej=late
    try:
        r,c,_=await asyncio.wait_for(run(data),3)
        assert not r['complete'] and r['remote_unknown'] and r['latched_fault']=='stop_unverified'
        assert [x[0] for x in c.write_calls]==['movej','stop_move']
        release.set();await asyncio.sleep(0)
        assert r['latched_fault']=='stop_unverified' and len(c.write_calls)==2
    finally:release.set();await asyncio.sleep(0)


async def test_model_change_during_inverse_never_reaches_native_motion():
    data=fixture();c=data[1];model=data[-1];original=c.kinematics_inverse
    async def changed(*args):
        result=await original(*args);model.model_sha256='0'*64;return result
    c.kinematics_inverse=changed;r,c,_=await run(data)
    assert not r['complete'] and r['reason']=='position_site_model_changed' and not c.write_calls


async def test_tail_task_reply_cannot_validate_an_expired_snapshot():
    data=fixture();c=data[1];clock=data[2];original=c.get_motion_state
    async def delayed(*args):
        if c.start_ns is not None and clock.now_ns()-c.start_ns>350_000_000:await clock.sleep(.15)
        return await original(*args)
    c.get_motion_state=delayed;r,c,_=await run(data)
    assert not r['complete'] and not r['tail_complete'] and r['reason']=='position_site_tail_feedback_expired'
    assert [x[0] for x in c.write_calls]==['movej','stop_move']


def test_theoretical_cache_reuses_only_pure_math_and_returns_fresh_arrays(monkeypatch):
    m=api();model=fixture()[-1];q=(.123,-1.1,1.2,.1,1.3,.2)
    point=m.mdh_tcp_jacobian;rate=m.mdh_tcp_jacobian_rate;calls=[]
    def count_point(*args):calls.append('point');return point(*args)
    def count_rate(*args):calls.append('rate');return rate(*args)
    monkeypatch.setattr(m,'mdh_tcp_jacobian',count_point);monkeypatch.setattr(m,'mdh_tcp_jacobian_rate',count_rate)
    model.forward(q);j=model.jacobian(q);expected=j.copy();j[:]=0
    assert np.array_equal(model.jacobian(q),expected)
    first=model.jdot(q,(.025,.01,-.005,0.,0.,.002));second=model.jdot(q,(.0125,.005,-.0025,0.,0.,.001))
    assert np.allclose(second,first*.5)
    assert np.allclose(first,rate(q,(.025,.01,-.005,0.,0.,.002),model.tool)[2],rtol=1e-12,atol=1e-12)
    assert calls.count('point')==calls.count('rate')==1


async def test_static_math_initialization_precedes_first_sdk_read():
    data=fixture();c=data[1];model=data[-1];original=model.jdot;order=[]
    def observed(*args):order.append(not c.read_calls);return original(*args)
    model.jdot=observed;r,c,_=await run(data)
    assert r['complete'] and order[0] is True
