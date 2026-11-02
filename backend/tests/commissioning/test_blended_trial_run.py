"""Whole scoped runner with an analytical in-memory controller; no SDK/network."""
import asyncio
import time
from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from scripts import real_robot_blended_trial as entry
from tests.commissioning.test_finite_position_site import fixture


@pytest.mark.parametrize('case',['normal','no_progress','unknown','bad_fk','wrong_ready','cancel_before','cancel_after'])
async def test_full_trial(case,tmp_path,monkeypatch):
    s,c,clock,_,model=fixture()
    s=replace(s,ip='192.0.2.1')
    writes=[];c.advance=lambda:None
    async def movel(p,a,v,t,r):
        writes.append('movel')
        if case=='unknown':raise TimeoutError('injected')
        if case=='cancel_after':raise asyncio.CancelledError()
        if case!='no_progress':
            q=await c.kinematics_inverse(p,c.kin_data['actual_joint_pose'])
            for side in ('actual','target'):
                c.kin_data[side+'_joint_pose']=q
                c.kin_data[side+'_tcp_pose']=p
        c.robot_state='IDLE';c.running_motion=0;c.motion_state='FINISHED'
        return len(writes)+10
    c.movel=movel
    if case=='cancel_before':
        async def ik(*args):raise asyncio.CancelledError()
        c.kinematics_inverse=ik
    if case=='wrong_ready':s=replace(s,teleop_ready_q=tuple(q+.01 for q in s.teleop_ready_q))
    if case=='bad_fk':
        original=c.kinematics_forward
        async def fk(q):
            p=await original(q);p['z']+=.01;return p
        c.kinematics_forward=fk
    monkeypatch.setattr(entry.single,'existing_rpc_connections',lambda:0)
    monkeypatch.setattr(entry,'preview',lambda p:{'injected':True})
    async def factory(ip):assert ip=='192.0.2.1';return c
    p=dict(settings=s,model=model,output=tmp_path/'run')
    inputs=NS(cold=lambda:None,read=lambda:None,write=lambda:None)
    result=await entry.run(p,inputs,client_factory=factory)
    if case=='normal':
        assert result['complete'],result
        assert result['counts']['movel']==2 and result['counts'].get('stop_move',0)==0
        assert len(result['tail'])>=3
    else:
        assert not result['complete'],result
        assert result['counts'].get('movel',0)<=2
    if case in ('bad_fk','wrong_ready','cancel_before'):assert not writes
    if case=='unknown':assert result['unknown_writes']==['movel'] and writes==['movel']
    if case=='cancel_after':assert result['unknown_writes']==['movel'] and writes==['movel']
    assert (p['output']/'result.json').exists()
