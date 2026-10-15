"""Bounded read-only survey against an injected synthetic client only."""
import asyncio
import copy
from dataclasses import replace
import json
from unittest.mock import AsyncMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.commissioning import readonly_jog_jacobian as survey
from app.commissioning import readonly_kinematics as original
from app.commissioning.native_jog_mdh import MdhTool
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import Pose
from tests.commissioning.test_native_jog_mdh import independent_transform
from tests.commissioning.test_readonly_kinematics import settings
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q


class JacobianClient(FakeLebaiClient):
    def __init__(self):
        super().__init__()
        self.running_motion=0
        self.q=tuple(IDLE_Q)
        self.tool=MdhTool((0.,0.,.175),(0.,0.,0.,1.))
        self.fk_inputs=[]
        self.kin_data['actual_tcp_pose']=self.fk(self.q)
        self.factor=100

    def fk(self,q):
        t=independent_transform(q,self.tool)
        return pose_to_lebai(Pose(p=tuple(t[:3,3]),q=tuple(Rotation.from_matrix(t[:3,:3]).as_quat())))

    async def kinematics_forward(self,q):
        self.read_calls.append('kinematics_forward')
        self.fk_inputs.append(list(q))
        return self.fk(q)

    async def get_velocity_factor(self):
        self.read_calls.append('get_velocity_factor')
        return self.factor

    async def kinematics_inverse(self,*args):
        pytest.fail('IK is outside scope')


async def collect(settings,client,*,admit=lambda:None):
    report=survey.new_report(simulation=True)
    return await survey.collect(settings,AsyncMock(return_value=client),admit,report)


@pytest.mark.parametrize('state',['IDLE','PAUSED',5,6])
async def test_fixed_scope_collects_six_independent_directions_without_writes(settings,state):
    client=JacobianClient();client.robot_state=state
    r=await collect(settings,client)
    assert r['complete'] and r['feedback_fk_consistent'] and r['jacobian_collected']
    assert len(r['calls'])==27 and len(client.fk_inputs)==13
    assert client.fk_inputs[0]==list(IDLE_Q)
    for i in range(6):
        minus,plus=client.fk_inputs[1+2*i:3+2*i]
        want=np.zeros(6);want[i]=.001
        np.testing.assert_allclose(np.array(plus)-IDLE_Q,want,atol=2e-16)
        np.testing.assert_allclose(np.array(minus)-IDLE_Q,-want,atol=2e-16)
    assert np.max(np.abs(np.array(r['controller_jacobian'])-r['theoretical_jacobian']))<2e-7
    assert client.write_calls==[] and client.network_calls==client.discovery_calls==0
    assert not r['motion_authorized'] and not r['continuous_control_admitted']
    assert not r['hardware_stop_verified'] and not r['joint_bounds_verified']


@pytest.mark.parametrize('damage',['moving','estop','running','unknown_running','speed','tcp','factor','bool_factor','joints','limit'])
async def test_bad_initial_observation_has_no_fk_or_cleanup_writes(settings,damage):
    c=JacobianClient()
    if damage=='moving':c.robot_state='MOVING'
    elif damage=='estop':c.estop_reason=4
    elif damage=='running':c.running_motion=12;c.motion_state='RUNNING'
    elif damage=='unknown_running':c.running_motion=None
    elif damage=='speed':c.kin_data['actual_joint_speed'][0]=.00001
    elif damage=='tcp':c.tcp['z']+=.002
    elif damage=='factor':c.factor=101
    elif damage=='bool_factor':c.factor=True
    elif damage=='joints':c.kin_data['actual_joint_pose'][0]=True
    else:c.kin_data['actual_joint_pose'][0]=settings.soft_joint_max_rad[0]-settings.joint_limit_margin_rad-.0005
    r=await collect(settings,c)
    assert not r['complete'] and not c.fk_inputs and c.write_calls==[]


async def test_retained_finished_id_is_queried_twice_within_29_calls(settings):
    c=JacobianClient();c.running_motion=503;queried=[]
    async def status(motion_id):
        queried.append(motion_id)
        return 'FINISHED'
    c.get_motion_state=status
    r=await collect(settings,c)
    assert r['complete'] and r['jacobian_collected']
    assert queried==[503,503] and len(r['calls'])==29
    assert r['initial']['running_motion']==r['final']['running_motion']==503
    assert r['initial']['motion_state']==r['final']['motion_state']=='FINISHED'
    assert len(c.fk_inputs)==13 and c.write_calls==[]
    assert not r['hardware_stop_verified'] and not r['continuous_control_admitted']


@pytest.mark.parametrize('value',['RUNNING','WAITING','PAUSED','UNKNOWN',None,True,2])
async def test_retained_id_requires_explicit_finished_before_any_fk(settings,value):
    c=JacobianClient();c.running_motion=503;c.motion_state=value
    r=await collect(settings,c)
    assert r['error']=='jog_survey_running_motion_not_finished'
    assert not r['complete'] and not c.fk_inputs and c.write_calls==[]
    assert [row['method'] for row in r['calls']][-1]=='get_motion_state'


@pytest.mark.parametrize('value',[-1,True,'503',None,2**32])
async def test_invalid_motion_id_never_enters_motion_state_query(settings,value):
    c=JacobianClient();c.running_motion=value
    r=await collect(settings,c)
    assert r['error']=='jog_survey_invalid_running_motion'
    assert not c.fk_inputs and c.write_calls==[]
    assert not any(row['method']=='get_motion_state' for row in r['calls'])


async def test_finished_motion_id_change_during_survey_cannot_certify_data(settings):
    c=JacobianClient();c.running_motion=503;old=c.kinematics_forward
    async def change(q):
        value=await old(q)
        if len(c.fk_inputs)==13:c.running_motion=504
        return value
    c.kinematics_forward=change
    r=await collect(settings,c)
    assert not r['complete'] and not r['jacobian_collected']
    assert r['error']=='jog_survey_observation_changed'
    assert len(c.fk_inputs)==13 and len(r['calls'])==29 and c.write_calls==[]


async def test_retained_id_cannot_become_running_after_fk(settings):
    c=JacobianClient();c.running_motion=503;old=c.kinematics_forward
    async def change(q):
        value=await old(q)
        if len(c.fk_inputs)==13:c.motion_state='RUNNING'
        return value
    c.kinematics_forward=change
    r=await collect(settings,c)
    assert not r['complete'] and r['error']=='jog_survey_running_motion_not_finished'
    assert len(c.fk_inputs)==13 and c.write_calls==[]


async def test_feedback_fk_mismatch_ends_after_single_query(settings):
    c=JacobianClient();c.kin_data['actual_tcp_pose']['x']+=.002
    r=await collect(settings,c)
    assert not r['complete'] and len(c.fk_inputs)==1 and c.write_calls==[]
    assert r['error']=='jog_survey_fk_feedback_mismatch'


@pytest.mark.parametrize('damage',['state','position','tcp','factor'])
async def test_final_change_cannot_certify_derivatives(settings,damage):
    c=JacobianClient();old=c.kinematics_forward
    async def change(q):
        value=await old(q)
        if len(c.fk_inputs)==13:
            if damage=='state':c.robot_state='PAUSED'
            elif damage=='position':c.kin_data['actual_joint_pose'][0]+=.002
            elif damage=='tcp':c.tcp['z']+=.00001
            else:c.factor=50
        return value
    c.kinematics_forward=change
    r=await collect(settings,c)
    assert not r['complete'] and not r['jacobian_collected']
    assert len(c.fk_inputs)==13 and c.write_calls==[]


async def test_authority_revocation_before_later_fk_stops_without_retry(settings):
    c=JacobianClient();old=c.kinematics_forward;allowed=True
    async def revoke(q):
        nonlocal allowed
        value=await old(q);allowed=False;return value
    c.kinematics_forward=revoke
    def admit():
        if not allowed:raise ValueError('revoked')
    r=await collect(settings,c,admit=admit)
    assert not r['complete'] and len(c.fk_inputs)==1 and c.write_calls==[]


@pytest.mark.parametrize('method',['is_connected','get_velocity_factor','kinematics_forward','get_motion_state'])
async def test_timeout_stubborn_read_never_waits_for_remote_cancel(settings,monkeypatch,method):
    c=JacobianClient();release=asyncio.Event()
    if method=='get_motion_state':c.running_motion=503
    async def stuck(*args):
        try:await release.wait()
        except asyncio.CancelledError:await release.wait()
        return 100
    setattr(c,method,stuck)
    monkeypatch.setattr(original,'READ_TIMEOUT_S',.01)
    try:
        r=await asyncio.wait_for(collect(settings,c),.3)
        assert not r['complete'] and r['pending_remote_result_unknown']
        frozen=json.dumps(r,sort_keys=True)
        release.set();await asyncio.sleep(.02)
        assert json.dumps(r,sort_keys=True)==frozen
        assert sum(row['method']==method for row in r['calls'])==1
        assert c.write_calls==[]
    finally:
        release.set();await asyncio.sleep(0)


async def test_revocation_after_retained_id_prevents_motion_state_entry(settings):
    c=JacobianClient();allowed=True;entered=[]
    async def running():
        nonlocal allowed
        allowed=False
        return 503
    async def status(motion_id):
        entered.append(motion_id)
        return 'FINISHED'
    def admit():
        if not allowed:raise ValueError('revoked')
    c.get_running_motion=running;c.get_motion_state=status
    r=await collect(settings,c,admit=admit)
    assert not r['complete'] and not entered and not c.fk_inputs and c.write_calls==[]


async def test_control_mode_refused_before_connection(settings):
    factory=AsyncMock()
    with pytest.raises(ValueError):
        await survey.collect(replace(settings,mode='control'),factory,lambda:None,survey.new_report(simulation=True))
    factory.assert_not_called()


async def test_old_query_policy_does_not_allow_new_read_method():
    r=original.new_report(simulation=True);q=original._Queries(lambda:None,r)
    with pytest.raises(original.SurveyError):await q.call('get_velocity_factor',AsyncMock(return_value=100))
    assert r['calls']==[]


async def test_analysis_cannot_mark_complete_after_total_budget(settings,monkeypatch):
    clock=[0]
    monkeypatch.setattr(original,'monotonic_ns',lambda:clock[0])
    old=survey._finish_analysis
    def slow(*args):
        old(*args)
        clock[0]=15_000_000_000
    monkeypatch.setattr(survey,'_finish_analysis',slow)
    r=await collect(settings,JacobianClient())
    assert not r['complete'] and not r['jacobian_collected']
    assert r['error']=='survey_budget_expired'


async def test_cancelled_owner_preserves_unknown_and_no_late_report_changes(settings):
    c=JacobianClient();entered=asyncio.Event();release=asyncio.Event()
    async def stuck(q):
        entered.set()
        try:await release.wait()
        except asyncio.CancelledError:await release.wait()
        return c.fk(q)
    c.kinematics_forward=stuck
    report=survey.new_report(simulation=True)
    task=asyncio.create_task(survey.collect(settings,AsyncMock(return_value=c),lambda:None,report))
    try:
        await asyncio.wait_for(entered.wait(),1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):await asyncio.wait_for(task,.3)
        assert report['pending_remote_result_unknown'] and not report['complete']
        frozen=json.dumps(report,sort_keys=True)
        release.set();await asyncio.sleep(.01)
        assert json.dumps(report,sort_keys=True)==frozen
        assert c.write_calls==[]
    finally:release.set()
