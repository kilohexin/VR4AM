"""Offline phase checks: old feedback cannot renew input or grant another write."""
import asyncio
import copy
from dataclasses import replace

import pytest

from app.commissioning import native_jog_packet as packets
from app.commissioning import native_jog_site_run as workflow
from tests.commissioning.test_native_jog_packet import packet, SHA
from tests.commissioning.test_native_jog_site_run import setup
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.test_lebai_adapter_control import FakeClock


def test_inflight_input_check_keeps_original_deadline_without_feedback_retimestamp():
    p=packet();frozen=copy.deepcopy(p)
    packets.validate_inflight_input(p,now_ns=1_050_000_000,generation=0,config_sha256=SHA)
    assert p==frozen and p.expires_ns==1_100_000_000


@pytest.mark.parametrize('change',[dict(now_ns=1_100_000_000),dict(now_ns=999_999_999),
    dict(generation=1),dict(config_sha256='b'*64),dict(now_ns=True)])
def test_inflight_input_still_rejects_expiry_future_and_changed_identity(change):
    args=dict(now_ns=1_050_000_000,generation=0,config_sha256=SHA);args.update(change)
    with pytest.raises(ValueError):packets.validate_inflight_input(packet(),**args)


async def phase_run(*,before_ms=70,reply_ms=40,change=None,monkeypatch=None):
    clock=FakeClock();fk_done=False;aged=False;entered=False;denied=False;holder={}
    def hot():
        nonlocal aged
        if fk_done and not aged:clock.advance_ms(before_ms);aged=True
    def read():
        if denied:raise ValueError('native_jog_explicit_revocation')
    s,c=setup(hot_check=hot,read_check=read);fk=c.kinematics_forward;speed=c.speedl
    if change=='no_redundant_math':
        from app.commissioning.native_jog_site_scope import _SiteRegionGuard
        original_validate=_SiteRegionGuard.validate
        def validate(self,*args,**kwargs):
            assert not entered,'already-admitted geometry recomputed while awaiting RPC'
            return original_validate(self,*args,**kwargs)
        monkeypatch.setattr(_SiteRegionGuard,'validate',validate)
    async def measured_fk(q):
        nonlocal fk_done
        value=await fk(q);fk_done=True;return value
    c.kinematics_forward=measured_fk
    async def measured_speed(*args):
        nonlocal entered,denied
        value=await speed(*args);entered=True
        clock.advance_ms(reply_ms)
        if change=='revoke':denied=True
        elif change=='config':object.__setattr__(s.settings.control,'max_joint_speed_radps',.04)
        elif change=='renew':
            object.__setattr__(holder['packet'],'input_ns',clock.now_ns())
            object.__setattr__(holder['packet'],'expires_ns',clock.now_ns()+100_000_000)
        elif change=='generation':holder['adapter']._pump.invalidate()
        elif change=='tool':holder['adapter']._snapshot.tcp_setting.p=(0.,0.,.2)
        elif change=='feedback':
            pose=holder['adapter']._snapshot.actual_tcp
            pose.p=(pose.p[0]+.00001,*pose.p[1:])
        elif change=='snapshot_stamp':object.__setattr__(holder['adapter']._snapshot,'captured_ns',clock.now_ns())
        await asyncio.sleep(0)
        return value
    c.speedl=measured_speed
    if change=='renew':
        original=workflow.plan_jog
        def captured(*args,**kwargs):holder['packet']=original(*args,**kwargs);return holder['packet']
        monkeypatch.setattr(workflow,'plan_jog',captured)
    if change in ('generation','tool','feedback','snapshot_stamp'):
        original_submit=workflow.SiteJogDiagnosticAdapter.submit_jog
        async def captured_adapter(self,*args,**kwargs):
            holder['adapter']=self
            return await original_submit(self,*args,**kwargs)
        monkeypatch.setattr(workflow.SiteJogDiagnosticAdapter,'submit_jog',captured_adapter)
    async def sleep(seconds):
        # Let immediate observer/read completions run before advancing a timer.
        for _ in range(20):await asyncio.sleep(0)
        await clock.sleep(seconds)
    async def factory(ip):return c
    recorder=Recorder()
    r=await workflow.run_site_window(s,recorder,factory,clock=clock.now_ns,sleep=sleep)
    ownership=[e for e in recorder.events if e.get('kind')=='motion_rpc_ownership']
    return r,c,ownership,entered


async def test_feedback_can_age_during_known_rpc_but_not_authorize_a_new_send():
    r,c,owned,entered=await phase_run()
    assert entered and r['feedback_fk']['age_ms']==70
    assert owned[-1]['calls']['speedl']['outcome']=='returned',r
    assert r['speedl_call']['outcome']=='returned' and not r['speedl_remote_result_unknown']
    assert [v[0] for v in c.write_calls]==['speedl','stop_move']
    assert r['complete'],r


@pytest.mark.parametrize('age',[100,101])
async def test_phase_cannot_bypass_feedback_expiry_before_entry(age):
    r,c,_,entered=await phase_run(before_ms=age)
    assert not entered and r['speedl_entries']==r['stop_entries']==0 and not c.write_calls


@pytest.mark.parametrize('change',['input_expiry','revoke','config','renew','generation','tool','feedback','snapshot_stamp'])
async def test_inflight_phase_keeps_input_revocation_binding_and_no_renewal(change,monkeypatch):
    r,c,owned,entered=await phase_run(reply_ms=101 if change=='input_expiry' else 40,
        change=change,monkeypatch=monkeypatch)
    assert entered and not r['complete'] and r['latched_fault']=='stop_unverified'
    assert r['speedl_entries']==r['stop_entries']==1
    assert [v[0] for v in c.write_calls]==['speedl','stop_move']
    assert owned[-1]['calls']['speedl']['outcome']!='returned'
    assert 'speedl' in owned[-1]['unknown_methods']
    assert owned[-1]['calls']['speedl']['abandon_reason']=='authority_revoked'
    expected={
        'input_expiry':'native_jog_inflight_input_expired',
        'revoke':'native_jog_explicit_revocation',
        'config':'native_jog_region_binding_changed',
        'renew':'native_jog_packet_binding_changed',
        # The owner rejects its invalid generation before invoking authority.
        'generation':None,
        'tool':'native_jog_snapshot_binding_changed',
        'feedback':'native_jog_snapshot_binding_changed',
        'snapshot_stamp':'native_jog_snapshot_binding_changed',
    }
    assert r.get('authority_rejection')==expected[change],r


async def test_inflight_checks_preserve_bound_values_without_recomputing_admitted_math(monkeypatch):
    r,_,owned,_=await phase_run(change='no_redundant_math',monkeypatch=monkeypatch)
    assert owned[-1]['calls']['speedl']['outcome']=='returned',r
    assert r['complete'],r
