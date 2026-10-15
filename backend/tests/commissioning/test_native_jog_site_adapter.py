import asyncio
from dataclasses import replace
import pytest
from app.commissioning.native_jog_site_scope import SiteJogScope,SiteJogDiagnosticAdapter
from app.commissioning.native_jog_packet import plan_jog
from app.robots.lebai_codec import pose_to_lebai
from app.robots.base import BackendCommandError,StopReason
from tests.commissioning.test_native_jog_site_scope import arguments,Q,SHA
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.test_lebai_adapter_control import FakeClock

async def connected():
    args,pose=arguments();scope=SiteJogScope(**args);clock=FakeClock()
    c=FakeLebaiClient.idle(q=list(Q));c._jog_scope=scope;c.entries=0
    c.kin_data['actual_tcp_pose']=pose_to_lebai(pose);c.kin_data['target_tcp_pose']=pose_to_lebai(pose)
    c._native_motion_entry_count=lambda method:c.entries if method=='speedl' else 0
    async def speedl(*values):c.entries+=1;c.write_calls.append(('speedl',*values));return 7
    c.speedl=speedl
    async def factory(ip):assert ip==scope.settings.ip;return c
    a=SiteJogDiagnosticAdapter(scope,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep,event_callback=None)
    await a.connect();assert (await a.preflight()).ready
    p=plan_jog((0,0,1),1,limits=scope.limits,actual_p=pose.p,snapshot_ns=clock.now_ns(),now_ns=clock.now_ns(),generation=a._pump.generation,sequence=1,input_ns=clock.now_ns(),input_expires_ns=clock.now_ns()+100_000_000,duration_s=.08,config_sha256=SHA)
    return a,c,p

async def test_single_site_adapter_original_stop():
    a,c,p=await connected()
    try:
        assert await a.submit_jog(p,authorize=lambda:None)==7
        await a.stop(StopReason.GRIP_RELEASED)
        assert a._stop_transaction.confirmed and [v[0] for v in c.write_calls]==['speedl','stop_move']
        with pytest.raises(BackendCommandError):await a.submit_jog(p,authorize=lambda:None)
    finally:await a.disconnect()

async def test_non_fixed_direction_consumes_without_write():
    a,c,p=await connected()
    try:
        with pytest.raises((ValueError,BackendCommandError)):await a.submit_jog(replace(p,velocity_mps=(.005,0.,0.)),authorize=lambda:None)
        assert not c.write_calls
    finally:await a.disconnect()

def test_site_adapter_requires_explicit_scope_and_factory():
    with pytest.raises(ValueError):SiteJogDiagnosticAdapter(None,client_factory=None,clock=lambda:0,sleep=asyncio.sleep,event_callback=None)
