"""Offline injected candidate races; no network access."""
import asyncio
from dataclasses import replace

import pytest

from app.commissioning.native_jog_candidate import NativeJogCandidateAdapter
from app.commissioning.native_jog_packet import NativeJogLimits, plan_jog
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock

SHA = 'a'*64
LIMITS = NativeJogLimits(.005,.02,(.1,-.3,.1),(.5,.3,.6),(.001,)*3)


async def connected():
    clock = FakeClock()
    client = FakeLebaiClient.idle()
    async def speedl(a,v,t,frame):
        client.write_calls.append(('speedl',a,v,t,frame))
        return 7
    client.speedl = speedl
    async def factory(host):
        assert host == '127.0.0.1'
        return client
    adapter = NativeJogCandidateAdapter(control_settings(ip='127.0.0.1'), limits=LIMITS,
        config_sha256=SHA, client_factory=factory, clock=clock.now_ns,
        sleep=clock.sleep, event_callback=None)
    await adapter.connect()
    assert (await adapter.preflight()).ready
    p = plan_jog((0,0,1),1,limits=LIMITS,actual_p=(.3,0,.4), snapshot_ns=clock.now_ns(),
        now_ns=clock.now_ns(), generation=adapter._pump.generation, sequence=1,
        input_ns=clock.now_ns(), input_expires_ns=clock.now_ns()+100_000_000,
        duration_s=.08, config_sha256=SHA)
    return adapter,client,clock,p


async def cleanup(a):
    try:
        await a.disconnect()
    except BackendCommandError:
        pass  # Original stop failure is asserted by the calling test.


async def test_injected_once_speedl_uses_original_stop():
    a,c,_,p = await connected()
    try:
        assert await a.submit_jog(p,authorize=lambda:None) == 7
        await a.stop(StopReason.GRIP_RELEASED)
        assert a._stop_transaction.confirmed
        with pytest.raises(BackendCommandError,match='consumed'):
            await a.submit_jog(p,authorize=lambda:None)
        assert [r[0] for r in c.write_calls] == ['speedl','stop_move']
        assert c.network_calls == c.discovery_calls == 0
        assert a.native_jog_entries == 1
    finally:
        await cleanup(a)


@pytest.mark.parametrize('host',['172.16.2.162','localhost','::1','127.0.0.2'])
def test_nonloopback_factory_refused_before_connect(host):
    async def forbidden(_):
        pytest.fail('must reject before factory')
    with pytest.raises(ValueError):
        NativeJogCandidateAdapter(control_settings(ip=host), limits=LIMITS,config_sha256=SHA,
            client_factory=forbidden,clock=lambda:0,sleep=asyncio.sleep,event_callback=None)


@pytest.mark.parametrize('change',['authority','expires','generation','snapshot','preflight','client'])
async def test_authority_revoked_while_waiting_lock_has_zero_native_entry(change):
    a,c,clock,p = await connected()
    allowed = True
    def auth():
        if not allowed:
            raise BackendCommandError('revoked')
    await a._sdk_lock.acquire()
    task = asyncio.create_task(a.submit_jog(p,authorize=auth))
    try:
        await asyncio.sleep(0)
        if change == 'authority': allowed = False
        elif change == 'expires': clock.advance_ms(100)
        elif change == 'generation': a._pump.invalidate()
        elif change == 'snapshot': a._snapshot = replace(a._snapshot, read_completed_ns=1)
        elif change == 'preflight': a._preflight_ready = False
        else: a._client = None
        a._sdk_lock.release()
        result = (await asyncio.gather(task,return_exceptions=True))[0]
        assert isinstance(result, (BackendCommandError,ValueError))
        assert a.native_jog_entries == 0
        assert not a._motion_write_stop_required
        assert not a._motion_calls.unknown_methods
        assert not c.write_calls
    finally:
        a._client = c
        if not task.done(): task.cancel()
        await cleanup(a)


@pytest.mark.parametrize('bad',[True,0,-1,1.5,'7',None])
async def test_invalid_or_late_motion_id_cannot_restore_owner(bad):
    a,c,_,p = await connected()
    async def speedl(*args):
        c.write_calls.append(('speedl',))
        return bad
    c.speedl = speedl
    try:
        with pytest.raises(BackendCommandError):
            await a.submit_jog(p,authorize=lambda:None)
        assert 'speedl' in a._motion_calls.unknown_methods
        assert a._latched_fault == 'stop_unverified'
        with pytest.raises(BackendCommandError):
            await a.stop(StopReason.GRIP_RELEASED)
        assert not a._stop_transaction.confirmed
    finally:
        await cleanup(a)


@pytest.mark.parametrize('cancel',[False,True])
async def test_cancel_does_not_wait_for_stubborn_native(cancel):
    a,c,_,p = await connected()
    entered, release = asyncio.Event(), asyncio.Event()
    async def stubborn(*args):
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        return 7
    c.speedl = stubborn
    task = asyncio.create_task(a.submit_jog(p,authorize=lambda:None))
    try:
        await asyncio.wait_for(entered.wait(),1)
        if cancel: task.cancel()
        result = await asyncio.wait_for(asyncio.gather(task,return_exceptions=True),.3)
        assert isinstance(result[0], (BackendCommandError,asyncio.CancelledError))
        assert a._latched_fault == 'stop_unverified'
        assert a._motion_calls.unknown_methods == {'speedl'}
        release.set()
        await asyncio.sleep(.02)
        assert a._motion_calls.unknown_methods == {'speedl'}
        assert not (await a.preflight()).ready
        with pytest.raises(BackendCommandError):
            await a.stop(StopReason.GRIP_RELEASED)
        assert not a._stop_transaction.confirmed
    finally:
        release.set()
        await cleanup(a)


async def test_packet_expires_at_native_boundary(monkeypatch):
    a,c,clock,p = await connected()
    old = a._motion_calls.call
    async def advance(*args,**kwargs):
        clock.advance_ms(100)
        return await old(*args,**kwargs)
    monkeypatch.setattr(a._motion_calls,'call',advance)
    try:
        with pytest.raises((ValueError,BackendCommandError)):
            await a.submit_jog(p,authorize=lambda:None)
        assert a.native_jog_entries == 0
        assert not a._motion_calls.unknown_methods
        assert not c.write_calls
    finally:
        await cleanup(a)


async def test_other_write_routes_are_disabled():
    a,c,_,_ = await connected()
    try:
        for name in ('submit_tcp','submit_catch_up','home','prepare','set_gripper'):
            with pytest.raises(BackendCommandError):
                await getattr(a,name)(None)
        assert not c.write_calls
    finally:
        await cleanup(a)


def test_old_pvat_and_gripper_unknown_policy_unchanged():
    assert RealLebaiAdapter._stop_blocking_motion_methods == {'move_pvat','set_claw'}
    assert RealLebaiAdapter._immediate_stop_latch_methods == {'move_pvat'}


@pytest.mark.parametrize('race',['stubborn','before_return','after_operation'])
async def test_submission_authority_revocation_survives_native_completion(race):
    a,c,_,p = await connected()
    entered,release = asyncio.Event(),asyncio.Event()
    allowed=True
    def revoke():
        nonlocal allowed
        allowed=False
    def authorize():
        if not allowed:raise BackendCommandError('input_revoked')
    async def speedl(*args):
        entered.set()
        if race=='after_operation':
            asyncio.get_running_loop().call_soon(revoke)
        else:
            try:await release.wait()
            except asyncio.CancelledError:await release.wait()
        return 7
    c.speedl=speedl
    task=asyncio.create_task(a.submit_jog(p,authorize=authorize))
    try:
        await asyncio.wait_for(entered.wait(),1)
        if race!='after_operation':revoke()
        if race=='before_return':release.set()
        if race=='stubborn':
            # Prompt local abandonment, rather than waiting for the 60ms
            # deadline or a native cancellation acknowledgement.
            done,_=await asyncio.wait({task},timeout=.04)
            assert task in done
        result=(await asyncio.gather(task,return_exceptions=True))[0]
        assert isinstance(result,BackendCommandError)
        assert a._motion_calls.unknown_methods=={'speedl'}
        assert a._latched_fault=='stop_unverified'
        release.set()
        await asyncio.sleep(.01)
        assert a._motion_calls.unknown_methods=={'speedl'}
        with pytest.raises(BackendCommandError):await a.stop(StopReason.GRIP_RELEASED)
        assert not a._stop_transaction.confirmed
    finally:
        release.set()
        await asyncio.gather(task,return_exceptions=True)
        await cleanup(a)
