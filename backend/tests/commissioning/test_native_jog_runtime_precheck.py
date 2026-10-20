"""Relocated metadata precheck is single-use; actual write check stays live."""
import asyncio
import copy
import pytest
from app.commissioning.native_jog_site_scope import SiteJogDiagnosticAdapter
from app.commissioning.native_jog_site_client import SiteJogClient
from app.commissioning.native_jog_packet import plan_jog
from app.robots.base import StopReason
from tests.commissioning.test_native_jog_site_run import setup,run
from tests.robots.test_lebai_adapter_control import FakeClock


async def connected(**changes):
    scope,native=setup(**changes);clock=FakeClock();proxy=None
    owner=asyncio.current_task()
    async def factory(ip):
        nonlocal proxy
        proxy=SiteJogClient(native,scope,owner=owner,clock=clock.now_ns,
            motion_permission=lambda:None,stop_permission=lambda:None)
        return proxy
    adapter=SiteJogDiagnosticAdapter(scope,client_factory=factory,clock=clock.now_ns,
        sleep=clock.sleep,event_callback=None)
    await adapter.connect();assert (await adapter.preflight()).ready
    return adapter,proxy,native,clock


def precheck(adapter):
    method=getattr(adapter,'precheck_jog_runtime',None)
    assert callable(method), 'missing single-use runtime precheck'
    method()


async def paired(adapter,proxy,clock):
    old=adapter._snapshot
    await proxy.feedback_pair(old.actual_q,lambda:adapter._read_snapshot(read_purpose='motion'))
    return plan_jog((0,0,1),1,limits=adapter._site_scope.limits,
        actual_p=adapter._snapshot.actual_tcp.p,snapshot_ns=adapter._snapshot.captured_ns,
        now_ns=clock.now_ns(),generation=adapter._pump.generation,sequence=1,
        input_ns=clock.now_ns(),input_expires_ns=clock.now_ns()+100_000_000,
        duration_s=.08,config_sha256=adapter._site_scope.config_sha256)


async def test_precheck_scan_moves_before_pair_without_skipping_native_write_check(monkeypatch):
    adapter,proxy,native,clock=await connected();events=[]
    scope_type=type(adapter._site_scope);hot=scope_type.hot
    def counted(self):events.append('metadata');return hot(self)
    monkeypatch.setattr(scope_type,'hot',counted)
    try:
        precheck(adapter)
        assert events==['metadata']
        p=await paired(adapter,proxy,clock);events.clear()
        assert await adapter.submit_jog(p,authorize=lambda:None)==7
        # The proxy's write_check is a separate callback: real Inputs.write
        # performs its full metadata scan here. Adapter retains return scan.
        assert events==['metadata']  # Publication only, no submit scan.
        assert proxy.speedl_entries==1
        await adapter.stop(StopReason.SHUTDOWN)
    finally:await adapter.disconnect()


@pytest.mark.parametrize('damage',['extra_snapshot','extra_native_read','generation','client',
    'snapshot_owner','foreign_task','no_pair'])
async def test_precheck_binding_rejects_once_without_native_motion(damage):
    adapter,proxy,native,clock=await connected()
    original_client=adapter._client;original_owner=adapter._snapshot_calls
    try:
        precheck(adapter)
        p=await paired(adapter,proxy,clock)
        if damage=='extra_snapshot':await adapter._read_snapshot(read_purpose='state')
        elif damage=='extra_native_read':await proxy.client.get_robot_state()
        elif damage=='generation':adapter._pump._generation+=1
        elif damage=='client':adapter._client=SiteJogClient(native,adapter._site_scope,
            owner=asyncio.current_task(),clock=clock.now_ns,motion_permission=lambda:None,stop_permission=lambda:None)
        elif damage=='snapshot_owner':adapter._snapshot_calls=type(original_owner)()
        elif damage=='no_pair':proxy._feedback_pair=None
        operation=adapter.submit_jog(p,authorize=lambda:None)
        with pytest.raises(ValueError,match='native_jog_precheck'):
            if damage=='foreign_task':await asyncio.create_task(operation)
            else:await operation
        adapter._client=original_client;adapter._snapshot_calls=original_owner
        with pytest.raises(Exception,match='consumed'):await adapter.submit_jog(p,authorize=lambda:None)
        assert proxy.speedl_entries==proxy.sdk_stop_entries==0 and not native.write_calls
    finally:
        adapter._client=original_client;adapter._snapshot_calls=original_owner
        await adapter.disconnect()


async def test_failed_or_repeated_precheck_cannot_fall_back_to_old_submission():
    adapter,proxy,native,clock=await connected()
    try:
        precheck(adapter)
        with pytest.raises(ValueError,match='precheck'):precheck(adapter)
        p=await paired(adapter,proxy,clock)
        with pytest.raises(ValueError,match='precheck'):await adapter.submit_jog(p,authorize=lambda:None)
        assert not native.write_calls
    finally:await adapter.disconnect()


async def test_file_change_after_precheck_is_rejected_by_original_write_callback():
    changed=False;writes=[]
    def write():
        writes.append(True)
        if changed:raise ValueError('native_jog_runtime_metadata_changed')
    adapter,proxy,native,clock=await connected(write_check=write)
    try:
        precheck(adapter);p=await paired(adapter,proxy,clock);changed=True
        with pytest.raises(ValueError,match='metadata_changed'):
            await adapter.submit_jog(p,authorize=lambda:None)
        assert writes and proxy.speedl_entries==proxy.sdk_stop_entries==0 and not native.write_calls
    finally:await adapter.disconnect()


async def test_revocation_after_precheck_still_rejects():
    adapter,proxy,native,clock=await connected()
    try:
        precheck(adapter);p=await paired(adapter,proxy,clock)
        def deny():raise ValueError('native_jog_test_revoked')
        with pytest.raises(Exception,match='revoked'):
            await adapter.submit_jog(p,authorize=deny)
        assert proxy.speedl_entries==proxy.sdk_stop_entries==0 and not native.write_calls
    finally:await adapter.disconnect()


async def test_stale_packet_is_not_renewed_by_precheck():
    adapter,proxy,native,clock=await connected()
    try:
        precheck(adapter);p=await paired(adapter,proxy,clock);clock.advance_ms(100)
        with pytest.raises(Exception):await adapter.submit_jog(p,authorize=lambda:None)
        assert proxy.speedl_entries==proxy.sdk_stop_entries==0 and not native.write_calls
    finally:await adapter.disconnect()


async def test_runner_prechecks_before_unique_pair_and_reports_policy(monkeypatch):
    events=[];method=getattr(SiteJogDiagnosticAdapter,'precheck_jog_runtime',None)
    assert callable(method), 'runner has no precheck method to bind'
    original_pair=SiteJogClient.feedback_pair
    def checked(self):events.append('precheck');return method(self)
    async def pair(self,*args,**kwargs):events.append('pair');return await original_pair(self,*args,**kwargs)
    monkeypatch.setattr(SiteJogDiagnosticAdapter,'precheck_jog_runtime',checked)
    monkeypatch.setattr(SiteJogClient,'feedback_pair',pair)
    result,_=await run()
    assert result['complete'],result
    assert events==['precheck','pair']
    assert result['runtime_precheck']['consumed']
    assert result['runtime_precheck']['policy']=='single_pre_pair_scan_original_native_write_scan'


async def test_proof_cannot_be_transferred_even_to_shallow_adapter_copy():
    adapter,proxy,native,clock=await connected()
    try:
        precheck(adapter);p=await paired(adapter,proxy,clock)
        other=copy.copy(adapter)
        with pytest.raises(ValueError,match='precheck'):
            await other.submit_jog(p,authorize=lambda:None)
        assert proxy.speedl_entries==0 and not native.write_calls
    finally:await adapter.disconnect()


async def test_failed_precheck_has_no_submission_fallback(monkeypatch):
    adapter,proxy,native,clock=await connected();cls=type(adapter._site_scope)
    original=cls.hot
    def fail(self):raise ValueError('native_jog_test_metadata_changed')
    try:
        monkeypatch.setattr(cls,'hot',fail)
        with pytest.raises(ValueError,match='metadata_changed'):precheck(adapter)
        monkeypatch.setattr(cls,'hot',original)
        p=await paired(adapter,proxy,clock)
        with pytest.raises(ValueError,match='precheck'):await adapter.submit_jog(p,authorize=lambda:None)
        assert proxy.speedl_entries==0 and not native.write_calls
    finally:await adapter.disconnect()


@pytest.mark.parametrize('method',['get_robot_state','get_motion_state'])
async def test_raw_read_before_pair_invalidates_proof(method):
    adapter,proxy,native,clock=await connected()
    try:
        precheck(adapter)
        await getattr(proxy.client,method)(*([123] if method=='get_motion_state' else []))
        p=await paired(adapter,proxy,clock)
        with pytest.raises(ValueError,match='precheck'):await adapter.submit_jog(p,authorize=lambda:None)
        assert proxy.speedl_entries==0 and not native.write_calls
    finally:await adapter.disconnect()


@pytest.mark.parametrize('changed_owner',['proxy','pair'])
async def test_proxy_owner_change_cannot_reuse_original_task_proof(changed_owner):
    adapter,proxy,native,clock=await connected()
    owner=proxy.owner;foreign=asyncio.create_task(asyncio.Event().wait())
    try:
        precheck(adapter);p=await paired(adapter,proxy,clock)
        if changed_owner=='proxy':proxy.owner=foreign
        else:proxy._feedback_pair['owner']=foreign
        with pytest.raises(ValueError,match='precheck'):
            await adapter.submit_jog(p,authorize=lambda:None)
        assert proxy.speedl_entries==0 and not native.write_calls
    finally:
        proxy.owner=owner
        if proxy._feedback_pair is not None:proxy._feedback_pair['owner']=owner
        foreign.cancel()
        await asyncio.gather(foreign,return_exceptions=True)
        if proxy.speedl_entries:await adapter.stop(StopReason.SHUTDOWN)
        await adapter.disconnect()


async def test_failed_admission_does_not_report_an_old_consumed_proof():
    def deny():raise ValueError('native_jog_test_revoked')
    scope,native=setup(authorize=deny)
    result,_=await run(scope,native,report={'runtime_precheck':{'consumed':True}})
    assert result['primary_rejection']=='native_jog_test_revoked'
    assert 'runtime_precheck' not in result
