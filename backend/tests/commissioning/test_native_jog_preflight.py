"""Synthetic preflight regression. Never connects to a controller."""
import asyncio
import time
from dataclasses import replace
import pytest

from app.commissioning.native_jog_site_scope import SiteJogDiagnosticAdapter
from app.commissioning.native_jog_site_client import SiteJogClient
from app.commissioning.idle_stop_baseline import AdmissionRejected, validate_idle_sample
from app.commissioning.paused_stop_cleanup import _cached_sample
from tests.commissioning.test_native_jog_site_run import setup, run


def proxy(scope, native):
    return SiteJogClient(native,scope,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)


async def test_preflight_reads_six_fields_concurrently_and_retains_original_freshness():
    s,c=setup();active=peak=0
    names=('get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion')
    for name in names:
        original=getattr(c,name)
        async def delayed(original=original):
            nonlocal active,peak
            active+=1;peak=max(peak,active)
            try:await asyncio.sleep(.02);return await original()
            finally:active-=1
        setattr(c,name,delayed)
    p=proxy(s,c)
    async def factory(ip):return p
    a=SiteJogDiagnosticAdapter(s,client_factory=factory,clock=time.perf_counter_ns,
        sleep=asyncio.sleep,event_callback=None)
    try:
        await a.connect();peak=0
        assert (await a.preflight()).ready
        assert peak==6
        validate_idle_sample(_cached_sample(a),time.perf_counter_ns())
    finally:await a.disconnect()
    assert not c.write_calls


async def test_two_distinct_groups_archive_owners_without_resetting_global_counts():
    r,_=await run()
    assert r['complete'],r
    assert [row['purpose'] for row in r['snapshot_groups']]==['preflight','motion']
    assert all(row['outcome']=='returned' for row in r['snapshot_groups'])
    assert r['archived_snapshot_owners']==6
    assert r['read_entries']==len(r['native_reads'])<=256


async def test_completed_preflight_group_cannot_be_reused_as_a_retry():
    s,c=setup();p=proxy(s,c);await p.get_estop_reason()
    with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):await p.get_robot_state()
    first=p.snapshot_read_owners['get_robot_state']
    with p._parallel_snapshot_scope(lambda:None,purpose='motion'):await p.get_robot_state()
    assert first in p._snapshot_owner_archive
    with pytest.raises(AdmissionRejected,match='group_not_admitted'):
        with p._parallel_snapshot_scope(lambda:None,purpose='motion'):pass


async def test_failed_first_group_permanently_blocks_next_group():
    s,c=setup();p=proxy(s,c);await p.get_estop_reason()
    with pytest.raises(ValueError):
        with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):
            await p.get_robot_state();raise ValueError('local injected failure')
    assert p.read_gate_closed
    with pytest.raises(AdmissionRejected):
        with p._parallel_snapshot_scope(lambda:None,purpose='motion'):pass


async def test_archived_unknown_owner_cannot_be_hidden_by_a_new_group():
    s,c=setup();p=proxy(s,c);await p.get_estop_reason()
    with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):await p.get_robot_state()
    old=p.snapshot_read_owners['get_robot_state']
    with p._parallel_snapshot_scope(lambda:None,purpose='motion'):await p.get_robot_state()
    old.read_remote_result_unknown=True
    with pytest.raises(AdmissionRejected,match='archive_not_clear'):p.require_clear_read_gate()


@pytest.mark.parametrize('outcome',[True,'yes'])
async def test_light_read_permission_must_return_none(outcome):
    s,c=setup();s=replace(s,read_check=lambda:outcome);p=proxy(s,c)
    with pytest.raises(ValueError,match='authorization_result'):await p.client.get_robot_state()
    assert p.read_entries==0


async def test_light_reads_do_not_call_whole_runtime_check():
    calls=[];s,c=setup();s=replace(s,hot_check=lambda:calls.append('full'),read_check=lambda:calls.append('read'))
    p=proxy(s,c);await p.get_robot_state()
    assert calls==['read'] and p.read_entries==1
    s.hot();assert calls[-1]=='full'


async def test_frozen_stale_preflight_rejection_is_reported_explicitly(monkeypatch):
    from app.commissioning import native_jog_site_run as workflow
    def reject(*args):raise AdmissionRejected('idle_baseline_sample_not_current')
    monkeypatch.setattr(workflow,'validate_idle_sample',reject)
    r,c=await run()
    assert r['primary_rejection']=='idle_baseline_sample_not_current'
    assert r['rejection_phase']=='preflight'
    assert not c.write_calls and r['speedl_entries']==r['stop_entries']==0


async def test_vendor_error_text_is_not_exposed(monkeypatch):
    from app.commissioning import native_jog_site_run as workflow
    def reject(*args):raise RuntimeError('vendor secret path password')
    monkeypatch.setattr(workflow,'validate_idle_sample',reject)
    r,_=await run()
    assert r['primary_rejection']=='native_jog_diagnostic_failed'
    assert 'vendor secret' not in str(r)


async def test_cancel_ignoring_first_group_read_blocks_every_new_group():
    s,c=setup();entered=asyncio.Event();release=asyncio.Event();old=c.get_kin_data
    async def stubborn():
        entered.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return await old()
    c.get_kin_data=stubborn;p=proxy(s,c);await p.get_estop_reason();task=None
    try:
        with pytest.raises(ValueError):
            with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):
                task=asyncio.create_task(p.get_kin_data());await asyncio.wait_for(entered.wait(),1)
                raise ValueError('abort group')
        task.cancel();await asyncio.gather(task,return_exceptions=True)
        assert p.read_remote_result_unknown
        with pytest.raises(AdmissionRejected):
            with p._parallel_snapshot_scope(lambda:None,purpose='motion'):pass
        assert len(p.snapshot_groups)==1 and p._snapshot_read_pending()
    finally:
        release.set()
        for _ in range(10):await asyncio.sleep(0)


async def test_foreign_read_cannot_join_an_existing_group():
    s,c=setup();p=proxy(s,c);await p.get_estop_reason();go=asyncio.Event()
    async def foreign():await go.wait();return await p.get_robot_state()
    task=asyncio.create_task(foreign()) # Created outside the snapshot context.
    with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):
        go.set()
        with pytest.raises(AdmissionRejected,match='foreign_read'):await task
    assert p.read_gate_closed


async def test_early_timer_wake_does_not_start_a_read_with_a_truncated_tail_budget(monkeypatch):
    from app.commissioning import native_jog_site_run as workflow
    from tests.robots.test_lebai_adapter_control import FakeClock
    from tests.commissioning.test_idle_stop_baseline import Recorder
    clock=FakeClock();budgets=[];tail_reads=0;injected=False
    class CompletedObservation:
        blocks_reads=False
        def __init__(self,*args,**kwargs):pass
        def start(self):pass
        def revoke(self):pass
        def snapshot(self):return dict(outcome='returned',remote_result_unknown=False)
    monkeypatch.setattr(workflow,'MotionObservation',CompletedObservation)
    original=workflow._bounded
    async def bounded(awaitable,seconds):
        nonlocal tail_reads
        if seconds<=.2:
            tail_reads+=1;budgets.append(seconds)
        return await original(awaitable,seconds)
    async def early_sleep(seconds):
        nonlocal injected
        await clock.sleep(seconds)
        if tail_reads==10 and not injected:
            clock.advance_ms(-1);injected=True
    monkeypatch.setattr(workflow,'_bounded',bounded)
    s,c=setup()
    async def factory(ip):return c
    await workflow.run_site_window(s,Recorder(),factory,clock=clock.now_ns,sleep=early_sleep)
    assert injected and budgets and min(budgets)>=.199


async def test_local_authority_rejection_is_preserved_when_native_was_never_entered(monkeypatch):
    from app.commissioning import native_jog_site_run as workflow
    denied=False
    def hot():
        if denied:raise ValueError('native_jog_test_scope_expired')
    s,c=setup(hot_check=hot);original=workflow.plan_jog
    def plan(*args,**kwargs):
        nonlocal denied
        value=original(*args,**kwargs);denied=True;return value
    monkeypatch.setattr(workflow,'plan_jog',plan)
    r,_=await run(s,c)
    assert r['speedl_entries']==r['stop_entries']==0
    assert r['primary_rejection']=='native_jog_test_scope_expired'
    assert r['authority_rejection']=='native_jog_test_scope_expired'


async def test_direct_read_surface_cannot_duplicate_a_group_native_entry():
    s,c=setup();p=proxy(s,c);await p.get_estop_reason()
    with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):
        await p.client.get_robot_state()
        with pytest.raises(AdmissionRejected,match='native_duplicate'):await p.client.get_robot_state()
    assert c.read_calls.count('get_robot_state')==1 and p.read_gate_closed


async def test_direct_read_surface_cannot_bypass_foreign_group_gate():
    s,c=setup();p=proxy(s,c);await p.get_estop_reason();go=asyncio.Event()
    async def foreign():await go.wait();return await p.client.get_robot_state()
    task=asyncio.create_task(foreign())
    with p._parallel_snapshot_scope(lambda:None,purpose='preflight'):
        go.set()
        with pytest.raises(AdmissionRejected,match='native_foreign'):await task
    assert 'get_robot_state' not in c.read_calls and p.read_gate_closed
