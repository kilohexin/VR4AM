"""Two owned pre-send reads, never live robot access."""
import asyncio
import time
from dataclasses import replace

import pytest

from app.commissioning.native_jog_site_client import SiteJogClient
from tests.commissioning.test_native_jog_site_run import setup, run
from tests.commissioning.test_native_jog_site_scope import Q


def overlap_fixture(client, *, damage=None):
    fk_started, snapshot_started, release = asyncio.Event(), asyncio.Event(), asyncio.Event()
    cancelled = asyncio.Event()
    fk_original, kin_original = client.kinematics_forward, client.get_kin_data
    async def fk(q):
        fk_started.set()
        if damage in ('fk_error','late_snapshot'):
            await snapshot_started.wait()
            raise ValueError('test_math_failed')
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                cancelled.set()
                if damage != 'late_fk': raise
        return await fk_original(q)
    async def kin():
        if fk_started.is_set():
            snapshot_started.set()
            if damage == 'late_snapshot':
                while not release.is_set():
                    try:await release.wait()
                    except asyncio.CancelledError:cancelled.set()
            if damage == 'q': client.kin_data['actual_joint_pose'][0] += .00001
            if damage == 'tool': client.tcp['z'] += .00001
            if damage == 'snapshot_error': raise ValueError('test_read_failed')
            if damage not in ('late_fk','cancel'): release.set()
        return await kin_original()
    client.kinematics_forward, client.get_kin_data = fk,kin
    return fk_started,snapshot_started,release,cancelled


async def test_unique_fk_overlaps_fresh_snapshot_and_original_stop_is_preserved():
    scope,client = setup()
    fk,snapshot,release,_ = overlap_fixture(client)
    try:
        result,_ = await run(scope,client)
        assert fk.is_set() and snapshot.is_set()
        assert result['complete'],result
        assert result['fk_entries'] == result['speedl_entries'] == result['stop_entries'] == 1
        assert result['feedback_read_pair']['outcome'] == 'returned'
        assert [x[0] for x in client.write_calls] == ['speedl','stop_move']
    finally: release.set()


@pytest.mark.parametrize('damage',['q','tool','fk_error','snapshot_error','late_fk','late_snapshot'])
async def test_pair_failure_or_unknown_results_cannot_send_motion(damage):
    scope,client = setup()
    _,snapshot,release,cancelled = overlap_fixture(client,damage=damage)
    try:
        result,_ = await run(scope,client)
        assert snapshot.is_set(),result
        assert not result['complete'] and result['speedl_entries'] == result['stop_entries'] == 0
        assert not client.write_calls and result['fk_entries'] <= 1
        if damage in ('late_fk','late_snapshot'):
            await asyncio.wait_for(cancelled.wait(),.5)
            assert result['read_remote_result_unknown']
            release.set(); await asyncio.sleep(.01)
            assert not client.write_calls  # Late math is never installed or retried.
    finally: release.set()


async def test_cancelling_pair_with_native_fk_pending_consumes_it_without_motion():
    scope,client = setup()
    _,snapshot,release,cancelled = overlap_fixture(client,damage='cancel')
    report = {}
    try:
        task = asyncio.create_task(run(scope,client,report))
        await asyncio.wait_for(snapshot.wait(),.5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError): await task
        assert not client.write_calls
        assert report['speedl_entries'] == report['stop_entries'] == 0
    finally: release.set()


async def test_previewed_pair_cannot_be_reused_for_a_second_math_read():
    scope,client = setup()
    proxy = SiteJogClient(client,scope,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    await proxy.get_estop_reason()
    with pytest.raises(Exception):
        # The pair is not an unrestricted gather callback: snapshot group
        # completion must be owned by the motion-snapshot scope.
        await proxy.feedback_pair(Q,lambda:proxy.kinematics_forward(Q))
    assert proxy.special_entries['kinematics_forward'] <= 1
    assert proxy.read_gate_closed and not client.write_calls


async def test_foreign_read_during_pair_closes_the_gate():
    scope,client = setup()
    proxy = SiteJogClient(client,scope,owner=asyncio.current_task(),clock=time.perf_counter_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    assert callable(getattr(proxy,'feedback_pair',None)), 'owned feedback pair missing'
    await proxy.get_estop_reason()
    ready,release = asyncio.Event(),asyncio.Event()
    old = client.kinematics_forward
    async def fk(q): ready.set(); await release.wait(); return await old(q)
    client.kinematics_forward = fk
    async def snapshot():
        with proxy._parallel_snapshot_scope(lambda:None,purpose='motion'):
            return await proxy.get_kin_data()
    async def foreign():
        await ready.wait()
        with pytest.raises(Exception): await proxy.get_tcp()
        release.set()
    intruder = asyncio.create_task(foreign())
    try:
        with pytest.raises(Exception): await proxy.feedback_pair(Q,snapshot)
        assert proxy.read_gate_closed and not client.write_calls
    finally:
        release.set(); await intruder


async def test_user_revocation_while_both_reads_are_pending_has_zero_motion():
    scope,client = setup()
    allowed=True
    def read_check():
        if not allowed:raise ValueError('native_jog_test_revoked')
    scope=replace(scope,read_check=read_check)
    _,snapshot,release,_ = overlap_fixture(client,damage='cancel')
    try:
        task=asyncio.create_task(run(scope,client))
        await asyncio.wait_for(snapshot.wait(),.5)
        allowed=False;release.set()
        result,_=await task
        assert not result['complete'] and result['speedl_entries']==result['stop_entries']==0
        assert not client.write_calls and result['fk_entries']==1
    finally:release.set()


async def test_fk_from_previous_generation_cannot_join_a_new_generation_snapshot(monkeypatch):
    from app.commissioning.native_jog_site_scope import SiteJogDiagnosticAdapter
    original=SiteJogDiagnosticAdapter._read_snapshot
    async def changed(self,**kwargs):
        if kwargs.get('read_purpose')=='motion':self._pump.invalidate()
        return await original(self,**kwargs)
    monkeypatch.setattr(SiteJogDiagnosticAdapter,'_read_snapshot',changed)
    scope,client=setup()
    result,_=await run(scope,client)
    assert result['speedl_entries']==result['stop_entries']==0,result
    assert result['primary_rejection']=='native_jog_feedback_pair_owner_changed',result
    assert not client.write_calls


async def test_pair_keeps_native_fk_timestamp_before_local_permission_work():
    from tests.robots.test_lebai_adapter_control import FakeClock
    clock=FakeClock();received=advanced=False
    def read_check():
        nonlocal advanced
        if received and not advanced:clock.advance_ms(50);advanced=True
    scope,client=setup(read_check=read_check)
    proxy=SiteJogClient(client,scope,owner=asyncio.current_task(),clock=clock.now_ns,
        motion_permission=lambda:None,stop_permission=lambda:None)
    await proxy.get_estop_reason()
    old=client.kinematics_forward
    async def fk(q):
        nonlocal received
        value=await old(q);received=True;return value
    client.kinematics_forward=fk
    async def snapshot():
        with proxy._parallel_snapshot_scope(lambda:None,purpose='motion'):
            return await proxy.get_kin_data()
    _,stamp,_=await proxy.feedback_pair(Q,snapshot)
    native=next(r for r in proxy.native_reads if r['method']=='kinematics_forward')
    assert advanced and clock.now_ns()-native['completed_ns']==50_000_000
    assert stamp==native['completed_ns']
