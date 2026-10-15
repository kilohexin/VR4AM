import asyncio
from dataclasses import replace
import importlib

import pytest

from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.fake_lebai import FakeLebaiClient, ACTUAL_TCP
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


def module():
    try:
        return importlib.import_module('app.commissioning.finite_window_run')
    except ModuleNotFoundError:
        pytest.fail('scoped finite-window workflow not implemented')


class ZClient(FakeLebaiClient):
    def __init__(self):
        super().__init__()
        self.fk_calls = []

    async def kinematics_inverse(self, pose, joints):
        self.ik_calls.append((pose.copy(), list(joints)))
        return [pose['z'] - .4, *joints[1:]]

    async def kinematics_forward(self, joints):
        self.fk_calls.append(list(joints))
        return {**ACTUAL_TCP, 'z': .4 + joints[0]}

    async def move_pvat(self, p, v, a, t):
        await super().move_pvat(p, v, a, t)
        self.kin_data['actual_joint_pose'] = list(p)
        self.kin_data['actual_joint_speed'] = list(v)
        self.kin_data['actual_joint_acc'] = list(a)
        self.kin_data['actual_tcp_pose']['z'] = .4 + p[0]
        return 1


async def run(client=None, *, recorder=None, authorize=lambda: None, hot=lambda: None, clock=None):
    client, clock = client or ZClient(), clock or FakeClock()
    settings = control_settings()
    settings = replace(settings, control=replace(settings.control, max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2))
    async def factory(_ip):
        return client
    report = await module().run_window(settings, recorder or Recorder(), factory, authorize, hot,
                                       clock=clock.now_ns, sleep=clock.sleep)
    return report, client


async def test_one_scoped_packet_has_zero_terminal_and_one_stop():
    result, client = await run()
    assert result['complete'] and result['stop_confirmed'] and result['displacement_consistent']
    assert result['pvat_entries'] == result['stop_entries'] == 1
    assert len(client.ik_calls) == 1 and len(client.fk_calls) == 4
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
    assert client.write_calls[0][2] == client.write_calls[0][3] == [0.]*6
    assert not result['hardware_verified'] and not result['motion_authorized']


async def test_small_configured_window_requires_the_full_endpoint():
    settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.2,max_tcp_step_m=.00005))
    client,clock=ZClient(),FakeClock()
    settings=replace(settings,teleop_ready_q=tuple(client.kin_data['actual_joint_pose']))
    async def factory(_ip):return client
    result=await module().run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep)
    assert result['complete'] and result['full_endpoint_required'] and result['full_endpoint_feasible']
    assert client.ik_calls[0][0]['z']==pytest.approx(.40005)
    assert client.kin_data['actual_tcp_pose']['z']==pytest.approx(.40005)
    assert result['pvat_entries']==result['stop_entries']==1


async def test_small_window_rejects_a_partial_endpoint_before_native_entry():
    settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.001,max_tcp_step_m=.00005))
    client,clock=ZClient(),FakeClock()
    settings=replace(settings,teleop_ready_q=tuple(client.kin_data['actual_joint_pose']))
    async def factory(_ip):return client
    result=await module().run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep)
    assert not result['complete'] and result['full_endpoint_required']
    assert not result['full_endpoint_feasible']
    assert result['pvat_entries']==0
    assert client.write_calls==[('stop_move',)]


async def test_small_window_rejects_a_changed_ready_pose_before_admission():
    settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.2,max_tcp_step_m=.00005))
    client,clock=ZClient(),FakeClock()
    async def factory(_ip):return client
    result=await module().run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep)
    assert not result['complete'] and not result['scope_admitted']
    assert result['pvat_entries']==0 and client.write_calls==[]


@pytest.mark.parametrize('tolerance,admitted',[(.0001,False),(.001,True)])
async def test_two_feedback_ticks_need_explicit_ready_tolerance(tolerance,admitted):
    settings=control_settings();client=ZClient();clock=FakeClock()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.2,max_tcp_step_m=.00005),
        teleop_ready_q=tuple(client.kin_data['actual_joint_pose']))
    client.kin_data['actual_joint_pose'][4]-=.0001917475984856143
    async def factory(_ip):return client
    report=await module().run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,ready_joint_tolerance_rad=tolerance)
    assert report['scope_admitted'] is admitted
    assert report['ready_max_joint_delta_rad']==pytest.approx(.0001917475984856143)
    if admitted:assert report['complete']
    else:
        assert report['primary_rejection']=='finite_scope_ready_pose_changed'
        assert client.write_calls==[] and report['pvat_entries']==0


@pytest.mark.parametrize('bad',[0,.00001,.00101,float('nan'),True,'0.001'])
async def test_ready_tolerance_range_rejected_before_connection(bad):
    calls=[]
    async def factory(_ip):calls.append(True)
    with pytest.raises((TypeError,ValueError)):
        await module().run_window(control_settings(),Recorder(),factory,lambda:None,lambda:None,
            ready_joint_tolerance_rad=bad)
    assert calls==[]


async def test_pre_native_refusal_records_local_reason_without_sdk_entry():
    m=module();native=ZClient()
    def deny(*_args):raise m.AdmissionRejected('idle_baseline_sample_not_current')
    proxy=m.FiniteScopeClient(native,deny,lambda:None,lambda:None,
        asyncio.current_task(),lambda:1)
    with pytest.raises(m.AdmissionRejected):
        await proxy.move_pvat([0.]*6,[0.]*6,[0.]*6,.08)
    assert proxy.local_motion_rejection=='idle_baseline_sample_not_current'
    assert proxy.pvat_entries==0 and native.write_calls==[]


async def test_expensive_source_check_runs_once_at_actual_native_entry():
    m=module();native=ZClient();checks=[]
    def source():
        assert proxy.pvat_entries==0
        checks.append('source')
    proxy=m.FiniteScopeClient(native,lambda *_:None,lambda:None,lambda:None,
        asyncio.current_task(),lambda:1,native_write_check=source)
    proxy._motion_permission([0.]*6,[0.]*6,[0.]*6,.08)
    assert checks==[]
    await proxy.move_pvat([0.]*6,[0.]*6,[0.]*6,.08)
    assert checks==['source'] and proxy.pvat_entries==1


async def test_native_source_check_refusal_cannot_enter_sdk():
    m=module();native=ZClient()
    def source():raise ValueError('finite_scope_loaded_runtime_metadata_changed')
    proxy=m.FiniteScopeClient(native,lambda *_:None,lambda:None,lambda:None,
        asyncio.current_task(),lambda:1,native_write_check=source)
    with pytest.raises(ValueError):await proxy.move_pvat([0.]*6,[0.]*6,[0.]*6,.08)
    assert proxy.pvat_attempted and proxy.pvat_entries==0 and native.write_calls==[]
    assert proxy.local_motion_rejection=='finite_scope_loaded_runtime_metadata_changed'
    assert not proxy.pvat_remote_result_unknown


@pytest.mark.parametrize('scan_ms,expected_entries',[(8,1),(12,0)])
async def test_one_final_scan_keeps_original_100ms_gate(monkeypatch,scan_ms,expected_entries):
    m=module();clock=FakeClock();scans=[];advanced=False
    begin=m.FiniteWindowDiagnosticAdapter._begin_motion_epoch
    def epoch(adapter):
        nonlocal advanced
        begin(adapter)
        if not advanced:clock.advance_ms(88);advanced=True
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_begin_motion_epoch',epoch)
    def source():scans.append(True);clock.advance_ms(scan_ms)
    client=ZClient();settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    r=await m.run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,write_check=source)
    assert len(scans)==1 and r['pvat_entries']==expected_entries
    assert r['stop_entries']==1
    if not expected_entries:assert r['local_motion_rejection']=='idle_baseline_sample_not_current'


def parallel_proxy(native):
    return module().FiniteScopeClient(native,lambda *_:None,lambda:None,lambda:None,
        asyncio.current_task(),lambda:1,parallel_fk=True)


async def test_parallel_scoped_reads_share_quota_deadline_and_block_ordinary_reads():
    client=ZClient();proxy=parallel_proxy(client);started=asyncio.Event();release=asyncio.Event()
    original=client.kinematics_forward
    async def blocked(q):
        started.set();await release.wait();return await original(q)
    client.kinematics_forward=blocked
    task=asyncio.create_task(proxy._kinematics_forward_guarded([0.]*6,lambda:None))
    try:
        await started.wait()
        assert proxy.fk_read_owners[0].read_deadline_ns==proxy.read_deadline_ns
        with pytest.raises(module().AdmissionRejected):await proxy.is_connected()
        release.set()
        with pytest.raises(module().AdmissionRejected):await task
        assert proxy.read_gate_closed and not client.write_calls
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)


async def test_parallel_native_boundary_rechecks_guard_after_scheduling(monkeypatch):
    client=ZClient();proxy=parallel_proxy(client);allowed=True;original=asyncio.create_task
    def create(coro,**kwargs):
        nonlocal allowed
        if kwargs.get('name')=='paused-cleanup-kinematics_forward':allowed=False
        return original(coro,**kwargs)
    def guard():
        if not allowed:raise module().AdmissionRejected('finite_scope_revoked')
    monkeypatch.setattr(asyncio,'create_task',create)
    with pytest.raises(module().AdmissionRejected):
        await proxy._kinematics_forward_guarded([0.]*6,guard)
    assert client.fk_calls==[] and proxy.math_entries['kinematics_forward']==0


async def test_parallel_scope_uses_exactly_four_fk_and_original_stop():
    client=ZClient();clock=FakeClock();settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    report=await module().run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,parallel_fk_audit=True)
    assert report['complete'] and report['parallel_fk_audit'] is True
    assert report['math_entries']==dict(kinematics_inverse=1,kinematics_forward=4)
    assert not report['owned_fk_pending_at_exit']
    assert [c[0] for c in client.write_calls]==['move_pvat','stop_move']


async def test_parallel_motion_snapshot_keeps_connection_first_and_motion_id_dependency(monkeypatch):
    m=module();client=ZClient();clock=FakeClock();started=set();finished=set();release=asyncio.Event()
    client.running_motion=7
    motion_queries=[]
    active=False
    original_fields=m.FiniteWindowDiagnosticAdapter._read_snapshot_fields
    async def fields(adapter,*args,**kwargs):
        nonlocal active
        active=kwargs['read_purpose']=='motion'
        return await original_fields(adapter,*args,**kwargs)
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_read_snapshot_fields',fields)
    names=['get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion']
    for name in names:
        original=getattr(client,name)
        async def read(*args,_name=name,_original=original):
            if active:
                started.add(_name)
                if len(started)==6:release.set()
                await release.wait();finished.add(_name)
            return await _original(*args)
        setattr(client,name,read)
    original_motion=client.get_motion_state
    async def motion(mid):
        if active:
            assert len(finished)==6
            motion_queries.append(mid)
        return await original_motion(mid)
    client.get_motion_state=motion
    settings=control_settings();settings=replace(settings,control=replace(settings.control,
        max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    report=await asyncio.wait_for(m.run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,parallel_fk_audit=True,parallel_motion_reads=True),1.)
    assert report['complete'] and report['parallel_motion_reads']
    assert len(started)==6 and report['math_entries']==dict(kinematics_inverse=1,kinematics_forward=4)
    assert not report['owned_snapshot_pending_at_exit']
    assert motion_queries==[7]


def snapshot_proxy(client):
    proxy=module().FiniteScopeClient(client,lambda *_:None,lambda:None,lambda:None,
        asyncio.current_task(),lambda:1,parallel_motion_reads=True)
    proxy.raw_estop_clear=True
    return proxy


async def test_parallel_snapshot_group_cannot_be_reused_by_inherited_context():
    client=ZClient();proxy=snapshot_proxy(client);release=asyncio.Event()
    async def later():await release.wait();return await proxy.get_kin_data()
    with proxy._parallel_snapshot_scope(lambda:None):task=asyncio.create_task(later())
    release.set()
    with pytest.raises(module().AdmissionRejected,match='group_revoked'):await task
    assert proxy._native_snapshot_entry_count('get_kin_data')==0


async def test_parallel_snapshot_actual_entry_rechecks_expiry(monkeypatch):
    client=ZClient();proxy=snapshot_proxy(client);allowed=True;create=asyncio.create_task
    def guard():
        if not allowed:raise module().AdmissionRejected('finite_scope_expired')
    def schedule(coro,**kwargs):
        nonlocal allowed
        if kwargs.get('name')=='paused-cleanup-get_kin_data':allowed=False
        return create(coro,**kwargs)
    monkeypatch.setattr(asyncio,'create_task',schedule)
    with proxy._parallel_snapshot_scope(guard):
        with pytest.raises(module().AdmissionRejected):await proxy.get_kin_data()
    assert proxy._native_snapshot_entry_count('get_kin_data')==0


async def test_snapshot_duplicate_method_and_math_inside_group_are_rejected():
    for method in ['get_kin_data','kinematics_inverse']:
        proxy=snapshot_proxy(ZClient())
        with proxy._parallel_snapshot_scope(lambda:None):
            if method=='get_kin_data':await proxy.get_kin_data()
            with pytest.raises(module().AdmissionRejected):
                if method=='get_kin_data':await proxy.get_kin_data()
                else:await proxy.kinematics_inverse({},[0.]*6)
        assert proxy.read_gate_closed


async def test_parallel_snapshot_pending_native_evidence_is_not_parent_completed_read():
    client=ZClient();proxy=snapshot_proxy(client);started=asyncio.Event();release=asyncio.Event()
    original=client.get_kin_data
    async def stubborn():
        started.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return await original()
    client.get_kin_data=stubborn
    with proxy._parallel_snapshot_scope(lambda:None):
        await proxy.is_connected()
        task=asyncio.create_task(proxy.get_kin_data())
        try:
            await started.wait()
            assert proxy.pending_read.done()
            assert proxy._native_snapshot_entry_count('get_kin_data')==1
            assert proxy._native_snapshot_result_unknown('get_kin_data') is True
            task.cancel()
            with pytest.raises(asyncio.CancelledError):await task
            assert proxy._native_snapshot_result_unknown('get_kin_data') is True
        finally:
            release.set();await proxy.snapshot_read_owners['get_kin_data'].pending_read
    assert proxy.read_gate_closed and proxy.read_remote_result_unknown
    with pytest.raises(module().AdmissionRejected):proxy.require_clear_read_gate()


@pytest.mark.parametrize('method',['get_robot_state','get_estop_reason','get_kin_data',
    'get_tcp','get_claw','get_running_motion'])
async def test_each_parallel_snapshot_failure_blocks_all_math_and_pvat(monkeypatch,method):
    m=module();client=ZClient();clock=FakeClock();active=False
    fields=m.FiniteWindowDiagnosticAdapter._read_snapshot_fields
    async def wrapped(adapter,*args,**kw):
        nonlocal active
        active=kw['read_purpose']=='motion'
        try:return await fields(adapter,*args,**kw)
        finally:active=False
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_read_snapshot_fields',wrapped)
    original=getattr(client,method)
    async def fail(*args):
        if active:raise RuntimeError('injected_snapshot_failure')
        return await original(*args)
    setattr(client,method,fail)
    settings=control_settings();settings=replace(settings,control=replace(settings.control,
        max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    report=await m.run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,parallel_fk_audit=True,parallel_motion_reads=True)
    assert not report['complete'] and report['pvat_entries']==0 and report['stop_entries']==1
    assert report['math_entries']==dict(kinematics_inverse=0,kinematics_forward=0)
    assert client.write_calls==[('stop_move',)]


async def test_parallel_snapshot_outer_timeout_retains_unknown_and_never_publishes_late_snapshot(monkeypatch):
    m=module();client=ZClient();clock=FakeClock();active=False;published=[]
    release=asyncio.Event();started=asyncio.Event();finished=asyncio.Event();report={}
    fields=m.FiniteWindowDiagnosticAdapter._read_snapshot_fields
    before=m.FiniteWindowDiagnosticAdapter._before_snapshot_install
    async def wrapped(adapter,*args,**kw):
        nonlocal active
        active=kw['read_purpose']=='motion'
        try:return await fields(adapter,*args,**kw)
        finally:active=False
    def install(adapter,snapshot,read):
        if snapshot.read_purpose=='motion':published.append(snapshot)
        return before(adapter,snapshot,read)
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_read_snapshot_fields',wrapped)
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_before_snapshot_install',install)
    original=client.get_kin_data
    async def stubborn():
        if active:
            started.set()
            while not release.is_set():
                try:await release.wait()
                except asyncio.CancelledError:pass
            finished.set()
        return await original()
    client.get_kin_data=stubborn
    settings=control_settings();settings=replace(settings,control=replace(settings.control,
        max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    task=asyncio.create_task(m.run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,parallel_fk_audit=True,parallel_motion_reads=True,report=report))
    try:
        await asyncio.wait_for(asyncio.shield(task),.8)
        assert started.is_set() and not published
        assert report['pvat_entries']==0 and report['stop_entries']==1
        assert report['owned_snapshot_pending_at_exit'] and report['read_remote_result_unknown']
        assert 'get_kin_data' in report['snapshot_rpc_ownership']['unknown_methods']
        assert report['math_entries']==dict(kinematics_inverse=0,kinematics_forward=0)
        frozen=repr(report);release.set();await asyncio.wait_for(finished.wait(),.2);await asyncio.sleep(0)
        assert repr(report)==frozen and not published
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)


@pytest.mark.parametrize('bad',[1,'yes',None])
async def test_parallel_snapshot_requires_boolean_before_connection(bad):
    with pytest.raises(TypeError):snapshot_proxy_with_option=module().FiniteScopeClient(
        ZClient(),lambda *_:None,lambda:None,lambda:None,asyncio.current_task(),lambda:1,
        parallel_motion_reads=bad)


async def test_parallel_fk_shared_native_quota_and_original_deadline():
    client=ZClient();proxy=parallel_proxy(client);deadline=proxy.read_deadline_ns
    for _ in range(4):await proxy._kinematics_forward_guarded([0.]*6,lambda:None)
    assert len(client.fk_calls)==4 and all(c.read_deadline_ns==deadline for c in proxy.fk_read_owners)
    with pytest.raises(module().AdmissionRejected):
        await proxy._kinematics_forward_guarded([0.]*6,lambda:None)
    assert len(client.fk_calls)==4 and proxy.read_gate_closed


async def test_parallel_fk_rejects_pending_parent_read():
    client=ZClient();proxy=parallel_proxy(client);started=asyncio.Event();release=asyncio.Event()
    async def pending():started.set();await release.wait();return True
    task=asyncio.create_task(proxy.read('is_connected',pending))
    try:
        await started.wait()
        with pytest.raises(module().AdmissionRejected):
            await proxy._kinematics_forward_guarded([0.]*6,lambda:None)
        assert not client.fk_calls
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)


async def test_parallel_stubborn_native_read_stays_owned_and_late_success_cannot_reopen():
    client=ZClient();proxy=parallel_proxy(client);started=asyncio.Event();release=asyncio.Event()
    async def stubborn(q):
        started.set()
        while not release.is_set():
            try:await release.wait()
            except asyncio.CancelledError:pass
        return ACTUAL_TCP
    client.kinematics_forward=stubborn
    task=asyncio.create_task(proxy._kinematics_forward_guarded([0.]*6,lambda:None))
    try:
        await started.wait();task.cancel()
        with pytest.raises(asyncio.CancelledError):await task
        assert proxy._fk_read_pending() and proxy.read_remote_result_unknown and proxy.read_gate_closed
        with pytest.raises(module().AdmissionRejected):proxy.require_clear_read_gate()
        release.set();await proxy.fk_read_owners[0].pending_read
        assert proxy.read_remote_result_unknown and proxy.read_gate_closed
        with pytest.raises(module().AdmissionRejected):await proxy.is_connected()
        assert client.write_calls==[]
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)


async def test_parallel_scoped_triplet_timeout_freezes_owned_native_unknown_report():
    client=ZClient();clock=FakeClock();report={};release=asyncio.Event();finished=asyncio.Event()
    original=client.kinematics_forward
    async def stubborn(q):
        result=await original(q)
        if len(client.fk_calls)==3:
            while not release.is_set():
                try:await release.wait()
                except asyncio.CancelledError:pass
            finished.set()
        return result
    client.kinematics_forward=stubborn
    settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,
        max_joint_acceleration_radps2=.2))
    async def factory(_ip):return client
    task=asyncio.create_task(module().run_window(settings,Recorder(),factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,parallel_fk_audit=True,report=report))
    try:
        await asyncio.wait_for(asyncio.shield(task),.8)
        assert report['pvat_entries']==0 and report['stop_entries']==1
        assert not report['complete'] and report['owned_fk_pending_at_exit']
        assert report['read_remote_result_unknown'] and report['fk_read_gate_closed']
        assert client.write_calls==[('stop_move',)]
        frozen=repr(report);release.set();await asyncio.wait_for(finished.wait(),.2)
        await asyncio.sleep(0)
        assert repr(report)==frozen
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)


@pytest.mark.parametrize('bad',[1,'yes',None])
async def test_parallel_option_requires_boolean(bad):
    client=ZClient()
    with pytest.raises(TypeError):
        module().FiniteScopeClient(client,lambda *_:None,lambda:None,lambda:None,
            asyncio.current_task(),lambda:1,parallel_fk=bad)


async def test_local_checks_crossing_snapshot_age_keep_the_original_rejection(monkeypatch):
    m=module();clock=FakeClock();active=False;delayed=False
    original=m.FiniteWindowDiagnosticAdapter._begin_motion_epoch
    def begin(adapter):
        nonlocal active
        original(adapter);clock.advance_ms(94);active=True
    def hot():
        nonlocal delayed
        if active and not delayed:clock.advance_ms(15);delayed=True
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_begin_motion_epoch',begin)
    result,client=await run(clock=clock,hot=hot)
    assert result['pvat_entries']==0 and result['stop_confirmed']
    assert result['motion_sample_age_ms']==109.
    assert result['local_motion_rejection']=='idle_baseline_sample_not_current'
    assert client.write_calls==[('stop_move',)]


@pytest.mark.parametrize('kind', ['paused', 'speed', 'acceleration', 'ambiguous_estop', 'bad_tcp'])
async def test_initial_rejection_has_no_robot_writes(kind):
    client = ZClient()
    if kind == 'paused': client.robot_state = 'PAUSED'
    elif kind == 'speed': client.kin_data['actual_joint_speed'][0] = 1e-9
    elif kind == 'acceleration': client.kin_data['actual_joint_acc'][0] = 1e-9
    elif kind == 'ambiguous_estop': client.estop_reason = None
    else: client.tcp['z'] += .01
    result, _ = await run(client)
    assert not result['complete'] and client.write_calls == [] and result['pvat_entries'] == 0


async def test_invalid_authorization_never_calls_factory():
    entered = []
    async def factory(ip): entered.append(ip)
    report = await module().run_window(control_settings(), Recorder(), factory, lambda: False, lambda: None)
    assert entered == [] and not report['complete']


@pytest.mark.parametrize('kind', ['expired', 'recording', 'raw_state'])
async def test_final_native_pvat_guard_blocks_changed_authority(monkeypatch, kind):
    m = module(); client = ZClient()
    revoked = False
    original = m.FiniteWindowDiagnosticAdapter._begin_motion_epoch
    recorder = Recorder()
    def change(adapter):
        nonlocal revoked
        original(adapter)
        if kind == 'expired': revoked = True
        elif kind == 'recording': recorder.fatal_error = 'disk_failure'
        else: adapter._snapshot = replace(adapter._snapshot, raw_robot_state='MOVING')
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter, '_begin_motion_epoch', change)
    def hot():
        if revoked: raise ValueError('expired')
    result, _ = await run(client, recorder=recorder, hot=hot)
    assert not result['complete'] and result['pvat_entries'] == 0
    assert client.write_calls == [('stop_move',)]


@pytest.mark.parametrize('kind', ['late', 'moving'])
async def test_stop_failure_is_preserved_and_not_retried(kind):
    client, clock = ZClient(), FakeClock()
    original = client.stop_move
    async def stop():
        await original()
        if kind == 'late': clock.advance_ms(621)
        else:
            client.robot_state = 'MOVING'
            client.kin_data['actual_joint_speed'] = [.01]*6
    client.stop_move = stop
    result, _ = await run(client, clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == ('stop_unverified' if kind=='late' else 'stop_incomplete')
    assert [c[0] for c in client.write_calls] == ['move_pvat','stop_move']


async def test_expired_motion_permission_still_allows_reserved_single_cleanup_stop():
    client = ZClient(); expired = False
    original = client.move_pvat
    async def move(*args):
        nonlocal expired
        result = await original(*args); expired = True
        return result
    client.move_pvat = move
    def hot():
        if expired: raise ValueError('scope_expired_after_native_entry')
    result, _ = await run(client, hot=hot)
    assert result['pvat_entries'] == result['stop_entries'] == 1 and not result['complete']
    assert [c[0] for c in client.write_calls] == ['move_pvat','stop_move']


async def test_fk_pending_timeout_cannot_stall_or_reenter_reads():
    client = ZClient(); release, finished = asyncio.Event(), asyncio.Event()
    async def pending(joints):
        client.fk_calls.append(list(joints))
        try: await release.wait()
        except asyncio.CancelledError: await release.wait()
        finished.set()
        return {**ACTUAL_TCP}
    client.kinematics_forward = pending
    task = asyncio.create_task(run(client))
    try:
        result, _ = await asyncio.wait_for(asyncio.shield(task), .9)
        assert not result['complete'] and result['read_remote_result_unknown'] and result['pvat_entries']==0
        assert len(client.fk_calls)==1 and client.write_calls == [('stop_move',)]
        frozen=str(result); release.set(); await finished.wait(); await asyncio.sleep(0)
        assert str(result)==frozen
    finally:
        release.set(); await asyncio.gather(task, return_exceptions=True)


async def test_cancel_after_pvat_keeps_one_reserved_stop_and_saved_fault():
    client, recorder = ZClient(), Recorder(); owner=[None]
    original = client.move_pvat
    async def cancel(*args):
        result = await original(*args); owner[0].cancel(); return result
    client.move_pvat = cancel
    report={}
    m=module(); settings=control_settings()
    settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    async def factory(ip): return client
    owner[0]=asyncio.create_task(m.run_window(settings,recorder,factory,lambda:None,lambda:None,report=report))
    with pytest.raises(asyncio.CancelledError): await owner[0]
    assert report['interrupted'] and not report['complete'] and report['stop_entries']==1
    assert [c[0] for c in client.write_calls]==['move_pvat','stop_move']


async def test_bad_first_tail_sample_cannot_be_hidden_by_later_good_samples(monkeypatch):
    m=module(); client=ZClient(); original=m.FiniteWindowDiagnosticAdapter.read_stop_observation
    count=0
    async def observe(adapter):
        nonlocal count
        count+=1
        sample=await original(adapter)
        if count==3: sample['actual_tcp']['p'][0] += .0001
        return sample
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'read_stop_observation',observe)
    result,_=await run(client)
    assert result['tail_complete'] and not result['displacement_consistent'] and not result['complete']


async def test_native_entry_rejects_packet_when_cleanup_reservation_has_expired(monkeypatch):
    m=module(); clock=FakeClock(); client=ZClient()
    original=m.FiniteWindowDiagnosticAdapter.submit_tcp
    async def delayed(adapter,target,command_id):
        clock.advance_ms(5100)
        adapter._snapshot=replace(adapter._snapshot, captured_ns=clock.now_ns(),
            read_requested_ns=clock.now_ns(), read_completed_ns=clock.now_ns())
        return await original(adapter,target,command_id)
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'submit_tcp',delayed)
    result,_=await run(client,clock=clock)
    assert not result['complete'] and result['pvat_entries']==0
    assert not any(c[0]=='move_pvat' for c in client.write_calls)


async def test_joint_drift_in_first_tail_frame_prevents_complete(monkeypatch):
    m=module();original=m.FiniteWindowDiagnosticAdapter.read_stop_observation;count=0
    async def observe(adapter):
        nonlocal count
        count+=1;sample=await original(adapter)
        if count==3:sample['actual_q'][1]+=.003
        return sample
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'read_stop_observation',observe)
    result,_=await run()
    assert not result['complete'] and not result['tail_complete']


@pytest.mark.parametrize('cancel_owner',[False,True])
async def test_stubborn_pvat_releases_lock_for_single_stop_and_bounded_exit(cancel_owner):
    client,recorder=ZClient(),Recorder();entered,release,finished=asyncio.Event(),asyncio.Event(),asyncio.Event()
    original=client.move_pvat
    async def pending(*args):
        await original(*args);entered.set()
        try:await release.wait()
        except asyncio.CancelledError:await release.wait()
        finished.set();return 1
    client.move_pvat=pending
    m=module();settings=control_settings();settings=replace(settings,control=replace(settings.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    clock=FakeClock();report={}
    async def factory(ip):return client
    task=asyncio.create_task(m.run_window(settings,recorder,factory,lambda:None,lambda:None,
        clock=clock.now_ns,sleep=clock.sleep,report=report))
    try:
        await entered.wait()
        if cancel_owner:task.cancel()
        if cancel_owner:
            with pytest.raises(asyncio.CancelledError):await asyncio.wait_for(asyncio.shield(task),.8)
        else:await asyncio.wait_for(asyncio.shield(task),.8)
        assert report['pvat_remote_result_unknown'] and not report['complete'] and not report['stop_confirmed']
        assert report['latched_fault']=='stop_unverified'
        assert [c[0] for c in client.write_calls]==['move_pvat','stop_move']
        frozen=str(report);release.set();await finished.wait();await asyncio.sleep(0)
        assert str(report)==frozen
    finally:
        release.set();await asyncio.gather(task,return_exceptions=True)


async def test_native_child_rechecks_permission_after_parent_guard(monkeypatch):
    m=module();revoked=False;original=asyncio.create_task
    def schedule(coro,*args,**kwargs):
        nonlocal revoked
        if kwargs.get('name')=='scoped-native-pvat':revoked=True
        return original(coro,*args,**kwargs)
    monkeypatch.setattr(asyncio,'create_task',schedule)
    def hot():
        if revoked:raise ValueError('expired_between_tasks')
    result,client=await run(hot=hot)
    assert not result['complete'] and result['pvat_entries']==0 and result['stop_entries']==1
    assert client.write_calls==[('stop_move',)]


async def test_native_pvat_future_is_supported_without_duplicate_dispatch():
    client=ZClient();original=client.move_pvat
    def future(*args):return asyncio.ensure_future(original(*args))
    client.move_pvat=future
    result,_=await run(client)
    assert result['complete'] and result['pvat_entries']==result['stop_entries']==1


async def test_checks_cannot_consume_last_cleanup_second_before_native_entry(monkeypatch):
    m=module();clock=FakeClock();active=False;original=m.FiniteWindowDiagnosticAdapter._begin_motion_epoch
    monkeypatch.setattr(m,'CLEANUP_STOP_S',1.03)
    def begin(adapter):
        nonlocal active
        original(adapter);active=True
    monkeypatch.setattr(m.FiniteWindowDiagnosticAdapter,'_begin_motion_epoch',begin)
    def hot():
        if active:clock.advance_ms(6)
    result,client=await run(clock=clock,hot=hot)
    assert result['pvat_entries']==0 and not any(c[0]=='move_pvat' for c in client.write_calls)


@pytest.mark.parametrize('stage', ['final_event', 'close'])
@pytest.mark.parametrize('signal', [KeyboardInterrupt, SystemExit])
async def test_final_recorder_interruption_is_preserved(stage, signal):
    class InterruptedRecorder(Recorder):
        async def write_critical_event(self, name, payload, now_ns):
            if stage == 'final_event' and name == 'finite_scope_result':
                raise signal()
            return await super().write_critical_event(name, payload, now_ns)

        async def close(self):
            if stage == 'close':
                raise signal()
            return await super().close()

    client, clock, report = ZClient(), FakeClock(), {}
    settings = control_settings()
    settings = replace(settings, control=replace(settings.control,
        max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2))
    async def factory(_ip):
        return client
    with pytest.raises(signal):
        await module().run_window(settings, InterruptedRecorder(), factory,
            lambda: None, lambda: None, clock=clock.now_ns, sleep=clock.sleep,
            report=report)
    assert report['interrupted'] and not report['complete']
    assert report['error_type'] == signal.__name__
    assert report['pvat_entries'] == report['stop_entries'] == 1
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']


async def test_proxy_rejection_before_native_entry_is_not_remote_unknown(monkeypatch):
    m = module()
    async def reject(proxy, *args):
        raise m.AdmissionRejected('scope_rejected_before_native')
    monkeypatch.setattr(m.FiniteScopeClient, 'move_pvat', reject)
    report, client = await run()
    assert report['pvat_entries'] == 0 and report['stop_entries'] == 1
    assert report['original_stop_confirmed'] and report['stop_confirmed']
    assert not report['pvat_remote_result_unknown'] and report['latched_fault'] is None
    assert client.write_calls == [('stop_move',)] and not report['complete']
