"""One explicitly scoped call to the existing prepare API. No native factory."""
import asyncio
import copy
import math
import re
import time
from dataclasses import asdict

from app.commissioning.joint_move_speed import JointMoveSpeedAudit
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient, _geometry
from app.commissioning.position_snapshot_groups import PositionSnapshotGroups
from app.robots.base import HomeOptions, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.schemas.messages import BackendState

READY_TOLERANCE_RAD = math.radians(.1)
OPTIONS = HomeOptions(.05, 60., READY_TOLERANCE_RAD, .02, .3)
# A 60s prepare can poll every 20ms: up to nine independent reads per poll,
# plus connection/preflight, the 2s tail and reserved fault-stop verification.
# The former 512-field limit could expire before a settling motion completed.
MAX_READS = 32768


def _q(value):
    if (not isinstance(value, (tuple,list)) or len(value)!=6
            or any(type(x) not in (int,float) or not math.isfinite(x) for x in value)):
        raise ValueError('ready_return_invalid_joint_vector')
    return tuple(float(x) for x in value)


def _permit(callback):
    value=callback()
    if value is not None:
        if asyncio.iscoroutine(value):value.close()
        raise ValueError('ready_return_invalid_permission_result')


def _reason(error):
    value=str(error)
    return value if (re.fullmatch(r'[a-z0-9_:.]{1,128}',value)
        and value.startswith(('ready_return_','sdk_timeout:','stop_','paused_cleanup_',
                              'preflight_not_ready:','snapshot_','home_timeout'))) else 'ready_return_failed'


class _PrepareReadClient(BoundedStopOnlyClient):
    read_entry_limit=MAX_READS


class _PrepareClient(PositionSnapshotGroups,BoundedStopOnlyClient):
    READS=BoundedStopOnlyClient.READS|{'get_velocity_factor'}
    read_entry_limit=MAX_READS
    snapshot_read_client_type=_PrepareReadClient

    def __init__(self,native,stop_permission,read_permission,write_permission,initial,owner,clock):
        super().__init__(native,stop_permission,read_permission,owner,deadline_clock=clock)
        self.write_permission,self.initial=write_permission,initial
        self.movej_entries=0;self.movej_attempted=False;self.movej_unknown=False
        self.movej_call=None;self.movej_task=None;self.movej_entry_rejection=None

    async def movej(self,p,a,v,t,r):
        if self.movej_attempted:raise ValueError('ready_return_movej_consumed')
        self.movej_attempted=True
        args=(_q(p),a,v,t,r)
        row=self.movej_call=dict(outcome='not_started',remote_result_unknown=False)
        async def send():
            try:
                self.write_permission();self.require_clear_read_gate();self.initial()
                if args!=(self.target_q,.2,.05,0.,0.):raise ValueError('ready_return_fixed_movej_required')
            except BaseException as error:
                self.movej_entry_rejection=_reason(error);raise
            self.movej_entries+=1;row.update(outcome='entered',started_ns=self.deadline_clock())
            return await self.client.movej(list(args[0]),*args[1:])
        task=self.movej_task=asyncio.create_task(send(),name='ready-return-native-movej')
        task.add_done_callback(lambda done:None if done.cancelled() else done.exception())
        try:
            done,_=await asyncio.wait({task},timeout=.2)
            if not done:raise TimeoutError('ready_return_movej_timeout')
            value=task.result()
            if type(value) is not int or value<=0:raise ValueError('ready_return_invalid_motion_id')
            if self.deadline_clock()-row['started_ns']>=200_000_000:
                raise TimeoutError('ready_return_movej_late')
            row.update(outcome='returned',completed_ns=self.deadline_clock(),motion_id=value)
            return value
        except BaseException:
            if self.movej_entries:
                self.movej_unknown=True;row.update(outcome='unknown',remote_result_unknown=True)
            if not task.done() and not task.cancelling():task.cancel()
            raise


class _PrepareAdapter(RealLebaiAdapter):
    """Owned snapshot groups for this isolated return; stop reads stay serial."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._prepare_group_guard=None;self._finite_snapshot_wrappers=set()

    async def _read_snapshot_impl(self,read,**kwargs):
        if kwargs['read_purpose']=='stop_verification':
            return await super()._read_snapshot_impl(read,**kwargs)
        client=self._client;owner=self._snapshot_calls;generation=self._pump.generation
        def guard():
            if (client is not self._client or read.client is not client or owner is not self._snapshot_calls
                    or owner.closed or owner.unknown_methods or generation!=self._pump.generation):
                raise ValueError('ready_return_snapshot_binding_changed')
        if self._prepare_group_guard is not None:raise ValueError('ready_return_snapshot_group_busy')
        self._prepare_group_guard=guard
        try:
            with client.position_snapshot_scope(guard,kwargs['read_purpose']):
                return await super()._read_snapshot_impl(read,**kwargs)
        finally:self._prepare_group_guard=None

    async def _read_snapshot_fields(self,client,latencies,deadline_ns,**kwargs):
        if self._prepare_group_guard is None:
            return await super()._read_snapshot_fields(client,latencies,deadline_ns,**kwargs)
        from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
        return await FiniteWindowDiagnosticAdapter._parallel_snapshot_fields(self,client,latencies,deadline_ns,
            check=self._prepare_group_guard,include_raw_kinematics=kwargs['include_raw_kinematics'])

    def _before_snapshot_install(self,snapshot,read):
        if self._prepare_group_guard is not None:self._client.verify_position_snapshot_group()
        return super()._before_snapshot_install(snapshot,read)


async def run_ready_return(settings,start_q,recorder,client_factory,authorize,*,
                           read_authorize=None,write_authorize=None,clock=time.perf_counter_ns,
                           sleep=asyncio.sleep,report=None):
    """Uses a new local adapter; a caller must supply scope, factory and consent."""
    start_q=_q(start_q);target=_q(settings.teleop_ready_q)
    if (settings.mode!='control' or settings.control.max_joint_speed_radps!=.05
            or settings.control.max_joint_acceleration_radps2!=.2
            or not all(callable(x) for x in (client_factory,authorize,clock,sleep))):
        raise ValueError('ready_return_explicit_profile_required')
    binding=copy.deepcopy(asdict(settings));owner=asyncio.current_task()
    started=clock();deadline=started+90_000_000_000;reserve=deadline+5_000_000_000
    report={} if report is None else report
    report.update(complete=False,connect_entries=0,movej_entries=0,stop_entries=0,
        movej_remote_result_unknown=False,hardware_verified=False,motion_time_stop_verified=False,
        continuous_control_admitted=False,native_connection_release_verified=False,
        recording_closed_verified=False,process_exit_required=True,
        local_resources_released=False,tail_complete=False,latched_fault=None,
        options=asdict(OPTIONS),read_calls=[],cleanup_errors=[],
        snapshot_read_policy='owned_six_fields_parallel_except_stop',snapshot_groups=[])
    adapter=proxy=None;settled=False;interrupted=False;started_recorder=False
    audit=JointMoveSpeedAudit(.05)

    def health():
        if asdict(settings)!=binding:raise ValueError('ready_return_configuration_changed')
        if (getattr(recorder,'fatal_error',None) is not None
                or getattr(recorder,'dropped_normal_events',0)):
            raise ValueError('ready_return_recording_unavailable')

    def permission():
        health();_permit(authorize)
        if clock()>=deadline:raise ValueError('ready_return_session_expired')

    def read_permission():
        health()
        if proxy is not None and proxy.sdk_stop_entries:
            if clock()>=reserve:raise ValueError('ready_return_stop_reserve_expired')
        else:
            permission()
            if read_authorize is not None and read_authorize is not authorize:_permit(read_authorize)

    def stop_permission():
        if proxy is None or proxy.movej_entries!=1 or clock()>=reserve:
            raise ValueError('ready_return_stop_not_reserved')

    def initial():
        s=adapter._snapshot
        if (adapter._client is not proxy or s is None or s.robot_state is not BackendState.IDLE
                or s.estop or s.running_motion is not None or s.motion_state not in (None,'FINISHED')
                or not 0<=clock()-s.captured_ns<100_000_000):
            raise ValueError('ready_return_initial_state_or_freshness')
        if any(max(abs(a-b) for a,b in zip(q,start_q,strict=True))>READY_TOLERANCE_RAD for q in (s.actual_q,s.target_q)):
            raise ValueError('ready_return_initial_pose_changed')
        if any(x!=0 for v in (s.actual_qd,s.actual_qdd,s.target_qd,s.target_qdd) for x in v):
            raise ValueError('ready_return_initial_motion_reported')
        _geometry(adapter)

    def write_permission():
        permission();_permit(write_authorize or authorize)

    async def event(row,stamp):
        audit.observe(row);await recorder.write_event(row,stamp)

    async def factory(ip):
        nonlocal proxy
        permission()
        if report['connect_entries'] or ip!=settings.ip:raise ValueError('ready_return_connection_consumed')
        report['connect_entries']+=1
        native=await client_factory(ip)
        permission()
        proxy=_PrepareClient(native,stop_permission,read_permission,write_permission,initial,owner,clock)
        proxy.read_deadline_ns=reserve;proxy.target_q=target
        return proxy

    async def preflight():
        snapshot=await adapter._read_snapshot(read_purpose='preflight')
        result=adapter._preflight_from_snapshot(snapshot)
        if not result.ready:raise ValueError('preflight_not_ready:'+str(result.reason))

    async def stop_once():
        if proxy is not None and proxy.movej_entries and not proxy.sdk_stop_entries:
            try:await adapter.stop(StopReason.SHUTDOWN)
            except BaseException as error:report['stop_error']=_reason(error)

    try:
        permission();await recorder.start();started_recorder=True
        adapter=_PrepareAdapter(settings,client_factory=factory,clock=clock,sleep=sleep,event_callback=event)
        async with asyncio.timeout(85.):
            await adapter.connect();await preflight();initial()
            if await proxy.get_velocity_factor()!=100:raise ValueError('ready_return_velocity_factor_changed')
            initial()
            await recorder.write_critical_event('ready_return_admission',dict(start_q=start_q,target_q=target),clock())
            permission()
            await adapter.prepare(OPTIONS,lambda phase:None,authorize=permission)
            await preflight();report['post_preflight_ready']=True
            tail_end=clock()+2_000_000_000
            while clock()<tail_end:
                permission();await adapter.get_state();s=adapter._snapshot
                _geometry(adapter)
                if (s.robot_state is not BackendState.IDLE or s.estop or s.running_motion is not None
                        or max(abs(a-b) for a,b in zip(s.actual_q,target,strict=True))>OPTIONS.position_tolerance_rad
                        or any(abs(x)>.02 for x in s.actual_qd)):
                    raise ValueError('ready_return_tail_not_settled')
                await sleep(.1)
            report['tail_complete']=True;settled=True
            if audit.report()['status']!='within_observed_limit':
                raise ValueError('ready_return_speed_evidence_not_accepted')
            if proxy.movej_unknown:raise ValueError('ready_return_movej_result_unknown')
            report['complete']=True
    except BaseException as error:
        report.update(complete=False,error_type=type(error).__name__,error=_reason(error))
        interrupted=(isinstance(error,(asyncio.CancelledError,KeyboardInterrupt,SystemExit))
            or owner.cancelling()>0)
        if not settled:await asyncio.shield(stop_once())
    finally:
        if adapter is not None:
            if proxy is None or not proxy.movej_entries:
                # This new isolated adapter has proof of zero native motion.
                # Drop only its local pre-entry obligation; no fault is cleared.
                adapter._motion_accepted=adapter._motion_write_stop_required=False
            try:await adapter.disconnect()
            except BaseException as error:report['cleanup_errors'].append(type(error).__name__)
            report['latched_fault']=adapter._latched_fault
            report['local_resources_released']=adapter._client is None and not adapter._pump.running
        if proxy is not None:
            proxy.read_gate_closed=True
            report.update(movej_entries=proxy.movej_entries,stop_entries=proxy.sdk_stop_entries,
                movej_remote_result_unknown=proxy.movej_unknown,movej_call=copy.deepcopy(proxy.movej_call),
                movej_entry_rejection=proxy.movej_entry_rejection,
                read_calls=copy.deepcopy(proxy.read_calls),read_remote_result_unknown=proxy.read_remote_result_unknown)
            report['snapshot_groups']=copy.deepcopy(proxy.snapshot_groups)
            if not proxy.movej_entries and proxy.movej_entry_rejection:
                report['error']=proxy.movej_entry_rejection
            if proxy.movej_unknown:report.update(complete=False,latched_fault='stop_unverified')
        report['speed_audit']=audit.report();report['duration_s']=(clock()-started)/1e9
        report['complete'] &= not report['cleanup_errors'] and report['latched_fault'] is None
        if started_recorder:
            provisional=copy.deepcopy(report)
            provisional.update(complete=False,result_is_provisional=True,
                execution_checks_passed_before_close=bool(report['complete']))
            try:await recorder.write_critical_event('ready_return_result',provisional,clock())
            except BaseException as error:report.update(complete=False,recording_error=type(error).__name__)
            try:
                await recorder.close();health();report['recording_closed_verified']=True
            except BaseException as error:
                report['cleanup_errors'].append(type(error).__name__);report['complete']=False
    if interrupted:raise asyncio.CancelledError
    return report
