"""Separately scoped one-position diagnostic. No SDK loader or UI registration."""
import asyncio
import copy
from dataclasses import asdict,replace
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np

from scipy.spatial.transform import Rotation
from app.commissioning.native_jog_mdh import MdhTool,mdh_tcp_jacobian,mdh_tcp_jacobian_rate
from app.commissioning.native_jog_site_scope import site_address,permission
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient
from app.commissioning.position_snapshot_groups import PositionSnapshotGroups
from app.control.finite_position_types import PositionIntent,INPUT_NS
from app.robots.base import StopReason
from app.robots.finite_position_adapter import _FinitePositionCore
from app.robots.finite_position_plan import evaluate_position_arrival
from app.schemas.messages import BackendState

DISTANCE_M=.0005
MAX_READS=512


@lru_cache(maxsize=64)
def _point(q,tool):
    matrix,j=mdh_tcp_jacobian(q,tool)
    return tuple(tuple(float(x) for x in row) for row in matrix),tuple(tuple(float(x) for x in row) for row in j)


@lru_cache(maxsize=64)
def _directional_rate(q,direction,tool):
    _,_,rate=mdh_tcp_jacobian_rate(q,direction,tool)
    return tuple(tuple(float(x) for x in row) for row in rate)


class MdhPositionModel:
    """Published theoretical model, checked against native FK on each path."""
    factory_calibrated=False

    def __init__(self,tool):
        if type(tool) is not MdhTool:raise ValueError('position_site_model_tool_required')
        self.tool=tool
        source=Path(__file__).with_name('native_jog_mdh.py').read_bytes()
        self.model_sha256=hashlib.sha256(source+json.dumps(asdict(tool),sort_keys=True).encode()).hexdigest()

    @classmethod
    def from_settings(cls,s):
        t=s.expected_tcp
        return cls(MdhTool((t.x,t.y,t.z),tuple(float(x) for x in Rotation.from_euler('ZYX',[t.rz,t.ry,t.rx]).as_quat())))

    def forward(self,q):
        matrix=np.array(_point(tuple(float(x) for x in q),self.tool)[0])
        return tuple(float(x) for x in matrix[:3,3]),tuple(float(x) for x in Rotation.from_matrix(matrix[:3,:3]).as_quat())

    def jacobian(self,q):return np.array(_point(tuple(float(x) for x in q),self.tool)[1])

    def jdot(self,q,qd):
        q=tuple(float(x) for x in q);qd=tuple(float(x) for x in qd);speed=max(abs(x) for x in qd)
        if not math.isfinite(speed):raise ValueError('position_site_invalid_rate')
        if speed==0:return np.zeros((6,6))
        # Analytic Jdot is linear in qd. Cache math only, never state or admission.
        return np.array(_directional_rate(q,tuple(x/speed for x in qd),self.tool))*speed


class _SingleClient(PositionSnapshotGroups,BoundedStopOnlyClient):
    READS=BoundedStopOnlyClient.READS|{'kinematics_inverse','kinematics_forward','get_velocity_factor'}

    def __init__(self,native,stop,read,write,owner,clock):
        super().__init__(native,stop,read,owner,deadline_clock=clock)
        self.write=write;self.movej_entries=0;self.movej_attempted=False;self.movej_unknown=False
        self.movej_returned=False;self.fixed_move=None
        self.math_entries=dict(kinematics_inverse=0,kinematics_forward=0,get_velocity_factor=0)

    async def read(self,name,invoke):
        if len(self.read_calls)>=MAX_READS:raise ValueError('position_site_read_budget')
        if name in self.math_entries:
            cap=5 if name=='kinematics_forward' else 1
            if self.math_entries[name]>=cap:raise ValueError('position_site_math_consumed')
            self.math_entries[name]+=1
        return await super().read(name,invoke)

    async def movej(self,q,a,v,t,r):
        if self.movej_attempted:raise ValueError('position_site_move_consumed')
        self.movej_attempted=True
        self.write();self.require_clear_read_gate()
        if self.fixed_move is None or (tuple(q),a,v,t,r)!=self.fixed_move:
            raise ValueError('position_site_move_changed')
        self.movej_entries+=1
        try:
            result=await self.client.movej(list(q),a,v,t,r);self.movej_returned=True;return result
        except BaseException:self.movej_unknown=True;raise

    async def stop_move(self):
        if self.movej_entries!=1:raise ValueError('position_site_stop_not_reserved')
        return await super().stop_move()


class _SiteAdapter(_FinitePositionCore):
    # Explicit standalone use only; never select the web session coordinator.
    position_candidate=False

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._position_group_guard=None;self._finite_snapshot_wrappers=set()

    async def _read_snapshot_impl(self,read,**kwargs):
        purpose=kwargs['read_purpose'];client=self._client
        scope=getattr(client,'position_snapshot_scope',None)
        if purpose=='stop_verification' or not callable(scope):
            return await super()._read_snapshot_impl(read,**kwargs)
        owner,generation=self._snapshot_calls,self._pump.generation
        def guard():
            if (client is not self._client or read.client is not client or owner is not self._snapshot_calls
                    or owner.closed or owner.unknown_methods or self._pump.generation!=generation):
                raise ValueError('position_snapshot_binding_changed')
        if self._position_group_guard is not None:raise ValueError('position_snapshot_group_busy')
        self._position_group_guard=guard
        try:
            with scope(guard,purpose):return await super()._read_snapshot_impl(read,**kwargs)
        finally:self._position_group_guard=None

    async def _read_snapshot_fields(self,client,latencies,deadline_ns,**kwargs):
        if self._position_group_guard is None:
            return await super()._read_snapshot_fields(client,latencies,deadline_ns,**kwargs)
        from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
        return await FiniteWindowDiagnosticAdapter._parallel_snapshot_fields(self,client,latencies,deadline_ns,
            check=self._position_group_guard,include_raw_kinematics=kwargs['include_raw_kinematics'])

    def _before_snapshot_install(self,snapshot,read):
        if self._position_group_guard is not None:self._client.verify_position_snapshot_group()
        return super()._before_snapshot_install(snapshot,read)

    async def connect(self):
        self._site_connecting=True
        try:return await super().connect()
        finally:self._site_connecting=False

    def _preflight_from_snapshot(self,snapshot,**kwargs):
        result=super()._preflight_from_snapshot(snapshot,**kwargs)
        if getattr(self,'_site_connecting',False):
            return replace(result,ready=False,reason=result.reason or 'position_site_explicit_single_only')
        return result

    def _position_native_entered(self):return self._client.movej_entries==1

    def _position_native_unknown(self):
        return self._client.movej_unknown or self._client.movej_entries==1 and not self._client.movej_returned


def _reason(error):
    value=str(error)
    return value if value.startswith(('position_','stop_','snapshot_','paused_cleanup_')) and len(value)<128 else 'position_site_failed'


async def run_single_position(settings,recorder,client_factory,authorize,*,config_sha256,model,
        read_authorize=None,write_authorize=None,clock=time.perf_counter_ns,sleep=asyncio.sleep):
    """Exactly one +Z target, no continuation/retry; caller owns live consent."""
    site_address(settings.ip)
    if (settings.mode!='control' or type(model) is not MdhPositionModel
            or model.model_sha256!=MdhPositionModel.from_settings(settings).model_sha256
            or not 0<settings.control.max_joint_speed_radps<=.05
            or not 0<settings.control.max_joint_acceleration_radps2<=.2
            or type(config_sha256) is not str or len(config_sha256)!=64
            or any(c not in '0123456789abcdef' for c in config_sha256)
            or not all(callable(x) for x in (client_factory,authorize,clock,sleep))):
        raise ValueError('position_site_profile_or_model_rejected')
    binding=copy.deepcopy(asdict(settings));owner=asyncio.current_task();deadline=clock()+15_000_000_000
    model_binding=copy.deepcopy((model.model_sha256,model.tool))
    reserve=deadline+5_000_000_000;adapter=proxy=None;plan=None;started=False;motion_done=False
    report=dict(complete=False,hardware_verified=False,continuous_control_admitted=False,
        snapshot_read_policy='owned_six_fields_parallel',snapshot_groups=[],
        motion_time_stop_verified=False,natural_arrival_confirmed=False,tail_complete=False,
        connect_entries=0,movej_entries=0,stop_entries=0,ik_entries=0,fk_entries=0,
        local_resources_released=False,native_connection_release_verified=False,process_exit_required=True,
        latched_fault=None,remote_unknown=False,cleanup_errors=[])

    def health():
        if asdict(settings)!=binding:raise ValueError('position_site_configuration_changed')
        if (model.model_sha256,model.tool)!=model_binding:raise ValueError('position_site_model_changed')
        if getattr(recorder,'fatal_error',None) is not None or getattr(recorder,'dropped_normal_events',0):
            raise ValueError('position_site_recorder_unavailable')

    def permit():
        health();permission(authorize)
        if owner.cancelling() or clock()>=deadline:raise ValueError('position_site_expired_or_cancelled')

    def read():
        if proxy is not None and proxy.sdk_stop_entries:
            if clock()>=reserve:raise ValueError('position_site_cleanup_expired')
        else:
            permit()
            if read_authorize is not None:permission(read_authorize)

    def entry():
        permit()
        if write_authorize is not None:permission(write_authorize)
        # This CLI authorizes one fixed target, not a streaming held-input lease.
        # Scope expiry and revocation apply throughout; the adapter re-reads the start.
        if intent is None:raise ValueError('position_site_input_missing')

    def stop():
        if proxy is None or proxy.movej_entries!=1 or clock()>=reserve:
            raise ValueError('position_site_stop_not_reserved')

    async def factory(ip):
        nonlocal proxy
        permit()
        if report['connect_entries'] or ip!=settings.ip:raise ValueError('position_site_connection_consumed')
        report['connect_entries']+=1;native=await client_factory(ip);permit()
        proxy=_SingleClient(native,stop,read,entry,owner,clock);proxy.read_deadline_ns=reserve
        return proxy

    async def event(row,stamp):await recorder.write_event(row,stamp)

    async def stop_once():
        if proxy is not None and proxy.movej_entries and not proxy.sdk_stop_entries:
            try:await adapter.stop(StopReason.FAULT)
            except BaseException as error:report['stop_error']=_reason(error)

    intent=None
    try:
        permit();await recorder.start();started=True
        def initialize_math():
            model.forward(settings.teleop_ready_q);model.jacobian(settings.teleop_ready_q)
            model.jdot(settings.teleop_ready_q,(.001,0.,0.,0.,0.,0.))
        await asyncio.to_thread(initialize_math);permit()
        report['static_math_initialized_before_connect']=True
        adapter=_SiteAdapter(settings,client_factory=factory,backend_label='LEBAI',position_model=model,
            clock=clock,sleep=sleep,event_callback=event)
        async with asyncio.timeout(15.):
            await adapter.connect();adapter.bind_position_generation(1)
            sample=await adapter._read_snapshot(read_purpose='single_position_preflight')
            if (not adapter._preflight_from_snapshot(sample).ready
                    or max(abs(a-b) for a,b in zip(sample.actual_q,settings.teleop_ready_q))>.001
                    or max(abs(a-b) for a,b in zip(sample.target_q,settings.teleop_ready_q))>.001
                    or any(x!=0 for v in (sample.actual_qd,sample.actual_qdd,sample.target_qd,sample.target_qdd) for x in v)):
                raise ValueError('position_site_initial_state')
            if await proxy.get_velocity_factor()!=100:raise ValueError('position_site_velocity_factor')
            intent=PositionIntent('standalone-single-position',1,1,clock(),'held',(0.,0.,1.))
            plan=await adapter.plan_step(intent,authorize=entry)
            proxy.fixed_move=(plan.goal_joints,plan.joint_acceleration,plan.joint_speed,0.,0.)
            result=await adapter.execute_step(plan,before_entry=entry,monitor_authorize=permit,on_entered=lambda:None)
            report.update(natural_arrival_confirmed=result.complete,motion_id=result.motion_id,
                reason=result.reason,remote_unknown=result.remote_unknown,
                target_p=list(plan.target_p),start_p=list(plan.start.tcp_p),model_audit=json.loads(json.dumps(asdict(plan.evidence))))
            if not result.complete:raise ValueError(result.reason or 'position_site_motion_failed')
            tail_end=clock()+2_000_000_000
            while clock()<tail_end:
                permit();s=await adapter._read_snapshot(read_purpose='single_position_tail')
                state=await proxy.get_motion_state(result.motion_id)
                current=adapter._position_sample(s,str(state).upper());adapter._position_feedback_limits(s)
                permit()
                if not 0<=clock()-current.captured_ns<INPUT_NS:
                    raise ValueError('position_site_tail_feedback_expired')
                if (evaluate_position_arrival(plan,current,motion_id=result.motion_id) is not None
                        or math.dist(current.tcp_p,result.final_sample.tcp_p)>1e-5
                        or max(abs(a-b) for a,b in zip(current.actual_q,result.final_sample.actual_q))>1e-5):
                    motion_done=False;raise ValueError('position_site_tail_not_stable')
                await sleep(.1)
            motion_done=True;report.update(tail_complete=True,complete=True)
    except BaseException as error:
        report.update(complete=False,reason=_reason(error),error_type=type(error).__name__)
        if not motion_done:await asyncio.shield(stop_once())
    finally:
        if adapter is not None:
            report['remote_unknown'] |= bool(adapter._snapshot_calls.unknown_methods)
            if proxy is None or proxy.movej_entries==0:
                adapter._motion_accepted=adapter._motion_write_stop_required=False
            report['latched_fault']=adapter._latched_fault
            try:await adapter.disconnect()
            except BaseException as error:report['cleanup_errors'].append(type(error).__name__)
            report['local_resources_released']=adapter._client is None and not adapter._pump.running
        if proxy is not None:
            proxy.read_gate_closed=True
            report.update(movej_entries=proxy.movej_entries,stop_entries=proxy.sdk_stop_entries,
                snapshot_groups=copy.deepcopy(proxy.snapshot_groups),
                ik_entries=proxy.math_entries['kinematics_inverse'],fk_entries=proxy.math_entries['kinematics_forward'],
                read_entries=len(proxy.read_calls),read_calls=copy.deepcopy(proxy.read_calls),
                remote_unknown=report['remote_unknown'] or proxy.movej_unknown or proxy.read_remote_result_unknown)
        if report['latched_fault'] or report['remote_unknown']:report['complete']=False
        if started:
            try:
                await recorder.write_critical_event('single_finite_position_result',
                    dict(report,complete=False,result_is_provisional=True,execution_checks_passed_before_close=report['complete']),clock())
                await recorder.close();health();report['recording_closed_verified']=True
            except BaseException as error:report['complete']=False;report['cleanup_errors'].append(type(error).__name__)
    return report
