"""Fixed native SDK matrix against an owned IPv4 listener; no real target option."""
import argparse
import asyncio
import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts.native_jog_loopback_probe import _sdk_identity,owned_port,_same_wire,_unique_object
from scripts.sdk_loopback_stop_probe import LoopbackPeer
from scripts.application_stop_loopback_probe import fixture_result,loopback_settings
from app.commissioning.native_jog_mdh import MdhTool,mdh_tcp_jacobian
from app.commissioning.native_jog_packet import NativeJogLimits
from app.commissioning.native_jog_site_scope import SiteJogScope
from app.commissioning.native_jog_site_run import run_site_window
from app.commissioning.qpc_event_loop import new_qpc_loop
from app.recording.noop import NoopRecorder
from scipy.spatial.transform import Rotation

HOST='127.0.0.1'
Q=(0.,-1.2,1.8,-.6,1.,0.)
TOOL=MdhTool((0.,0.,.175),(0.,0.,0.,1.))
CASES=('normal','preflight_read_delay_20ms','speedl_late_100ms','stop_late_250ms','observation_stuck',
    'feedback_ages_reply_35ms','revoked_during_speedl','late_effect_after_stop')
READS={'get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion','get_kin_factor'}


def wire_pose(p,e=(0.,0.,0.)):
    return dict(position=dict(zip(('x','y','z'),p)),rotation=dict(euler_zyx=dict(zip(('x','y','z'),e))))


def tcp():
    t,_=mdh_tcp_jacobian(Q,TOOL)
    rz,ry,rx=Rotation.from_matrix(t[:3,:3]).as_euler('ZYX')
    return wire_pose(tuple(t[:3,3]),(rx,ry,rz))


def speed_request():
    return dict(speed=wire_pose((0.,0.,.005)),param=dict(acc=.01,time=.08,constrained=True),
        frame=dict(position_kind='CUSTOM',rotation_kind='CUSTOM',**wire_pose((0.,0.,0.))))


def parse_request(raw, *, active_motion_id=503):
    if type(active_motion_id) is not int or active_motion_id not in (7,503):
        raise ValueError('invalid fixed owned motion id')
    r=json.loads(raw,object_pairs_hook=_unique_object)
    if (type(r) is not dict or set(r)!={'jsonrpc','id','method','params'} or r['jsonrpc']!='2.0'
            or type(r['id']) not in (int,str) or type(r['params']) is not list or len(r['params'])!=1):
        raise ValueError('invalid owned peer request')
    method=r['method']
    if method=='speed_linear':expected=speed_request()
    elif method=='get_forward_kin':expected=dict(pose=dict(kind='JOINT',joint=dict(joint=list(Q))))
    elif method=='get_motion_state':expected=dict(id=active_motion_id)
    elif method in READS|{'stop_move'}:expected={}
    else:raise ValueError('forbidden owned peer method')
    if not _same_wire(r['params'][0],expected):raise ValueError('invalid owned peer parameters')
    return r


class Peer(LoopbackPeer):
    def __init__(self,case):
        super().__init__();self.case=case;self.speed_seen=False;self.stopped=False
        self.late_effect_started_ns=None

    async def _late_effect(self):
        await asyncio.sleep(.035)
        self.late_effect_started_ns=time.perf_counter_ns()

    async def handler(self,connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    r=parse_request(raw,active_motion_id=7 if self.late_effect_started_ns is not None else 503)
                    key=(type(r['id']),r['id'])
                    if key in self._ids:raise ValueError('duplicate owned peer id')
                    self._ids.add(key)
                    if r['method']=='speed_linear':
                        if self.speed_seen:raise ValueError('duplicate owned speed')
                        self.speed_seen=True
                    if r['method']=='stop_move':
                        if self.stopped or not self.speed_seen:raise ValueError('invalid owned stop')
                        self.stopped=True
                        if self.case=='late_effect_after_stop':
                            effect=asyncio.create_task(self._late_effect());self._tasks.add(effect)
                            effect.add_done_callback(self._reply_done)
                except ValueError:
                    self.errors.append('owned_protocol_rejected');await connection.close(code=1008);return
                row={**copy.deepcopy(r),'received_ns':time.perf_counter_ns()}
                self.records.append(row)
                task=asyncio.create_task(self.reply(connection,row));self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        finally:self._connections.discard(connection)

    async def reply(self,connection,row):
        method=row['method']
        if (self.case=='preflight_read_delay_20ms' and not self.speed_seen
                and method in READS|{'get_motion_state'}):await asyncio.sleep(.02)
        if method=='speed_linear' and self.case in ('speedl_late_100ms','late_effect_after_stop'):
            await asyncio.sleep(.1)
        if method=='speed_linear' and self.case=='feedback_ages_reply_35ms':await asyncio.sleep(.035)
        if method=='speed_linear' and self.case=='revoked_during_speedl':await asyncio.sleep(.05)
        if method=='stop_move' and self.case=='stop_late_250ms':await asyncio.sleep(.25)
        if method=='get_kin_data' and self.case=='observation_stuck' and self.speed_seen and not self.stopped:
            await asyncio.Event().wait()
        if method=='speed_linear':result=dict(id=7)
        elif method=='get_kin_factor':result=dict(speed_factor=100)
        elif method=='get_forward_kin':result=tcp()
        elif method=='get_motion_state':result=dict(state='RUNNING' if self.late_effect_started_ns is not None else 'FINISHED')
        elif method=='get_running_motion' and self.case=='preflight_read_delay_20ms':result=dict(id=503)
        elif method=='get_running_motion' and self.late_effect_started_ns is not None:result=dict(id=7)
        else:
            result=fixture_result(method,moving=self.late_effect_started_ns is not None)
            if method=='get_kin_data':
                for prefix in ('actual','target'):
                    result[prefix+'_joint_pose']=list(Q);result[prefix+'_tcp_pose']=tcp()
                result['actual_flange_pose']=tcp()
        if method=='get_robot_state':row['synthetic_state']=result['state']
        row['send_started_ns']=time.perf_counter_ns()
        await connection.send(json.dumps(dict(jsonrpc='2.0',id=row['id'],result=result)))
        row['send_completed_ns']=time.perf_counter_ns()


def synthetic_scope():
    t,_=mdh_tcp_jacobian(Q,TOOL);lo=tuple(float(v)-.1 for v in t[:3,3]);hi=tuple(float(v)+.1 for v in t[:3,3])
    s=loopback_settings();s=replace(s,teleop_ready_q=Q,startup_tcp_min_m=lo,startup_tcp_max_m=hi,
        control=replace(s.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    return SiteJogScope(s,NativeJogLimits(.005,.01,lo,hi,(.001,)*3),
        hashlib.sha256(b'fixed-owned-site-jog-synthetic-v1').hexdigest(),TOOL,lambda:None,lambda:None,lambda:None)


def matches_expected_case(result):
    records=result['rpc_records']
    speeds=sum(r['method']=='speed_linear' for r in records)
    stops=sum(r['method']=='stop_move' for r in records)
    if not result['local_connection_closed'] or result['protocol_errors']:return False
    if result['case']=='preflight_read_delay_20ms':
        preflight=result.get('preflight',{})
        return (result['complete'] is False and speeds==stops==0
            and result['speedl_entries']==result['stop_entries']==result['fk_entries']==0
            and result['latched_fault'] is None
            and result.get('primary_rejection')=='native_jog_loopback_preflight_only'
            and preflight.get('ready') is True and 0<=preflight.get('snapshot_age_ms',100)<100)
    if speeds!=1 or stops!=1 or result['speedl_entries']!=1 or result['stop_entries']!=1:return False
    speed=next(r for r in records if r['method']=='speed_linear')
    stop=next(r for r in records if r['method']=='stop_move')
    late_reply=(type(speed.get('send_completed_ns')) is int and type(stop.get('send_completed_ns')) is int
        and stop['send_completed_ns']<speed['send_completed_ns'])
    known_speed=(result.get('speedl_call',{}).get('outcome')=='returned'
        and result.get('speedl_remote_result_unknown') is False)
    failed=(result['complete'] is False and result['latched_fault']=='stop_unverified')
    if result['case']=='normal':return result['complete'] is True and result['stop_confirmed'] is True and known_speed
    if result['case']=='feedback_ages_reply_35ms':
        fks=[r for r in result.get('native_reads',[]) if r['method']=='kinematics_forward' and r['outcome']=='returned']
        if len(fks)!=1 or not known_speed:return False
        completed=fks[0]['completed_ns']
        return (result['complete'] is True and result['stop_confirmed'] is True
            and 0<=result['speedl_started_ns']-completed<100_000_000
            and result['speedl_call']['native_completed_ns']-completed>=100_000_000)
    if result['case']=='revoked_during_speedl':
        return (failed and result.get('authority_rejection')=='native_jog_loopback_revoked'
            and result.get('speedl_remote_result_unknown') is True
            and result.get('speedl_call',{}).get('outcome')=='unknown' and late_reply)
    if result['case']=='late_effect_after_stop':
        effect=result.get('late_effect_started_ns')
        return (failed and result.get('primary_rejection')=='stop_incomplete'
            and result.get('original_stop_confirmed') is False
            and result.get('speedl_remote_result_unknown') is True
            and result.get('speedl_call',{}).get('outcome')=='unknown' and late_reply
            and type(effect) is int and stop['send_completed_ns']<effect
            and any(r['method']=='get_robot_state' and r.get('synthetic_state')=='MOVING'
                and r['received_ns']>=effect for r in records))
    if result['case']=='stop_late_250ms':
        return failed and known_speed and result.get('primary_rejection')=='sdk_timeout:stop_move'
    if result['case']=='speedl_late_100ms':return failed and result.get('speedl_remote_result_unknown') is True
    if result['case']=='observation_stuck':
        return failed and known_speed and result.get('motion_observation',{}).get('remote_result_unknown') is True
    return False


async def one_case(sdk,case):
    from websockets.asyncio.server import serve
    peer=Peer(case);client=None
    async with serve(peer.handler,HOST,0) as server:
        port=owned_port([s.getsockname() for s in server.sockets])
        async def factory(ip):
            nonlocal client
            if ip!=HOST:raise ValueError('owned loopback only')
            client=await sdk.connect(HOST,port,{'request_timeout':2})
            return client
        try:
            report={}
            scope=synthetic_scope()
            if case=='preflight_read_delay_20ms':
                # This case measures preflight only. Deliberate revocation
                # before FK makes zero writes an explicit requirement, rather
                # than treating a later stale motion rejection as a success.
                def preflight_only():
                    if report.get('preflight',{}).get('ready'):
                        raise ValueError('native_jog_loopback_preflight_only')
                scope=replace(scope,authorize=preflight_only)
            elif case=='feedback_ages_reply_35ms':
                aged=False
                def age_feedback():
                    nonlocal aged
                    if not aged and any(r['method']=='get_forward_kin' for r in peer.records):
                        aged=True;time.sleep(.065)
                scope=replace(scope,hot_check=age_feedback,read_check=lambda:None)
            elif case=='revoked_during_speedl':
                def revoke():
                    if peer.speed_seen:raise ValueError('native_jog_loopback_revoked')
                scope=replace(scope,read_check=revoke)
            result=await run_site_window(scope,NoopRecorder(),factory,clock=time.perf_counter_ns,report=report)
        finally:
            await peer.close()
            if client is not None:await asyncio.wait_for(client.wait_disconnect(),2)
        closed=client is not None and not await client.is_connected()
        result.update(case=case,rpc_records=copy.deepcopy(peer.records),local_connection_closed=closed,
            listener=dict(host=HOST,port=port),protocol_errors=list(peer.errors),
            late_effect_started_ns=peer.late_effect_started_ns)
        result['expected_outcome_matched']=matches_expected_case(result)
        return result


async def matrix():
    if getattr(asyncio.get_running_loop(),'vr4arm_timing_clock',None)!='time.perf_counter_ns':
        raise ValueError('owned_qpc_required')
    sdk,identity=_sdk_identity();sdk.init()
    results=[]
    for case in CASES:results.append(await one_case(sdk,case))
    return dict(offline_only=True,real_robot_connections=0,hardware_verified=False,
        complete=all(case['expected_outcome_matched'] for case in results),
        runtime='SIMULATION',sdk=identity,cases=results,synthetic_q_rad=Q)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args(argv)
    try:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with a.output.open('x',encoding='utf-8') as stream:
            try:
                with asyncio.Runner(loop_factory=new_qpc_loop) as runner:r=runner.run(matrix())
            except BaseException:
                json.dump(dict(complete=False,offline_only=True,reason='owned_site_matrix_failed'),stream)
                raise
            json.dump(r,stream,indent=2,allow_nan=False)
        print(json.dumps(dict(output=str(a.output),cases=len(r['cases']),
            complete=r['complete'],real_robot_connections=0)))
        return 0 if r['complete'] else 2
    except (OSError,ValueError,RuntimeError) as error:parser.error(str(error))


if __name__=='__main__':raise SystemExit(main())
