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
CASES=('normal','preflight_read_delay_20ms','speedl_late_100ms','stop_late_250ms','observation_stuck')
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


def parse_request(raw):
    r=json.loads(raw,object_pairs_hook=_unique_object)
    if (type(r) is not dict or set(r)!={'jsonrpc','id','method','params'} or r['jsonrpc']!='2.0'
            or type(r['id']) not in (int,str) or type(r['params']) is not list or len(r['params'])!=1):
        raise ValueError('invalid owned peer request')
    method=r['method']
    if method=='speed_linear':expected=speed_request()
    elif method=='get_forward_kin':expected=dict(pose=dict(kind='JOINT',joint=dict(joint=list(Q))))
    elif method=='get_motion_state':expected=dict(id=503)
    elif method in READS|{'stop_move'}:expected={}
    else:raise ValueError('forbidden owned peer method')
    if not _same_wire(r['params'][0],expected):raise ValueError('invalid owned peer parameters')
    return r


class Peer(LoopbackPeer):
    def __init__(self,case):
        super().__init__();self.case=case;self.speed_seen=False;self.stopped=False

    async def handler(self,connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    r=parse_request(raw);key=(type(r['id']),r['id'])
                    if key in self._ids:raise ValueError('duplicate owned peer id')
                    self._ids.add(key)
                    if r['method']=='speed_linear':
                        if self.speed_seen:raise ValueError('duplicate owned speed')
                        self.speed_seen=True
                    if r['method']=='stop_move':
                        if self.stopped or not self.speed_seen:raise ValueError('invalid owned stop')
                        self.stopped=True
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
        if method=='speed_linear' and self.case=='speedl_late_100ms':await asyncio.sleep(.1)
        if method=='stop_move' and self.case=='stop_late_250ms':await asyncio.sleep(.25)
        if method=='get_kin_data' and self.case=='observation_stuck' and self.speed_seen and not self.stopped:
            await asyncio.Event().wait()
        if method=='speed_linear':result=dict(id=7)
        elif method=='get_kin_factor':result=dict(speed_factor=100)
        elif method=='get_forward_kin':result=tcp()
        elif method=='get_motion_state':result=dict(state='FINISHED')
        elif method=='get_running_motion' and self.case=='preflight_read_delay_20ms':result=dict(id=503)
        else:
            result=fixture_result(method)
            if method=='get_kin_data':
                for prefix in ('actual','target'):
                    result[prefix+'_joint_pose']=list(Q);result[prefix+'_tcp_pose']=tcp()
                result['actual_flange_pose']=tcp()
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
    if result['case']=='normal':return result['complete'] is True and result['stop_confirmed'] is True
    return result['case'] in CASES and result['complete'] is False and result['latched_fault']=='stop_unverified'


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
            result=await run_site_window(scope,NoopRecorder(),factory,clock=time.perf_counter_ns,report=report)
        finally:
            await peer.close()
            if client is not None:await asyncio.wait_for(client.wait_disconnect(),2)
        closed=client is not None and not await client.is_connected()
        result.update(case=case,rpc_records=copy.deepcopy(peer.records),local_connection_closed=closed,
            listener=dict(host=HOST,port=port),protocol_errors=list(peer.errors))
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
