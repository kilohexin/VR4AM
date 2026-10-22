"""Native SDK / single-position executor on a self-owned localhost peer only."""
import argparse
import asyncio
import copy
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts.finite_position_loopback import Peer as BasePeer,parse_request as base_parse,Q,_sdk_identity,owned_port
from scripts.application_stop_loopback_probe import fixture_result,loopback_settings
from app.commissioning.finite_position_site import MdhPositionModel,run_single_position,DISTANCE_M
from app.commissioning.qpc_event_loop import new_qpc_loop
from app.recording.noop import NoopRecorder

CASES=('normal','read_latency','no_actual_progress','move_reply_late','stop_reply_late','late_effect')
MODEL=MdhPositionModel.from_settings(loopback_settings())


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=(*CASES,'all'),default='all')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--arrival-policy',choices=('strict','coarse_grasp'),default='strict')
    return p


def wire(q):
    p,r=MODEL.forward(q);rz,ry,rx=Rotation.from_quat(r).as_euler('ZYX')
    return dict(position=dict(zip(('x','y','z'),p)),rotation=dict(euler_zyx=dict(x=float(rx),y=float(ry),z=float(rz))))


def parse_request(raw):
    from scripts.native_jog_loopback_probe import _unique_object
    row=json.loads(raw,object_pairs_hook=_unique_object);normalized=copy.deepcopy(row)
    if row.get('method')=='get_kin_factor':normalized['method']='get_robot_state'
    elif row.get('method')=='get_inverse_kin':
        pose=row['params'][0]['pose'];cart=pose['cart'];p=cart['position'];r=cart['rotation']['euler_zyx']
        if (set(pose)!={'kind','cart'} or set(cart)!={'position','rotation'}
                or set(cart['rotation'])!={'euler_zyx'} or set(p)!={'x','y','z'} or set(r)!={'x','y','z'}
                or any(type(v) not in (int,float) or not np.isfinite(v) for v in (*p.values(),*r.values()))):
            raise ValueError('invalid owned inverse pose')
        initial=wire(Q)
        if (max(abs(p[k]-initial['position'][k]-(DISTANCE_M if k=='z' else 0.)) for k in p)>1e-8
                or max(abs(r[k]-initial['rotation']['euler_zyx'][k]) for k in r)>1e-8):
            raise ValueError('outside fixed synthetic +Z target')
        normalized['params'][0]['pose']['cart']['position']=dict(x=.3,y=0.,z=.4)
    base_parse(json.dumps(normalized))
    return row


class Peer(BasePeer):
    async def handler(self,connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    row=parse_request(raw);key=(type(row['id']),row['id'])
                    if key in self._ids or len(self.records)>=1024:raise ValueError('duplicate or excess')
                    if row['method']=='move_joint':
                        self.moves+=1
                        if self.moves>1:raise ValueError('second move')
                    if row['method']=='stop_move':
                        self.stops+=1
                        if self.stops>1 or self.moves!=1:raise ValueError('unreserved stop')
                    self._ids.add(key)
                except Exception:
                    self.errors.append('protocol_rejected');await connection.close(code=1008);return
                row['received_ns']=time.perf_counter_ns();self.records.append(row)
                task=asyncio.create_task(self.reply(connection,row));self._tasks.add(task);task.add_done_callback(self._reply_done)
        finally:self._connections.discard(connection)

    def advance(self):
        old=self.case
        if old=='stop_reply_late':self.case='no_actual_progress'
        try:super().advance()
        finally:self.case=old

    async def reply(self,connection,row):
        m=row['method'];p=row['params'][0]
        if self.case=='read_latency' and m in ('get_robot_state','get_estop_reason','get_kin_data',
                'get_tcp','get_claw','get_running_motion','get_motion_state'):
            await asyncio.sleep(.015)
        self.advance()
        if m=='move_joint':
            self.start=self.q;self.goal=tuple(p['pose']['joint']['joint'])
            if self.case!='late_effect':self.motion_start=time.perf_counter();self.active=True
            if self.case in ('move_reply_late','late_effect'):await asyncio.sleep(.25)
            if self.case=='late_effect':self.motion_start=time.perf_counter();self.active=True
            result=dict(id=7)
        elif m=='stop_move':
            self.advance();self.active=False;self.motion_start=None;self.stopped=True
            if self.case=='stop_reply_late':await asyncio.sleep(.25)
            result={}
        elif m=='get_inverse_kin':
            cart=p['pose']['cart'];target=np.array([cart['position'][k] for k in ('x','y','z')]);target_q=Rotation.from_euler('ZYX',[cart['rotation']['euler_zyx'][k] for k in ('z','y','x')])
            joints=np.array(self.q)
            for _ in range(10):
                pos,rot=MODEL.forward(joints);error=np.r_[target-pos,(target_q*Rotation.from_quat(rot).inv()).as_rotvec()]
                if np.linalg.norm(error)<1e-10:break
                joints+=np.linalg.solve(MODEL.jacobian(joints),error)
            result=dict(joint=[float(x) for x in joints])
        elif m=='get_forward_kin':result=wire(p['pose']['joint']['joint'])
        elif m=='get_kin_factor':result=dict(speed_factor=100)
        elif m=='get_motion_state':result=dict(state='RUNNING' if self.active else 'FINISHED')
        else:
            result=fixture_result(m,moving=self.active)
            if m=='get_running_motion':result=dict(id=7 if self.active else 0)
            if m=='get_kin_data':
                for side,q in (('actual',self.q),('target',self.goal)):
                    result[side+'_joint_pose']=list(q);result[side+'_tcp_pose']=wire(q)
                result['actual_flange_pose']=wire(self.q)
        row['reply_started_ns']=time.perf_counter_ns()
        await connection.send(json.dumps(dict(jsonrpc='2.0',id=row['id'],result=result)))


class Recorder(NoopRecorder):
    fatal_error=None
    dropped_normal_events=0
    def __init__(self):self.events=[]
    async def write_event(self,event,stamp):self.events.append(dict(event,server_mono_ns=stamp))
    async def write_critical_event(self,kind,payload,stamp):await self.write_event(dict(payload,kind=kind),stamp)


async def run_case(case,output,*,arrival_policy='strict'):
    from websockets.asyncio.server import serve
    sdk,identity=_sdk_identity();sdk.init();peer=Peer(case);client=None;recorder=Recorder()
    async with serve(peer.handler,'127.0.0.1',0) as server:
        port=owned_port([s.getsockname() for s in server.sockets])
        async def factory(ip):
            nonlocal client
            if ip!='127.0.0.1' or client is not None:raise ValueError('self-owned listener only')
            client=await asyncio.wait_for(sdk.connect('127.0.0.1',port,{'request_timeout':2}),3);return client
        try:
            p,_=MODEL.forward(Q);settings=replace(loopback_settings(),ip='127.0.0.1',home_q=Q,teleop_ready_q=Q,
                startup_tcp_min_m=tuple(x-.01 for x in p),startup_tcp_max_m=tuple(x+.01 for x in p),
                control=replace(loopback_settings().control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
            report=await run_single_position(settings,recorder,factory,lambda:None,config_sha256='a'*64,model=MODEL,
                arrival_policy=arrival_policy)
            if peer._tasks:await asyncio.wait(tuple(peer._tasks),timeout=.6)
            expected=(report['complete'] and peer.moves==1 and peer.stops==0) if case in ('normal','read_latency') else (not report['complete'] and peer.moves==1 and peer.stops==1)
            if case=='read_latency':
                delayed=[r for r in peer.records if r['method'] in ('get_robot_state','get_estop_reason','get_kin_data',
                    'get_tcp','get_claw','get_running_motion','get_motion_state')]
                expected &= bool(delayed) and all(r.get('reply_started_ns',0)-r['received_ns']>=15_000_000 for r in delayed)
            if case in ('move_reply_late','stop_reply_late','late_effect'):expected &= report['latched_fault']=='stop_unverified'
            result=dict(case=case,expected_outcome_observed=bool(expected and not peer.errors),
                real_robot_connections=0,offline_only=True,hardware_verified=False,sdk=identity,
                listener=dict(host='127.0.0.1',port=port),report=report,protocol_errors=peer.errors,
                rpc_records=peer.records,events=recorder.events)
            output=Path(output);output.mkdir(parents=True,exist_ok=True)
            with (output/(case+'.json')).open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
            return result
        finally:
            await peer.close()
            if client is not None:await asyncio.wait_for(client.wait_disconnect(),2)


async def matrix(args):
    results=[]
    for case in CASES if args.case=='all' else (args.case,):
        r=await run_case(case,args.output,arrival_policy=args.arrival_policy);results.append(r)
        print(json.dumps(dict(case=case,expected_outcome_observed=r['expected_outcome_observed'],
            complete=r['report']['complete'],move_entries=r['report']['movej_entries'],stop_entries=r['report']['stop_entries'],
            reason=r['report'].get('reason'),latched_fault=r['report']['latched_fault'])),flush=True)
    return all(r['expected_outcome_observed'] for r in results)


def main():
    args=parser().parse_args()
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:ok=runner.run(matrix(args))
    raise SystemExit(0 if ok else 2)


if __name__=='__main__':main()
