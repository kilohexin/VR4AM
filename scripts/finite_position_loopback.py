"""Finite-position SDK matrix on a listener owned by this process only."""
import argparse
import asyncio
import copy
from dataclasses import asdict,replace
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts.native_jog_loopback_probe import _sdk_identity,_unique_object
from scripts.sdk_loopback_stop_probe import LoopbackPeer,owned_port
from scripts.application_stop_loopback_probe import fixture_result,loopback_settings,_wire_pose
from app.robots.finite_position_adapter import FinitePositionAdapter
from app.control.finite_position_session import FinitePositionSession
from app.control.finite_position_types import PositionIntent
from app.robots.base import StopReason
from app.commissioning.qpc_event_loop import new_qpc_loop

CASES=('normal','release_before_entry','release_during_step','reverse_latest','no_actual_progress',
    'foreign_task','move_reply_late','stop_reply_late','late_effect')
READS={'get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion'}
Q=(0.,-1.,1.,0.,1.57,0.)


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=(*CASES,'all'),default='all')
    p.add_argument('--output',type=Path,required=True)
    return p


def six(q):
    if (type(q) is not list or len(q)!=6 or any(type(x) not in (float,int) or not np.isfinite(x) for x in q)
            or max(abs(a-b) for a,b in zip(q,Q))>.01):raise ValueError('invalid synthetic joints')
    return q


def parse_request(raw):
    r=json.loads(raw,object_pairs_hook=_unique_object)
    if (type(r) is not dict or set(r)!={'jsonrpc','id','method','params'} or r['jsonrpc']!='2.0'
            or type(r['id']) not in (int,str) or type(r['params']) is not list or len(r['params'])!=1
            or type(r['params'][0]) is not dict):raise ValueError('invalid envelope')
    method=r['method'];p=r['params'][0]
    if method in READS|{'stop_move'}:
        if p!={}:raise ValueError('invalid read')
    elif method=='get_motion_state':
        if set(p)!={'id'} or type(p['id']) is not int or not 7<=p['id']<=999:raise ValueError('invalid task')
    elif method in ('get_forward_kin','move_joint'):
        if set(p)!=({'pose'} if method=='get_forward_kin' else {'pose','param'}):raise ValueError('invalid pose envelope')
        pose=p['pose']
        if type(pose) is not dict or set(pose)!={'kind','joint'} or pose['kind']!='JOINT':raise ValueError('invalid joint pose')
        if type(pose['joint']) is not dict or set(pose['joint'])!={'joint'}:raise ValueError('invalid joint payload')
        six(pose['joint']['joint'])
        if method=='move_joint':
            v=p['param']
            if (type(v) is not dict or set(v)!={'velocity','acc','time','radius'}
                    or any(type(x) not in (int,float) or not np.isfinite(x) for x in v.values())
                    or v['time']!=0. or v['radius']!=0. or not 0<v['velocity']<=.05 or not 0<v['acc']<=.2):
                raise ValueError('invalid finite move')
    elif method=='get_inverse_kin':
        if set(p)!={'pose','refer'} or p['pose'].get('kind')!='CARTESIAN' or set(p['refer'])!={'joint'}:
            raise ValueError('invalid inverse')
        six(p['refer']['joint']);position=p['pose']['cart']['position']
        if (set(position)!={'x','y','z'} or any(type(x) not in (int,float) or not np.isfinite(x) for x in position.values())
                or abs(position['x']-.3)>.01 or abs(position['y'])>.01 or abs(position['z']-.4)>.01):
            raise ValueError('invalid inverse target')
    else:raise ValueError('forbidden method')
    return r


class SyntheticModel:
    model_sha256='c'*64
    factory_calibrated=False
    def forward(self,q):return ((.3+q[0],1.+q[1],q[2]-.6),(0.,0.,0.,1.))
    def jacobian(self,q):return np.diag((1.,1.,1.,0.,0.,0.))
    def jdot(self,q,qd):return np.zeros((6,6))


class Peer(LoopbackPeer):
    def __init__(self,case):
        super().__init__();self.case=case;self.q=Q;self.goal=Q;self.start=Q
        self.motion_start=None;self.moves=0;self.stops=0;self.stopped=False;self.active=False
        self.model=SyntheticModel()

    def advance(self):
        if self.motion_start is None:return
        u=min(1.,max(0.,(time.perf_counter()-self.motion_start)/.15))
        self.active=u<1
        if self.case!='no_actual_progress':self.q=tuple(a+(b-a)*u for a,b in zip(self.start,self.goal))

    async def handler(self,connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    r=parse_request(raw);key=(type(r['id']),r['id'])
                    if key in self._ids or len(self.records)>=1000:raise ValueError('duplicate or excess read')
                    if r['method']=='move_joint':
                        self.moves+=1
                        if self.moves>2:raise ValueError('too many moves')
                    if r['method']=='stop_move':
                        self.stops+=1
                        if self.stops>1:raise ValueError('duplicate stop')
                    self._ids.add(key)
                except Exception:
                    self.errors.append('protocol_rejected');await connection.close(code=1008);return
                row={**r,'received_ns':time.perf_counter_ns()};self.records.append(row)
                task=asyncio.create_task(self.reply(connection,row));self._tasks.add(task);task.add_done_callback(self._reply_done)
        finally:self._connections.discard(connection)

    async def reply(self,connection,r):
        self.advance();m=r['method'];p=r['params'][0]
        if m=='move_joint':
            self.start=self.q;self.goal=tuple(p['pose']['joint']['joint'])
            if self.case!='late_effect':self.motion_start=time.perf_counter();self.active=True
            if self.case in ('move_reply_late','late_effect'):await asyncio.sleep(.25)
            if self.case=='late_effect':self.motion_start=time.perf_counter();self.active=True
            result=dict(id=self.moves+6)
        elif m=='stop_move':
            self.advance();self.active=False;self.motion_start=None;self.stopped=True
            if self.case=='stop_reply_late':await asyncio.sleep(.25)
            result={}
        elif m=='get_inverse_kin':
            xyz=p['pose']['cart']['position'];result=dict(joint=[xyz['x']-.3,xyz['y']-1,xyz['z']+.6,*self.q[3:]])
        elif m=='get_forward_kin':
            pos,_=self.model.forward(p['pose']['joint']['joint']);result=_wire_pose(*pos)
        elif m=='get_motion_state':result=dict(state='RUNNING' if self.active else 'FINISHED')
        else:
            result=fixture_result(m,moving=self.active)
            if m=='get_running_motion':result=dict(id=(999 if self.case=='foreign_task' else self.moves+6) if self.active else 0)
            if m=='get_kin_data':
                for side,q in (('actual',self.q),('target',self.goal)):
                    result[side+'_joint_pose']=list(q);pos,_=self.model.forward(q);result[side+'_tcp_pose']=_wire_pose(*pos)
                result['actual_flange_pose']=result['actual_tcp_pose']
        await connection.send(json.dumps(dict(jsonrpc='2.0',id=r['id'],result=result)))


class Recorder:
    fatal_error=None
    dropped_normal_events=0
    def __init__(self):self.events=[]
    async def write_event(self,event,stamp):self.events.append(dict(event,stamp=stamp))


async def run_case(case,output):
    if case not in CASES:raise ValueError('unknown case')
    from websockets.asyncio.server import serve
    sdk,identity=_sdk_identity();sdk.init();peer=Peer(case);adapter=None;client=None;session=None
    async with serve(peer.handler,'127.0.0.1',0) as server:
        port=owned_port([s.getsockname() for s in server.sockets])
        async def factory(ip):
            nonlocal client
            if ip!='digital-twin' or client is not None:raise ValueError('owned listener only')
            client=await asyncio.wait_for(sdk.connect('127.0.0.1',port,{'request_timeout':2}),3)
            return client
        try:
            settings=replace(loopback_settings(),ip='digital-twin')
            recorder=Recorder()
            adapter=FinitePositionAdapter(settings,client_factory=factory,backend_label='LEBAI_FAKE',
                position_model=SyntheticModel(),clock=time.perf_counter_ns,event_callback=recorder.write_event)
            await adapter.connect();binding=adapter.bind_position_generation(1)
            def authorize():
                if case=='stop_reply_late' and peer.motion_start is not None and time.perf_counter()-peer.motion_start>.04:
                    raise ValueError('position_synthetic_abort')
            session=FinitePositionSession(adapter,binding,clock=time.perf_counter_ns,recorder=recorder,authorize=authorize)
            started=time.perf_counter();seq=0;released=False
            while time.perf_counter()-started<4:
                if session.view().phase=='fault':break
                if not released:
                    if case=='stop_reply_late' and peer.motion_start is not None and time.perf_counter()-peer.motion_start>.04:
                        await session.abort(StopReason.FAULT);break
                    vector=(-1.,0.,0.) if case=='reverse_latest' and peer.moves>=1 else (1.,0.,0.)
                    seq+=1;session.offer(PositionIntent('owned-loopback',1,seq,time.perf_counter_ns(),'held',vector))
                    if (case=='release_before_entry' or case=='release_during_step' and peer.moves>=1
                            or case in ('normal','reverse_latest') and peer.moves>=2):
                        seq+=1;session.offer(PositionIntent('owned-loopback',1,seq,time.perf_counter_ns(),'release',(0.,)*3));released=True
                await session.advance()
                if session.view().phase=='fault' or released and not session.busy:break
                await asyncio.sleep(.01)
            else:raise RuntimeError('matrix case did not terminate')
            # Observe delayed effects before sealing. Never retry a failed command.
            if peer._tasks:await asyncio.wait(tuple(peer._tasks),timeout=.6)
            result=dict(case=case,offline_only=True,hardware_verified=False,real_robot_connections=0,
                listener=dict(host='127.0.0.1',port=port),sdk=identity,view=asdict(session.view()),
                move_entries=peer.moves,stop_entries=peer.stops,latched_fault=adapter._latched_fault,
                remote_unknown=bool(adapter._motion_calls.unknown_methods),events=recorder.events,
                rpc_records=copy.deepcopy(peer.records),protocol_errors=list(peer.errors))
            if case=='release_before_entry':ok=peer.moves==peer.stops==0
            elif case in ('normal','reverse_latest'):ok=peer.moves==2 and peer.stops==0 and session.view().phase=='ready'
            elif case=='release_during_step':ok=peer.moves==1 and peer.stops==0 and session.view().phase=='ready'
            else:ok=peer.moves==1 and peer.stops==1 and session.view().phase=='fault'
            if case in ('move_reply_late','late_effect','stop_reply_late'):
                ok &= adapter._latched_fault=='stop_unverified'
            result['expected_outcome_observed']=bool(ok and not peer.errors)
            path=Path(output);path.mkdir(parents=True,exist_ok=True)
            with (path/(case+'.json')).open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
            return result
        finally:
            if session is not None:await session.abort(StopReason.SHUTDOWN)
            if adapter is not None:
                try:await adapter.disconnect()
                except Exception:pass
            await peer.close()
            if client is not None:await asyncio.wait_for(client.wait_disconnect(),2)


async def matrix(args):
    cases=CASES if args.case=='all' else (args.case,);results=[]
    for case in cases:
        r=await run_case(case,args.output);results.append(r)
        print(json.dumps({k:r[k] for k in ('case','move_entries','stop_entries','latched_fault','expected_outcome_observed')}),flush=True)
    return all(r['expected_outcome_observed'] for r in results)


def main():
    args=parser().parse_args()
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:ok=runner.run(matrix(args))
    raise SystemExit(0 if ok else 2)


if __name__=='__main__':main()
