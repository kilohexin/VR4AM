"""Exercise the scoped runner through the pinned SDK and an owned localhost peer."""
import asyncio
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts.finite_position_site_loopback import MODEL,Q,wire
from scripts.sdk_loopback_stop_probe import LoopbackPeer,owned_port
from scripts.native_jog_loopback_probe import _sdk_identity,_unique_object
from scripts.application_stop_loopback_probe import fixture_result,loopback_settings
from scripts import real_robot_blended_trial as entry


class Peer(LoopbackPeer):
    def __init__(self,case):
        super().__init__();self.q=Q;self.case=case;self.moves=0;self.stops=0

    def inverse(self,cart):
        target=np.array([cart['position'][k] for k in ('x','y','z')])
        r=cart['rotation']['euler_zyx'];rot=Rotation.from_euler('ZYX',[r[k] for k in ('z','y','x')])
        origin,_=MODEL.forward(Q)
        if np.linalg.norm(target-origin)>.008001:raise ValueError('synthetic_target_range')
        joints=np.array(self.q)
        for _ in range(12):
            p,q=MODEL.forward(joints)
            error=np.r_[target-p,(rot*Rotation.from_quat(q).inv()).as_rotvec()]
            if np.linalg.norm(error)<1e-10:return tuple(joints)
            joints+=np.linalg.solve(MODEL.jacobian(joints),error)
        raise ValueError('synthetic_ik_failed')

    async def handler(self,connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                row=json.loads(raw,object_pairs_hook=_unique_object)
                method=row.get('method');params=row.get('params')
                if (row.get('jsonrpc')!='2.0' or type(params) is not list or len(params)!=1
                        or row['id'] in self._ids or len(self.records)>=4096):
                    raise ValueError('loopback_envelope')
                self._ids.add(row['id']);self.records.append(row)
                try:result=await self.reply(method,params[0])
                except Exception as error:
                    self.errors.append(str(error));await connection.close(code=1008);return
                await connection.send(json.dumps(dict(jsonrpc='2.0',id=row['id'],result=result)))
        finally:self._connections.discard(connection)

    async def reply(self,method,p):
        if method=='get_inverse_kin':return dict(joint=self.inverse(p['pose']['cart']))
        if method=='get_forward_kin':return wire(p['pose']['joint']['joint'])
        if method=='get_kin_factor':return dict(speed_factor=100)
        if method=='move_linear':
            self.moves+=1
            if self.moves>2:raise ValueError('synthetic_move_budget')
            v=p['param']
            if not 0<v['velocity']<=.02 or not 0<v['acc']<=.1 or v['time']!=0 or v['radius']!=.001:
                raise ValueError('synthetic_motion_parameters')
            q=self.inverse(p['pose']['cart'])
            if self.case!='no_progress':self.q=q
            if self.case=='late_reply':await asyncio.sleep(.25)
            return dict(id=10+self.moves)
        if method=='get_motion_state':return dict(state='FINISHED')
        if method=='stop_move':
            self.stops+=1
            if self.stops>1:raise ValueError('synthetic_stop_budget')
        result=fixture_result(method)
        if method=='get_kin_data':
            for side in ('actual','target'):
                result[side+'_joint_pose']=list(self.q);result[side+'_tcp_pose']=wire(self.q)
            result['actual_flange_pose']=wire(self.q)
        return result


async def exercise(case,output):
    from websockets.asyncio.server import serve
    sdk,identity=_sdk_identity();sdk.init();peer=Peer(case)
    async with serve(peer.handler,'127.0.0.1',0) as server:
        port=owned_port([s.getsockname() for s in server.sockets]);connected=False
        async def factory(ip):
            nonlocal connected
            if ip!='192.0.2.1' or connected:raise ValueError('owned_loopback_only')
            connected=True
            return await sdk.connect('127.0.0.1',port,{'request_timeout':2})
        p,_=MODEL.forward(Q);s=loopback_settings()
        s=replace(s,ip='192.0.2.1',teleop_ready_q=Q,home_q=Q,
            startup_tcp_min_m=tuple(x-.01 for x in p),startup_tcp_max_m=tuple(x+.01 for x in p),
            control=replace(s.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
        params=dict(settings=s,model=MODEL,output=output,authorization_expires_utc=None,input_hashes={},runtime_hashes={})
        try:result=await entry.run(params,SimpleNamespace(cold=lambda:None,read=lambda:None,write=lambda:None),client_factory=factory)
        finally:await peer.close()
    passed=(result['complete'] and peer.moves==2 and peer.stops==0) if case=='normal' else (
        not result['complete'] and peer.moves>=1 and peer.stops==1)
    result.update(case=case,loopback_passed=passed and not peer.errors,sdk_identity=identity,
        peer_errors=peer.errors,wire_methods=[r['method'] for r in peer.records])
    (output/'loopback.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:result.get(k) for k in ('case','loopback_passed','complete','error','peer_errors','counts')}))
    return result['loopback_passed']


def parser():
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=('normal','no_progress','late_reply'),default='normal')
    p.add_argument('--output',type=Path,required=True);return p


if __name__=='__main__':
    from app.commissioning.qpc_event_loop import new_qpc_loop
    args=parser().parse_args()
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:ok=runner.run(exercise(args.case,args.output))
    raise SystemExit(0 if ok else 1)
