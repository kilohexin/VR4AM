"""Bounded speedl semantics experiment against the admitted local Docker only.

Not a production control mode or evidence of hardware braking. Default: preview.
"""
import argparse
import asyncio
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))


async def probe(client,reverse=False,blend=False,bounded=False):
    from app.robots.lebai_motion_calls import OwnedMotionCalls
    owner=OwnedMotionCalls();stop_owner=OwnedMotionCalls()
    blend=blend or bounded
    report=dict(case='bounded_blend' if bounded else 'blend' if blend else 'reverse' if reverse else 'steady',commands=[],samples=[],unknown=[],stop_sent=False)
    origin=time.perf_counter();entered=False
    async def sample():
        state,kin=await asyncio.wait_for(asyncio.gather(client.get_robot_state(),client.get_kin_data()),1.)
        qd=kin['actual_joint_speed'];pose=kin['actual_tcp_pose']
        if len(qd)!=6 or not all(math.isfinite(x) for x in qd):raise ValueError('invalid_feedback')
        row=dict(t=time.perf_counter()-origin,state=state,qd=qd,pose=pose)
        report['samples'].append(row)
        return row
    try:
        initial=await sample()
        if initial['state']!='IDLE' or max(map(abs,initial['qd']))>.002:raise ValueError('requires_idle')
        start=initial['pose'];frame=dict.fromkeys(('x','y','z','rx','ry','rz'),0.)
        pending=[]
        for i in range(6 if bounded else 4 if blend else 8):
            if bounded and len(pending)==2:
                deadline=time.perf_counter()+2.
                while True:
                    state=await asyncio.wait_for(client.get_motion_state(pending[0]),.5)
                    if state=='FINISHED':pending.pop(0);break
                    if state!='RUNNING' or time.perf_counter()>deadline:raise ValueError('queue_did_not_advance')
                    await sample();await asyncio.sleep(.02)
            row=await sample()
            if math.dist([row['pose'][k] for k in ('x','y','z')],[start[k] for k in ('x','y','z')])>.025:
                raise ValueError('probe_displacement_limit')
            velocity={**frame,'z':-.01 if reverse and i>=4 else .01}
            stamp=time.perf_counter();entered=True
            if blend:
                target={**start,'z':start['z']+.004*(i+1)}
                operation=lambda:client.movel(target,.1,.02,0.,.001 if bounded or i<3 else 0.)
            else:operation=lambda:client.speedl(.1,velocity,.2,frame)
            ident=await owner.call('movel' if blend else 'speedl',operation,.5,revoked=lambda:False)
            if type(ident) is not int or ident<=0:raise ValueError('invalid_motion_id')
            pending.append(ident)
            report['commands'].append(dict(t=stamp-origin,reply_t=time.perf_counter()-origin,id=ident,
                **(dict(target=target,r=.001 if bounded or i<3 else 0.) if blend else dict(z=velocity['z']))))
            await asyncio.sleep(max(0,.08-(time.perf_counter()-stamp)))
        report['release_t']=time.perf_counter()-origin
        # No new command after release. Measure whether finite duration drains.
        deadline=time.perf_counter()+3.;stable=0
        while time.perf_counter()<deadline:
            row=await sample()
            stable=stable+1 if row['state']=='IDLE' and max(map(abs,row['qd']))<=.002 else 0
            if stable>=3:
                report['natural_finish_t']=row['t'];break
            await asyncio.sleep(.02)
        else:raise ValueError('natural_finish_timeout')
        report['passed']=True
    except Exception as error:
        report['passed']=False;report['error']=type(error).__name__+':'+str(error)
        if entered:
            report['stop_sent']=True
            try:
                await stop_owner.call('stop_move',client.stop_move,1.,revoked=lambda:False)
                row=await sample();report['stop_observation']=row
            except Exception as stop_error:report['stop_error']=type(stop_error).__name__
    finally:
        report['unknown']=sorted(owner.unknown_methods|stop_owner.unknown_methods)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--reverse',action='store_true')
    parser.add_argument('--blend',action='store_true')
    parser.add_argument('--bounded-blend',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from app.digital_twin.docker_runtime import verify_local_docker
    admission=verify_local_docker()
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admission,speedl_max=0 if args.blend or args.bounded_blend else 8,
            movel_max=6 if args.bounded_blend else 4 if args.blend else 0,duration_s=.2,period_s=.08,speed_mps=.02 if args.blend or args.bounded_blend else .01)))
        return 0
    from app.robots.lebai_sdk_bridge import connect_docker_simulator_client
    from app.commissioning.qpc_event_loop import new_qpc_loop
    async def run():
        client=await connect_docker_simulator_client()
        result=await probe(client,args.reverse,args.blend,args.bounded_blend);result['admission']=admission
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','admission')}))
        return 0 if result['passed'] else 1
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:return runner.run(run())


if __name__=='__main__':raise SystemExit(main())
