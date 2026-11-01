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


async def probe(client,reverse=False,blend=False,bounded=False,geometric=False):
    from app.robots.lebai_motion_calls import OwnedMotionCalls
    owner=OwnedMotionCalls();stop_owner=OwnedMotionCalls()
    bounded=bounded or geometric
    blend=blend or bounded
    case=('geometric_blend_reverse' if reverse else 'geometric_blend') if geometric else (
        'bounded_blend' if bounded else 'blend' if blend else 'reverse' if reverse else 'steady')
    report=dict(case=case,commands=[],samples=[],unknown=[],stop_sent=False)
    origin=time.perf_counter();entered=False
    async def sample():
        captured_ns=time.perf_counter_ns()
        state,kin=await asyncio.wait_for(asyncio.gather(client.get_robot_state(),client.get_kin_data()),1.)
        qd=kin['actual_joint_speed'];pose=kin['actual_tcp_pose']
        if len(qd)!=6 or not all(math.isfinite(x) for x in qd):raise ValueError('invalid_feedback')
        row=dict(t=time.perf_counter()-origin,captured_ns=captured_ns,state=state,qd=qd,pose=pose)
        report['samples'].append(row)
        return row
    try:
        initial=await sample()
        if initial['state']!='IDLE' or max(map(abs,initial['qd']))>.002:raise ValueError('requires_idle')
        start=initial['pose'];frame=dict.fromkeys(('x','y','z','rx','ry','rz'),0.)
        gate=None;binding=object();seq=0
        if geometric:
            from app.control.blended_position_gate import BlendedPositionGate
            p=tuple(start[k] for k in ('x','y','z'))
            gate=BlendedPositionGate(binding,tuple(x-.03 for x in p),tuple(x+.03 for x in p),clock=time.perf_counter_ns)
        pending=[]
        for i in range(6 if bounded else 4 if blend else 8):
            direction=(0,0,-1 if reverse and i>=3 else 1)
            if gate and reverse and i==3:
                # Reversal never appends behind the previous direction. Retain
                # ownership until every old ID and a continuous quiet span agree.
                report['reverse_requested_t']=time.perf_counter()-origin
                seq+=1;gate.offer(binding,seq,time.perf_counter_ns(),direction)
                deadline=time.perf_counter()+3.
                while gate.draining:
                    finished=[]
                    for ident in pending:
                        state=await asyncio.wait_for(client.get_motion_state(ident),.5)
                        if state=='FINISHED':finished.append(ident)
                        elif state not in ('WAIT','RUNNING'):raise ValueError('reverse_task_unknown')
                    pending=[ident for ident in pending if ident not in finished]
                    row=await sample()
                    gate.observe(binding,tuple(row['pose'][k] for k in ('x','y','z')),row['captured_ns'],
                        finished=finished,idle=row['state']=='IDLE',stationary=max(map(abs,row['qd']))<=.002)
                    if time.perf_counter()>deadline:raise ValueError('reverse_drain_timeout')
                    if gate.draining:await asyncio.sleep(.02)
                report['reverse_drained_t']=time.perf_counter()-origin
            if bounded and len(pending)==2:
                deadline=time.perf_counter()+2.
                while True:
                    state=await asyncio.wait_for(client.get_motion_state(pending[0]),.5)
                    if state=='FINISHED':
                        done=pending.pop(0)
                        if gate:
                            row=await sample()
                            gate.observe(binding,tuple(row['pose'][k] for k in ('x','y','z')),row['captured_ns'],finished=(done,))
                        break
                    if state not in ('WAIT','RUNNING') or time.perf_counter()>deadline:raise ValueError('queue_did_not_advance')
                    await sample();await asyncio.sleep(.02)
            row=await sample()
            reserved=None;lead=None
            if gate:
                deadline=time.perf_counter()+2.
                while reserved is None:
                    seq+=1;gate.offer(binding,seq,time.perf_counter_ns(),direction)
                    actual=tuple(row['pose'][k] for k in ('x','y','z'))
                    reserved=gate.reserve(binding,actual,row['captured_ns'])
                    if reserved is not None:break
                    if time.perf_counter()>deadline:raise ValueError('geometry_did_not_advance')
                    await asyncio.sleep(.02);row=await sample()
                lead=math.dist(actual,gate.tail or actual)+gate.step
                gate.require_entry(binding,reserved,actual,row['captured_ns'])
            if math.dist([row['pose'][k] for k in ('x','y','z')],[start[k] for k in ('x','y','z')])>.025:
                raise ValueError('probe_displacement_limit')
            velocity={**frame,'z':-.01 if reverse and i>=4 else .01}
            stamp=time.perf_counter();entered=True
            if blend:
                target={**start,'z':start['z']+.004*(i+1)}
                if reserved:target={**start,**dict(zip(('x','y','z'),reserved.position))}
                operation=lambda:client.movel(target,.1,.02,0.,.001 if bounded or i<3 else 0.)
            else:operation=lambda:client.speedl(.1,velocity,.2,frame)
            ident=await owner.call('movel' if blend else 'speedl',operation,.5,revoked=lambda:False)
            if type(ident) is not int or ident<=0:raise ValueError('invalid_motion_id')
            if gate:gate.acknowledge(binding,reserved,ident)
            pending.append(ident)
            report['commands'].append(dict(t=stamp-origin,reply_t=time.perf_counter()-origin,id=ident,
                **(dict(target=target,r=.001 if bounded or i<3 else 0.,lead_m=lead) if blend else dict(z=velocity['z']))))
            await asyncio.sleep(max(0,.08-(time.perf_counter()-stamp)))
        report['release_t']=time.perf_counter()-origin
        if gate:gate.offer(binding,seq+1,time.perf_counter_ns(),(0,0,0))
        report['release_sample']=await sample()
        # No new command after release. Measure whether finite duration drains.
        deadline=time.perf_counter()+3.;stable=0;quiet_since=None
        while time.perf_counter()<deadline:
            finished=[]
            if gate:
                for ident in pending:
                    state=await asyncio.wait_for(client.get_motion_state(ident),.5)
                    if state=='FINISHED':finished.append(ident)
                    elif state not in ('WAIT','RUNNING'):raise ValueError('drain_task_unknown')
                pending=[ident for ident in pending if ident not in finished]
            row=await sample()
            quiet=row['state']=='IDLE' and max(map(abs,row['qd']))<=.002
            if gate:
                gate.observe(binding,tuple(row['pose'][k] for k in ('x','y','z')),row['captured_ns'],
                    finished=finished,idle=row['state']=='IDLE',stationary=max(map(abs,row['qd']))<=.002)
                quiet=quiet and not pending and gate.tail is None
            stable=stable+1 if quiet else 0
            if quiet_since is None and quiet:quiet_since=row['t']
            if not quiet:quiet_since=None
            if stable>=3 and (gate is not None or row['t']-quiet_since>=.2):
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
        if 'gate' in locals() and gate:gate.revoke()
        report['unknown']=sorted(owner.unknown_methods|stop_owner.unknown_methods)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--reverse',action='store_true')
    parser.add_argument('--blend',action='store_true')
    parser.add_argument('--bounded-blend',action='store_true')
    parser.add_argument('--geometric-blend',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.geometric_blend:args.bounded_blend=True
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
        result=await probe(client,args.reverse,args.blend,args.bounded_blend,args.geometric_blend);result['admission']=admission
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','admission')}))
        return 0 if result['passed'] else 1
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:return runner.run(run())


if __name__=='__main__':raise SystemExit(main())
