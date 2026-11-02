"""Default-preview, one separately scoped +Z two-segment trial. No UI service."""
import argparse
import asyncio
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts import real_robot_finite_position as single
from scripts.preview_real_blended_trial import preview as contract
from app.commissioning.blended_trial import RealBlendTrialAdapter,RealBlendTrialPort
from app.commissioning.blended_trial_window import TrialWindow,TrialClient
from app.control.blended_position_session import BlendedPositionSession
from app.control.finite_position_types import PositionIntent

CONFIRM='I_APPROVE_ONE_TWO_SEGMENT_Z_BLEND_TRIAL_AND_ONE_RESERVED_FAULT_STOP'


def preview(p):
    result=contract()
    result.update(scope='one_scoped_two_segment_z_trial',target_ip=p['settings'].ip,
        output=str(p['output']),authorization_expires_utc=p['authorization_expires_utc'],
        input_hashes=p['input_hashes'],runtime_hashes=p['runtime_hashes'],
        settings_binding=json.loads(json.dumps(asdict(p['settings']))),
        sdk_binary_sha256=single.identity.SDK_SHA,model_sha256=p['model'].model_sha256,
        session_s=20,cleanup_s=5,max_reads=4096,readonly_tail_s=2,
        prepare_tolerance_rad=.001,
        native_entry_available=True,
        pending=['current_readonly_site_survey','fresh_onsite_authorization','physical_empty_load_acceptance'])
    return result


class Inputs(single.Inputs):
    def confirmation(self):return CONFIRM
    def expected_preview(self):return preview(self.p)


async def run(p,inputs,*,client_factory=None,clock=time.perf_counter_ns,sleep=asyncio.sleep):
    inputs.cold()
    if single.existing_rpc_connections():raise ValueError('blend_trial_existing_rpc')
    if client_factory is None:
        single.identity.require_execution_runtime()
        if clock is not time.perf_counter_ns or sleep is not asyncio.sleep:raise ValueError('blend_trial_native_clock_override')
    output=p['output'];output.mkdir(parents=True,exist_ok=False)
    (output/'admission.json').write_text(json.dumps(preview(p),indent=2),encoding='utf-8')
    window=TrialWindow(inputs.write,clock);session=None;adapter=None;port=None
    result=dict(complete=False,connection_unknown=False,runtime='INJECTED' if client_factory else 'LIVE')
    async def factory(ip):
        inputs.cold()
        if ip!=p['settings'].ip or single.existing_rpc_connections():raise ValueError('blend_trial_connection_rejected')
        window.entry('connect')
        try:
            if client_factory:client=await client_factory(ip)
            else:
                sdk=single.load_sdk();inputs.cold();sdk.init();inputs.read()
                client=await sdk.connect(ip,False)
            inputs.cold();return TrialClient(client,window)
        except BaseException:
            result['connection_unknown']=True;raise
    try:
        adapter=RealBlendTrialAdapter(p['settings'],client_factory=factory,position_model=p['model'],
            authorize=inputs.read,clock=clock,sleep=sleep)
        await adapter.connect()
        if not (await adapter.preflight()).ready:raise ValueError('blend_trial_preflight')
        sample=adapter._snapshot
        if max(abs(a-b) for a,b in zip(sample.actual_q,p['settings'].teleop_ready_q))>.001:
            raise ValueError('blend_trial_ready_mismatch')
        binding=adapter.bind_position_generation(1)
        port=RealBlendTrialPort(adapter,binding,authorize=inputs.write)
        session=BlendedPositionSession(port,binding,lower=p['settings'].startup_tcp_min_m,
            upper=p['settings'].startup_tcp_max_m,clock=clock,authorize=inputs.read)
        seq=0;released=False
        while clock()-window.start<18_000_000_000:
            if session.closed:raise ValueError(session.reason)
            seq+=1;released=released or session.entered_steps>=2
            session.offer(PositionIntent('bounded-trial',1,seq,clock(),
                'release' if released else 'held',(0.,0.,0.) if released else (0.,0.,1.)))
            await session.advance()
            if session.closed:raise ValueError(session.reason)
            if released and not session.busy and session.phase=='ready':break
            await sleep(.02)
        else:raise ValueError('blend_trial_completion_timeout')
        # Observe the same fixed arrival anchor; do not install a new motion.
        import math
        expected=tuple(x+(.008 if i==2 else 0.) for i,x in enumerate(port.origin))
        if session.entered_steps!=2 or math.dist(port.sample.actual_tcp.p,expected)>.001:
            raise ValueError('blend_trial_endpoint_error')
        anchor=tuple(port.sample.actual_tcp.p);tail_start=clock();tail=[]
        while clock()-tail_start<2_000_000_000:
            feedback=await port.read(())
            if not feedback.idle or not feedback.stationary or math.dist(feedback.position,anchor)>.001:
                raise ValueError('blend_trial_tail_failed')
            tail.append(asdict(feedback));await sleep(.05)
        result.update(complete=True,tail=tail,view=asdict(session.view()))
    except BaseException as error:
        result.update(error=type(error).__name__+':'+str(error),complete=False)
        if session is not None:
            # A failed read-only tail still requires the reserved original stop.
            if session.entered_steps and not session._obligation:session._obligation=True
            await session.abort('blend_trial_failed')
            result['view']=asdict(session.view())
    finally:
        if session and session.worker:
            await asyncio.wait({session.worker},timeout=1.)
            if not session.worker.done():result['complete']=False
        if adapter:
            owner,motion=adapter._snapshot_calls,adapter._motion_calls
            try:await adapter.disconnect()
            except BaseException as error:result.update(complete=False,close_error=type(error).__name__)
            result.update(unknown_reads=sorted(owner.unknown_methods),
                unknown_writes=sorted(motion.unknown_methods),fault=adapter._latched_fault,
                pending_reads=sum(not t.done() for t in owner.tasks),
                pending_writes=sum(not t.done() for t in motion.tasks))
        result['counts']=dict(window.entries)
        if port is not None:result['observations']=port.observations
        if any(result.get(k) for k in ('fault','unknown_reads','unknown_writes','pending_reads','pending_writes','connection_unknown')):result['complete']=False
        (output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('config','reference-survey','ready-pose','output'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--execute-once',action='store_true')
    for key in ('approved-preview','approved-preview-sha256','authorization-expires-utc','confirm'):parser.add_argument('--'+key)
    args=parser.parse_args()
    p=single.prepare(args.config,args.reference_survey,args.ready_pose,args.output,expires=args.authorization_expires_utc)
    if not args.execute_once:print(json.dumps(preview(p),indent=2));return 0
    inputs=Inputs(p,args.approved_preview,args.approved_preview_sha256,args.authorization_expires_utc,args.confirm)
    from app.commissioning.qpc_event_loop import new_qpc_loop
    loop=new_qpc_loop();task=loop.create_task(run(p,inputs))
    try:result=loop.run_until_complete(task)
    except KeyboardInterrupt:
        task.cancel()
        try:loop.run_until_complete(asyncio.wait({task},timeout=5.))
        except KeyboardInterrupt:pass
        result=dict(complete=False,interrupted=True)
        if task.done() and not task.cancelled():result.update(task.result(),complete=False,interrupted=True)
    finally:pending=single.close_loop_bounded(loop)
    result['pending_tasks_at_process_exit']=pending
    if pending:result['complete']=False
    if p['output'].exists():(p['output']/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result));return 0 if result['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
