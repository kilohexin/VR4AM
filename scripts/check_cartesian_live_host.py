"""Bounded LIVE host planning/transport in network=none official Docker.

The native executable has NO actuator APIs. Synthetic current-time inputs,
single host math worker, independently renewed authority, deliberate late
math, owned-brake/hold/resume, release/expiry, actual two-bank transport.
This is NOT a UI entry or permission to send motion.
"""
import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from time import perf_counter_ns

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'scripts')]
from scipy.spatial.transform import Rotation
from app.commissioning.native_jog_mdh import MdhTool
from app.control.compiled_reference_geometry import CompiledReferenceModel
from app.control.cartesian_clock import ControllerClock
from app.control.cartesian_host_planner import CartesianHostPlanner
from app.control.cartesian_pvat_reference import JointReference
from cartesian_signal_channel import CartesianSignalChannel
from cartesian_native_pipe import arguments
from cartesian_signal_publisher import CartesianSignalPublisher
from check_native_cartesian_reference import native_admission
from check_native_cartesian_motion import transport,vec


def program(origin,session,model_hash,*,generation=1):
    if type(generation) is not int or not 1<=generation<2**31:raise ValueError('cartesian_program_generation')
    source=(ROOT/'scripts/native_cartesian_live_dry.lua').read_text(encoding='utf-8')
    for key,name in [('__BOOT_CLOCK__','cartesian_boot_clock.lua'),('__MAILBOX__','cartesian_bundle_mailbox.lua'),('__AUTHORITY__','cartesian_authority.lua'),
            ('__OWNER__','cartesian_bundle_owner.lua')]:
        source=source.replace(key,(ROOT/'scripts'/name).read_text(encoding='utf-8'))
    words='{'+','.join(str(v) for v in struct.unpack('<4i',bytes.fromhex(session)))+'}'
    binding="{session='"+session+"',model='"+model_hash+"',generation="+str(generation)+",words="+words+'}'
    return source.replace('__ORIGIN__','{q='+vec(origin.q)+',v='+vec(origin.v)+',a='+vec(origin.a)+'}').replace('__BINDING__',binding)


def model_fingerprint():
    sha=hashlib.sha256()
    for path in ('backend/app/control/cartesian_pvat_reference.py','backend/app/control/cartesian_reference_bundle.py',
            'backend/app/control/compiled_reference_geometry.py','backend/app/commissioning/native_jog_mdh.py',
            'backend/app/control/canonical_joint_quintic.py','backend/app/control/cartesian_bundle_wire.py',
            'backend/app/control/cartesian_host_planner.py','backend/app/control/cartesian_pose_input.py',
            'backend/app/control/vr_pose_reference.py'):
        sha.update(path.encode());sha.update((ROOT/path).read_bytes())
    sha.update(b'zero-tool;q+-3;reference-defaults;workspace=.35;segment=24;curves=4;joint_fit;owned-braking-prefix=1')
    return sha.hexdigest()


def run_local(coroutine):
    # Match the product launcher: QPC owns BOTH time readings and timers.
    # Merely calling perf_counter in callbacks does not fix the default
    # Windows event loop's GetTickCount64 scheduling resolution.
    from app.commissioning.qpc_event_loop import new_qpc_loop
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:
        return runner.run(coroutine)


def synthetic_target(base,time,fixture='mixed'):
    """Current-time diagnostic inputs, never replaying stale capture times."""
    if fixture=='pitch-reversal':
        # Go through the SAME grip/tool basis mapping as a held XR controller.
        # Keep the endpoint position exactly fixed while crossing pitch zero.
        from app.control.vr_pose_reference import VRPoseReference
        from app.schemas.messages import Pose
        hand=Pose(p=(0.,0.,0.),q=(0.,0.,0.,1.))
        reference=VRPoseReference(clock=lambda:0,translation_scale=1.,orientation_mode='tool')
        reference.capture(hand,base,generation=1,hand_ns=0,tcp_ns=0)
        turned=hand.model_copy(update={'q':tuple(Rotation.from_euler('x',.12*math.sin(2*math.pi*time/4)).as_quat())})
        return reference.target(turned,generation=1,seq=1,captured_ns=0)
    if fixture!='mixed':raise ValueError('cartesian_diagnostic_fixture')
    return base.model_copy(update={'p':tuple(base.p[i]+.006*math.sin(time*(i+1)) for i in range(3)),
        'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('xyz',(.04*math.sin(time),.03*math.sin(2*time),.04*math.sin(.7*time)))).as_quat())})


async def execute(admission,*,expire=False,_source_factory=None,fixture='mixed',_input_source=None,_ready=None):
    if fixture not in ('mixed','pitch-reversal'):raise ValueError('cartesian_diagnostic_fixture')
    from app.control.cartesian_pose_input import CartesianPoseInput
    if _input_source is not None and type(_input_source) is not CartesianPoseInput:
        raise ValueError('cartesian_live_input_source')
    generation=1 if _input_source is None else _input_source.binding.generation
    if type(generation) is not int or not 1<=generation<2**31:raise ValueError('cartesian_live_generation')
    capture=await asyncio.to_thread(transport,admission,"""program_begin(0)
disable_auto_sync()
assert(lebai:get_robot_state()=='IDLE','dry_capture_requires_idle')
local k=lebai:get_kin_data();local q={}
for i=1,6 do assert(math.abs(k.actual_joint_speed[i])<1e-9,'dry_not_quiet');q[i]=string.format('%.17g',k.actual_joint_pose[i]) end
print('DRY_ORIGIN',table.concat(q,'\\t'))
print('DRY_CAPTURE_DONE',true,nil)
program_end(0)
""",'DRY_CAPTURE_DONE')
    rows=[r.strip().split('\t') for r in capture['raw_output'].splitlines() if r.startswith('DRY_ORIGIN\t')]
    if capture.get('transport_returncode')!=0 or len(rows)!=1:raise ValueError('live_dry_capture')
    origin=JointReference(tuple(float(x) for x in rows[0][1:]))
    model=await asyncio.to_thread(CompiledReferenceModel,MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    base=model.pose(origin.q);session=__import__('secrets').token_hex(16);fingerprint=model_fingerprint()
    if _input_source is not None:
        if _input_source.closed:raise ValueError('cartesian_live_input_retired')
        anchor=_input_source.anchor_tcp
        if (math.dist(anchor.p,base.p)>1e-6 or
                (Rotation.from_quat(anchor.q).inv()*Rotation.from_quat(base.q)).magnitude()>1e-6):
            raise ValueError('cartesian_live_capture_changed')
    source=(program if _source_factory is None else _source_factory)(origin,session,fingerprint,generation=generation)
    if await asyncio.to_thread(native_admission,admission['container_id'])!=admission:raise ValueError('live_dry_identity')
    proc=await asyncio.create_subprocess_exec(*arguments(admission,'CART_DONE'),
        stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.PIPE)
    proc.stdin.write(source.encode());await proc.stdin.drain();proc.stdin.close()
    ready=asyncio.get_running_loop().create_future();acks={};raw=[];heads=[];h=None;error=None
    async def read():
        nonlocal error
        try:
            while line:=await proc.stdout.readline():
                row=line.decode('utf-8').strip().split('\t');raw.append(row)
                if row[0]=='CART_READY':ready.set_result(int(row[1]))
                elif row[0]=='CART_ACK':
                    pass  # Logs are evidence; signal-channel ACKs drive control.
                elif row[0]=='CART_HEAD':
                    if len(row)!=22:raise ValueError('live_dry_head_shape')
                    values=tuple(float(x) for x in row[4:]);state=JointReference(values[:6],values[6:12],values[12:])
                    heads.append((int(row[1]),int(row[2]),row[3]))
                elif row[0]=='CART_DONE':return
        except BaseException as exc:error=exc;raise
    reader=asyncio.create_task(read());rpc=CartesianSignalChannel(admission);tasks=[];planner_task=None;cleaning=False
    stats=dict(planned=0,committed=0,rejected=0,discarded=0,late_injected=False,resumed_after_stall=False,
        math_ms=[],publications=[],authority_writes=0,expire=expire,fixture=fixture,
        input_provider='synthetic' if _input_source is None else 'position_intent_pose',generation=generation)
    try:
        epoch=await asyncio.wait_for(ready,3);await rpc.open()
        # Explicit no-I/O clock calibration, before any input is admitted.
        # Keep every sample; the narrowest valid interval is the tightest
        # conservative bound, not a refreshed measurement of old input.
        calibration=[]
        for nonce in range(1,6):
            sent=perf_counter_ns();await rpc.call('set_signal',250,nonce)
            async with asyncio.timeout(.1):
                while True:
                    values=await rpc.call('get_signals',251,2);received=perf_counter_ns()
                    if values[1]==nonce:break
                    await asyncio.sleep(.001)
            from types import SimpleNamespace
            calibration.append(SimpleNamespace(sent_ns=sent,received_ns=received,sampled_ms=epoch+values[0]))
        stats['calibration_roundtrip_ms']=[(s.received_ns-s.sent_ns)/1e6 for s in calibration]
        if any(b.sampled_ms<a.sampled_ms or b.sent_ns<a.received_ns
                for a,b in zip(calibration,calibration[1:])):
            raise ValueError('cartesian_clock_rollback')
        reply=min(calibration,key=lambda r:r.received_ns-r.sent_ns)
        pair=ControllerClock(reply.sent_ns,reply.received_ns,reply.sampled_ms,resolution_ms=10)
        for sample in calibration:
            if sample.sent_ns>=pair.received_ns:pair.check_pair(sample.sent_ns,sample.received_ns,sample.sampled_ms)
        # Host scheduling uses the earliest native bound. It is only a work
        # hint, never motion admission. The native commit AND owner still
        # check actual current time, original input age and the exact due grid.
        # Using the upper bound here AND an already lowered capture would
        # count the entire clock uncertainty twice and starve all resumption.
        clock=lambda:pair.capture_lower_ms(perf_counter_ns())-epoch
        h=CartesianHostPlanner(model,origin,session=session,model_sha256=fingerprint,generation=generation,
            clock=clock,planning_braking_prefix=True,braking_segment_limit=24,braking_curve_limit=4)
        publisher=CartesianSignalPublisher(rpc,h,epoch_ms=epoch,clock_pair=pair,host_clock=perf_counter_ns)
        latest=[None];active=True;start=perf_counter_ns();stall_finished=False
        async def monitor():
            last_ack=0;last_frame=0
            phases={0:'waiting',1:'armed',2:'active',3:'holding',4:'finishing',5:'ready',6:'fault'}
            while not cleaning and (active or h.pending is not None):
                frame=await rpc.call('get_signals',253,2);before=frame[0]
                if frame[1]!=0:
                    h.close('cartesian_controller_fault');raise ValueError('cartesian_controller_fault')
                if before and before%2==0:
                    values=await rpc.call('get_signals',246,10)
                    after=await rpc.call('get_signal',253)
                    if before==after and values[7]==before and before!=last_frame:
                        last_frame=before;seq=values[2]
                        if values[8]!=0:
                            h.close('cartesian_controller_fault');raise ValueError('cartesian_controller_fault')
                        if seq>last_ack:
                            accepted=values[3]==1
                            h.acknowledge(seq,accepted);last_ack=seq
                            if seq in acks and not acks[seq].done():acks[seq].set_result(accepted)
                        if not h.closed:h.observe_owned_cursor(values[0],values[9],phases[values[1]],generation=generation)
                await asyncio.sleep(.002)
        async def inputs():
            sequence=0
            while active:
                stamp=perf_counter_ns();time=(stamp-start)/1e9;sequence+=1
                if _input_source is None:
                    target=synthetic_target(base,time,fixture)
                    h.offer(target,captured_ms=pair.capture_lower_ms(stamp)-epoch,sequence=sequence,generation=generation)
                    latest[0]=stamp
                else:
                    sample=_input_source.sample()
                    if sample is not None and sample.seq>h.input_sequence and sample.captured_ns>=pair.received_ns:
                        h.offer(sample.pose,captured_ms=pair.capture_lower_ms(sample.captured_ns)-epoch,
                            sequence=sample.seq,generation=generation)
                        latest[0]=sample.captured_ns
                await asyncio.sleep(.005)
        async def authority():
            while active:
                if latest[0] is not None and 0<=perf_counter_ns()-latest[0]<100_000_000:
                    await publisher.offer_authority(latest[0]);stats['authority_writes']+=1
                await asyncio.sleep(.02)
        async def planning():
            nonlocal stall_finished
            while active:
                if h.captured_ms is None:await asyncio.sleep(.003);continue
                tip=h.planning_head()
                due=(clock()+400 if h.reference_epoch is None else
                    h.reference_epoch+max(0,(tip or (0,))[0]-1)*80)
                job=h.begin(due) if tip is not None and due>clock() else None
                if job is None:await asyncio.sleep(.003);continue
                began=perf_counter_ns();calculated=await asyncio.to_thread(h.calculate,job)
                stats['math_ms'].append((perf_counter_ns()-began)/1e6);stats['planned']+=1
                if stats['planned']==8:
                    stats['late_injected']=True;await asyncio.sleep(.6);stall_finished=True
                packet=h.complete(calculated)
                if packet is None:stats['discarded']+=1;continue
                acks[packet.sequence]=asyncio.get_running_loop().create_future()
                publication=dict(sequence=packet.sequence,parent=packet.parent_tick,captured=packet.captured_ms,
                    due=packet.due_ms,began=clock(),native_head=h.head_tick)
                committed=await publisher.publish(packet)
                publication['returned']=clock();stats['publications'].append(publication)
                if not committed:
                    h.acknowledge(packet.sequence,False);stats['rejected']+=1;continue
                accepted=await asyncio.wait_for(acks[packet.sequence],.5)
                stats['committed' if accepted else 'rejected']+=1
                if stall_finished and accepted:stats['resumed_after_stall']=True
        tasks=[asyncio.create_task(inputs()),asyncio.create_task(authority()),asyncio.create_task(monitor())]
        if _ready is not None:_ready.set()
        await asyncio.sleep(.025);planner_task=asyncio.create_task(planning())
        async with asyncio.timeout(8):
            while (perf_counter_ns()-start)/1e9<6:
                if _input_source is not None and _input_source.closed:break
                if _input_source is not None and reader.done():
                    reader.result()
                    quiet=[r for r in raw if r[0]=='CART_QUIET']
                    done=[r for r in raw if r[0]=='CART_DONE']
                    if (len(quiet)==1 and quiet[0][2]=='ready' and len(done)==1
                            and done[0][1:]==['true','nil']):
                        _input_source.close('input_expired' if quiet[0][3] in ('expired','input_expired') else 'release')
                        break
                for task in tasks+[planner_task,reader]:
                    if task.done():task.result();raise ValueError('live_dry_early_exit')
                await asyncio.sleep(.02)
        active=False;h.close('tracking_lost' if expire else
            _input_source.reason if _input_source is not None and _input_source.closed else 'release')
        await asyncio.gather(*tasks,planner_task)
        if not expire:await publisher.release()
        await asyncio.wait_for(reader,7);rc=await proc.wait()
        done=[r for r in raw if r[0]=='CART_DONE'];quiet=[r for r in raw if r[0]=='CART_QUIET']
        segments=[r for r in raw if r[0]=='CART_SEGMENT']
        following=sum(r[2]=='following' for r in segments);braking=sum(r[2]=='braking' for r in segments)
        stats.update(native_done=done,native_quiet=quiet,following=following,braking=braking,
            clock_roundtrip_ms=(reply.received_ns-reply.sent_ns)/1e6)
        passed=(rc==0 and len(done)==1 and done[0][1:]==['true','nil'] and len(quiet)==1
            and quiet[0][2]=='ready' and stats['late_injected'] and stats['resumed_after_stall']
            and quiet[0][3] in ({'expired','input_expired'} if expire else {'released'})
            and following>=8 and braking>0 and not rpc.unknown and not h.publish_unknown)
        entries=[r for r in raw if r[0]=='CART_ACTUAL_ENTER']
        return dict(passed=passed,statistics=stats,raw_output=raw,heads=heads,motion_calls=len(entries),
            signal_writes=True,unknown=rpc.unknown,program_sha256=hashlib.sha256(source.encode()).hexdigest(),
            model_sha256=fingerprint,product_admission_complete=False,hardware_verified=False)
    except Exception as exc:
        if h is not None:h.close(type(exc).__name__)
        # No actuator exists here. Preserve signal/transport uncertainty and
        # allow the finite native owner to expire; never restart/retry it.
        return dict(passed=False,error=type(exc).__name__+':'+str(exc),raw_output=raw,heads=heads,
            host_reason=None if h is None else h.reason,
            unresolved_publication=bool(h is not None and h.pending is not None),
            statistics=stats,motion_calls=sum(r[0]=='CART_ACTUAL_ENTER' for r in raw),
            unknown=rpc.unknown,program_sha256=hashlib.sha256(source.encode()).hexdigest(),
            model_sha256=fingerprint,product_admission_complete=False)
    finally:
        active=False;cleaning=True
        if h is not None:h.close('cleanup')
        if planner_task and not planner_task.done():
            # Retire output, await actual math completion rather than claiming
            # that cancellation stopped a worker thread.
            if h is not None:h.close('cleanup')
            try:await asyncio.wait_for(asyncio.shield(planner_task),3)
            except Exception:pass
        # Let already-issued SDK signal reads/writes settle. Cancelling them
        # here would manufacture unknown results during ordinary cleanup.
        await asyncio.gather(*tasks,return_exceptions=True)
        await rpc.close()
        if not reader.done():
            try:await asyncio.wait_for(reader,7)
            except Exception:pass
        if proc.returncode is None:proc.terminate();await proc.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true')
    p.add_argument('--expire',action='store_true');args=p.parse_args();admitted=native_admission(args.container)
    if not args.run:print(json.dumps(dict(admission=admitted,execute=False,motion_calls=0)));return 0
    report=run_local(execute(admitted,expire=args.expire));report['admission']=admitted
    report['scheduler_clock']='time.perf_counter'
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    summary={k:v for k,v in report.items() if k not in ('raw_output','heads','admission','statistics')}
    summary['statistics']={k:v for k,v in report.get('statistics',{}).items() if k not in ('math_ms','publications')}
    summary['max_math_ms']=max(report.get('statistics',{}).get('math_ms',[]),default=0)
    print(json.dumps(summary))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
