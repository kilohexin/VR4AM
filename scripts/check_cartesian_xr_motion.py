"""Finite Cartesian Docker motion from genuine PositionIntent-shaped inputs.

Preview default, pinned network=none image only. Requires quiet IDLE; no
startup/prepare, claw, physical selector or retry. Same <=128 PVAT and one
exception stop limits as check_cartesian_live_motion. Synthetic hand frames
exercise the product input/mapping contract, not a browser/headset claim.
"""
import argparse
import asyncio
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
from time import perf_counter_ns

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'scripts')]
from scipy.spatial.transform import Rotation
from app.control.cartesian_pose_input import CartesianPoseInput
from app.control.compiled_reference_geometry import CompiledReferenceModel
from app.commissioning.native_jog_mdh import MdhTool
from app.control.finite_position_types import PositionBinding,PositionIntent
from app.control.cartesian_tool_binding import ZERO_TOOL,checked_tool,tool_digest,tool_arguments
from app.robots.lebai_codec import pose_from_lebai
from app.schemas.messages import Pose
from scripts import check_cartesian_live_host as live
from scripts import check_cartesian_live_motion as motion
from scripts.cartesian_signal_channel import CartesianSignalChannel
from scripts.check_native_cartesian_reference import native_admission


async def execute(admission,output,*,expire=False,tool=ZERO_TOOL):
    checked_tool(tool)
    # Warm pure math before the clutch capture, never by resetting its age.
    await asyncio.to_thread(CompiledReferenceModel,tool,(-3.,)*6,(3.,)*6)
    binding=PositionBinding(object(),7,live.model_fingerprint(tool),tool_digest(tool))
    hand=Pose(p=(.1,.2,.3),q=(0.,0.,0.,1.))
    channel=CartesianSignalChannel(admission)
    try:
        await channel.open()
        if await channel.call('get_robot_state')!='IDLE':raise ValueError('xr_cartesian_requires_idle')
        data=await channel.call('get_kin_data');stamp=perf_counter_ns()
        if any(abs(x)>1e-9 for x in data['actual_joint_speed']):raise ValueError('xr_cartesian_requires_quiet')
        tcp=pose_from_lebai(data['actual_tcp_pose'])
        intent=PositionIntent('xr-finite-cartesian-probe',7,1,stamp,'held',(0.,0.,0.),hand_pose=hand)
        inputs=CartesianPoseInput(binding,intent,tcp,tcp_ns=stamp,clock=perf_counter_ns)
    except BaseException:
        await channel.close();raise
    ready=asyncio.Event();finished=False;count=1;max_gap=0;last=stamp;driver_error=None
    async def feed():
        nonlocal count,max_gap,last
        started=None
        while not finished and not inputs.closed:
            now=perf_counter_ns()
            if ready.is_set() and started is None:started=now
            elapsed=0. if started is None else (now-started)/1e9
            if elapsed>=5.5:
                if not expire:
                    count+=1;inputs.offer(replace(intent,seq=count,received_ns=now,kind='release',hand_pose=None))
                return
            count+=1;max_gap=max(max_gap,now-last);last=now
            # Full-pose pitch reversal; fixed XYZ. Same signed tool mapping as
            # VR, distinct UI generation 7 must reach native admission intact.
            turned=hand.model_copy(update={'q':tuple(Rotation.from_euler('x',.12*math.sin(2*math.pi*elapsed/4)).as_quat())})
            inputs.offer(replace(intent,seq=count,received_ns=now,hand_pose=turned))
            await asyncio.sleep(.02)
    def frozen_program(origin,session,model_hash,*,generation=1,tool=ZERO_TOOL):
        source=motion.program(origin,session,model_hash,generation=generation,tool=tool)
        output.parent.mkdir(parents=True,exist_ok=True);output.with_suffix('.lua').write_text(source,encoding='utf-8')
        return source
    async def guarded_feed():
        nonlocal driver_error
        try:await feed()
        except Exception as error:
            driver_error=local_error(error);inputs.close('input_driver_fault')
    driver=asyncio.create_task(guarded_feed())
    try:
        # Keep fresh genuine frames arriving while capture I/O drains. Do not
        # close the read channel and later pretend its old capture is new.
        await channel.close()
        result=motion.analyze(await live.execute(admission,expire=expire,_source_factory=frozen_program,
            _input_source=inputs,_ready=ready,tool=tool))
        output.with_suffix('.host.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        motion.analyze_pitch(result,tool)
        result.update(input_frames=count,max_input_gap_ms=max_gap/1e6,input_exit=inputs.reason,
            driver_error=driver_error,scheduler_clock='time.perf_counter',real_headset_verified=False,admission=admission)
        result['passed']=bool(result['passed'] and driver_error is None)
        return result
    finally:
        finished=True;await driver;await channel.close()


def local_error(error):
    import re
    value=str(error)
    return value if re.fullmatch(r'(?:cartesian|vr_pose|xr_cartesian)_[a-z_]+',value) else type(error).__name__


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--run',action='store_true')
    p.add_argument('--expire',action='store_true')
    p.add_argument('--tool-report',type=Path);p.add_argument('--tool-report-sha256')
    args=p.parse_args();tool=tool_arguments(p,args);admitted=native_admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,pvat_max=128,exception_stop_max=1,
            input='PositionIntent',generation=7,tool_sha256=tool_digest(tool),
            tool_report_sha256=args.tool_report_sha256,real_headset_verified=False,hardware_verified=False)));return 0
    if args.output.exists() or args.output.with_suffix('.lua').exists():
        p.error('evidence output already exists; preserve the previous attempt')
    try:report=live.run_local(execute(admitted,args.output,expire=args.expire,tool=tool))
    except Exception as error:
        built=args.output.with_suffix('.lua').exists()
        report=dict(passed=False,error_type=type(error).__name__,error_code=local_error(error),program_created=built,
            unknown_motion_write=built,stop_unverified=built,admission=admitted,
            hardware_verified=False,real_headset_verified=False,product_admission_complete=False)
    report.update(tool_sha256=tool_digest(tool),tool_report_sha256=args.tool_report_sha256)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('raw_output','heads','statistics','admission')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
