"""Range/time regression using the exact product Lua in network=none Docker.

No host SDK connection or published test port. A synthetic pose producer is
injected only into this probe; the product code keeps its real mailbox input.
Preview by default. No retry, container restart or unknown-result cleanup.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
sys.path.insert(0,str(ROOT/'scripts'))
from docker_pose_transport_probe import admission
from app.control.docker_pose_session import build_pose_program
from app.digital_twin.docker_runtime import docker_settings
from app.commissioning.finite_position_site import MdhPositionModel


def program(scenario):
    if scenario not in ('continuation','boundary','human'):
        raise ValueError('invalid_continuation_scenario')
    settings=docker_settings().lebai
    _,q=MdhPositionModel.from_settings(settings).forward(settings.teleop_ready_q)
    source=build_pose_program(SimpleNamespace(settings=settings,_position_pitch_reference=q))
    feed=(ROOT/'scripts/docker_pose_continuation_source.lua').read_text(encoding='utf-8')
    feed=feed.replace('__BOUNDARY__','true' if scenario in ('boundary','branch') else 'false')
    feed=feed.replace('__BRANCH__','true' if scenario=='branch' else 'false')
    feed=feed.replace('__HUMAN__','true' if scenario=='human' else 'false')
    marker='    wait_all(receive,plan,motion)'
    if source.count(marker)!=1:
        raise ValueError('continuation_source_marker')
    source=source.replace(marker,feed+'\n    wait_all(producer,receive,plan,motion)')
    lines=source.splitlines()
    if lines[0]!='program_begin(0)' or lines[-1]!='program_end(0)':
        raise ValueError('continuation_program_frame')
    body='\n'.join(lines[1:-1])+'\nreturn ok,err,moves\n'
    wrapper=(ROOT/'scripts/docker_pose_continuation_probe.lua').read_text(encoding='utf-8')
    return wrapper.replace('__PRODUCT_BODY__',body)


def analyze(raw,scenario):
    rows=[line.strip().split('\t') for line in raw.splitlines()]
    final=[r for r in rows if r[0]=='CONT_DONE']
    done=[r for r in rows if r[0]=='LIVE_DONE']
    points=[r for r in rows if r[0]=='LIVE_POINT']
    initial=[r for r in rows if r[0]=='CONT_REFERENCE']
    source=[r for r in rows if r[0]=='CONT_SOURCE_END']
    boundary=[r for r in rows if r[0]=='LIVE_BOUNDARY']
    result=dict(passed=False,completed=bool(final),scenario=scenario,points=len(points),raw_output=raw,
                aborted=any(r[0] in ('LIVE_ABORT','CONT_PREP_ABORT') for r in rows))
    try:
        if (len(final)!=1 or len(final[0])<4 or final[0][1]!='true'
                or len(done)!=1 or done[0][1]!='true' or len(initial)!=1 or len(source)!=1
                or result['aborted'] or not points or not any(r[0]=='LIVE_QUIET' for r in rows)):
            return result
        if int(final[0][3])!=len(points) or int(done[0][3])!=len(points):
            return result
        numeric=[tuple(float(v) for v in r[1:]) for r in points]
        if any(len(r)!=6 or any(not math.isfinite(x) for x in r) for r in numeric):
            return result
        origin=tuple(float(v) for v in initial[0][1:4])
        if (len(origin)!=3 or not all(math.isfinite(v) for v in origin)
                or any(b[0]<=a[0] for a,b in zip(numeric,numeric[1:]))):
            return result
        result['max_send_gap_ms']=max((b[0]-a[0] for a,b in zip(numeric,numeric[1:])),default=0)
        result['strict_interval_cadence']=result['max_send_gap_ms']<80
        result['last_point_ms']=numeric[-1][0]
        result['source_samples']=int(source[0][2])
        result['reference_joint_span_rad']=float(source[0][3])
        if not math.isfinite(result['reference_joint_span_rad']):
            return result
        result['sequence_wrap_observed']=any(b[1]<a[1] for a,b in zip(numeric,numeric[1:]))
        result['x_peak_progress_m']=max(r[2]-origin[0] for r in numeric)
        result['max_tcp_displacement_m']=max(math.dist(r[2:5],origin) for r in numeric)
        result['boundary']=boundary[-1][2] if boundary else None
        result['saturated']=any(r[:2]==['LIVE_SATURATED','true'] for r in rows)
        if scenario in ('continuation','human'):
            result['passed']=bool(len(points)>400 and result['last_point_ms']>43000
                and result['source_samples']>1024 and result['x_peak_progress_m']>.022
                and result['sequence_wrap_observed']
                and result['reference_joint_span_rad']>.08 and not boundary
                and any(r[0]=='LIVE_EXPIRED' for r in rows))
        elif scenario=='boundary':
            result['passed']=bool(result['saturated'] and not boundary and len(points)>60
                                  and result['max_tcp_displacement_m']<=.302)
        elif scenario=='branch':  # Offline analysis of archived pre-stepper evidence only.
            result['passed']=bool(result['boundary']=='ik_branch' and result['max_tcp_displacement_m']<.002)
        result['passed']=result['passed'] and result['max_send_gap_ms']<160
    except (ValueError,IndexError):
        pass
    return result


def execute(admitted,scenario):
    bridge=('exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; '
        'while IFS= read -r -t 30 line <&3; do printf "%s\\n" "$line"; '
        'case "$line" in *CONT_DONE*) break;; esac; done')
    try:
        response=subprocess.run(['docker','exec','-i',admitted['container_id'],'bash','-c',bridge],
            input=program(scenario),text=True,encoding='utf-8',capture_output=True,timeout=90)
        result=analyze(response.stdout,scenario)
        result.update(transport_returncode=response.returncode,transport_stderr=response.stderr)
        result['passed']=result['passed'] and response.returncode==0
    except subprocess.TimeoutExpired as error:
        raw=error.stdout or b''
        if isinstance(raw,bytes):raw=raw.decode('utf-8',errors='replace')
        result=dict(passed=False,completed=False,raw_output=raw,error='execution_unknown',stop_unverified=True)
    return dict(result,admission=admitted)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container',required=True)
    parser.add_argument('--scenario',choices=('continuation','boundary','human'),default='continuation')
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();admitted=admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,scenario=args.scenario,
            prepare_movej=1,duration_s=44 if args.scenario in ('continuation','human') else 8,
            product_grip_span_mm=300,product_window_s=300)))
        return 0
    result=execute(admitted,args.scenario)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('admission','raw_output')}))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
