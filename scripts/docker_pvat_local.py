"""Execute one bounded PVAT reference inside verified Docker's Lua 3010 service.

Uses docker exec/stdin only; no new port, installed package or persistent scene.
Default is preview. Caller must close product services first. Raw response is
retained even on failure; transport loss never implies cancellation in Lua.
"""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))


def code(interrupt=False,phase_clock=False,direct_api=False,pvt=False):
    if pvt and (not phase_clock or direct_api):raise ValueError('pvt_requires_sdk_phase_clock')
    return Path(__file__).with_suffix('.lua').read_text(encoding='utf-8').replace(
        '__INTERRUPT__','true' if interrupt else 'false').replace(
        '__PHASE_CLOCK__','true' if phase_clock else 'false').replace(
        '__DIRECT_API__','true' if direct_api else 'false').replace('__PVT__','true' if pvt else 'false')


def analyze(output, *, interrupt=None):
    rows=[line.strip().split('\t') for line in output.splitlines()]
    points=[r for r in rows if r[0]=='PVAT_POINT']
    tails=[r for r in rows if r[0]=='PVAT_TAIL']
    done=[r for r in rows if r[0]=='PVAT_DONE']
    begins=[r for r in rows if r[0]=='PVAT_BEGIN']
    releases=[r for r in rows if r[0]=='PVAT_RELEASE']
    result=dict(completed=bool(done),controller_passed=bool(done and done[-1][1]=='true'),
                points=len(points),raw_output=output)
    if points:
        result['max_send_gap_ms']=max((float(b[1])-float(a[1]) for a,b in zip(points,points[1:])),default=0)
        result['max_rpc_ms']=max(float(r[5]) for r in points)
        result['max_same_sample_error_rad']=max(abs(float(r[2])-float(r[3])) for r in points)
        middle=[r for r in points if 400<=float(r[1])<=1600]
        result['middle_samples']=len(middle)
        result['middle_near_zero_velocity_samples']=sum(abs(float(r[4]))<.0001 for r in middle)
        if all(len(r)>=8 for r in points):
            result['max_sampled_joint_acceleration_radps2']=max(abs(float(r[7])) for r in points)
            result['max_sample_delay_ms']=max(float(r[6])-float(r[1]) for r in points)
    if begins and tails:
        result['final_displacement_rad']=float(tails[-1][3])-float(begins[0][1])
        if points:
            result['last_point_sample_to_final_rad']=float(tails[-1][3])-float(points[-1][3])
    if releases and tails:
        result['release_to_quiet_ms']=float(tails[-1][1])-float(releases[-1][1])
    result['passed']=bool(result['controller_passed'] and len(points)>0 and
        abs(result.get('final_displacement_rad',0))>=.001)
    result['aborted']=any(r[0]=='PVAT_ABORT' for r in rows)
    if result['aborted']:result['passed']=False
    if interrupt is False:
        result['endpoint_error_rad']=abs(result.get('final_displacement_rad',0)-.01)
        result['passed']=result['passed'] and result['endpoint_error_rad']<=.0002
    # Completion says the bounded Lua program ended, not that failed motion
    # calls had known outcomes or that physical stopping was validated.
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--interrupt',action='store_true')
    parser.add_argument('--phase-clock',action='store_true')
    parser.add_argument('--direct-api',action='store_true',help='Call the built-in Lua motion API; 20 ms phase intervals')
    parser.add_argument('--pvt',action='store_true',help='Compare SDK PVT under the same phase clock and bounds')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    program=code(args.interrupt,args.phase_clock,args.direct_api,args.pvt)
    from app.digital_twin.docker_runtime import verify_local_docker
    admission=verify_local_docker()
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admission,service='container_loopback:3010',
            period_ms=20 if args.direct_api or not args.phase_clock else 40,
            horizon_ms=(20 if args.direct_api else 40) if args.phase_clock else 80,
            max_packets=50 if args.phase_clock and not args.direct_api else 100,
            joint_delta_rad=.01,interrupt=args.interrupt,direct_api=args.direct_api,method='move_pvt' if args.pvt else 'move_pvat')))
        return 0
    for port in (8773,8774):
        with socket.socket() as check:
            check.settimeout(.2)
            if check.connect_ex(('127.0.0.1',port))==0:raise ValueError('close_docker_web_before_probe')
    bridge='exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; while IFS= read -r -t 12 line <&3; do printf "%s\\n" "$line"; case "$line" in *PVAT_DONE*) break;; esac; done'
    try:
        response=subprocess.run(['docker','exec','-i',admission['container_id'],'bash','-c',bridge],
            input=program,text=True,encoding='utf-8',capture_output=True,timeout=15)
        result=analyze(response.stdout,interrupt=args.interrupt)
        result.update(transport_returncode=response.returncode,transport_stderr=response.stderr)
        result['passed']=result['passed'] and response.returncode==0
    except subprocess.TimeoutExpired as error:
        result=dict(passed=False,completed=False,error='transport_timeout_execution_unknown',
            raw_output=(error.stdout or b'').decode('utf-8',errors='replace'))
    result.update(admission=admission,interrupt=args.interrupt,phase_clock=args.phase_clock,direct_api=args.direct_api,
        method='move_pvt' if args.pvt else 'move_pvat')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('admission','raw_output')}))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
