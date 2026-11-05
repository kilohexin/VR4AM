"""One explicitly approved no-actuator native3011 capability query.

Default preview. Fixed program, no arbitrary code/port/target option, no SDK
import, signal write, actuator, stop or retry. A separate onsite approval
must pin the program, this launcher and tool report before a connection.
"""
import argparse
from datetime import datetime,timezone,timedelta
import hashlib
import json
from pathlib import Path
import socket
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'scripts')]
from scripts.preview_real_pvat_capability import PROGRAM,preview
from scripts.local_rpc_connections import read_rpc_connections
from app.control.cartesian_tool_binding import load_tool_report

TARGET=('172.16.2.162',3011)
CONFIRM='ONE_NATIVE_PVAT_CAPABILITY_QUERY_NO_ROBOT_WRITES'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(authorization):
    a=json.loads(Path(authorization).read_text(encoding='utf-8'))
    expiry=datetime.fromisoformat(a['expires_utc']);now=datetime.now(timezone.utc)
    if (a.get('approved') is not True or a.get('confirmation')!=CONFIRM
            or a.get('target')!=list(TARGET) or expiry.utcoffset()!=timedelta(0)
            or not now<expiry<=now+timedelta(minutes=10)
            or a.get('program_sha256')!=preview()['source_sha256']
            or a.get('launcher_sha256')!=sha(__file__)
            or a.get('tool_report_sha256')!=sha(a['tool_report'])
            or a.get('scope_sha256')!=sha(a['scope'])):
        raise ValueError('capability_scope_expired_or_changed')
    return a


def analyze(rows):
    done=[r for r in rows if r and r[0]=='PVAT_CAP_DONE']
    clock=[r for r in rows if r and r[0]=='PVAT_CAP_CLOCK']
    tools=[r for r in rows if r and r[0]=='PVAT_CAP_TOOL']
    fk=[r for r in rows if r and r[0]=='PVAT_CAP_FK']
    apis=[r for r in rows if r and r[0]=='PVAT_CAP_APIS']
    try:
        valid=(done==[['PVAT_CAP_DONE','true','nil']] and len(clock)==5
            and [int(r[1]) for r in clock]==list(range(1,6))
            and all(float(b[2])>=float(a[2]) for a,b in zip(clock,clock[1:]))
            and len(tools)==len(fk)==len(apis)==1 and len(tools[0])==len(fk[0])==7
            and apis[0][1:]==['function']*4)
        import math
        valid=valid and all(math.isfinite(float(v)) for r in tools+fk for v in r[1:])
        valid=valid and all(math.isfinite(float(r[2])) and float(r[2])>=0 for r in clock)
    except (ValueError,IndexError):valid=False
    return dict(complete=bool(valid),native_done=done,robot_writes=0,signal_writes=0,
        motion_admitted=False,monotonic_clock_guaranteed=False,hardware_motion_verified=False)


def completion_known(rows):
    done=[r for r in rows if r and r[0]=='PVAT_CAP_DONE']
    return bool(len(done)==1 and len(done[0])==3 and done[0][1] in ('true','false')
        and done[0][2] and (done[0][1]=='false' or done[0][2]=='nil'))


def execute(authorization,output,*,connector=socket.create_connection):
    a=validate(authorization);output=Path(output)
    expected_tool=load_tool_report(a['tool_report'],a['tool_report_sha256'])
    if read_rpc_connections()!=0:raise ValueError('existing_sdk_connection')
    # Consume the authorization bytes, not a caller-chosen output path. A
    # copied approval or changed --output must never grant another attempt.
    claims=ROOT/'.tmp/pvat-capability-claims';claims.mkdir(parents=True,exist_ok=True)
    with (claims/(sha(authorization)+'.claim')).open('x',encoding='utf-8') as claim:
        json.dump(dict(authorization_sha256=sha(authorization),output=str(output.resolve())),claim)
    output.mkdir(parents=True,exist_ok=False)  # Consumed before even one connect.
    (output/'authorization.json').write_bytes(Path(authorization).read_bytes())
    report=dict(complete=False,connect_attempted=False,program_submitted=False,rows=[],robot_writes=0,
        signal_writes=0,automatic_retry=False,program_sha256=preview()['source_sha256'])
    try:
        validate(authorization);deadline=time.perf_counter()+15
        report['connect_attempted']=True
        with connector(TARGET,timeout=5) as connection:
            validate(authorization)
            if time.perf_counter()>=deadline:raise TimeoutError('capability_launch_expired')
            # Unknown submission is retained even if sendall fails partially.
            report['program_submitted']=True
            connection.sendall(PROGRAM.encode());buffer=b'';total=0;finished=False
            while not finished:
                remaining=deadline-time.perf_counter()
                if remaining<=0:raise TimeoutError('capability_query_expired')
                connection.settimeout(min(1.,remaining));chunk=connection.recv(4096)
                if not chunk:break
                total+=len(chunk)
                if total>65536:raise ValueError('capability_response_budget')
                buffer+=chunk
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1)
                    row=line.decode('utf-8',errors='strict').strip().split('\t')
                    report['rows'].append(row)
                    if len(report['rows'])>128:raise ValueError('capability_response_budget')
                    if completion_known(report['rows']):finished=True
        report.update(analyze(report['rows']))
        if report['complete']:
            from scipy.spatial.transform import Rotation
            import math
            from app.robots.lebai_codec import pose_from_lebai
            row=next(r for r in report['rows'] if r[0]=='PVAT_CAP_TOOL')
            actual=pose_from_lebai(dict(zip(('x','y','z','rx','ry','rz'),map(float,row[1:]))))
            matches=(math.dist(actual.p,expected_tool.translation_m)<1e-9 and
                (Rotation.from_quat(actual.q).inv()*Rotation.from_quat(expected_tool.rotation_xyzw)).magnitude()<1e-6)
            report['tool_matches_saved_readonly']=bool(matches)
            report['complete']=bool(matches)
    except Exception as error:report['error_type']=type(error).__name__
    finally:
        report['unconfirmed_program_completion']=bool(report['program_submitted'] and not completion_known(report['rows']))
        (output/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute-once',action='store_true')
    p.add_argument('--authorization',type=Path);p.add_argument('--output',type=Path);args=p.parse_args(argv)
    if not args.execute_once:
        print(json.dumps(dict(**preview(),launcher_sha256=sha(__file__)),indent=2));return 0
    if not args.authorization or not args.output:p.error('approved authorization and unique output required')
    result=execute(args.authorization,args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}));return 0 if result['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
