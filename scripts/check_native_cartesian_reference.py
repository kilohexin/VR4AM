"""No-motion native FK audit of coordinated references and braking tails.

Only a pinned official, network=none Docker instance is admitted. No SDK,
physical selector, actuation, signal writes, startup or settings changes.
Default preview. A failed/unknown calculation is retained, never retried.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT/'scripts')]
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel,OfflineCartesianFollower,plan_cartesian_braking
from docker_pose_transport_probe import IMAGE

IMAGE_ID='sha256:87e406742de187064c994fc4e5570de1934e92643c31848ce11433201ba38694'


def validate_native_isolated(row):
    config,host,network=row['Config'],row['HostConfig'],row['NetworkSettings']
    if (row['Image']!=IMAGE_ID or config['Image'] not in (IMAGE+':latest',IMAGE_ID)
            or config['Cmd']!=['/data/startup.sh'] or not row['State']['Running']
            or host['NetworkMode']!='none' or host['Privileged'] or host.get('Devices')
            or host.get('PortBindings') or row.get('Mounts') or network['Ports']
            or set(network.get('Networks',{}))!={'none'}):
        raise ValueError('native_reference_isolation_changed')
    return dict(container_id=row['Id'],image_id=row['Image'],network='none',ports=[])


def native_admission(container):
    if os.environ.get('DOCKER_HOST'):raise ValueError('native_reference_nonlocal_docker')
    endpoint=json.loads(subprocess.run(['docker','context','inspect','--format','{{json .Endpoints.docker.Host}}'],
        capture_output=True,text=True,check=True,timeout=5).stdout)
    if not isinstance(endpoint,str) or not endpoint.startswith(('npipe:////./pipe/','unix:///')):
        raise ValueError('native_reference_nonlocal_docker')
    rows=json.loads(subprocess.run(['docker','inspect',container],capture_output=True,text=True,check=True,timeout=5).stdout)
    return validate_native_isolated(rows[0])


def build_cases():
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start=JointReference((0.,-.8,1.4,-.6,1.57,0.));base=model.pose(start.q)
    follower=OfflineCartesianFollower(model,start);cases=[];brakes=0
    def add(state):
        p=model.pose(state.q)
        cases.append(dict(q=state.q,p=p.p,r=tuple(float(v) for v in Rotation.from_quat(p.q).as_matrix().ravel())))
    for tick in range(32):
        angle=.15 if tick<16 else -.15
        goal=base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',angle)).as_quat())})
        segment,_=follower.advance(goal)
        add(segment.state(.04));add(segment.end)
        if tick in (8,16,24,31):
            plan,_=plan_cartesian_braking(model,segment.end);brakes+=1
            for item in plan.segments:
                add(item.state(item.duration/2));add(item.end)
    if not cases or len(cases)>256:raise ValueError('native_reference_case_budget')
    return cases,brakes


def program(cases):
    if not 0<len(cases)<=256:raise ValueError('native_reference_case_budget')
    entries=[]
    for case in cases:
        if (len(case['q'])!=6 or len(case['p'])!=3 or len(case['r'])!=9
                or any(type(v) not in (int,float) or not math.isfinite(v) for k in ('q','p','r') for v in case[k])):
            raise ValueError('native_reference_invalid_case')
        entries.append('{'+','.join(k+'={'+','.join(repr(float(v)) for v in case[k])+'}' for k in ('q','p','r'))+'}')
    return '''program_begin(0)
disable_auto_sync()
local function matrix(p)
 local sx,cx=math.sin(p.rx),math.cos(p.rx)
 local sy,cy=math.sin(p.ry),math.cos(p.ry)
 local sz,cz=math.sin(p.rz),math.cos(p.rz)
 return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
end
local cases={__CASES__}
local count=0
local ok,err=pcall(function()
 local tool=lebai:get_tcp()
 for _,key in ipairs({'x','y','z','rx','ry','rz'}) do assert(type(tool[key])=='number' and math.abs(tool[key])<1e-12,'nonzero_tool') end
 local before=lebai:get_kin_data()
 for i=1,6 do assert(math.abs(before.actual_joint_speed[i])<1e-9,'not_quiet') end
 for index,case in ipairs(cases) do
  local started=timestamp()
  local p=lebai:kinematics_forward(case.q)
  local elapsed=timestamp()-started
  local d=math.sqrt((p.x-case.p[1])^2+(p.y-case.p[2])^2+(p.z-case.p[3])^2)
  local actual=matrix(p);local rotation=0
  for i=1,9 do rotation=math.max(rotation,math.abs(actual[i]-case.r[i])) end
  assert(d==d and d<=1e-6 and rotation==rotation and rotation<=1e-5,'fk_mismatch')
  assert(elapsed>=0 and elapsed<80,'fk_budget')
  count=count+1;print('REFERENCE_FK_POINT',index,elapsed,d,rotation)
 end
 local after=lebai:get_kin_data()
 for i=1,6 do assert(math.abs(after.actual_joint_pose[i]-before.actual_joint_pose[i])<1e-9 and math.abs(after.actual_joint_speed[i])<1e-9,'changed_joint_state') end
end)
print('REFERENCE_FK_DONE',ok,tostring(err),count)
program_end(0)
'''.replace('__CASES__',','.join(entries))


def analyze(raw,expected):
    rows=[line.strip().split('\t') for line in raw.splitlines()]
    points=[r for r in rows if r[0]=='REFERENCE_FK_POINT'];done=[r for r in rows if r[0]=='REFERENCE_FK_DONE']
    result=dict(passed=False,motion_calls=0,signal_writes=0,native_interpolation_verified=False,raw_output=raw)
    try:
        if (len(done)!=1 or len(done[0])!=4 or done[0][1]!='true' or int(done[0][3])!=expected
                or len(points)!=expected or any(len(r)!=5 for r in points)):
            return result
        values=[tuple(float(x) for x in r[2:]) for r in points]
        if (any(not math.isfinite(v) or v<0 for row in values for v in row)
                or [int(r[1]) for r in points]!=list(range(1,expected+1))):return result
        result.update(samples=len(points),max_fk_ms=max(r[0] for r in values),
            max_position_error_m=max(r[1] for r in values),max_rotation_matrix_error=max(r[2] for r in values))
        result['passed']=result['max_fk_ms']<80 and result['max_position_error_m']<=1e-6 and result['max_rotation_matrix_error']<=1e-5
    except (ValueError,IndexError):pass
    return result


def execute(admitted,cases):
    if admitted.get('image_id')!=IMAGE_ID:raise ValueError('native_reference_image_not_pinned')
    source=program(cases)
    if native_admission(admitted['container_id'])!=admitted:raise ValueError('native_reference_isolation_changed')
    bridge=('exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; '
        'while IFS= read -r -t 20 line <&3; do printf "%s\\n" "$line"; '
        'case "$line" in *REFERENCE_FK_DONE*) break;; esac; done')
    try:
        response=subprocess.run(['docker','exec','-i',admitted['container_id'],'bash','-c',bridge],
            input=source,capture_output=True,text=True,encoding='utf-8',timeout=25)
        report=analyze(response.stdout,len(cases));report['transport_returncode']=response.returncode
        report['passed']=report['passed'] and response.returncode==0
    except subprocess.TimeoutExpired as error:
        raw=error.stdout or b''
        if isinstance(raw,bytes):raw=raw.decode('utf-8',errors='replace')
        report=dict(passed=False,read_unknown=True,motion_calls=0,signal_writes=0,native_interpolation_verified=False,raw_output=raw)
    report['program_sha256']=hashlib.sha256(source.encode()).hexdigest()
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--container',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true')
    args=p.parse_args();cases,brakes=build_cases();admitted=native_admission(args.container)
    if admitted['image_id']!=IMAGE_ID:raise ValueError('native_reference_image_not_pinned')
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,cases=len(cases),brake_plans=brakes,motion_calls=0,signal_writes=0)));return 0
    report=execute(admitted,cases);report.update(admission=admitted,brake_plans=brakes)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('raw_output','admission')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
