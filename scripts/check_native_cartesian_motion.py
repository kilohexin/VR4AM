"""One finite fixed-position pitch-reversal trial in pinned isolated Docker.

Preview by default. No host SDK or real-address option. One startup if needed,
one virtual preparation movej, <=64 prevalidated PVAT points including a braking
tail, and at most one exception stop across the two stages. Never retries.
This is a diagnostic, not the product pose-follow entry or hardware admission.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT/'scripts')]
from check_native_cartesian_reference import native_admission
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor

COMMON='''
local entered=false
local function tool_check()
 local t=lebai:get_tcp()
 for _,key in ipairs({'x','y','z','rx','ry','rz'}) do assert(type(t[key])=='number' and math.abs(t[key])<1e-12,'tool_changed') end
end
local function read(cap)
 local state=lebai:get_robot_state();local k=lebai:get_kin_data()
 assert(state=='IDLE' or state=='MOVING','robot_state')
 for i=1,6 do
  assert(k.actual_joint_pose[i]==k.actual_joint_pose[i] and math.abs(k.actual_joint_pose[i])<=3,'joint_position')
  assert(k.actual_joint_speed[i]==k.actual_joint_speed[i] and math.abs(k.actual_joint_speed[i])<=cap,'joint_speed')
 end
 return state,k
end
local function quiet(milliseconds,stable_ms,cap,observe)
 local deadline=timestamp()+milliseconds;local since=nil;local previous=timestamp()
 while timestamp()<deadline do
  local state,k=read(cap);local now=timestamp();local stopped=state=='IDLE'
  if observe then observe(k,now) end
  for i=1,6 do stopped=stopped and math.abs(k.actual_joint_speed[i])<=.000001 end
  if now<previous or now-previous>=100 then since=nil end;previous=now
  if stopped then
   since=since or now
   if now-since>=stable_ms then return k end
  else since=nil end
  rt_sleep(20)
 end
 error('quiet_unverified')
end
local function abort()
 local started=timestamp();local returned=pcall(function() lebai:stop_move() end)
 returned=returned and timestamp()-started<1000
 local observed=pcall(function() quiet(2500,200,.6,nil) end)
 print('FINITE_ABORT',returned,observed)
end
'''


def preparation_program():
    return '''program_begin(0)
disable_auto_sync()
'''+COMMON+'''
local ok,err=pcall(function()
 tool_check()
 local state=lebai:get_robot_state()
 if state=='STOP' or state=='ROBOT_ON' then
  lebai:start_sys();local deadline=timestamp()+8000
  repeat
   state=lebai:get_robot_state();if state=='IDLE' then break end
   assert(state=='STOP' or state=='ROBOT_ON' or state=='STARTING' or state=='BOOTING','startup_state')
   rt_sleep(50)
  until timestamp()>=deadline
 end
 assert(state=='IDLE','requires_idle')
 if lebai:get_velocity_factor()~=100 then lebai:set_velocity_factor(100) end
 assert(lebai:get_velocity_factor()==100,'velocity_factor')
 local ready={0,-.8,1.4,-.6,1.57,0}
 entered=true;print('FINITE_PREP_ENTER')
 lebai:movej(ready,.2,.2,0,0)
 print('FINITE_PREP_RETURNED')
 local k=quiet(20000,500,.6,nil)
 for i=1,6 do assert(math.abs(k.actual_joint_pose[i]-ready[i])<.002,'prepare_error') end
 local values={}
 for i=1,6 do values[i]=string.format('%.17g',k.actual_joint_pose[i]) end
 print('FINITE_ORIGIN',table.concat(values,'\t'))
end)
if not ok and entered then abort() end
print('FINITE_PREP_DONE',ok,tostring(err))
program_end(0)
'''


def build_trajectory(origin):
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start=JointReference(tuple(origin));base=model.pose(start.q)
    supervisor=OfflineReferenceSupervisor(model,start);segments=[]
    for tick in range(46):
        angle=.15 if tick<16 else -.15
        target=base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',angle)).as_quat())})
        packet=supervisor.advance(target)
        if packet.mode!='following':raise ValueError('finite_trial_reference_rejected')
        segments.append(packet.segment)
    while supervisor.phase!='ready':segments.append(supervisor.finish().segment)
    if len(segments)>64 or any(s.duration!=.08 for s in segments):raise ValueError('finite_trial_point_budget')
    if any(a.end!=b.start for a,b in zip(segments,segments[1:])):raise ValueError('finite_trial_chain')
    if max(abs(x) for x in segments[-1].end.v+segments[-1].end.a)>1e-8:raise ValueError('finite_trial_not_terminal')
    return model,base,segments


def vec(values):
    if len(values)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        raise ValueError('finite_trial_bad_vector')
    return '{'+','.join(repr(float(v)) for v in values)+'}'


def motion_program(origin,base,segments,*,cadence_probe=False):
    if type(cadence_probe) is not bool:raise ValueError('finite_trial_cadence_configuration')
    if not 0<len(segments)<=64:raise ValueError('finite_trial_point_budget')
    points=','.join('{p='+vec(s.end.q)+',v='+vec(s.end.v)+',a='+vec(s.end.a)+'}' for s in segments)
    source='''program_begin(0)
disable_auto_sync()
'''+COMMON+'''
local origin=__ORIGIN__
local final=__FINAL__
local base={x=__X__,y=__Y__,z=__Z__}
local base_rotation={__ROTATION__}
local points={__POINTS__}
local count=0;local samples=0;local failure=nil;local finish=false
local max_drift=0;local start=timestamp()
local max_pitch=-math.huge;local min_pitch=math.huge;local last_pitch=0;local previous_sample=nil
local function rotation(p)
 local sx,cx=math.sin(p.rx),math.cos(p.rx);local sy,cy=math.sin(p.ry),math.cos(p.ry)
 local sz,cz=math.sin(p.rz),math.cos(p.rz)
 return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
end
local function observe(k,now)
 local p=k.actual_tcp_pose
 local drift=math.sqrt((p.x-base.x)^2+(p.y-base.y)^2+(p.z-base.z)^2)
 max_drift=math.max(max_drift,drift);samples=samples+1
 local r=rotation(p);local relative={}
 for row=1,3 do for column=1,3 do
  local value=0;for axis=1,3 do value=value+base_rotation[(axis-1)*3+row]*r[(axis-1)*3+column] end
  relative[(row-1)*3+column]=value
 end end
 last_pitch=math.atan2(relative[8]-relative[6],relative[5]+relative[9]);max_pitch=math.max(max_pitch,last_pitch);min_pitch=math.min(min_pitch,last_pitch)
 assert(math.acos(math.max(-1,math.min(1,(relative[1]+relative[5]+relative[9]-1)/2)))<=.20,'orientation_region')
 print('FINITE_SAMPLE',now-start,drift,p.z,last_pitch)
 assert(drift==drift and drift<=.001,'tcp_path_deviation')
 assert(not previous_sample or (now>=previous_sample and now-previous_sample<100),'feedback_gap');previous_sample=now
end
local ok,err=pcall(function()
 tool_check();assert(lebai:get_velocity_factor()==100,'velocity_factor_changed')
 local state,k=read(.3);assert(state=='IDLE','requires_idle')
 for i=1,6 do
  assert(math.abs(k.actual_joint_pose[i]-origin[i])<=.000001,'origin_changed')
  assert(math.abs(k.actual_joint_speed[i])<=.000001,'origin_not_quiet')
 end
 observe(k,timestamp())
 local function worker(fn)
  return async_task(function()
   local passed,reason=pcall(fn)
   if not passed then failure=failure or reason;finish=true end
  end)
 end
 local monitor=worker(function()
  while not finish do local _,f=read(.3);observe(f,timestamp());rt_sleep(20) end
 end)
 local sender=worker(function()
  local epoch=timestamp();local previous=nil
  for index,item in ipairs(points) do
   if finish then return end
   local due=epoch+math.max(0,(index-2)*70)
   if timestamp()<due then rt_sleep(due-timestamp()) end
   if finish then return end
   local now=timestamp();assert(not previous or now-previous<80,'pvat_cadence')
   entered=true;previous=now;print('FINITE_PVAT_ENTER',index,now-start)
   lebai:move_pvat(item.p,item.v,item.a,.08);count=count+1
  end
  finish=true
 end)
 wait_all(monitor,sender);assert(not failure,tostring(failure))
 local last=quiet(2500,200,.3,observe)
 for i=1,6 do assert(math.abs(last.actual_joint_pose[i]-final[i])<.002,'endpoint_error') end
 assert(max_pitch>.02 and min_pitch<-.005 and max_pitch-last_pitch>.01,'pitch_reversal_missing')
 print('FINITE_QUIET',timestamp()-start,max_drift,samples,max_pitch,last_pitch,min_pitch)
end)
if not ok and entered then abort() end
print('FINITE_MOTION_DONE',ok,tostring(err),count)
program_end(0)
'''
    for token,value in {'__ORIGIN__':vec(origin),'__FINAL__':vec(segments[-1].end.q),
        '__X__':repr(base.p[0]),'__Y__':repr(base.p[1]),'__Z__':repr(base.p[2]),'__POINTS__':points,
        '__ROTATION__':','.join(repr(float(x)) for x in Rotation.from_quat(base.q).as_matrix().ravel())}.items():source=source.replace(token,value)
    if cadence_probe:
        if source.count('(index-2)*70')!=1 or source.count('now-previous<80')!=1:
            raise ValueError('finite_trial_cadence_template')
        source=source.replace('(index-2)*70','(index-2)*80').replace('now-previous<80','now-previous<100')
        marker=" print('FINITE_SAMPLE',now-start,drift,p.z,last_pitch)"
        if source.count(marker)!=1:raise ValueError('finite_trial_cadence_template')
        source=source.replace(marker,marker+'''
 local values={}
 for _,field in ipairs({'actual_joint_pose','actual_joint_speed','target_joint_pose'}) do
  for i=1,6 do
   local value=k[field][i];assert(type(value)=='number' and value==value and math.abs(value)<1000000,'kinematics_nonfinite')
   values[#values+1]=string.format('%.17g',value)
  end
 end
 print('FINITE_KIN',now-start,table.concat(values,'\\t'))''')
    return source


def transport(admitted,source,marker):
    if native_admission(admitted['container_id'])!=admitted:raise ValueError('finite_trial_identity_changed')
    bridge=('exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; '
        'while IFS= read -r -t 45 line <&3; do printf "%s\\n" "$line"; '
        f'case "$line" in *{marker}*) break;; esac; done')
    try:
        response=subprocess.run(['docker','exec','-i',admitted['container_id'],'bash','-c',bridge],input=source,
            capture_output=True,text=True,encoding='utf-8',timeout=60)
        return dict(raw_output=response.stdout,transport_returncode=response.returncode,
            program_sha256=hashlib.sha256(source.encode()).hexdigest())
    except subprocess.TimeoutExpired as error:
        raw=error.stdout or b''
        if isinstance(raw,bytes):raw=raw.decode('utf-8',errors='replace')
        return dict(raw_output=raw,transport_unknown=True,stop_unverified=True,
            program_sha256=hashlib.sha256(source.encode()).hexdigest())


def finished(result,marker):
    rows=[line.strip().split('\t') for line in result['raw_output'].splitlines()]
    done=[r for r in rows if r[0]==marker]
    return (not result.get('transport_unknown') and result.get('transport_returncode')==0
        and len(done)==1 and len(done[0])>=3 and done[0][1]=='true')


def analyze_motion(result,expected_points,*,cadence_probe=False):
    """Require complete, finite sampled evidence; Lua success alone is insufficient."""
    report=dict(passed=False,unknown_motion_write=True,stop_unverified=True)
    rows=[line.strip().split('\t') for line in result['raw_output'].splitlines()]
    done=[r for r in rows if r[0]=='FINITE_MOTION_DONE']
    entries=[r for r in rows if r[0]=='FINITE_PVAT_ENTER']
    quiet=[r for r in rows if r[0]=='FINITE_QUIET']
    samples=[r for r in rows if r[0]=='FINITE_SAMPLE']
    aborts=[r for r in rows if r[0]=='FINITE_ABORT']
    try:
        returned=int(done[0][3]) if len(done)==1 and len(done[0])==4 else 0
        if not 0<=returned<=expected_points:raise ValueError('invalid_return_count')
        report['unknown_motion_write']=len(entries)>returned
        report['stop_unverified']=bool(result.get('transport_unknown') or result.get('stop_unverified')
            or result.get('transport_returncode')!=0 or len(entries)>returned
            or len(aborts)>1 or any(r[1:]!=['true','true'] for r in aborts))
        report['exception_stops']=len(aborts)
        if not finished(result,'FINITE_MOTION_DONE') or aborts or len(quiet)!=1 or len(quiet[0])!=7:
            return report
        if not 0<expected_points<=64 or returned!=expected_points or len(entries)!=expected_points:
            return report
        if any(len(r)!=3 or int(r[1])!=i for i,r in enumerate(entries,1)):return report
        if len(samples)<20 or any(len(r)!=5 for r in samples):return report
        data=[tuple(float(v) for v in r[1:]) for r in samples]
        summary=tuple(float(v) for v in quiet[0][1:])
        entry_times=[float(r[2]) for r in entries]
        if any(not math.isfinite(v) for row in data+[summary,tuple(entry_times)] for v in row):return report
        elapsed,drift,count,peak,final,low=summary
        if (count!=len(samples) or not 0<=drift<=.001 or elapsed<200 or elapsed>10000
            or any(not 0<=r[1]<=.001 or abs(r[3])>.20 for r in data)
            or any(not 0<=b[0]-a[0]<100 for a,b in zip(data,data[1:]))
            or any(not 0<=b-a<(100 if cadence_probe else 80) for a,b in zip(entry_times,entry_times[1:]))
            or any(not 0<=t<=elapsed for t in entry_times)
            or any(not 0<=r[0]<=elapsed for r in data)
            or abs(drift-max(r[1] for r in data))>1e-8
            or abs(peak-max(r[3] for r in data))>1e-6
            or abs(low-min(r[3] for r in data))>1e-6
            or abs(final-data[-1][3])>1e-6
            or not (peak>.02 and low<-.005 and peak-final>.01)):
            return report
        report.update(passed=not report['stop_unverified'],max_sampled_tcp_drift_m=drift,
            feedback_samples=len(samples),max_pitch_rad=peak,final_pitch_rad=final,min_pitch_rad=low)
        if cadence_probe:
            kin=[r for r in rows if r[0]=='FINITE_KIN']
            values=[tuple(float(v) for v in r[1:]) for r in kin]
            leads=[index*80-(stamp-entry_times[0]) for index,stamp in enumerate(entry_times,1)]
            report.update(max_send_gap_ms=max((b-a for a,b in zip(entry_times,entry_times[1:])),default=0),
                strict_send_interval_observed=all(b-a<80 for a,b in zip(entry_times,entry_times[1:])),
                max_modeled_queue_lead_ms=max(leads),min_modeled_queue_lead_ms=min(leads))
            report['passed']=bool(report['passed'] and len(values)==len(data)
                and all(len(r)==19 and all(math.isfinite(v) for v in r) for r in values)
                and [r[0] for r in values]==[r[0] for r in data]
                and min(leads)>=0 and max(leads)<=160)
    except (ValueError,IndexError,OverflowError):
        report['evidence_invalid']=True
    return report


def execute(admitted,*,cadence_probe=False):
    first=transport(admitted,preparation_program(),'FINITE_PREP_DONE')
    return complete_prepared_trial(admitted,first,cadence_probe=cadence_probe)


def complete_prepared_trial(admitted,first,*,cadence_probe=False):
    """Finish the same finite scope without repeating a returned preparation.

    The motion program independently requires the exact quiet captured origin.
    This helper never starts or prepares the virtual robot.
    """
    report=dict(passed=False,hardware_verified=False,product_admission_complete=False,
        native_interpolation_verified=False,realtime_verified=False,
        simulated_motion_only=True,precomputed_trajectory=True,cadence_probe=cadence_probe,admission=admitted)
    report['preparation']=first
    prep_entered='FINITE_PREP_ENTER' in first['raw_output'];prep_returned='FINITE_PREP_RETURNED' in first['raw_output']
    prep_aborts=[line.split('\t') for line in first['raw_output'].splitlines() if line.startswith('FINITE_ABORT\t')]
    report['stop_unverified']=bool(first.get('transport_unknown') or first.get('transport_returncode')!=0
        or (prep_entered and not prep_returned) or len(prep_aborts)>1
        or any(r[1:]!=['true','true'] for r in prep_aborts))
    report['exception_stops']=len(prep_aborts)
    if (not finished(first,'FINITE_PREP_DONE') or report['stop_unverified'] or prep_aborts
        or not prep_entered or not prep_returned):return report
    rows=[line.strip().split('\t') for line in first['raw_output'].splitlines() if line.startswith('FINITE_ORIGIN\t')]
    if len(rows)!=1 or len(rows[0])!=7:report['error']='finite_trial_origin_missing';return report
    try:
        origin=tuple(float(v) for v in rows[0][1:]);vec(origin)
        _,base,segments=build_trajectory(origin)
    except ValueError as error:
        report['error']=str(error);return report
    second=transport(admitted,motion_program(origin,base,segments,cadence_probe=cadence_probe),'FINITE_MOTION_DONE');report['motion']=second
    report.update(planned_points=len(segments),**analyze_motion(second,len(segments),cadence_probe=cadence_probe))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--run',action='store_true')
    p.add_argument('--cadence-probe',action='store_true');args=p.parse_args()
    admitted=native_admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,startup_max=1,velocity_factor_writes_max=1,
            prepare_movej_max=1,pvat_points_max=64,exception_stops_max=1,real_selector=False,cadence_probe=args.cadence_probe)));return 0
    report=execute(admitted,cadence_probe=args.cadence_probe);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('preparation','motion','admission')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
