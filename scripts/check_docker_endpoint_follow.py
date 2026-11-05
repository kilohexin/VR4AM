"""Finite empty towardj evaluation in verified isolated official Docker only.

No host IK/interpolation, physical address, API fallback or product unlocking.
Three changed endpoints, final arrival and fixed hold; no normal stop request.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'scripts')]
from scripts.check_native_cartesian_reference import native_admission
from scripts.check_native_cartesian_motion import preparation_program,transport,finished

PROGRAM=r'''program_begin(0)
disable_auto_sync()
local events={};local entered=0;local returned=0;local pending=false
local failure=nil;local finish=false;local previous=nil;local count=0;local samples=0
local function clock()
 local n=timestamp();assert(type(n)=='number' and n==n and (not previous or n>=previous),'clock')
 previous=n;return n
end
local function record(...) events[#events+1]={...};assert(#events<600,'record_budget') end
local function finite(x) assert(type(x)=='number' and x==x and math.abs(x)<1000000,'nonfinite');return x end
local base=nil;local final=nil;local epoch=clock();local last_read=nil;local stable=nil;local stable_p=nil
local last_input=nil
local function angle(a,b)
 local function r(p)
  local sx,cx=math.sin(p.rx),math.cos(p.rx);local sy,cy=math.sin(p.ry),math.cos(p.ry)
  local sz,cz=math.sin(p.rz),math.cos(p.rz)
  return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
 end
 local ar,br=r(a),r(b);local trace=0;for i=1,9 do trace=trace+ar[i]*br[i] end
 return math.acos(math.max(-1,math.min(1,(trace-1)/2)))
end
local function dist(a,b) return math.sqrt((a.x-b.x)^2+(a.y-b.y)^2+(a.z-b.z)^2) end
local function read()
 count=count+1;assert(count<=400,'read_budget')
 local capture=clock();local state=lebai:get_robot_state();local k=lebai:get_kin_data();local now=clock()
 assert(now-capture<200 and (not last_read or now-last_read<200),'feedback_gap');last_read=capture
 assert(state=='IDLE' or state=='MOVING','state')
 local p=k.actual_tcp_pose
 for _,field in ipairs({'x','y','z','rx','ry','rz'}) do finite(p[field]) end
 for i=1,6 do
  finite(k.actual_joint_pose[i]);finite(k.actual_joint_speed[i])
  assert(math.abs(k.actual_joint_pose[i])<3 and math.abs(k.actual_joint_speed[i])<=.3,'joint_bounds_or_speed')
 end
 if base then
  for _,axis in ipairs({'x','y','z'}) do assert(math.abs(p[axis]-base[axis])<=.03,'workspace') end
  assert(angle(p,base)<=.01,'orientation')
 end
 if final then
  samples=samples+1
  record('END_SAMPLE',capture-epoch,now-epoch,dist(p,final),angle(p,final),math.max(math.abs(k.actual_joint_speed[1]),math.abs(k.actual_joint_speed[2]),math.abs(k.actual_joint_speed[3]),math.abs(k.actual_joint_speed[4]),math.abs(k.actual_joint_speed[5]),math.abs(k.actual_joint_speed[6])),p.x,p.y,p.z)
 end
 return state,k,p,capture
end
local ok,err=pcall(function()
 local tool=lebai:get_tcp();for _,key in ipairs({'x','y','z','rx','ry','rz'}) do assert(finite(tool[key])==0,'requires_zero_virtual_tool') end
 assert(lebai:get_velocity_factor()==100,'velocity_factor')
 local state,k,p=read();assert(state=='IDLE','requires_idle');base=p
 for i=1,6 do assert(k.actual_joint_speed[i]==0 and k.actual_joint_acc[i]==0,'requires_quiet') end
 local function target(x,y,z)
  return {x=base.x+x,y=base.y+y,z=base.z+z,rx=base.rx,ry=base.ry,rz=base.rz}
 end
 local targets={target(.01,0,0),target(.02,0,.005),target(.005,.01,.005)};final=targets[3]
 local function worker(fn)
  return async_task(function()
   disable_auto_sync()
   local good,reason=pcall(fn);if not good then failure=failure or reason;finish=true end
  end)
 end
 local monitor=worker(function()
  while not finish do
   local s,feedback,tcp,capture=read()
   if returned==3 then
    assert(clock()-last_input<7000,'arrival_deadline')
    local quiet=s=='IDLE' and dist(tcp,final)<=.001 and angle(tcp,final)<=.0035
    for i=1,6 do quiet=quiet and math.abs(feedback.actual_joint_speed[i])<=.002 end
    if quiet then
     if stable_p and dist(tcp,stable_p)>.0001 then stable=nil;stable_p=nil end
     if not stable then
      stable=capture;stable_p=tcp;record('END_STABLE',capture-epoch,tcp.x,tcp.y,tcp.z,clock()-epoch)
     end
     if capture-stable>=700 then record('END_HELD',capture-epoch,dist(tcp,final),samples,clock()-epoch);finish=true;return end
    else stable=nil;stable_p=nil end
   end
   assert(clock()-epoch<10000,'session_deadline');rt_sleep(20)
  end
 end)
 local sender=worker(function()
  for index,item in ipairs(targets) do
   if index>1 then rt_sleep(200) end
   if finish then return end
   local before=clock();pending=true;entered=index;record('END_ENTER',index,before-epoch)
   local good,id=pcall(function() return lebai:towardj(item,.2,.1,0,0) end)
   pending=false
   if not good then error('towardj_write_unknown') end
   assert(type(id)=='number' and id==math.floor(id) and id>=0,'towardj_receipt')
   returned=returned+1;record('END_RETURN',index,clock()-epoch,id);last_input=clock()
   assert(last_input-before<200,'reply_deadline')
  end
 end)
 wait_all(sender,monitor);assert(not failure,tostring(failure));assert(returned==3,'incomplete')
end)
if not ok and entered>0 and not pending then
 record('END_STOP_ENTER',entered)
 local stopped=pcall(function() lebai:stop_move() end)
 record('END_STOP_RETURN',stopped and 'returned' or 'unknown')
end
for _,row in ipairs(events) do
 local out={};for i,v in ipairs(row) do out[i]=type(v)=='number' and string.format('%.17g',v) or tostring(v) end
 print(table.concat(out,'\t'))
end
print('END_DONE',ok,tostring(err),returned)
program_end(0)
'''


def analyze(response):
    result=dict(passed=False,hardware_verified=False,motion_unknown=True,stop_unverified=True)
    try:
        rows=[s.strip().split('\t') for s in response['raw_output'].splitlines()]
        pick=lambda name:[r for r in rows if r[0]==name]
        if response.get('transport_unknown') or response.get('transport_returncode')!=0:return result
        if pick('END_DONE')!=[['END_DONE','true','nil','3']] or rows[-1]!=['END_DONE','true','nil','3'] or any(r[0].startswith('END_STOP') for r in rows):return result
        entries=pick('END_ENTER');returns=pick('END_RETURN');samples=pick('END_SAMPLE');held=pick('END_HELD')
        if len(entries)!=3 or len(returns)!=3 or len(held)!=1 or len(held[0])!=5 or len(samples)<20:return result
        for i,(a,b) in enumerate(zip(entries,returns),1):
            if len(a)!=3 or len(b)!=4 or int(a[1])!=i or int(b[1])!=i or not 0<=int(b[2])-int(a[2])<200 or int(b[3])<0:return result
        data=[];pending=None;entered=0;returned=0;previous_time=-1;stable=None;end=None;held_seen=False
        for row in rows[:-1]:
            kind=row[0]
            if held_seen:return result
            if kind=='END_ENTER':
                if pending is not None or int(row[1])!=entered+1:return result
                entered+=1;pending=entered;stamp=float(row[2])
            elif kind=='END_RETURN':
                if pending!=int(row[1]):return result
                returned+=1;pending=None;stamp=float(row[2])
            elif kind=='END_SAMPLE':
                v=tuple(map(float,row[1:]))
                if len(v)!=8 or any(not math.isfinite(x) for x in v):return result
                capture,stamp,error,rotation,speed,*position=v
                if not 0<=capture<=stamp or stamp-capture>=200 or not 0<=error<=.1 or not 0<=rotation<=.01 or not 0<=speed<=.3:return result
                if data and (not 0<capture-data[-1][0]<200 or stamp-data[-1][0]>=200):return result
                data.append(v)
            elif kind=='END_STABLE':
                if len(row)!=6 or returned!=3 or not data:return result
                values=tuple(map(float,row[1:]))
                if any(not math.isfinite(v) for v in values):return result
                capture,*rest=values;stamp=rest[-1];xyz=tuple(rest[:3])
                if capture!=data[-1][0] or xyz!=tuple(data[-1][5:]):return result
                stable=(capture,xyz)
            elif kind=='END_HELD':
                end=tuple(map(float,row[1:]));stamp=end[-1]
                if any(not math.isfinite(v) for v in end) or pending is not None or returned!=3 or stable is None:return result
                if end[2]!=len(data) or end[0]!=data[-1][0] or end[0]-stable[0]<700:return result
                tail=[r for r in data if r[0]>=stable[0]]
                if any(r[2]>.001 or r[3]>.0035 or r[4]>.002 or math.dist(r[5:],stable[1])>.0001 for r in tail):return result
                if stable[0]<float(returns[-1][2]) or end[0]-float(returns[-1][2])>=7000:return result
                if abs(end[1]-data[-1][2])>1e-9:return result
                held_seen=True
            else:return result
            if not math.isfinite(stamp) or not previous_time<=stamp<10000:return result
            previous_time=stamp
        if not held_seen:return result
        result.update(passed=True,motion_unknown=False,stop_unverified=False,writes=3,samples=len(data),
            final_error_m=end[1],native_interpolation_verified=False)
    except (ValueError,TypeError,IndexError,OverflowError):result['invalid_evidence']=True
    return result


def run(container,output):
    admission=native_admission(container);out=Path(output);out.mkdir(parents=True,exist_ok=False)
    report=dict(passed=False,hardware_verified=False)
    try:
        prep=transport(admission,preparation_program(),'FINITE_PREP_DONE');report['preparation']=prep
        if not finished(prep,'FINITE_PREP_DONE') or 'FINITE_ABORT' in prep['raw_output']:return report
        response=transport(admission,PROGRAM,'END_DONE');report['motion']=response
        report.update(analyze(response));report['program_sha256']=hashlib.sha256(PROGRAM.encode()).hexdigest()
        return report
    finally:(out/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run:
        print(json.dumps(dict(admission=native_admission(a.container),execute=False,towardj_max=3,real_selector=False)));return 0
    r=run(a.container,a.output);print(json.dumps({k:v for k,v in r.items() if k not in ('preparation','motion')}));return 0 if r['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
