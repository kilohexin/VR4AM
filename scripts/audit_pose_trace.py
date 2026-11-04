"""Offline observation audit. No SDK, Docker, sockets or motion API.

Finding no violation does not verify motion safety. This measures recorded
Cartesian error and drops under stable requested height for regression use.
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path


def analyze(rows,*,scale=1.,span=.3):
    kin=sorted((r for r in rows if r.get('kind')=='robot_kinematics'),key=lambda r:r['server_mono_ns'])
    kt=[r['server_mono_ns'] for r in kin]
    groups=[];group=[]
    for r in rows:
        if r.get('kind')!='finite_position_input':continue
        if r.get('event')=='held' and r.get('hand_pose'):group.append(r)
        elif group:groups.append(group);group=[]
    if group:groups.append(group)
    samples=[];windows=[];max_error=0.;max_descent=0.
    for hand in groups:
        ht=[r['server_mono_ns'] for r in hand];anchor=bisect.bisect_right(kt,ht[0])-1
        if anchor<0:continue
        z0=kin[anchor]['actual_tcp']['p'][2];hy0=hand[0]['hand_pose']['p'][1]
        current=[]
        for k in kin:
            t=k['server_mono_ns']
            if not ht[0]<=t<=ht[-1]:continue
            h=hand[bisect.bisect_right(ht,t)-1]
            target=z0+max(-span,min(span,(h['hand_pose']['p'][1]-hy0)*scale))
            actual=k['actual_tcp']['p'][2];max_error=max(max_error,target-actual)
            if current:
                dt=(t-current[-1][0])/1e9
                if dt>0:max_descent=max(max_descent,(current[-1][2]-actual)/dt)
            low=high=target
            for old in reversed(current):
                dt=(t-old[0])/1e9
                if dt>5:break
                low=min(low,old[1]);high=max(high,old[1])
                if high-low>.01:break
                drop=old[2]-actual
                if dt>=.1 and drop>.03 and actual<low-.02:
                    windows.append(dict(start_ns=old[0],end_ns=t,drop_m=drop,goal_span_m=high-low,undershoot_m=low-actual))
            current.append((t,target,actual))
        samples.extend(current)
    return dict(sdk_connections=0,motion_writes=0,motion_safety_verified=False,
        samples=len(samples),max_downward_error_m=max_error,max_descent_mps=max_descent,
        stable_height_drop_observed=bool(windows),worst_windows=sorted(windows,key=lambda w:w['drop_m'],reverse=True)[:5])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();data=args.input.read_bytes()
    report=analyze([json.loads(line) for line in data.decode('utf-8').splitlines()])
    report['source_sha256']=hashlib.sha256(data).hexdigest()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
