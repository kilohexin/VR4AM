"""Read-only sampled curve witnesses from a frozen actual-Docker report.

Finds an explicit point on one of the <=8 recently sent assumed Hermite
curves. A found witness bounds that SAMPLE's distance; this is not a proof
of native interpolation, an accurate clock alignment, or unsampled motion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend')]
import numpy as np
from scipy.spatial.transform import Rotation
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference,PvatSegment,_evaluate
from app.control.compiled_reference_geometry import CompiledReferenceModel
from app.control.cartesian_tool_binding import ZERO_TOOL,checked_tool,tool_digest,tool_arguments


def witness(coefficients,target):
    r=coefficients.copy();r[0]-=target
    derivative=np.zeros(10)
    for axis in range(6):
        d=np.arange(1,6)*r[1:,axis]
        terms=np.polynomial.polynomial.polymul(r[:,axis],d)
        derivative[:len(terms)]+=terms
    candidates=[0.,1.]
    for root in np.polynomial.polynomial.polyroots(derivative):
        if abs(root.imag)<1e-8 and 0<=root.real<=1:candidates.append(float(root.real))
    u=min(candidates,key=lambda x:np.linalg.norm(_evaluate(coefficients,x)-target))
    return u,_evaluate(coefficients,u)


def analyze(report,tool=ZERO_TOOL):
    checked_tool(tool)
    recorded=[v for v in (report.get('tool_sha256'),report.get('statistics',{}).get('tool_sha256')) if v is not None]
    if (any(v!=tool_digest(tool) for v in recorded) or not recorded and tool!=ZERO_TOOL):
        raise ValueError('native_feedback_tool_binding_mismatch')
    admission=report.get('admission',{})
    if (admission.get('image_id')!='sha256:87e406742de187064c994fc4e5570de1934e92643c31848ce11433201ba38694'
            or admission.get('network')!='none' or admission.get('ports')!=[]):
        raise ValueError('native_feedback_requires_frozen_isolated_docker')
    rows=report.get('raw_output',[]);heads={};segments=[];samples=[];current_enter=None
    for row in rows:
        if row[0]=='CART_ACTUAL_ENTER':
            current_enter=(int(row[1]),int(row[2]),int(row[3]))
        elif row[0]=='CART_HEAD':
            data=tuple(map(float,row[4:]));state=JointReference(data[:6],data[6:12],data[12:])
            tick=int(row[2]);heads[tick]=state
            if current_enter and current_enter[2]==tick and tick-1 in heads:
                segments.append((current_enter[0],current_enter[1],PvatSegment(heads[tick-1],state,.08).coefficients()))
                current_enter=None
        elif row[0]=='CART_ACTUAL_KIN':
            if len(row)!=32:raise ValueError('native_feedback_shape')
            actual=np.array(list(map(float,row[2:8])));target=np.array(list(map(float,row[8:14])))
            if not np.isfinite(actual).all() or not np.isfinite(target).all():raise ValueError('native_feedback_nonfinite')
            samples.append((int(row[1]),actual,target))
    if not samples or not segments:raise ValueError('native_feedback_missing_curve_evidence')
    model=CompiledReferenceModel(tool,(-3.,)*6,(3.,)*6)
    initial=heads[0];hold=PvatSegment(initial,initial,.08).coefficients()
    output=[]
    for time,actual,target in samples:
        available=[s for s in segments if s[1]<=time][-8:] or [(0,0,hold)]
        choices=[]
        for identity,_,c in available:
            u,q=witness(c,target);choices.append((float(np.linalg.norm(q-target)),identity,u,q))
        _,identity,u,q=min(choices,key=lambda x:x[0])
        p=model.pose(q);a=model.pose(actual);t=model.pose(target)
        output.append(dict(time_ms=time,segment=identity,u=u,
            target_joint_distance_rad=float(np.max(np.abs(q-target))),
            target_position_distance_m=float(np.linalg.norm(np.array(p.p)-t.p)),
            actual_position_distance_m=float(np.linalg.norm(np.array(p.p)-a.p)),
            target_angle_distance_rad=float((Rotation.from_quat(p.q).inv()*Rotation.from_quat(t.q)).magnitude()),
            actual_angle_distance_rad=float((Rotation.from_quat(p.q).inv()*Rotation.from_quat(a.q)).magnitude())))
    return dict(samples=len(output),emitted_curves=len(segments),tool_sha256=tool_digest(tool),
        source_passed=report.get('passed'),source_stop_unverified=report.get('stop_unverified'),
        source_unknown_motion_write=report.get('unknown_motion_write'),
        max_target_joint_distance_rad=max(r['target_joint_distance_rad'] for r in output),
        max_target_position_distance_m=max(r['target_position_distance_m'] for r in output),
        max_actual_position_distance_m=max(r['actual_position_distance_m'] for r in output),
        max_target_angle_distance_rad=max(r['target_angle_distance_rad'] for r in output),
        max_actual_angle_distance_rad=max(r['actual_angle_distance_rad'] for r in output),
        witnesses=output,sdk_connections=0,motion_writes=0,native_interpolation_verified=False,hardware_verified=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--tool-report',type=Path);p.add_argument('--tool-report-sha256')
    args=p.parse_args();tool=tool_arguments(p,args)
    raw=args.report.read_bytes();result=analyze(json.loads(raw),tool)
    result['source_sha256']=hashlib.sha256(raw).hexdigest()
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='witnesses'}));return 0


if __name__=='__main__':raise SystemExit(main())
