"""One finite precomputed bundle-owner motion trial in isolated Docker only.

Preview by default; one preparation, <=64 PVAT points including owned tail,
one exception stop maximum, no retries, real selector, gripper or UI bypass.
The injected synthetic publisher and finite whitelist are NOT a live driver.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT/'scripts')]
from check_native_cartesian_reference import native_admission
from check_native_cartesian_motion import transport,preparation_program,finished,COMMON,vec
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor
from app.control.cartesian_bundle_wire import CompactBundle,encode


def build_packets(origin):
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    initial=JointReference(tuple(origin));base=model.pose(initial.q)
    supervisor=OfflineReferenceSupervisor(model,initial,braking_method='joint_fit')
    packets=[]
    for tick in range(46):
        angle=.15 if tick<16 else -.15
        target=base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',angle)).as_quat())})
        approved=supervisor.advance(target)
        if approved.mode!='following':raise ValueError('owned_reference_rejected')
        packet=CompactBundle('15'*16,'a9'*32,1,tick+1,tick,200+max(0,tick-1)*80,
            max(0,tick-1)*80,approved.segment,approved.braking_tail[-1].end,len(approved.braking_tail))
        if packet.expand_tail()!=approved.braking_tail:raise ValueError('owned_compaction_mismatch')
        packets.append(packet)
    if len(packets)+packets[-1].braking_ticks>64:raise ValueError('owned_point_budget')
    return model,base,packets


def program(origin,base,packets,*,native_emitter=False):
    if type(native_emitter) is not bool:raise ValueError('owned_emitter_configuration')
    if native_emitter:raise ValueError('owned_native_emitter_return_unverified')
    words='{'+','.join('{'+','.join(str(x) for x in encode(p))+'}' for p in packets)+'}'
    source=(ROOT/'scripts/native_owned_motion.lua').read_text(encoding='utf-8')
    replacements={'__COMMON__':COMMON,'__MAILBOX__':(ROOT/'scripts/cartesian_bundle_mailbox.lua').read_text(encoding='utf-8'),
        '__OWNER__':(ROOT/'scripts/cartesian_bundle_owner.lua').read_text(encoding='utf-8'),
        '__SDK_RESULT__':(ROOT/'scripts/pvat_sdk_result.lua').read_text(encoding='utf-8'),
        '__PACKETS__':words,'__ORIGIN__':vec(origin),'__FINAL__':vec(packets[-1].terminal.q),
        '__BASE__':'{x='+repr(base.p[0])+',y='+repr(base.p[1])+',z='+repr(base.p[2])+'}',
        '__ROTATION__':'{'+','.join(repr(float(x)) for x in Rotation.from_quat(base.q).as_matrix().ravel())+'}'}
    for key,value in replacements.items():
        if source.count(key)!=1:raise ValueError('owned_program_template')
        source=source.replace(key,value)
    return source


def analyze(raw,packets):
    rows=[line.strip().split('\t') for line in raw.splitlines()]
    done=[r for r in rows if r[0]=='OWNED_DONE'];quiet=[r for r in rows if r[0]=='OWNED_QUIET']
    points=[r for r in rows if r[0]=='OWNED_POINT'];entered=[r for r in rows if r[0]=='OWNED_ENTER']
    aborts=[r for r in rows if r[0]=='FINITE_ABORT'];samples=[r for r in rows if r[0]=='OWNED_SAMPLE']
    result=dict(passed=False,precomputed_only=True,hardware_verified=False,product_admission_complete=False,
        native_interpolation_verified=False,motion_points=len(points),exception_stops=len(aborts),
        unknown_motion_write=len(entered)>len(points),stop_unverified=bool(len(entered)>len(points)
            or len(aborts)>1 or any(r[1:]!=['true','true'] for r in aborts)))
    try:
        expected=len(packets)+packets[-1].braking_ticks
        acks=[r for r in rows if r[0]=='OWNED_ACK']
        if (len(done)!=1 or len(done[0])!=4 or done[0][1]!='true' or int(done[0][3])!=expected
            or len(points)!=expected or len(entered)!=expected or aborts or len(quiet)!=1 or len(quiet[0])!=4):return result
        if any(len(r)!=5 for r in points) or [int(r[1]) for r in points]!=list(range(1,expected+1)):return result
        if [int(r[3]) for r in points]!=list(range(1,expected+1)):return result
        if [r[4] for r in points]!=['following']*len(packets)+['braking']*packets[-1].braking_ticks:return result
        if len(acks)!=len(packets) or any(len(r)!=5 or r[2]!='true' for r in acks):return result
        if [int(r[1]) for r in acks]!=list(range(1,len(packets)+1)):return result
        values=[tuple(float(x) for x in r[1:]) for r in samples]
        if len(values)<30 or any(len(r)!=4 or not all(np.isfinite(x) for x in r) for r in values):return result
        if any(not 0<=r[1]<=.001 for r in values) or any(not 0<=b[0]-a[0]<100 for a,b in zip(values,values[1:])):return result
        time,drift,count=map(float,quiet[0][1:])
        if not np.isfinite([time,drift,count]).all() or count!=len(values) or abs(drift-max(r[1] for r in values))>1e-8:return result
        result.update(passed=not result['stop_unverified'],max_sampled_tcp_drift_m=drift,feedback_samples=len(values),
            following_points=len(packets),owned_braking_points=packets[-1].braking_ticks)
        times=[float(r[2]) for r in entered]
        # Packet due_ms and native records share the controller epoch. The
        # first invocation can be late; it does NOT reset the planned grid.
        leads=[index*80-(stamp-packets[0].due_ms) for index,stamp in enumerate(times,1)]
        result.update(max_send_gap_ms=max((b-a for a,b in zip(times,times[1:])),default=0),
            planned_epoch_ms=packets[0].due_ms,
            max_scheduled_reference_lead_ms=max(leads),min_scheduled_reference_lead_ms=min(leads))
        result['passed']=bool(result['passed'] and np.isfinite(times+leads).all()
            and all(b>=a for a,b in zip(times,times[1:])) and min(leads)>=0 and max(leads)<=160)
    except (ValueError,IndexError,OverflowError):return result
    return result


def execute(admitted,*,native_emitter=False):
    if native_emitter:return dict(passed=False,error='owned_native_emitter_return_unverified',motion_calls=0)
    first=transport(admitted,preparation_program(),'FINITE_PREP_DONE')
    result=dict(passed=False,preparation=first,admission=admitted,hardware_verified=False,
        product_admission_complete=False,native_emitter=native_emitter)
    entries=first['raw_output'].count('FINITE_PREP_ENTER');returns=first['raw_output'].count('FINITE_PREP_RETURNED')
    aborts=[line.strip().split('\t') for line in first['raw_output'].splitlines() if line.startswith('FINITE_ABORT')]
    result['stop_unverified']=bool(first.get('transport_unknown') or first.get('stop_unverified')
        or first.get('transport_returncode')!=0 or entries!=returns
        or any(row[1:]!=['true','true'] for row in aborts))
    if not finished(first,'FINITE_PREP_DONE') or aborts or result['stop_unverified']:return result
    rows=[line.strip().split('\t') for line in first['raw_output'].splitlines() if line.startswith('FINITE_ORIGIN\t')]
    if len(rows)!=1 or len(rows[0])!=7:result['error']='owned_origin_missing';return result
    try:
        origin=tuple(float(v) for v in rows[0][1:]);_,base,packets=build_packets(origin)
    except ValueError as error:result['error']=str(error);return result
    second=transport(admitted,program(origin,base,packets,native_emitter=native_emitter),'OWNED_DONE');result['motion']=second
    result.update(analyze(second['raw_output'],packets))
    result['stop_unverified']=bool(result['stop_unverified'] or second.get('stop_unverified') or second.get('transport_unknown'))
    result['passed']=result['passed'] and second.get('transport_returncode')==0 and not result['stop_unverified']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--run',action='store_true')
    p.add_argument('--native-emitter',action='store_true');args=p.parse_args()
    admitted=native_admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,prepare_max=1,pvat_max=64,
            exception_stop_max=1,native_emitter=args.native_emitter)));return 0
    result=execute(admitted,native_emitter=args.native_emitter);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('preparation','motion','admission')}))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
