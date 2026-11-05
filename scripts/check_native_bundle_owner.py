"""Zero-motion native reference/backup ownership contracts. No actuator APIs.

Fixtures come from approved model bundles and must compact bit-for-bit in
Python. The injected validator in the Lua harness is a TEST double only.
This does not register an executor, bypass quarantine or authorize motion.
"""
import argparse
import json
from pathlib import Path
import sys

from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT/'scripts')]
from check_native_cartesian_reference import native_admission
from check_native_cartesian_motion import transport
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor
from app.control.cartesian_bundle_wire import CompactBundle,encode

CHECKS={'precise_tail_reconstruction','full_backup_before_output','at_most_two','known_late_no_commit',
    'late_validation_no_commit','release_discards_future','authority_expiry_permanent','model_binding_fault',
    'bad_start_no_commit','caller_mutation_isolated','input_rollback_fault','unknown_validation_fault',
    'concurrent_stage_fault','holding_without_recenter','sdk_void_contract','native_joint_format',
    'bounded_preload_schedule','joints_unchanged','quiet_prefix_no_write','quiet_prefix_release',
    'moving_prefix_refused','reference_age_rechecked','binary_tail_identity',
    'curve_witness_exact_point','curve_witness_keeps_deviation','curve_witness_constant',
    'curve_witness_eight_curves','lua_mdh_matches_sdk',
    'braking_prefix_opt_in','braking_prefix_owned_then_follow','braking_prefix_release_old_tail',
    'braking_prefix_bad_start','braking_prefix_yield_recheck','lua_bound_tool_matches_python',
    'lua_bound_tool_copies_geometry','lua_bound_tool_invalid_rejected'}


def fixtures():
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    initial=JointReference((0.,-.8,1.4,-.6,1.57,0.));base=model.pose(initial.q)
    target=base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',.15)).as_quat())})
    supervisor=OfflineReferenceSupervisor(model,initial,continue_on_known_rejection=True,
        braking_method='joint_fit',braking_segment_limit=24,braking_curve_limit=4)
    packets=[]
    for tick in range(3):
        approved=supervisor.advance(target)
        if approved.mode!='following':raise ValueError('owner_probe_fixture_not_approved')
        packet=CompactBundle('01'*16,'a9'*32,1,23+tick,tick,200+max(0,tick-1)*80,100,
            approved.segment,approved.braking_tail[-1].end,len(approved.braking_tail))
        if packet.expand_tail()!=approved.braking_tail:raise ValueError('owner_probe_compaction_mismatch')
        packets.append(packet)
    return packets


def program():
    def numbers(values):return '{'+','.join(repr(v) for v in values)+'}'
    def state(s):return '{q='+numbers(s.q)+',v='+numbers(s.v)+',a='+numbers(s.a)+'}'
    packets=fixtures()
    prefix=braking_prefix_fixture()
    words='{'+','.join(numbers(encode(p)) for p in packets)+'}'
    tail='{'+','.join(state(s.end) for s in packets[0].expand_tail())+'}'
    tails='{'+','.join('{'+','.join(state(s.end) for s in p.expand_tail())+'}' for p in packets)+'}'
    source=(ROOT/'scripts/native_bundle_owner_probe.lua').read_text(encoding='utf-8')
    tool=MdhTool((.03,-.02,.07),tuple(float(v) for v in Rotation.from_euler('xyz',(.2,-.4,.3)).as_quat()))
    model=MdhReferenceModel(tool,(-3.,)*6,(3.,)*6);cases=[]
    for joints in (packets[0].lead.start.q,(.2,-.8,1.4,-.6,1.2,.3),(-.3,-.4,.9,.2,-1.,.4)):
        pose=model.pose(joints)
        cases.append('{joints='+numbers(joints)+',p='+numbers(pose.p)+',r='+
            numbers(tuple(float(v) for v in Rotation.from_quat(pose.q).as_matrix().ravel()))+'}')
    source=source.replace('__TOOL_CASES__','{p='+numbers(tool.translation_m)+',q='+numbers(tool.rotation_xyzw)+
        ',poses={'+','.join(cases)+'}}')
    return source.replace('__MAILBOX__',(ROOT/'scripts/cartesian_bundle_mailbox.lua').read_text(encoding='utf-8'))\
        .replace('__OWNER__',(ROOT/'scripts/cartesian_bundle_owner.lua').read_text(encoding='utf-8'))\
        .replace('__SDK_RESULT__',(ROOT/'scripts/pvat_sdk_result.lua').read_text(encoding='utf-8'))\
        .replace('__CURVE_WITNESS__',(ROOT/'scripts/cartesian_curve_witness.lua').read_text(encoding='utf-8'))\
        .replace('__LUA_MDH__',(ROOT/'scripts/cartesian_lm3_geometry.lua').read_text(encoding='utf-8'))\
        .replace('__PACKETS__',words).replace('__TAIL__',tail).replace('__ALL_TAILS__',tails)\
        .replace('__PREFIX_PACKET__',numbers(encode(prefix)))


def braking_prefix_fixture():
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    first=fixtures()[0];initial=first.lead.start;base=model.pose(initial.q)
    target=base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',.15)).as_quat())})
    supervisor=OfflineReferenceSupervisor(model,initial,continue_on_known_rejection=True,
        braking_method='joint_fit',braking_segment_limit=24,braking_curve_limit=4)
    approved=supervisor.advance(target)
    if approved.segment!=first.lead:raise ValueError('owner_prefix_fixture_origin')
    supervisor._consume_tail()  # Consume the already-approved first brake.
    next=supervisor.advance(target)
    if next.mode!='following':raise ValueError('owner_prefix_fixture_not_approved')
    packet=CompactBundle(first.session,first.model_sha256,1,first.sequence+1,2,280,100,
        next.segment,next.braking_tail[-1].end,len(next.braking_tail))
    if packet.expand_tail()!=next.braking_tail:raise ValueError('owner_prefix_compaction')
    return packet


def analyze(raw):
    rows=[line.strip().split('\t') for line in raw.splitlines()]
    checks=[r[1] for r in rows if len(r)==2 and r[0]=='OWNER_CHECK']
    done=[r for r in rows if r[0]=='OWNER_PROBE_DONE']
    return dict(passed=bool(len(done)==1 and len(done[0])==3 and done[0][1]=='true'
        and len(checks)==len(CHECKS) and set(checks)==CHECKS),checks=checks,motion_calls=0,
        product_admission_complete=False,native_interpolation_verified=False)


def execute(admitted):
    result=transport(admitted,program(),'OWNER_PROBE_DONE');report=analyze(result['raw_output'])
    report['passed']=report['passed'] and result.get('transport_returncode')==0 and not result.get('transport_unknown')
    return dict(report,transport=result,admission=admitted)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--run',action='store_true');args=p.parse_args()
    admitted=native_admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,motion_calls=0,signal_writes=0)));return 0
    report=execute(admitted);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('transport','admission')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
