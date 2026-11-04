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
    'moving_prefix_refused','reference_age_rechecked'}


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
    words='{'+','.join(numbers(encode(p)) for p in packets)+'}'
    tail='{'+','.join(state(s.end) for s in packets[0].expand_tail())+'}'
    source=(ROOT/'scripts/native_bundle_owner_probe.lua').read_text(encoding='utf-8')
    return source.replace('__MAILBOX__',(ROOT/'scripts/cartesian_bundle_mailbox.lua').read_text(encoding='utf-8'))\
        .replace('__OWNER__',(ROOT/'scripts/cartesian_bundle_owner.lua').read_text(encoding='utf-8'))\
        .replace('__SDK_RESULT__',(ROOT/'scripts/pvat_sdk_result.lua').read_text(encoding='utf-8'))\
        .replace('__PACKETS__',words).replace('__TAIL__',tail)


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
