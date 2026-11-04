"""Zero-motion bundle-wire audit in pinned network=none official Docker.

Default preview. Native calls are only signal I/O and before/after joint reads.
No startup, stop, motion, gripper, physical selector, reset or retries.
"""
import argparse
import json
from pathlib import Path
import struct
import sys
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT/'scripts')]
from check_native_cartesian_reference import native_admission
from check_native_cartesian_motion import transport
from app.control.cartesian_pvat_reference import JointReference,PvatSegment
from app.control.cartesian_bundle_wire import CompactBundle,encode

CHECKS={'lossless','every_word_crc','invalid_float_and_reserved','latest','torn_each_word',
    'duplicate_expiry','immutable_duplicate','binding','rollback','expiry_permanent','native_banks','joint_unchanged'}


def fixture(sequence=23,captured=340,due=400,change=False):
    q=(0.,-.8,1.4,-.6,1.57,-0.)
    end=JointReference((.0001 if not change else .0002,-.8002,1.4001,-.6,1.57,.0001),(.001,)*6,(.002,)*6)
    return CompactBundle('01'*16,'a9'*32,1,sequence,4,due,captured,
        PvatSegment(JointReference(q),end,.08),JointReference(end.q),10)


def damaged():
    result=[]
    for value in (float('nan'),float('inf'),-float('inf'),1e9,5e-324):
        w=list(encode(fixture()));w[19:21]=struct.unpack('<ii',struct.pack('<d',value));result.append(w)
    w=list(encode(fixture()));w[103]=1;result.append(w)
    for w in result:
        crc=zlib.crc32(struct.pack('<119i',*w[:-1]));w[-1]=crc if crc<2**31 else crc-2**32
    return result


def program():
    def vector(words):return '{'+','.join(str(v) for v in words)+'}'
    module=(ROOT/'scripts/cartesian_bundle_mailbox.lua').read_text(encoding='utf-8')
    harness=(ROOT/'scripts/native_bundle_mailbox_probe.lua').read_text(encoding='utf-8')
    return harness.replace('__MODULE__',module).replace('__FIXTURE__',vector(encode(fixture())))\
        .replace('__LATEST__',vector(encode(fixture(24,360,480))))\
        .replace('__MUTATED__',vector(encode(fixture(change=True))))\
        .replace('__INVALID__','{'+','.join(vector(w) for w in damaged())+'}')\
        .replace('__NATIVE_A__',vector(encode(fixture(23,0,10000))))\
        .replace('__NATIVE_B__',vector(encode(fixture(24,0,10000))))


def analyze(raw):
    rows=[line.strip().split('\t') for line in raw.splitlines()]
    checks=[r[1] for r in rows if len(r)==2 and r[0]=='BUNDLE_CHECK']
    done=[r for r in rows if r[0]=='BUNDLE_MAILBOX_DONE']
    reads=[r for r in rows if r[0]=='BUNDLE_NATIVE_READ']
    passed=(len(done)==1 and len(done[0])==3 and done[0][1]=='true'
        and len(checks)==len(CHECKS) and set(checks)==CHECKS and len(reads)==2)
    try:
        times=[int(r[2]) for r in reads if len(r)==3]
        passed=passed and len(times)==2 and [int(r[1]) for r in reads]==[23,24] and all(0<=t<80 for t in times)
    except (ValueError,IndexError):passed=False;times=[]
    return dict(passed=bool(passed),motion_calls=0,checks=checks,max_read_ms=max(times,default=None),
        product_admission_complete=False,native_interpolation_verified=False)


def execute(admitted):
    result=transport(admitted,program(),'BUNDLE_MAILBOX_DONE')
    report=analyze(result['raw_output'])
    report['passed']=report['passed'] and result.get('transport_returncode')==0 and not result.get('transport_unknown')
    return dict(report,transport=result,admission=admitted)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--run',action='store_true');args=p.parse_args()
    admitted=native_admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admitted,motion_calls=0,signal_banks=[0,120])));return 0
    report=execute(admitted);args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('transport','admission')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
