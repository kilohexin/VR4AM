"""Zero-motion native independent-input and quiet math-channel contracts."""
import argparse
import asyncio
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from check_native_bundle_owner import fixtures
from check_native_cartesian_reference import ROOT,native_admission
from check_native_cartesian_motion import transport
from cartesian_native_rpc import CartesianNativeRpc
from app.control.cartesian_bundle_wire import encode
from app.control.cartesian_clock import ControllerClock

CHECKS={'duplicates_never_renew','release_permanent','torn_not_committed',
    'early_bound_withdrawal','invalid_negative_word',*(f'withdraw_binding_{i}' for i in range(2,7)),
    *(f'binding_{i}' for i in range(2,7)),'capture_rollback','future_capture','clock_rollback',
    'independent_freshness','math_silence_uses_independent_authority','joints_unchanged'}


def program():
    source=(ROOT/'scripts/native_cartesian_authority_probe.lua').read_text(encoding='utf-8')
    for marker,file in [('__AUTHORITY__','cartesian_authority.lua'),('__MAILBOX__','cartesian_bundle_mailbox.lua')]:
        source=source.replace(marker,(ROOT/'scripts'/file).read_text(encoding='utf-8'))
    return source.replace('__PACKET__','{'+','.join(str(v) for v in encode(fixtures()[0]))+'}')


def analyze(raw):
    rows=[s.strip().split('\t') for s in raw.splitlines()]
    checks=[r[1] for r in rows if len(r)==2 and r[0]=='AUTHORITY_CHECK']
    done=[r for r in rows if r[0]=='AUTHORITY_DONE']
    return dict(passed=len(done)==1 and done[0][1:]==['true','nil'] and len(checks)==len(CHECKS)
        and set(checks)==CHECKS,checks=checks,motion_calls=0,signal_writes=0,product_admission_complete=False)


async def clock_probe(admission):
    rpc=CartesianNativeRpc(admission)
    try:
        await rpc.open();samples=[];pair=None
        warmup=await rpc.request('return lebai:get_signal(240)')
        for _ in range(5):
            r=await rpc.request('return lebai:get_signal(240)')
            samples.append(dict(sent_ns=r.sent_ns,received_ns=r.received_ns,sampled_ms=r.sampled_ms))
            if pair is None:pair=ControllerClock(r.sent_ns,r.received_ns,r.sampled_ms)
            else:pair=pair.check_pair(r.sent_ns,r.received_ns,r.sampled_ms)
            await asyncio.sleep(.02)
        return dict(passed=True,samples=samples,unknown=rpc.unknown,
            warmup_roundtrip_ms=(warmup.received_ns-warmup.sent_ns)/1e6,
            clock='time.perf_counter_ns')
    finally:await rpc.close()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true');args=p.parse_args()
    admission=native_admission(args.container)
    if not args.run:print(json.dumps(dict(admission=admission,execute=False,motion_calls=0)));return 0
    execution=transport(admission,program(),'AUTHORITY_DONE');result=analyze(execution['raw_output'])
    result.update(transport=execution,admission=admission)
    if result['passed'] and execution.get('transport_returncode')==0:
        try:result['clock_probe']=asyncio.run(clock_probe(admission))
        except Exception as error:result['clock_probe']=dict(passed=False,error=type(error).__name__)
    result['passed']=bool(result['passed'] and result.get('clock_probe',{}).get('passed'))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('transport','admission','clock_probe')}))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
