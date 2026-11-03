"""Bounded live-input PVAT candidate against the verified local Docker only.

No webpage registration. An atomic timestamp/direction mailbox uses signal 240
only when initially zero. Late or lost input ends the controller program.
"""
import argparse
import asyncio
import json
from pathlib import Path
import socket
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from app.control.docker_pvat_input import CHANNEL,MODULUS,encode,DockerPvatInput,ExecutorLease


def schedule(elapsed):
    # Same-clutch direction changes, explicit releases, then silent input loss.
    phases=((.4,0),(1.3,1),(1.8,0),(2.7,2),(3.2,0),(4.1,3),(4.6,0),(5.5,4),(6.,0),(6.9,5),(7.4,0),(8.3,6))
    return next((action for end,action in phases if elapsed<end),None)


def analyze(lines):
    rows=[line.split('\t') for line in lines]
    points=[r for r in rows if r[0]=='LIVE_POINT']
    releases=[r for r in rows if r[0]=='LIVE_RELEASE']
    quiet=[r for r in rows if r[0]=='LIVE_QUIET']
    done=[r for r in rows if r[0]=='LIVE_DONE']
    actions=sorted({int(r[2]) for r in points})
    spans={axis:max(float(r[i]) for r in points)-min(float(r[i]) for r in points)
           for axis,i in (('x',3),('y',4),('z',5))} if points else {}
    directional={}
    for action in range(1,7):
        samples=[r for r in points if int(r[2])==action]
        axis=(action-1)//2+3;sign=1 if action%2 else -1
        directional[str(action)]=(float(samples[-1][axis])-float(samples[0][axis]))*sign if len(samples)>1 else 0.
    return dict(points=len(points),actions=actions,tcp_spans_m=spans,directional_progress_m=directional,
        max_send_gap_ms=max((float(b[1])-float(a[1]) for a,b in zip(points,points[1:])),default=0),
        release_to_quiet_ms=float(quiet[-1][1])-float(releases[-1][1]) if quiet and releases else None,
        trace_passed=bool(done and done[-1][1]=='true' and quiet and releases
            and any(r[0]=='LIVE_EXPIRED' for r in rows)
            and not any(r[0]=='LIVE_ABORT' for r in rows)
            and actions==list(range(7)) and all(v>.001 for v in spans.values())
            and all(v>.0005 for v in directional.values())))


async def run(admission):
    from app.robots.lebai_sdk_bridge import connect_docker_simulator_client
    client=await connect_docker_simulator_client()
    if await client.get_signal(CHANNEL)!=0:raise ValueError('mailbox_occupied')
    bridge='exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; while IFS= read -r -t 12 line <&3; do printf "%s\\n" "$line"; case "$line" in *LIVE_DONE*) break;; esac; done'
    proc=await asyncio.create_subprocess_exec('docker','exec','-i',admission['container_id'],'bash','-c',bridge,
        stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.PIPE)
    proc.stdin.write(Path(__file__).with_suffix('.lua').read_bytes());await proc.stdin.drain();proc.stdin.close()
    report=dict(lines=[],inputs=[],passed=False,unknown_input=False)
    feeder=None;pump=None;done=False;finish=asyncio.Event()
    async def feed(controller_stamp):
        # Receipt time makes the encoded clock conservative by handshake delay.
        origin=time.perf_counter();seq=0
        while not finish.is_set():
            elapsed=time.perf_counter()-origin;action=schedule(elapsed)
            if action is None:return
            pump.offer(seq=seq,captured_ns=time.perf_counter_ns(),action=action);seq+=1
            report['inputs'].append(dict(t=elapsed,action=action))
            await asyncio.sleep(.025)
    try:
        async with asyncio.timeout(15):
            while line:=await proc.stdout.readline():
                line=line.decode('utf-8',errors='replace').strip();report['lines'].append(line)
                fields=line.split('\t')
                if fields[0]=='LIVE_READY':
                    if feeder is not None:raise ValueError('duplicate_ready')
                    pump=DockerPvatInput(ExecutorLease(),clock=time.perf_counter_ns,authorize=lambda:None)
                    pump.start(client,int(fields[1]))
                    feeder=asyncio.create_task(feed(int(fields[1])))
                if fields[0]=='LIVE_DONE':
                    done=True;finish.set();report['controller_passed']=fields[1]=='true'
                    if pump:await pump.withdraw()
                    break
            report['transport_returncode']=await proc.wait()
        if feeder:await feeder
        report['passed']=done and report.get('controller_passed',False) and not report['unknown_input'] and report['transport_returncode']==0
    except BaseException as error:
        report['error']=type(error).__name__+':'+str(error)
    finally:
        finish.set()
        if feeder and not feeder.done():
            feeder.cancel();await asyncio.gather(feeder,return_exceptions=True)
        # Only clear the dedicated mailbox after a known program completion.
        # No retry after an unknown write, and never terminate/restart motion.
        if pump:
            await pump.withdraw()
            report['unknown_input']=pump.unknown
            report['input_exit_reason']=pump.reason
            report['mailbox_writes']=len(pump.writes)
        if done and pump and report['controller_passed'] and any(s.startswith('LIVE_QUIET') for s in report['lines']):
            try:
                await pump.complete(client,controller_ok=True,quiet_observed=True)
                report['mailbox_cleared']=True
            except Exception:report['mailbox_cleared']=False
        if pump:
            report['unknown_input']=pump.unknown
            report['lease_released']=pump.lease.owner is None
        report['passed']=report['passed'] and report.get('mailbox_cleared',False)
        report['completed']=done
        report.update(analyze(report['lines']))
        report['passed']=report['passed'] and report['trace_passed']
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from app.digital_twin.docker_runtime import verify_local_docker
    admission=verify_local_docker()
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admission,signal=CHANNEL,input_expiry_ms=100,
            duration_s=10,maximum_packets=125,cartesian_speed_mps=.005,reference_joint_speed_cap=.06)))
        return 0
    for port in (8773,8774):
        with socket.socket() as check:
            check.settimeout(.2)
            if check.connect_ex(('127.0.0.1',port))==0:raise ValueError('close_docker_web_before_probe')
    from app.commissioning.qpc_event_loop import new_qpc_loop
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:result=runner.run(run(admission))
    result['admission']=admission
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('lines','inputs','admission')}))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
