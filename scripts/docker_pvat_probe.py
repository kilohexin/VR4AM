"""Bounded official-Docker PVAT cadence/stream-loss experiment; preview by default.

No physical selector, webpage registration or controller interpolation assumption.
The reference is a two-second, 0.01-radian minimum-jerk J1 displacement.
Sending and feedback run independently; missed slots are skipped, never replayed.
"""
import argparse
import asyncio
import json
import math
import socket
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
PERIOD = .02
HORIZON = .08
DURATION = 2.
AMPLITUDE = .01


def reference(initial, elapsed):
    if len(initial) != 6 or not all(math.isfinite(x) for x in initial) or not math.isfinite(elapsed):
        raise ValueError('invalid_reference')
    u = min(1., max(0., elapsed / DURATION))
    p = list(initial);v = [0.] * 6;a = [0.] * 6
    p[0] += AMPLITUDE * (10*u**3 - 15*u**4 + 6*u**5)
    v[0] = AMPLITUDE / DURATION * (30*u**2 - 60*u**3 + 30*u**4)
    a[0] = AMPLITUDE / DURATION**2 * (60*u - 180*u**2 + 120*u**3)
    return p, v, a


async def probe(client, *, interrupt=False):
    from app.robots.lebai_motion_calls import OwnedMotionCalls
    writes = OwnedMotionCalls();stops = OwnedMotionCalls()
    report = dict(interrupt=interrupt,period_s=PERIOD,horizon_s=HORIZON,
                  commands=[],samples=[],passed=False,stop_sent=False,unknown_reads=[])
    origin = time.perf_counter();entered = False;monitor = None;latest = None
    initial = None;finishing = False

    async def sample():
        nonlocal latest
        try:
            state, kin = await asyncio.wait_for(asyncio.gather(
                client.get_robot_state(), client.get_kin_data()), .2)
        except BaseException:
            report['unknown_reads'] = ['get_robot_state', 'get_kin_data']
            raise
        q = kin['actual_joint_pose'];qd = kin['actual_joint_speed']
        if any(len(x)!=6 or not all(math.isfinite(v) for v in x) for x in (q,qd)):
            raise ValueError('invalid_feedback')
        latest = dict(t=time.perf_counter()-origin,state=state,q=q,qd=qd)
        report['samples'].append(latest)
        if state not in ('IDLE','MOVING') or max(map(abs,qd))>.3:
            raise ValueError('feedback_limit')
        if initial and max(abs(a-b) for a,b in zip(q,initial))>.04:
            raise ValueError('displacement_limit')
        return latest

    async def observe():
        while not finishing:
            await sample()
            await asyncio.sleep(.01)

    try:
        row = await sample();initial = tuple(row['q'])
        if row['state']!='IDLE' or max(map(abs,row['qd']))>.002:
            raise ValueError('requires_idle')
        monitor = asyncio.create_task(observe())
        start = time.perf_counter();deadline = start;cutoff = 1. if interrupt else DURATION
        while time.perf_counter()-start < cutoff:
            await asyncio.sleep(max(0.,deadline-time.perf_counter()))
            now = time.perf_counter();elapsed = now-start
            if elapsed>=cutoff:break
            if monitor.done():monitor.result();raise ValueError('monitor_ended')
            if now-origin-latest['t']>=.1:raise ValueError('feedback_expired')
            if report['commands'] and now-origin-report['commands'][-1]['t']>=HORIZON:
                raise ValueError('send_gap')
            p,v,a = reference(initial, elapsed+HORIZON)
            entry = dict(t=now-origin,reference_t=elapsed+HORIZON,p=p,v=v,a=a)
            report['commands'].append(entry);entered = True
            await writes.call('move_pvat',lambda:client.move_pvat(p,v,a,HORIZON),.06,revoked=lambda:False)
            entry['reply_t'] = time.perf_counter()-origin
            # Absolute slots avoid accumulated drift; late slots are discarded.
            deadline = start + (math.floor((time.perf_counter()-start)/PERIOD)+1)*PERIOD
        report['release_t'] = time.perf_counter()-origin
        quiet = None;last_t = -1.;end = time.perf_counter()+3.
        while time.perf_counter()<end:
            if monitor.done():monitor.result();raise ValueError('monitor_ended')
            row = latest
            if time.perf_counter()-origin-row['t']>=.1:raise ValueError('feedback_expired')
            if row['t']>last_t:
                last_t = row['t']
                if row['state']=='IDLE' and max(map(abs,row['qd']))<=.002:
                    if quiet is None:quiet = row['t']
                    if row['t']-quiet>=.2:
                        report['settled_t'] = row['t'];break
                else:quiet = None
            await asyncio.sleep(.01)
        else:raise ValueError('natural_finish_timeout')
        if max(abs(row['q'][0]-initial[0]) for row in report['samples'])<.001:
            raise ValueError('no_observed_progress')
        report['passed'] = True
    except BaseException as error:
        report['error'] = type(error).__name__+':'+str(error)
        if entered:
            report['stop_sent'] = True
            try:await stops.call('stop_move',client.stop_move,1.,revoked=lambda:False)
            except BaseException as stop_error:report['stop_error'] = type(stop_error).__name__
            # A returned stop RPC is not a confirmed stop. Preserve the fault
            # and separately collect a continuous fresh quiet observation.
            if monitor:
                finishing = True
                await asyncio.gather(monitor,return_exceptions=True)
                monitor = None
            quiet = None;previous = None
            report['stop_observed'] = False
            try:
                end = time.perf_counter()+2.
                while time.perf_counter()<end:
                    row = await sample()
                    if previous is not None and row['t']-previous>=.1:quiet = None
                    previous = row['t']
                    if row['state']=='IDLE' and max(map(abs,row['qd']))<=.002:
                        if quiet is None:quiet = row['t']
                        if row['t']-quiet>=.2:
                            report['stop_observed'] = True;break
                    else:quiet = None
                    await asyncio.sleep(.01)
            except BaseException as observation_error:
                report['stop_observation_error'] = type(observation_error).__name__
    finally:
        if monitor:
            finishing = True
            await asyncio.gather(monitor,return_exceptions=True)
        report['unknown_writes'] = sorted(writes.unknown_methods|stops.unknown_methods)
        report['motion_ownership'] = writes.snapshot()
        if report['unknown_reads'] or report['unknown_writes']:
            report['passed'] = False
        report['max_send_gap_ms'] = max((1000*(b['t']-a['t']) for a,b in zip(report['commands'],report['commands'][1:])),default=0.)
        report['rpc_max_ms'] = max((1000*(x['reply_t']-x['t']) for x in report['commands'] if 'reply_t' in x),default=0.)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--interrupt',action='store_true')
    parser.add_argument('--output',type=Path,required=True);args = parser.parse_args()
    from app.digital_twin.docker_runtime import verify_local_docker
    admission = verify_local_docker()
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admission,period_s=PERIOD,horizon_s=HORIZON,
                              joint_delta_rad=AMPLITUDE,maximum_packets=100,interrupt=args.interrupt)))
        return 0
    # Existing product services own their own SDK lifecycle even without an
    # open page. Require them to be shut down before this isolated experiment.
    for port in (8773,8774):
        with socket.socket() as check:
            check.settimeout(.2)
            if check.connect_ex(('127.0.0.1',port))==0:
                raise ValueError('close_docker_web_before_probe')
    from app.robots.lebai_sdk_bridge import connect_docker_simulator_client
    from app.commissioning.qpc_event_loop import new_qpc_loop
    async def run():
        client = await connect_docker_simulator_client()
        result = await probe(client,interrupt=args.interrupt);result['admission'] = admission
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','commands','admission')}))
        return 0 if result['passed'] else 1
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:return runner.run(run())


if __name__=='__main__':raise SystemExit(main())
