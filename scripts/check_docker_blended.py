"""Default-preview, verified local Docker exercise of the blended native port."""
import argparse
import asyncio
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))


async def exercise():
    from app.digital_twin.docker_runtime import docker_settings
    from app.robots.finite_position_docker import DockerFinitePositionAdapter
    from app.robots.blended_position_docker import DockerBlendPort
    from app.control.blended_position_session import BlendedPositionSession
    from app.control.finite_position_types import PositionIntent
    from app.robots.base import HomeOptions
    a=DockerFinitePositionAdapter(docker_settings().lebai)
    report={'passed':False};session=None
    try:
        await a.connect()
        await a.prepare(HomeOptions(.2,60.,.002,.02,.3),lambda phase:None)
        binding=a.bind_position_generation(1)
        port=DockerBlendPort(a,binding,authorize=lambda:None)
        session=BlendedPositionSession(port,binding,lower=a.settings.startup_tcp_min_m,
            upper=a.settings.startup_tcp_max_m,clock=a._clock,authorize=lambda:None)
        seq=0;deadline=time.perf_counter()+20.;released=False
        while time.perf_counter()<deadline:
            seq+=1
            if session.entered_steps>=6:released=True
            session.offer(PositionIntent('web-blend-check',1,seq,a._clock(),
                'release' if released else 'held',(0.,0.,0.) if released else (0.,0.,1.)))
            await session.advance()
            if session.closed:break
            if released and not session.busy and session.phase=='ready':
                report['passed']=True;break
            await asyncio.sleep(.02)
        report['view']=asdict(session.view())
        if not report['passed'] and not session.closed:await session.abort('blend_test_timeout')
    except Exception as error:
        report['error']=type(error).__name__+':'+str(error)
        if session is not None:await session.abort('blend_test_failed')
    finally:
        if session is not None and session.worker is not None:
            await asyncio.wait({session.worker},timeout=1.)
        report['unknown_writes']=sorted(a._motion_calls.unknown_methods)
        report['unknown_reads']=sorted(a._snapshot_calls.unknown_methods)
        report['fault']=a._latched_fault
        report['moves']=getattr(a._client,'move_entries',0)
        await a.disconnect()
        report['close_fault']=a._latched_fault
        if a._latched_fault:report['passed']=False
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from app.digital_twin.docker_runtime import verify_local_docker
    admission=verify_local_docker()
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admission,prepare_movej_max=1,movel_max=6,
            direction='+Z',step_mm=4,max_lead_mm=8,stream_budget_s=20)))
        return 0
    from app.commissioning.qpc_event_loop import new_qpc_loop
    with asyncio.Runner(loop_factory=new_qpc_loop) as runner:report=runner.run(exercise())
    report['admission']=admission
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
