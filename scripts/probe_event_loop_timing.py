"""Offline scheduler comparison. No SDK, robot configuration, or network."""
import argparse
import asyncio
import json
from pathlib import Path
import platform
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from app.commissioning.qpc_event_loop import new_qpc_loop


async def trial():
    never=asyncio.create_task(asyncio.Event().wait())
    finished=False
    polls=0
    async def poll():
        nonlocal polls
        while not finished:
            await asyncio.wait({never},timeout=.005)
            polls+=1
    task=asyncio.create_task(poll())
    started=time.perf_counter_ns()
    try:
        await asyncio.wait({never},timeout=.060)
        return dict(elapsed_ms=(time.perf_counter_ns()-started)/1e6,polls=polls)
    finally:
        finished=True
        await task
        never.cancel()
        await asyncio.gather(never,return_exceptions=True)


def sample(factory,iterations):
    loop=factory()
    try:
        values=[loop.run_until_complete(trial()) for _ in range(iterations)]
        elapsed=[value['elapsed_ms'] for value in values]
        return dict(loop=type(loop).__name__,resolution_s=loop._clock_resolution,
            configured_timeout_ms=60,configured_poll_ms=5,min_ms=min(elapsed),
            median_ms=statistics.median(elapsed),max_ms=max(elapsed),samples=values)
    finally:loop.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--iterations',type=int,default=12)
    args=parser.parse_args(argv)
    if not 1<=args.iterations<=100:raise ValueError('iterations_out_of_range')
    value=dict(python=platform.python_version(),platform=platform.system(),measurement_clock='time.perf_counter_ns',
        monotonic_info=vars(time.get_clock_info('monotonic')),perf_counter_info=vars(time.get_clock_info('perf_counter')),
        sdk_imported=False,robot_connections=0,hardware_verified=False,
        stock=sample(asyncio.new_event_loop,args.iterations),qpc=sample(new_qpc_loop,args.iterations))
    with args.output.open('x',encoding='utf-8') as stream:json.dump(value,stream,indent=2)
    print(json.dumps({name:{key:value[name][key] for key in ['min_ms','median_ms','max_ms']} for name in ['stock','qpc']}))
    return 0


if __name__=='__main__':raise SystemExit(main())
