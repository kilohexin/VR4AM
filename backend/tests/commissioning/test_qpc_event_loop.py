"""Local timers only. No SDK, network, robot, or global event-loop policy."""
import asyncio
import importlib
import time


def test_qpc_loop_uses_one_monotonic_high_resolution_clock():
    module=importlib.import_module('app.commissioning.qpc_event_loop')
    policy=asyncio.get_event_loop_policy()
    loop=module.new_qpc_loop()
    try:
        before=time.perf_counter();value=loop.time();after=time.perf_counter()
        assert before<=value<=after
        assert loop._clock_resolution==time.get_clock_info('perf_counter').resolution
        assert asyncio.get_event_loop_policy() is policy
    finally:loop.close()


def test_qpc_60ms_deadline_is_not_triggered_early_by_5ms_polling():
    module=importlib.import_module('app.commissioning.qpc_event_loop')
    async def probe():
        never=asyncio.create_task(asyncio.Event().wait());finished=False;polls=0
        async def poll():
            nonlocal polls
            while not finished:
                await asyncio.wait({never},timeout=.005);polls+=1
        task=asyncio.create_task(poll());started=time.perf_counter_ns()
        try:
            await asyncio.wait({never},timeout=.06)
            elapsed=(time.perf_counter_ns()-started)/1e6
            assert elapsed>=59.5
            assert polls>=1  # Windows may dispatch later than a requested 5 ms.
        finally:
            finished=True;await task;never.cancel();await asyncio.gather(never,return_exceptions=True)
    loop=module.new_qpc_loop()
    try:
        for _ in range(5):loop.run_until_complete(probe())
    finally:loop.close()
