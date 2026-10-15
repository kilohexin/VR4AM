import asyncio
import time
import copy
import pytest
from tests.commissioning.test_native_jog_site_run import setup,run


@pytest.mark.parametrize('late',['speedl','stop'])
async def test_late_native_result_preserves_failure(late):
    s,c=setup(); old=c.speedl if late=='speedl' else c.stop_move
    async def delayed(*args):
        value=await old(*args)
        await asyncio.sleep(.1 if late=='speedl' else .25)
        return value
    if late=='speedl':c.speedl=delayed
    else:c.stop_move=delayed
    r,_=await run(s,c)
    assert not r['complete'] and r['latched_fault']=='stop_unverified'
    assert r['speedl_entries']==r['stop_entries']==1
    frozen=copy.deepcopy(r);await asyncio.sleep(.12);assert r==frozen


async def test_stubborn_observer_does_not_delay_original_stop():
    s,c=setup();release=asyncio.Event();stop_seen=asyncio.Event();stop_ns=[]
    read=c.get_kin_data;stop=c.stop_move
    async def blocked():
        if c.write_calls and not stop_seen.is_set():
            while not release.is_set():
                try:await release.wait()
                except asyncio.CancelledError:pass
        return await read()
    async def stopped():
        stop_ns.append(time.perf_counter_ns());stop_seen.set();return await stop()
    c.get_kin_data=blocked;c.stop_move=stopped
    try:
        r,_=await run(s,c)
        assert stop_seen.is_set() and (stop_ns[0]-r['speedl_started_ns'])/1e9 < .2
        assert r['motion_observation']['remote_result_unknown'] and not r['complete']
    finally:
        release.set()
        for _ in range(10):await asyncio.sleep(0)


async def test_revocation_after_native_entry_still_sends_one_stop():
    entered=False
    def hot():
        if entered:raise ValueError('native_jog_expired')
    s,c=setup(hot_check=hot);old=c.speedl
    async def speed(*args):
        nonlocal entered
        value=await old(*args);entered=True;return value
    c.speedl=speed
    r,_=await run(s,c)
    assert r['speedl_entries']==r['stop_entries']==1 and not r['complete']


async def test_actual_entry_clock_survives_prewrite_delay():
    before=[]
    def write():
        before.append(time.perf_counter_ns());time.sleep(.015)
    s,c=setup(write_check=write)
    r,_=await run(s,c)
    assert r['complete'],r
    assert r['speedl_started_ns']-before[0]>=15_000_000
    assert 80_000_000 <= r['stop_started_ns']-r['speedl_started_ns'] < 180_000_000


async def test_owner_cancellation_after_entry_still_stops_once():
    s,c=setup();entered=asyncio.Event();report={};old=c.speedl
    async def speed(*args):
        value=await old(*args);entered.set();await asyncio.sleep(.2);return value
    c.speedl=speed
    task=asyncio.create_task(run(s,c,report))
    await asyncio.wait_for(entered.wait(),2);task.cancel()
    with pytest.raises(asyncio.CancelledError):await asyncio.wait_for(task,4)
    assert report['speedl_entries']==report['stop_entries']==1
    assert not report['complete'] and report['latched_fault']=='stop_unverified'
