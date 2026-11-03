import asyncio
import pytest
from app.control.docker_pvat_input import DockerPvatInput,ExecutorLease,EXPIRY_NS,encode_direction


class Client:
    def __init__(self):self.writes=[];self.sent=asyncio.Event()
    async def set_signal(self,channel,value):self.writes.append(value);self.sent.set()
    async def get_signal(self,channel):return 0


def make():
    now=[1_000_000_000];lease=ExecutorLease()
    return now,lease,DockerPvatInput(lease,clock=lambda:now[0],authorize=lambda:None)


async def test_old_intent_timestamp_is_not_renewed_and_expiry_never_rearms():
    now,lease,pump=make();client=Client();pump.start(client,1234)
    pump.offer(seq=0,captured_ns=now[0],action=1)
    await client.sent.wait();first=client.writes[0]
    now[0]+=50_000_000;await asyncio.sleep(.035)
    assert client.writes[-1]==first
    now[0]+=EXPIRY_NS
    await asyncio.wait_for(pump.worker,.2)
    assert pump.closed and pump.reason=='pvat_input_expired'
    with pytest.raises(ValueError):pump.offer(seq=1,captured_ns=now[0],action=2)
    with pytest.raises(ValueError):lease.acquire()
    await pump.complete(client,controller_ok=True,quiet_observed=True)
    assert lease.owner is None and client.writes[-1]==0


async def test_withdraw_joins_inflight_write_before_control_handoff():
    now,lease,pump=make();entered=asyncio.Event();returned=asyncio.Event()
    class Slow(Client):
        async def set_signal(self,channel,value):
            if value:entered.set();await returned.wait()
            await super().set_signal(channel,value)
    client=Slow();pump.start(client,1234);pump.offer(seq=0,captured_ns=now[0],action=1)
    await entered.wait();closing=asyncio.create_task(pump.withdraw());await asyncio.sleep(0)
    assert not closing.done()
    with pytest.raises(ValueError):lease.acquire()
    returned.set();await closing
    assert len(client.writes)==1
    await pump.complete(client,controller_ok=True,quiet_observed=True)
    assert lease.acquire() is not None


async def test_unknown_write_keeps_lease_even_with_quiet_controller():
    now,lease,pump=make()
    class Unknown(Client):
        async def set_signal(self,*args):self.writes.append(args);raise TimeoutError()
    client=Unknown();pump.start(client,1234);pump.offer(seq=0,captured_ns=now[0],action=1)
    await pump.worker
    with pytest.raises(ValueError):await pump.complete(client,controller_ok=True,quiet_observed=True)
    assert len(client.writes)==1 and pump.unknown and lease.owner is pump.token


@pytest.mark.parametrize('quiet,okay',[(False,True),(True,False),(False,False)])
async def test_unverified_completion_cannot_clear_mailbox_or_release(quiet,okay):
    now,lease,pump=make();client=Client()
    with pytest.raises(ValueError):await pump.complete(client,controller_ok=okay,quiet_observed=quiet)
    assert not client.writes and lease.owner is pump.token


def test_wrong_owner_and_reordered_input_are_rejected():
    now,lease,pump=make()
    with pytest.raises(ValueError):lease.release(object())
    pump.offer(seq=2,captured_ns=now[0],action=1)
    for seq,stamp in [(2,now[0]),(1,now[0]),(3,now[0]+1),(3,now[0]-EXPIRY_NS)]:
        with pytest.raises(ValueError):pump.offer(seq=seq,captured_ns=stamp,action=2)


async def test_authority_loss_ends_pump_without_clearing_or_new_writes():
    now,lease,pump=make();client=Client();pump.start(client,1234)
    pump.offer(seq=0,captured_ns=now[0],action=1);await client.sent.wait()
    def revoked():raise ValueError('authority_revoked')
    pump.authorize=revoked
    await asyncio.wait_for(pump.worker,.2)
    assert len(client.writes)==1 and pump.closed and not pump.unknown
    assert 'authority_revoked' in pump.reason and lease.owner is pump.token


async def test_releasing_before_start_does_not_clear_an_unowned_mailbox():
    now,lease,pump=make();client=Client()
    with pytest.raises(ValueError):await pump.complete(client,controller_ok=True,quiet_observed=True)
    assert not client.writes and lease.owner is pump.token


@pytest.mark.parametrize('v',[(1,0,0),(-1,0,0),(1,1,0),(1,-2,3),(0,0,0),(-1,-1,-1)])
def test_atomic_vector_preserves_direction_and_bounds(v):
    import math
    word=encode_direction(65535,v)
    assert 0<word<2**31 and word//32768==65535
    p=word%32768;decoded=(p//1024-15,p//32%32-15,p%32-15)
    length=math.hypot(*v);dlen=math.hypot(*decoded)
    if length:assert sum(a*b for a,b in zip(v,decoded))/(length*dlen)>.998
    else:assert decoded==(0,0,0)
    assert encode_direction(65536+42,v)==encode_direction(42,v)


@pytest.mark.parametrize('v',[(float('nan'),0,0),(float('inf'),0,0),(True,0,0),(1,2)])
def test_invalid_vector_cannot_enter_mailbox(v):
    with pytest.raises(ValueError):encode_direction(1,v)
