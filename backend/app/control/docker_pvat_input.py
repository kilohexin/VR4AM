"""Latest-input mailbox and exclusive lease for the isolated Docker PVAT path.

No endpoint selection or automatic motion restart. The caller must obtain
Docker admission and observe controller completion before releasing ownership.
"""
import asyncio
import math
from dataclasses import dataclass

CHANNEL=240
MODULUS=33554432
EXPIRY_NS=100_000_000


def encode(stamp_ms,action):
    if type(stamp_ms) is not int or stamp_ms<0 or type(action) is not int or not 0<=action<=6:
        raise ValueError('invalid_live_input')
    return (stamp_ms%MODULUS)*64+action


def encode_direction(stamp_ms,direction):
    """One atomic int32: 16-bit ms clock plus three signed 5-bit components."""
    if (type(stamp_ms) is not int or stamp_ms<0 or type(direction) not in (tuple,list)
            or len(direction)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in direction)):
        raise ValueError('invalid_pvat_direction')
    length=math.hypot(*direction)
    if not math.isfinite(length):raise ValueError('invalid_pvat_direction')
    unit=tuple(x/length for x in direction) if length else (0.,0.,0.)
    quantized=tuple(max(-15,min(15,round(x*15)))+15 for x in unit)
    return (stamp_ms%65536)*32768+quantized[0]*1024+quantized[1]*32+quantized[2]


def encode_pose(stamp_ms,direction):
    """Atomic 31 bits: 11-bit clock and five signed 4-bit components.

    A >100ms input gap permanently retires the pump/controller; a delayed
    controller also rejects >160ms cadence loss before writing any point.
    """
    if (type(stamp_ms) is not int or stamp_ms<0 or type(direction) not in (tuple,list)
            or len(direction)!=5 or any(type(x) not in (int,float) or not math.isfinite(x) for x in direction)
            or any(abs(x)>1 for x in direction[3:])):raise ValueError('invalid_pvat_pose')
    length=math.hypot(*direction[:3])
    if not math.isfinite(length):raise ValueError('invalid_pvat_pose')
    v=tuple(x/length for x in direction[:3]) if length else (0.,0.,0.)
    word=stamp_ms%2048
    for x in (*v,*direction[3:]):word=word*16+max(0,min(14,round(x*7)+7))
    return word


class ExecutorLease:
    def __init__(self):self.owner=None

    def acquire(self):
        if self.owner is not None:raise ValueError('executor_occupied')
        self.owner=object()
        return self.owner

    def require(self,token):
        if token is not self.owner or token is None:raise ValueError('executor_revoked')

    def release(self,token):
        self.require(token);self.owner=None


@dataclass(frozen=True)
class LiveInput:
    seq:int
    captured_ns:int
    action:object


class DockerPvatInput:
    def __init__(self,lease,*,clock,authorize,encoder=encode):
        self.lease,self.clock,self.authorize=lease,clock,authorize
        self.token=lease.acquire()
        self.encoder=encoder
        self.latest=None;self.seq=-1;self.closed=False;self.unknown=False
        self.reason=None;self.worker=None;self.writes=[]
        self._finish=asyncio.Event();self.anchor=None

    def _check(self):
        self.lease.require(self.token);self.authorize()
        if self.closed:raise ValueError('pvat_input_closed')

    def offer(self,*,seq,captured_ns,action):
        self._check();self.encoder(0,action)
        if (type(seq) is not int or seq<=self.seq or type(captured_ns) is not int
                or not 0<=self.clock()-captured_ns<EXPIRY_NS
                or self.latest is not None and captured_ns<self.latest.captured_ns):
            raise ValueError('pvat_input_rejected')
        self.seq=seq;self.latest=LiveInput(seq,captured_ns,action)

    def start(self,client,controller_ms):
        self._check();encode(controller_ms,0)
        if self.worker is not None:raise ValueError('pvat_input_already_started')
        self.anchor=(self.clock(),controller_ms)
        self.worker=asyncio.create_task(self._pump(client))

    async def _pump(self,client):
        try:
            while not self._finish.is_set():
                self._check();sample=self.latest
                if sample is not None:
                    age=self.clock()-sample.captured_ns
                    if not 0<=age<EXPIRY_NS:
                        self.reason='pvat_input_expired';return
                    # Encode capture time, never refresh a cached intent while
                    # repeating the signal. Pre-handshake samples are rejected.
                    delta=sample.captured_ns-self.anchor[0]
                    if delta<0:raise ValueError('pvat_input_before_handshake')
                    word=self.encoder(self.anchor[1]+delta//1_000_000,sample.action)
                    try:await asyncio.wait_for(client.set_signal(CHANNEL,word),.1)
                    except BaseException:
                        self.unknown=True;raise
                    self.writes.append((sample.seq,word))
                try:await asyncio.wait_for(self._finish.wait(),.025)
                except TimeoutError:pass
        except asyncio.CancelledError:
            self.reason='pvat_input_cancelled';raise
        except Exception as error:self.reason=type(error).__name__+':'+str(error)
        finally:self.closed=True

    async def withdraw(self):
        # Join rather than cancelling an in-flight SDK write. Unknown outcomes
        # retain the lease even if subsequent feedback looks quiet.
        self._finish.set();self.closed=True
        if self.worker is not None:await asyncio.shield(self.worker)

    async def complete(self,client,*,controller_ok,quiet_observed):
        await self.withdraw();self.lease.require(self.token)
        if self.worker is None or not controller_ok or not quiet_observed or self.unknown:
            raise ValueError('pvat_completion_unverified')
        try:
            await asyncio.wait_for(client.set_signal(CHANNEL,0),.1)
            if await asyncio.wait_for(client.get_signal(CHANNEL),.1)!=0:
                raise ValueError('pvat_mailbox_not_cleared')
        except BaseException:
            self.unknown=True;raise
        self.lease.release(self.token)
