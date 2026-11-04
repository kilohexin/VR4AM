"""Private persistent Lua3010 RPC in a pinned network=none Docker only.

This pipe is NOT a motion owner. It must never launch an actuator program;
an unknown request retires the pipe and is reported, never retried.
"""
import asyncio
import re
from pathlib import Path
from dataclasses import dataclass
from time import perf_counter_ns

from scripts.check_native_cartesian_reference import native_admission
from scripts.cartesian_native_pipe import arguments


@dataclass(frozen=True)
class NativeReply:
    fields: tuple[str,...]
    sent_ns: int
    received_ns: int
    sampled_ms: int


class CartesianNativeRpc:
    def __init__(self,admission,*,clock=perf_counter_ns,boot_clock=False):
        self.admission=admission;self.clock=clock;self.proc=None;self.sequence=0
        self.lock=asyncio.Lock();self.closed=False;self.unknown=False;self.noise=[]
        if type(boot_clock) is not bool:raise ValueError('cartesian_rpc_clock_configuration')
        self.clock_source=(Path(__file__).with_name('cartesian_boot_clock.lua').read_text(encoding='utf-8')
            if boot_clock else 'local cartesian_now=timestamp\n')
        # Static helper only, no user code/string rewriting. Lua3010's source
        # ingestion is line-oriented; avoid re-sending comment/empty lines on
        # every private clock/signal request.
        self.clock_source=' '.join(line.strip() for line in self.clock_source.splitlines()
            if line.strip() and not line.lstrip().startswith('--'))

    async def open(self):
        if self.proc is not None or self.closed:raise ValueError('cartesian_rpc_retired')
        if await asyncio.to_thread(native_admission,self.admission['container_id'])!=self.admission:
            raise ValueError('cartesian_rpc_identity')
        self.proc=await asyncio.create_subprocess_exec(*arguments(self.admission),
            stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE)

    async def request(self,body):
        async with self.lock:
            if self.closed or self.proc is None or self.proc.returncode is not None:
                raise ValueError('cartesian_rpc_retired')
            # The driver supplies bounded signal/clock operations only.
            # Keep the actuator namespace outside this channel altogether.
            if (set(re.findall(r'lebai:(\w+)\s*\(',body))
                    -{'get_signal','get_signals','set_signal','set_signals'}
                    or re.search(r'\b(?:load|require|exec|dofile|move\w*|stop\w*|start\w*)\s*\(',body)):
                raise ValueError('cartesian_rpc_operation')
            self.sequence+=1;token=str(self.sequence)
            source=('program_begin(1)\ndisable_auto_sync()\n'
                +self.clock_source+'\n'
                +'local ok,value=pcall(function() '+body+' end)\n'
                +"print('CART_RPC',"+token+",ok,cartesian_now(),tostring(value))\nprogram_end(1)\n")
            sent=self.clock()
            try:
                self.proc.stdin.write(source.encode());await self.proc.stdin.drain()
                async with asyncio.timeout(2):
                    while raw:=await self.proc.stdout.readline():
                        received=self.clock()
                        row=tuple(raw.decode('utf-8',errors='strict').strip().split('\t'))
                        if row[:2]==('CART_RPC',token):
                            if len(row)<5 or row[2]!='true':raise ValueError('cartesian_rpc_native_error')
                            stamp=int(row[3])
                            if stamp<0:raise ValueError('cartesian_rpc_clock')
                            return NativeReply(row[4:],sent,received,stamp)
                        if len(self.noise)>=128:raise ValueError('cartesian_rpc_noise_budget')
                        self.noise.append(row)
                    raise ValueError('cartesian_rpc_eof')
            except BaseException:
                self.unknown=True;self.closed=True;raise

    async def close(self):
        self.closed=True
        if self.proc is not None:
            if self.proc.stdin and not self.proc.stdin.is_closing():self.proc.stdin.close()
            # No actuator program is ever launched on this pipe. Closing it
            # neither confirms nor cancels a separately owned motion program.
            if self.proc.returncode is None:self.proc.terminate()
            await self.proc.wait()
