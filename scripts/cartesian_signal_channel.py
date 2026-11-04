"""Explicit virtual3030-only SDK tunnel into a verified network=none Docker.

No Docker port publication, external bind, physical default or actuation
method. The host endpoint is an ephemeral 127.0.0.1 listener; the in-container
endpoint is the fixed VIRTUAL SDK3030. Unknown calls retire this channel.
"""
import asyncio
from time import perf_counter_ns

from scripts.cartesian_native_pipe import BRIDGE
from scripts.check_native_cartesian_reference import native_admission

SDK_BRIDGE=BRIDGE.replace('PeerPort=>3010','PeerPort=>3030')
ALLOWED=frozenset({'get_signal','get_signals','set_signal','set_signals','get_kin_data','get_robot_state'})


class CartesianSignalChannel:
    def __init__(self,admission):
        self.admission=admission;self.server=None;self.client=None;self.closed=False
        self.pipes=set();self.connections=set();self.unknown=[];self.pending=set();self.port=None

    async def _accept(self,reader,writer):
        if self.closed or len(self.connections)>=4:
            writer.close();return
        self.connections.add(writer);proc=None
        try:
            proc=await asyncio.create_subprocess_exec('docker','exec','-i',self.admission['container_id'],
                'perl','-e',SDK_BRIDGE,stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE)
            self.pipes.add(proc)
            async def incoming():
                while data:=await reader.read(16384):
                    proc.stdin.write(data);await proc.stdin.drain()
            async def outgoing():
                while data:=await proc.stdout.read(16384):
                    writer.write(data);await writer.drain()
            tasks=[asyncio.create_task(incoming()),asyncio.create_task(outgoing())]
            _,left=await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
            for task in left:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
        finally:
            if proc is not None:
                if proc.returncode is None:proc.terminate()
                await proc.wait();self.pipes.discard(proc)
            writer.close();self.connections.discard(writer)

    async def open(self):
        if self.server is not None or self.closed:raise ValueError('cartesian_signal_retired')
        if await asyncio.to_thread(native_admission,self.admission['container_id'])!=self.admission:
            raise ValueError('cartesian_signal_identity')
        self.server=await asyncio.start_server(self._accept,'127.0.0.1',0)
        self.port=self.server.sockets[0].getsockname()[1]
        import lebai_sdk
        # INT custom port is mandatory. Never invoke SDK's default3031 or
        # infer a real address from settings/environment.
        lebai_sdk.init()
        self.client=await lebai_sdk.connect('127.0.0.1',int(self.port),{'request_timeout':2})

    async def call(self,method,*args):
        if method not in ALLOWED:raise ValueError('cartesian_signal_operation')
        if self.closed or self.client is None:raise ValueError('cartesian_signal_retired')
        task=asyncio.ensure_future(getattr(self.client,method)(*args));self.pending.add(task)
        task.add_done_callback(self.pending.discard)
        try:return await asyncio.shield(task)
        except BaseException:
            self.unknown.append(method);self.closed=True;raise

    async def close(self):
        self.closed=True
        if self.pending:await asyncio.gather(*self.pending,return_exceptions=True)
        if self.server is not None:self.server.close();await self.server.wait_closed()
        for writer in list(self.connections):writer.close()
        for proc in list(self.pipes):
            if proc.returncode is None:proc.terminate()
        await asyncio.gather(*(proc.wait() for proc in list(self.pipes)),return_exceptions=True)


async def probe(admission):
    channel=CartesianSignalChannel(admission)
    try:
        await channel.open();samples=[]
        for _ in range(10):
            start=perf_counter_ns();value=await channel.call('get_signals',240,6)
            samples.append(dict(ms=(perf_counter_ns()-start)/1e6,values=value))
        return dict(samples=samples,unknown=channel.unknown,port=channel.port)
    finally:await channel.close()
