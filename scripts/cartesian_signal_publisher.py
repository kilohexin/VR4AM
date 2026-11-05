"""Direct virtual-SDK signal transactions; no motion method exposed."""
import asyncio
from scripts.cartesian_native_publisher import CartesianNativePublisher
from app.control.cartesian_bundle_wire import encode


class CartesianSignalPublisher(CartesianNativePublisher):
    def __init__(self,channel,*args,**kwargs):
        super().__init__(channel,*args,**kwargs)
        self.channel=channel;self.writes=asyncio.Lock()

    async def offer_authority(self,captured_ns):
        async with self.writes:
            if self.closed or self.planner.closed:return False
            capture=self.pair.capture_lower_ms(captured_ns)-self.epoch
            if not 0<=self.host_clock()-captured_ns<100_000_000 or capture<0:
                raise ValueError('cartesian_publish_input')
            self.authority_sequence+=1
            word=(capture+1)*1024+self.authority_sequence%1024
            if word>2**31-1:raise ValueError('cartesian_publish_clock_range')
            await self.channel.call('set_signals',241,[*self.words,self.planner.generation])
            if self.closed or self.planner.closed:return False
            await self.channel.call('set_signal',240,word)

    async def publish(self,packet):
        async with self.writes:
            p=self.planner
            if self.closed or p.closed:return False
            if p.pending is None or packet is not p.pending.bundle:
                raise ValueError('cartesian_publish_uncertified')
            words=encode(packet);offset=self.bank*120
            await self.channel.call('set_signal',offset,0)
            await self.channel.call('set_signals',offset+1,list(words[1:]))
            if self.closed or p.closed:return False
            # The native owner is the final actual-time/100ms admission gate.
            # No host timestamp is substituted for packet.captured_ms.
            await self.channel.call('set_signal',offset,packet.sequence)
            self.bank=1-self.bank;return True

    async def release(self):
        self.closed=True
        async with self.writes:
            await self.channel.call('set_signals',241,[*self.words,self.planner.generation])
            await self.channel.call('set_signal',240,-1)
