"""Private publisher: only a completed host-planner transaction can commit."""
import struct

from app.control.cartesian_bundle_wire import encode


class CartesianNativePublisher:
    def __init__(self,rpc,planner,*,epoch_ms,clock_pair,host_clock):
        self.rpc=rpc;self.planner=planner;self.epoch=epoch_ms;self.pair=clock_pair;self.host_clock=host_clock
        self.bank=0;self.authority_sequence=0;self.closed=False
        self.words=struct.unpack('<4i',bytes.fromhex(planner.session))

    async def offer_authority(self,captured_ns):
        if self.closed or self.planner.closed:raise ValueError('cartesian_publish_retired')
        capture=self.pair.capture_lower_ms(captured_ns)-self.epoch
        if not 0<=self.host_clock()-captured_ns<100_000_000 or capture<0:
            raise ValueError('cartesian_publish_input')
        self.authority_sequence+=1
        word=(capture+1)*1024+self.authority_sequence%1024
        if word>2**31-1:raise ValueError('cartesian_publish_clock_range')
        payload='{'+','.join(str(x) for x in (*self.words,self.planner.generation))+'}'
        result=await self.rpc.request('assert(set_signals(241,'+payload+'));'
            +'assert(set_signal(240,'+str(word)+"));return 'authority'")
        if result.fields!=('authority',):raise ValueError('cartesian_publish_reply')

    async def publish(self,packet):
        p=self.planner
        if (self.closed or p.closed or p.pending is None or packet is not p.pending.bundle):
            raise ValueError('cartesian_publish_uncertified')
        words=encode(packet);offset=self.bank*120
        payload='{'+','.join(str(x) for x in words[1:])+'}'
        # One bounded native transaction avoids an unnecessary host roundtrip.
        # Native time and independent lease are rechecked AFTER payload I/O.
        # An in-flight commit remains owned until its reply/ACK is known.
        due=self.epoch+packet.due_ms;capture=self.epoch+packet.captured_ms
        result=await self.rpc.request('assert(set_signal('+str(offset)+',0));assert(set_signals('
            +str(offset+1)+','+payload+'));if cartesian_now()>='+str(due)+' or cartesian_now()-'+str(capture)
            +">=100 or get_signal(240)==0 then return 'late' end;assert(set_signal("+str(offset)+','+str(packet.sequence)
            +"));return 'committed'")
        if result.fields==('late',):return False
        if result.fields!=('committed',):raise ValueError('cartesian_publish_reply')
        self.bank=1-self.bank;return True

    async def release(self):
        self.closed=True
        # Do not clear math banks while the native reader may be in flight.
        # A separate genuine-input release makes the owner drop future leads.
        return await self.rpc.request("assert(set_signal(240,0));return 'released'")
