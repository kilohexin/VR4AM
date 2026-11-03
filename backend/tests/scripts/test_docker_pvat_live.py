import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('docker_pvat_live',Path(__file__).resolve().parents[3]/'scripts/docker_pvat_live.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_atomic_mailbox_fits_int32_and_wraps_clock():
    for action in range(7):
        word=m.encode(m.MODULUS-1,action)
        assert 0<=word<2**31 and word%64==action
        assert word//64==m.MODULUS-1
        assert m.encode(m.MODULUS+42,action)==m.encode(42,action)


@pytest.mark.parametrize('stamp,action',[(-1,0),(True,0),(0,True),(1.5,1),(0,-1),(0,7)])
def test_invalid_inputs_cannot_encode(stamp,action):
    with pytest.raises(ValueError):m.encode(stamp,action)


def test_input_loss_is_silence_and_not_a_release_packet():
    assert m.schedule(0)==0 and m.schedule(.4)==1
    assert m.schedule(1.3)==0 and m.schedule(1.8)==2
    assert m.schedule(7.39)==0 and m.schedule(7.4)==6
    assert m.schedule(8.3) is None


def test_completion_without_motion_and_expiry_cannot_pass():
    assert not m.analyze(['LIVE_DONE\ttrue\tnil\t0'])['trace_passed']


def test_trace_requires_three_axis_progress_expiry_and_no_abort():
    lines=[]
    for action in range(7):
        for tick in range(2):
            d=tick*.002*(1 if action%2 else -1)
            lines.append(f'LIVE_POINT\t{len(lines)*80}\t{action}\t{d}\t{d}\t{d}\t0')
    lines+=['LIVE_EXPIRED\t1200\t14','LIVE_RELEASE\t1200\t14','LIVE_QUIET\t1500\t0\t0\t0','LIVE_DONE\ttrue\tnil\t14']
    result=m.analyze(lines)
    assert result['trace_passed'] and result['release_to_quiet_ms']==300
    assert result['max_send_gap_ms']==80
    assert not m.analyze(lines+['LIVE_ABORT\ttrue\ttrue'])['trace_passed']
    assert not m.analyze([s for s in lines if not s.startswith('LIVE_EXPIRED')])['trace_passed']


@pytest.mark.parametrize('unknown',[False,True])
async def test_controller_end_stops_input_and_unknown_write_is_never_cleared(monkeypatch,unknown):
    import asyncio
    from app.robots import lebai_sdk_bridge
    writes=[];sent=asyncio.Event()
    class Client:
        async def get_signal(self,channel):return 0
        async def set_signal(self,channel,value):
            writes.append(value);sent.set()
            if unknown:raise TimeoutError('unknown')
    async def connect():return Client()
    class Input:
        def write(self,data):pass
        async def drain(self):pass
        def close(self):pass
    class Output:
        count=0
        async def readline(self):
            self.count+=1
            if self.count==1:return b'LIVE_READY\t1234\n'
            await sent.wait()
            return b'LIVE_DONE\tfalse\tpreflight_failed\t0\n'
    class Process:
        stdin=Input();stdout=Output()
        async def wait(self):return 0
    async def spawn(*args,**kwargs):return Process()
    monkeypatch.setattr(lebai_sdk_bridge,'connect_docker_simulator_client',connect)
    monkeypatch.setattr(m.asyncio,'create_subprocess_exec',spawn)
    result=await m.run({'container_id':'fake-local'})
    assert result['completed'] and not result['passed']
    assert result['unknown_input']==unknown
    assert len(writes)==1
    assert 'mailbox_cleared' not in result and not result['lease_released']
