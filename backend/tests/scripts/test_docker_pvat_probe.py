import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('docker_pvat_probe',Path(__file__).resolve().parents[3]/'scripts/docker_pvat_probe.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_reference_derivatives_are_consistent_and_bounded():
    origin=(.1,-.2,.3,-.4,.5,-.6);eps=1e-5
    for t in np.linspace(.01,1.99,100):
        p,v,a=m.reference(origin,t)
        before=m.reference(origin,t-eps);after=m.reference(origin,t+eps)
        assert np.allclose((np.array(after[0])-before[0])/(2*eps),v,atol=1e-9)
        assert np.allclose((np.array(after[1])-before[1])/(2*eps),a,atol=1e-9)
        assert max(map(abs,v))<.01 and max(map(abs,a))<.015
        assert .1<=p[0]<=.11 and tuple(p[1:])==origin[1:]
    assert m.reference(origin,0)==(list(origin),[0.]*6,[0.]*6)
    assert m.reference(origin,2)[1:]==([0.]*6,[0.]*6)


@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_invalid_reference_time_is_rejected(value):
    with pytest.raises(ValueError):m.reference([0.]*6,value)


async def test_nonidle_preflight_never_sends_motion():
    class Client:
        async def get_robot_state(self):return 'MOVING'
        async def get_kin_data(self):return dict(actual_joint_pose=[0.]*6,actual_joint_speed=[0.]*6)
        async def move_pvat(self,*args):raise AssertionError('unexpected_write')
    result=await m.probe(Client())
    assert not result['passed'] and not result['commands'] and not result['stop_sent']
    assert result['error'].endswith('requires_idle')


async def test_unknown_rpc_never_retries_and_preserves_unknown_after_quiet_stop():
    import asyncio
    class Client:
        moves=0;stops=0
        async def get_robot_state(self):return 'IDLE'
        async def get_kin_data(self):return dict(actual_joint_pose=[0.]*6,actual_joint_speed=[0.]*6)
        async def move_pvat(self,*args):
            self.moves+=1
            await asyncio.sleep(1.)
        async def stop_move(self):self.stops+=1
    client=Client();result=await m.probe(client)
    assert client.moves==client.stops==1
    assert result['unknown_writes']==['move_pvat']
    assert result['stop_observed'] and not result['passed']
    assert not result['unknown_reads']
