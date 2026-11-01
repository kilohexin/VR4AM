from types import SimpleNamespace as NS
import asyncio
import pytest

from app.robots.blended_position_docker import DockerBlendPort, DockerBlendPlan
from app.robots.lebai_motion_calls import OwnedMotionCalls


def rig():
    writes=[]
    async def movel(*args):writes.append(args);return 42
    native=NS(movel=movel)
    client=NS(native=native,move_entries=0,move_returned=True,move_unknown=False)
    sample=NS(captured_ns=1,actual_tcp=NS(p=(0.,0.,0.)))
    adapter=NS(_clock=lambda:1,_position_feedback_limits=lambda s:None,
        _sdk_lock=asyncio.Lock(),_motion_calls=OwnedMotionCalls(),position_aborted=False,
        position_busy=False,_begin_motion_epoch=lambda:None,_latched_fault=None)
    # Isolate the port's native-entry boundary; constructor admission is tested
    # separately and no SDK or network is used by this fixture.
    port=object.__new__(DockerBlendPort)
    port.adapter=adapter;port.client=client;port.binding=object();port.closed=False
    port.check=lambda:None;port.sample=sample;port.ids=set()
    target=NS(position=(0.,0.,.004))
    plan=DockerBlendPlan(target,dict(x=0.,y=0.,z=.004,rx=0.,ry=0.,rz=0.),
        NS(acceleration=.025,speed=.01),port.binding,(0.,0.,0.))
    port.plan=plan
    return port,plan,writes


async def test_native_entry_is_once_and_counted_before_actual_call():
    p,plan,writes=rig();checks=[]
    ident=await p.submit(plan,entering=lambda:checks.append('entered'))
    assert ident==42 and checks==['entered'] and len(writes)==1
    assert writes[0][1:]==(.025,.01,0.,.001)
    assert p.adapter.position_busy and p.client.move_entries==1
    with pytest.raises(ValueError,match='obsolete'):await p.submit(plan,entering=lambda:None)
    assert len(writes)==1


@pytest.mark.parametrize('damage,reason',[
    ('expired','feedback_expired'),('off_path','path_changed'),('revoked','entry_revoked')])
async def test_entry_rejections_are_zero_write_and_not_remote_unknown(damage,reason):
    p,plan,writes=rig()
    if damage=='expired':p.adapter._clock=lambda:100_000_001
    if damage=='off_path':p.sample.actual_tcp.p=(.001,0.,0.)
    def entering():
        if damage=='revoked':raise ValueError('blend_entry_revoked')
    with pytest.raises(ValueError,match=reason):await p.submit(plan,entering=entering)
    assert not writes and not p.adapter._motion_calls.unknown_methods
    assert not p.adapter.position_busy


async def test_native_error_retains_unknown_write_and_does_not_retry():
    p,plan,writes=rig()
    async def fail(*args):writes.append(args);raise RuntimeError('native_failure')
    p.client.native.movel=fail
    with pytest.raises(RuntimeError):await p.submit(plan,entering=lambda:None)
    assert len(writes)==1 and p.client.move_unknown
    assert p.adapter._motion_calls.unknown_methods=={'movel'}
    assert p.adapter.position_busy and p.adapter._motion_write_stop_required


def test_real_or_generic_adapter_cannot_construct_port():
    with pytest.raises(ValueError,match='docker_adapter'):
        DockerBlendPort(NS(settings=NS(ip='127.0.0.1')),object(),authorize=lambda:None)


def test_movel_unknown_is_added_only_to_docker_stop_contract():
    from app.robots.finite_position_docker import DockerFinitePositionAdapter
    from app.robots.finite_position_web import RealFinitePositionAdapter
    assert 'movel' in DockerFinitePositionAdapter._stop_blocking_motion_methods
    assert 'movel' not in RealFinitePositionAdapter._stop_blocking_motion_methods
