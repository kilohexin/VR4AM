from types import SimpleNamespace as NS
import pytest
from app.control.finite_position_types import PositionIntent
from app.control import docker_motion_session as m


def test_install_during_prepare_does_not_request_control_authority(monkeypatch):
    class Port:
        policy='docker_blended_candidate'
        def __init__(self,*args,**kwargs):pass
    monkeypatch.setattr(m,'DockerBlendPort',Port)
    calls=[]
    def authorize():calls.append(1);raise ValueError('preparation_lock')
    adapter=NS(settings=NS(startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3))
    session=m.DockerMotionSession(adapter,NS(generation=1),clock=lambda:1,recorder=None,authorize=authorize)
    assert not calls
    with pytest.raises(ValueError,match='preparation_lock'):
        session.offer(PositionIntent('web',1,1,1,'held',(0,0,1)))
    assert calls==[1]


def test_rotation_switch_drains_then_requires_new_input(monkeypatch):
    class Port:
        policy='docker_blended_candidate';coordinate_limit=.72
        def __init__(self,*args,axis=None,**kwargs):self.axis=axis;self.closed=False
        def check(self):pass
    monkeypatch.setattr(m,'DockerBlendPort',Port);monkeypatch.setattr(m,'DockerRotationPort',Port)
    a=NS(position_blended_rotation=True,settings=NS(startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3))
    s=m.DockerMotionSession(a,NS(generation=1),clock=lambda:1,recorder=None,authorize=lambda:None)
    s.offer(PositionIntent('web',1,1,1,'held',(0,0,0),pitch=1.))
    assert s.kind=='pitch' and s.gate.latest==(1.,0.,0.)
    s.active.worker=object()
    s.offer(PositionIntent('web',1,2,1,'held',(0,0,0),spin=-1.))
    assert s.kind=='pitch' and s.gate.released
    s.active.worker=None
    s.offer(PositionIntent('web',1,3,1,'held',(0,0,0),spin=-1.))
    assert s.kind=='spin' and s.gate.latest==(0.,0.,-1.)
