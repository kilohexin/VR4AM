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


def test_combined_pose_keeps_one_executor_for_translation_rotation_and_mix(monkeypatch):
    class Port:
        policy='docker_blended_candidate';dimensions=6;radians_per_unit=2.
        def __init__(self,*args,**kwargs):pass
        def input_direction(self,i):return (*i.vector,i.pitch,0.,i.spin)
    monkeypatch.setattr(m,'DockerPosePort',Port)
    a=NS(position_blended_rotation=True,settings=NS(startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3))
    s=m.DockerMotionSession(a,NS(generation=1),clock=lambda:1,recorder=None,authorize=lambda:None)
    active=s.active
    for seq,(v,pitch,spin) in enumerate([((0,0,1),0.,0.),((0,0,1),1.,-1.),((0,0,0),1.,0.)],1):
        s.offer(PositionIntent('web',1,seq,1,'held',v,pitch=pitch,spin=spin))
        assert s.active is active and s.kind=='pose' and len(s.gate.latest)==6
