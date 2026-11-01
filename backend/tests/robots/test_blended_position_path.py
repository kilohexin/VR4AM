from dataclasses import replace
import numpy as np
import pytest
from app.digital_twin.docker_runtime import docker_settings
from app.robots.blended_position_path import audit_blended_path


class Model:
    model_sha256='a'*64
    gain=1.
    def forward(self,q):return (tuple(np.asarray(q[:3])*self.gain), (0.,0.,0.,1.))
    def jacobian(self,q):return np.eye(6)*self.gain
    def jdot(self,q,qd):return np.zeros((6,6))


def fixture():
    s=docker_settings().lebai
    s=replace(s,soft_joint_min_rad=(-3.,)*6,soft_joint_max_rad=(3.,)*6,
              startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3)
    p=[(0.,0.,z) for z in (0.,.001,.002,.003,.004)]
    q=[(*point,0.,0.,0.) for point in p]
    return p,q,s,Model()


def test_cartesian_path_uses_inverse_jacobian_rate_limits():
    p,q,s,m=fixture();m.gain=.1
    q=[tuple(x/.1 for x in row) for row in q]
    result=audit_blended_path(p,q,(0,0,0,1),model=m,settings=s)
    assert result.speed<=.02 and result.acceleration/.1<=s.control.max_joint_acceleration_radps2
    assert result.samples==5


@pytest.mark.parametrize('damage,reason',[
    ('curve','not_linear'),('singular','singular'),('workspace','workspace'),
    ('joint','joint_limit'),('fk','model_mismatch'),('nan','invalid_path')])
def test_bad_cartesian_path_never_produces_motion_parameters(damage,reason):
    p,q,s,m=fixture()
    if damage=='curve':p[2]=(.001,0.,.002)
    if damage=='singular':m.jacobian=lambda q:np.zeros((6,6))
    if damage=='workspace':s=replace(s,startup_tcp_max_m=(1.,1.,.003))
    if damage=='joint':s=replace(s,soft_joint_max_rad=(3.,3.,.003,3.,3.,3.))
    if damage=='fk':m.forward=lambda q:((1.,1.,1.),(0,0,0,1))
    if damage=='nan':q[1]=(float('nan'),)*6
    with pytest.raises(ValueError,match=reason):audit_blended_path(p,q,(0,0,0,1),model=m,settings=s)
