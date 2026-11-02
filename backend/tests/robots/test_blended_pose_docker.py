from dataclasses import replace
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from app.robots.blended_pose_docker import DockerPosePort,audit_pose_path
from app.digital_twin.docker_runtime import docker_settings

class Model:
    model_sha256='a'*64
    def forward(self,q):return tuple(q[:3]),tuple(Rotation.from_rotvec(q[3:]).as_quat())
    def jacobian(self,q):return np.eye(6)
    def jdot(self,q,qd):return np.zeros((6,6))

def fixture():
    s=docker_settings().lebai
    s=replace(s,soft_joint_min_rad=(-3.,)*6,soft_joint_max_rad=(3.,)*6,
        startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3)
    p=[(0.,0.,.004*f) for f in (0,.25,.5,.75,1)]
    j=[(*x,math.radians(.3)*i/4,0.,-math.radians(.3)*i/4) for i,x in enumerate(p)]
    r=[Model().forward(q)[1] for q in j]
    return p,r,j,s,Model()

def test_combined_path_audits_both_tcp_rates_and_joint_acceleration():
    p,r,j,s,m=fixture()
    a=audit_pose_path(p,r,j,model=m,settings=s,reference=(0,0,0,1))
    ratio=math.radians(.3)*math.sqrt(2)/.004
    assert a.speed<=s.control.max_tcp_speed_mps
    assert a.speed*ratio<=s.control.max_tcp_rotation_radps
    assert a.acceleration*ratio<=s.control.max_tcp_angular_acceleration_radps2
    assert a.samples==5

@pytest.mark.parametrize('damage,reason', [('workspace','workspace'),('fk','model_mismatch'),('curve','rotation_path'),('nan','invalid_path'),('singular','singular')])
def test_combined_path_rejects_invalid_model_or_geometry(damage,reason):
    p,r,j,s,m=fixture()
    if damage=='workspace':s=replace(s,startup_tcp_max_m=(1.,1.,.001))
    if damage=='fk':m.forward=lambda q:((1,1,1),(0,0,0,1))
    if damage=='curve':r[2]=tuple(Rotation.from_rotvec([.01,0,0]).as_quat())
    if damage=='nan':j[2]=(float('nan'),)*6
    if damage=='singular':m.jacobian=lambda q:np.zeros((6,6))
    with pytest.raises(ValueError,match=reason):audit_pose_path(p,r,j,model=m,settings=s,reference=(0,0,0,1))

def test_physical_adapter_cannot_use_pose_port():
    with pytest.raises(ValueError,match='docker_adapter'):
        DockerPosePort(NS(settings=NS(ip='172.16.2.162')),object(),authorize=lambda:None)

def test_tool_input_log_coordinates_match_small_local_rotation():
    p=object.__new__(DockerPosePort)
    p.orientation=(0,0,0,1);p.origin=Rotation.identity()
    rotation=Rotation.from_rotvec([.4,.2,-.3])
    p.sample=NS(actual_tcp=NS(q=tuple(rotation.as_quat()),p=(0,0,0)))
    i=NS(kind='held',vector=(0,0,1),pitch=1,spin=-1)
    d=p.input_direction(i)
    exact=((rotation*Rotation.from_rotvec([1e-6,0,-1e-6])).as_rotvec()-rotation.as_rotvec())/1e-6
    assert np.allclose(d[3:],exact,atol=1e-6)


def test_micrometre_translation_drift_does_not_collapse_rotation_speed():
    p,r,j,s,m=fixture()
    p=[(0.,0.,.000005*i/4) for i in range(5)]
    j=[(*point,*old[3:]) for point,old in zip(p,j)]
    a=audit_pose_path(p,r,j,model=m,settings=s,reference=(0,0,0,1))
    assert a.speed>.02 and a.speed<=.12
    assert a.acceleration<=.2
