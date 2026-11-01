"""Nominal pure-rotation path audit for the admitted Docker candidate."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from app.robots.blended_position_path import BlendPathAudit
from app.robots.finite_position_plan import angle


def audit_rotation_path(position, orientations, joints, *, model, settings, reference):
    if len(orientations)!=5 or len(joints)!=5:raise ValueError('blend_rotation_samples')
    q=np.asarray(joints,dtype=float)
    if q.shape!=(5,6) or not np.isfinite(q).all():raise ValueError('blend_invalid_path')
    lo=np.asarray(settings.soft_joint_min_rad)+settings.joint_limit_margin_rad
    hi=np.asarray(settings.soft_joint_max_rad)-settings.joint_limit_margin_rad
    if np.any(q<lo) or np.any(q>hi):raise ValueError('blend_joint_limit')
    if np.max(np.abs(q-q[0]))>settings.control.max_joint_step_rad:raise ValueError('blend_joint_jump')
    if any(not lo<=x<=hi for x,lo,hi in zip(position,settings.startup_tcp_min_m,settings.startup_tcp_max_m)):
        raise ValueError('blend_workspace')
    start=Rotation.from_quat(orientations[0])
    world_delta=(Rotation.from_quat(orientations[-1])*start.inv()).as_rotvec()
    length=np.linalg.norm(world_delta)
    if not 1e-6<length<=math.radians(1.01):raise ValueError('blend_rotation_length')
    twist=np.r_[np.zeros(3),world_delta/length];matrices=[]
    for i,(configuration,orientation) in enumerate(zip(q,orientations)):
        expected=(Rotation.from_rotvec(world_delta*i/4)*start).as_quat()
        if angle(expected,orientation)>1e-6:raise ValueError('blend_rotation_path')
        if angle(reference,orientation)>math.pi/2:raise ValueError('blend_rotation_range')
        p,r=model.forward(tuple(configuration))
        if math.dist(p,position)>1e-5 or angle(r,orientation)>1e-4:raise ValueError('blend_model_mismatch')
        j=np.asarray(model.jacobian(tuple(configuration)))
        if j.shape!=(6,6) or not np.isfinite(j).all() or np.linalg.cond(j)>1e6:raise ValueError('blend_singular_path')
        matrices.append(j)
    limits=settings.control
    speed=min(.02,limits.max_tcp_rotation_radps);acceleration=min(.1,limits.max_tcp_angular_acceleration_radps2)
    for _ in range(10):
        valid=True
        for configuration,j in zip(q,matrices):
            qd=np.linalg.solve(j,twist*speed)
            jd=np.asarray(model.jdot(tuple(configuration),tuple(qd)))
            if jd.shape!=(6,6) or not np.isfinite(jd).all():raise ValueError('blend_invalid_model')
            bound=np.abs(np.linalg.solve(j,twist*acceleration))+np.abs(np.linalg.solve(j,jd@qd))
            valid &= np.max(np.abs(qd))<=limits.max_joint_speed_radps and np.max(bound)<=limits.max_joint_acceleration_radps2
        if valid:return BlendPathAudit(speed,acceleration,model.model_sha256,5)
        speed/=2;acceleration/=4
    raise ValueError('blend_model_rates')
