"""Bounded combined translation/rotation stream for the local Docker only.

Scaled rotation-vector coordinates bound the total outstanding pose lead.
Native targets use linear TCP interpolation and quaternion SLERP, with five
IK/FK/J/Jdot samples. No physical selector imports this port.
"""
import asyncio
import math
import numpy as np
from scipy.spatial.transform import Rotation
from app.robots.blended_position_docker import DockerBlendPort, DockerBlendPlan
from app.robots.blended_position_path import BlendPathAudit, audited_rates
from app.robots.finite_position_plan import angle
from app.robots.lebai_codec import pose_to_lebai, joint_vector
from app.schemas.messages import Pose


class DockerPosePort(DockerBlendPort):
    dimensions = 6
    direction_cone_deg = 35
    input_expiry_drains = True
    radians_per_unit = math.radians(1.)/.004

    def input_direction(self, intent):
        if intent.kind == 'release':return (0.,)*6
        # Tool-local twists mapped to the reference rotation-vector tangent.
        # Refresh the mapping from measured pose, never from queued targets.
        local = np.array((intent.pitch, 0., intent.spin))
        if self.sample is not None:
            r = (self.origin.inv()*Rotation.from_quat(self.sample.actual_tcp.q)).as_rotvec()
            theta = np.linalg.norm(r)
            skew = np.array(((0.,-r[2],r[1]),(r[2],0.,-r[0]),(-r[1],r[0],0.)))
            c = 1/12 if theta < 1e-5 else (1-theta/(2*math.tan(theta/2)))/(theta*theta)
            local = (np.eye(3)+.5*skew+c*(skew@skew))@local
        return (*intent.vector, *map(float,local))

    def _initialize_reference(self, s):
        self.origin = Rotation.from_quat(self.orientation)

    def _coordinates(self, s):
        rotation = (self.origin.inv()*Rotation.from_quat(s.actual_tcp.q)).as_rotvec()/self.radians_per_unit
        return (*s.actual_tcp.p, *map(float,rotation))

    def _validate_pose(self, s):
        reference = self.adapter._position_pitch_reference
        if reference is None or angle(reference,s.actual_tcp.q)>math.pi/2:
            raise ValueError('blend_rotation_range')

    async def audit(self, target):
        self.check();a=self.adapter
        if self.sample is None:raise ValueError('blend_feedback_missing')
        start=self._coordinates(self.sample)
        goal=self.origin*Rotation.from_rotvec(np.array(target.position[3:])*self.radians_per_unit)
        first=Rotation.from_quat(self.sample.actual_tcp.q)
        rotation=(goal*first.inv()).as_rotvec()
        positions=[tuple(x+(y-x)*f for x,y in zip(start[:3],target.position[:3])) for f in (0.,.25,.5,.75,1.)]
        orientations=[tuple((Rotation.from_rotvec(rotation*f)*first).as_quat()) for f in (0.,.25,.5,.75,1.)]
        joints=[tuple(self.sample.actual_q)]
        async with asyncio.timeout(1.):
            for position,orientation in zip(positions[1:],orientations[1:]):
                pose=pose_to_lebai(Pose(p=position,q=orientation))
                raw=await a._position_math('kinematics_inverse',lambda:self.client.kinematics_inverse(pose,list(joints[-1])),self.check)
                if raw is None:raise ValueError('blend_ik_unreachable')
                joints.append(joint_vector(raw,'blend_ik_invalid'))
            audit=await asyncio.to_thread(audit_pose_path,positions,orientations,joints,
                model=a.position_model,settings=a.settings,reference=a._position_pitch_reference)
        self.check()
        self.plan=DockerBlendPlan(target,pose_to_lebai(Pose(p=positions[-1],q=orientations[-1])),audit,self.binding,start,tuple(joints))
        return self.plan


def audit_pose_path(positions, orientations, joints, *, model, settings, reference):
    if len(positions)!=5 or len(orientations)!=5 or len(joints)!=5:raise ValueError('blend_path_samples')
    p=np.asarray(positions,dtype=float);q=np.asarray(joints,dtype=float)
    if p.shape!=(5,3) or q.shape!=(5,6) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('blend_invalid_path')
    delta=p[-1]-p[0];distance=float(np.linalg.norm(delta))
    first=Rotation.from_quat(orientations[0])
    rotation=(Rotation.from_quat(orientations[-1])*first.inv()).as_rotvec()
    radians=float(np.linalg.norm(rotation))
    if distance>.008000001 or radians>math.radians(2.01) or max(distance,radians)<1e-6:
        raise ValueError('blend_pose_length')
    if not np.allclose(p,p[0]+np.linspace(0,1,5)[:,None]*delta,atol=1e-9,rtol=0):
        raise ValueError('blend_path_not_linear')
    lo=np.asarray(settings.soft_joint_min_rad)+settings.joint_limit_margin_rad
    hi=np.asarray(settings.soft_joint_max_rad)-settings.joint_limit_margin_rad
    if np.any(q<lo) or np.any(q>hi):raise ValueError('blend_joint_limit')
    if np.max(np.abs(q-q[0]))>settings.control.max_joint_step_rad:raise ValueError('blend_joint_jump')
    if np.any(p<settings.startup_tcp_min_m) or np.any(p>settings.startup_tcp_max_m):raise ValueError('blend_workspace')
    matrices=[]
    for i,(point,orientation,configuration) in enumerate(zip(p,orientations,q)):
        if angle((Rotation.from_rotvec(rotation*i/4)*first).as_quat(),orientation)>1e-6:
            raise ValueError('blend_rotation_path')
        if angle(reference,orientation)>math.pi/2:raise ValueError('blend_rotation_range')
        actual,quat=model.forward(tuple(configuration))
        if math.dist(actual,point)>1e-5 or angle(quat,orientation)>1e-4:raise ValueError('blend_model_mismatch')
        j=np.asarray(model.jacobian(tuple(configuration)))
        if j.shape!=(6,6) or not np.isfinite(j).all() or np.linalg.cond(j)>1e6:raise ValueError('blend_singular_path')
        matrices.append(j)
    # Classify sub-0.1mm translation as rotation-dominated for rate scaling.
    # Otherwise micrometre feedback drift collapses the native speed argument.
    # Both measured path components remain in the twist/J/Jdot audit; this is
    # a Docker-tested parameterization, not a physical SDK unit guarantee.
    translating=distance>.0001 or radians<1e-6
    length=distance if translating else radians
    twist=np.r_[delta,rotation]/length
    limits=settings.control
    speed=.02 if translating else .12
    acceleration=.1 if translating else .2
    for gain,v,a in ((distance/length,limits.max_tcp_speed_mps,limits.max_tcp_acceleration_mps2),
                     (radians/length,limits.max_tcp_rotation_radps,limits.max_tcp_angular_acceleration_radps2)):
        if gain>1e-12:speed=min(speed,v/gain);acceleration=min(acceleration,a/gain)
    speed,acceleration=audited_rates(q,matrices,twist,model=model,limits=limits,speed=speed,acceleration=acceleration)
    return BlendPathAudit(speed,acceleration,model.model_sha256,5)
