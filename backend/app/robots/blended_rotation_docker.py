"""Pure tool-X/tool-Z rotation, isolated from physical robot selectors."""
import asyncio
import math
from scipy.spatial.transform import Rotation
from app.robots.blended_position_docker import DockerBlendPort,DockerBlendPlan
from app.robots.blended_rotation_path import audit_rotation_path
from app.robots.finite_position_plan import angle
from app.robots.lebai_codec import pose_to_lebai,joint_vector
from app.schemas.messages import Pose


class DockerRotationPort(DockerBlendPort):
    radians_per_unit=math.radians(.5)/.004
    coordinate_limit=math.radians(90)/radians_per_unit

    def __init__(self,*args,axis,**kwargs):
        if axis not in ('pitch','spin'):raise ValueError('blend_rotation_axis')
        super().__init__(*args,**kwargs)
        self.axis=0 if axis=='pitch' else 2

    def _initialize_reference(self,s):
        self.fixed_position=tuple(s.actual_tcp.p)
        self.origin_rotation=Rotation.from_quat(self.orientation)

    def _coordinates(self,s):
        relative=(self.origin_rotation.inv()*Rotation.from_quat(s.actual_tcp.q)).as_rotvec()
        out=[0.,0.,0.];out[self.axis]=float(relative[self.axis])/self.radians_per_unit
        return tuple(out)

    def _validate_pose(self,s):
        if math.dist(self.fixed_position,s.actual_tcp.p)>.0001:raise ValueError('blend_rotation_tcp_drift')
        relative=(self.origin_rotation.inv()*Rotation.from_quat(s.actual_tcp.q)).as_rotvec()
        if math.hypot(*(x for i,x in enumerate(relative) if i!=self.axis))>math.radians(.05):
            raise ValueError('blend_rotation_axis_drift')
        reference=self.adapter._position_pitch_reference
        if reference is None or angle(reference,s.actual_tcp.q)>math.pi/2:raise ValueError('blend_rotation_range')

    async def audit(self,target):
        self.check();a=self.adapter
        if self.sample is None:raise ValueError('blend_feedback_missing')
        start=self._coordinates(self.sample)
        delta=[0.,0.,0.];delta[self.axis]=target.position[self.axis]*self.radians_per_unit
        goal=self.origin_rotation*Rotation.from_rotvec(delta)
        first=Rotation.from_quat(self.sample.actual_tcp.q)
        relative=(goal*first.inv()).as_rotvec()
        orientations=[tuple((Rotation.from_rotvec(relative*f)*first).as_quat()) for f in (0.,.25,.5,.75,1.)]
        joints=[tuple(self.sample.actual_q)]
        async with asyncio.timeout(1.):
            for orientation in orientations[1:]:
                pose=pose_to_lebai(Pose(p=self.fixed_position,q=orientation))
                raw=await a._position_math('kinematics_inverse',lambda:self.client.kinematics_inverse(pose,list(joints[-1])),self.check)
                if raw is None:raise ValueError('blend_ik_unreachable')
                joints.append(joint_vector(raw,'blend_ik_invalid'))
            audit=await asyncio.to_thread(audit_rotation_path,self.fixed_position,orientations,joints,
                model=a.position_model,settings=a.settings,reference=a._position_pitch_reference)
        self.check()
        self.plan=DockerBlendPlan(target,pose_to_lebai(Pose(p=self.fixed_position,q=orientations[-1])),audit,self.binding,start)
        return self.plan
