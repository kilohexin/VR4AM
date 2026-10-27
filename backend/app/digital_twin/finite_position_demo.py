"""Local finite-position webpage. Visual kinematics, no real target or physics."""
import math
import asyncio
import os
import time
from dataclasses import replace
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from fastapi.staticfiles import StaticFiles
from app.digital_twin.segmented_demo import SegmentedDemoClient,demo_settings as segmented_settings,validate_demo_environment,_validate_ready_path
from app.digital_twin.lebai_client import DigitalTwinLebaiClient
from app.sim.lm3_model import LM3Model,model_config_sha256
from app.sim.kinematics import forward_pose,geometric_jacobian
from app.sim.self_collision import is_self_colliding
from app.robots.lebai_codec import joint_vector,pose_from_lebai


def demo_settings():
    s=segmented_settings()
    return replace(s,segmented_target=None,transactional_target=None)


class PositionDemoModel:
    model_sha256=model_config_sha256()
    factory_calibrated=False
    def __init__(self):self.model=LM3Model()
    def forward(self,q):
        p=forward_pose(q,self.model);return tuple(p.p),tuple(p.q)
    def jacobian(self,q):return geometric_jacobian(q,self.model)
    def jdot(self,q,qd):
        q=np.asarray(q);v=np.asarray(qd);h=1e-4
        return (self.jacobian(q+h*v)-self.jacobian(q-h*v))/(2*h)
    def validate_path(self,start,goal,settings):
        _validate_ready_path(tuple(start),tuple(goal),settings,self.model)


class FinitePositionDemoClient(SegmentedDemoClient):
    # These methods only calculate from explicit joint/pose inputs and the
    # fixed local model. They never call a native SDK or mutate arm state.
    _position_pure_local_kinematics=True

    async def kinematics_inverse(self,pose,joints):
        await self._delay();self._require_connected()
        if self._faults.ik_failure:return None
        target=pose_from_lebai(pose);start=joint_vector(joints,'position_demo_ik')
        model=self.model
        def solve():
            q=np.array(start,dtype=float)
            for _ in range(8):
                actual=forward_pose(q,model)
                error=np.concatenate([np.array(target.p)-actual.p,
                    (Rotation.from_quat(target.q)*Rotation.from_quat(actual.q).inv()).as_rotvec()])
                if np.linalg.norm(error[:3])<=1e-8 and np.linalg.norm(error[3:])<=1e-7:break
                j=geometric_jacobian(q,model)
                try:delta=j.T@np.linalg.solve(j@j.T+1e-6*np.eye(6),error)
                except np.linalg.LinAlgError:return None
                candidate=q+np.clip(delta,-.01,.01)
                if (not np.isfinite(candidate).all() or np.any(abs(candidate-np.array(model.home_q))>model.joint_window_rad)
                        or is_self_colliding(candidate,model)):return None
                q=candidate
            return [float(x) for x in q]
        result=await asyncio.to_thread(solve)
        self._require_connected();return result

    async def movej(self,p,a,v,t,r):
        goal=joint_vector(p,'position_demo_joints')
        if goal==self.settings.teleop_ready_q:return await super().movej(p,a,v,t,r)
        if (t!=0. or r!=0. or any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in (a,v))):
            raise ValueError('position_demo_move_options')
        self._advance()
        if self._motion is not None:raise ValueError('position_demo_busy')
        distance=max(abs(x-y) for x,y in zip(self._q,goal))
        if distance>self.settings.control.max_joint_step_rad:raise ValueError('position_demo_joint_step')
        start=self._q;epoch=getattr(self,'_ready_epoch',0)
        await asyncio.to_thread(_validate_ready_path,start,goal,self.settings,self.model)
        if epoch!=getattr(self,'_ready_epoch',0) or self._q!=start or self._motion is not None:
            raise ValueError('position_demo_revoked')
        duration=max(.25,1.875*distance/min(v,self.settings.control.max_joint_speed_radps),
            math.sqrt(10/math.sqrt(3)*distance/min(a,self.settings.control.max_joint_acceleration_radps2)))
        motion_id=await DigitalTwinLebaiClient.movej(self,p,a,v,duration,0.)
        self._write_calls[-1]=('movej',list(p),a,v,t,r)
        return motion_id


def create_finite_position_demo_app(web_directory:Path):
    validate_demo_environment(os.environ)
    directory=Path(web_directory).resolve()
    if not (directory/'index.html').is_file():raise ValueError('position_demo_web_missing')
    from app.main import create_app
    settings=demo_settings();client=FinitePositionDemoClient(settings.lebai,clock=time.perf_counter_ns,
        initial_q=settings.lebai.home_q)
    class PositionClock:
        def now_ns(self):return time.perf_counter_ns()
    async def factory(ip):
        if ip!='digital-twin':raise ValueError('position_demo_target')
        return client
    app=create_app(settings=settings,client_factory=factory,backend_label='LEBAI_FAKE',
        offline_finite_position=True,offline_position_model=PositionDemoModel(),offline_position_clock=PositionClock())
    @app.get('/demo/identity')
    async def identity():
        return dict(runtime='LEBAI_FAKE',target_strategy='finite_position_offline',hardware_verified=False,
            physics_engine=False,headset_verified=False,log_session_dir=str(app.state.log_session_dir))
    app.mount('/',StaticFiles(directory=directory,html=True),name='finite-position-offline')
    return app
