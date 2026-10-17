"""Offline delivery app: local LM3 model, explicit fake factory, no native SDK."""
import asyncio
import os
import sys
import math
from functools import lru_cache
from dataclasses import replace
from pathlib import Path

import numpy as np
from fastapi.staticfiles import StaticFiles
from scipy.spatial.transform import Rotation

from app.digital_twin.lebai_client import DigitalTwinLebaiClient, _joint_tuple
from app.digital_twin.path_geometry import validate_ready_samples
from app.robots.lebai_codec import joint_vector, pose_from_lebai, pose_to_lebai
from app.sim.kinematics import forward_pose, geometric_jacobian
from app.sim.self_collision import is_self_colliding
from app.sim.lm3_model import LM3Model


def validate_demo_environment(environment):
    if ('VR4ARM_REAL_ROBOT_CONFIRM' in environment
            or environment.get('VR4ARM_CONFIG') not in (None,'config/fake-lebai.yaml')
            or 'VITE_TELEOP_WS_URL' in environment
            or 'lebai_sdk' in sys.modules or 'lebai_sdk_asyncio' in sys.modules):
        raise ValueError('unsafe_demo_environment')


def demo_settings():
    validate_demo_environment(os.environ)
    from app.digital_twin.runtime import load_digital_twin_settings
    from app.control.transactional_settings import PursuitSettings, SegmentedSettings
    settings=load_digital_twin_settings()
    # A visible local-model work-ready posture. It is synthetic and provides
    # no real-robot calibration or authorization.
    ready_q=tuple(a+b for a,b in zip(settings.lebai.home_q,(-.30,-.15,.10,0.,0.,0.)))
    control=replace(settings.lebai.control,pvat_horizon_s=.08,pvat_send_hz=25,
        max_tcp_speed_mps=.005,max_tcp_acceleration_mps2=.02,
        max_tcp_step_m=.0002,translation_scale=.2)
    result=replace(settings,lebai=replace(settings.lebai,control=control,teleop_ready_q=ready_q),rotation_dead_zone_deg=.001,
        segmented_target=SegmentedSettings(PursuitSettings(500,60,60,.001,.002,3),tool_spin=True,prepare_on_arm=True))
    _validate_ready_path(result.lebai.home_q,ready_q,result.lebai,LM3Model())
    return result


@lru_cache(maxsize=32)
def _validate_ready_path(start,goal,settings,model):
    # Local sampled geometric checks are not a hardware path certificate.
    validate_ready_samples(start,goal,settings,model)


class SegmentedDemoClient(DigitalTwinLebaiClient):
    """Quintic joint animation, no physics or calibrated real-robot model."""
    def _native_snapshot_entry_count(self,method):
        # Reads run in this process against local state. There is no native
        # transport to abandon, including when a browser cancels a read.
        return 0

    def _native_snapshot_result_unknown(self,method):
        return False

    async def kinematics_forward(self,joints):
        await self._delay()
        self._require_connected()
        return pose_to_lebai(forward_pose(joint_vector(joints,'demo_fk'),self.model))

    async def kinematics_inverse(self,pose,joints):
        await self._delay()
        self._require_connected()
        if self._faults.ik_failure:return None
        target=pose_from_lebai(pose)
        q=np.array(joint_vector(joints,'demo_ik'),dtype=float)
        # Bounded local numerical iterations inside one simulated SDK call.
        # The actual adapter still performs its original FK acceptance audit.
        for _ in range(8):
            actual=forward_pose(q,self.model)
            if (np.linalg.norm(np.array(actual.p)-target.p)<=1e-8
                    and (Rotation.from_quat(actual.q)*Rotation.from_quat(target.q).inv()).magnitude()<=1e-7):
                break
            error=np.concatenate([np.array(target.p)-actual.p,
                (Rotation.from_quat(target.q)*Rotation.from_quat(actual.q).inv()).as_rotvec()])
            jacobian=geometric_jacobian(q,self.model)
            try:
                delta=jacobian.T@np.linalg.solve(jacobian@jacobian.T+1e-6*np.eye(6),error)
            except np.linalg.LinAlgError:return None
            candidate=q+np.clip(delta,-.01,.01)
            if (not np.all(np.isfinite(candidate))
                    or np.any(candidate<np.array(self.model.home_q)-self.model.joint_window_rad)
                    or np.any(candidate>np.array(self.model.home_q)+self.model.joint_window_rad)
                    or is_self_colliding(candidate,self.model)):return None
            q=candidate
        return list(q)

    async def move_pvat(self,p,v,a,t):
        if joint_vector(v,'demo_velocity')!=(0.,)*6 or joint_vector(a,'demo_acceleration')!=(0.,)*6:
            raise ValueError('demo_zero_terminal_required')
        self._advance()
        if self._qd!=(0.,)*6 or self._qdd!=(0.,)*6:
            raise ValueError('demo_zero_start_required')
        return await super().move_pvat(p,v,a,t)

    async def movej(self,p,a,v,t,r):
        goal=joint_vector(p,'demo_ready')
        if goal!=self.settings.teleop_ready_q:
            raise ValueError('demo_ready_pose_required')
        if (not all(type(x) in (float,int) and math.isfinite(x) and x>0 for x in (a,v))
                or t!=0. or r!=0.):
            raise ValueError('demo_ready_options_invalid')
        self._advance()
        if self._motion is not None or self._qd!=(0.,)*6 or self._qdd!=(0.,)*6:
            raise ValueError('demo_zero_start_required')
        start_q=self._q
        epoch=getattr(self,'_ready_epoch',0)
        # Local sampled geometry is CPU work, not an SDK RPC. Keep it off
        # the event loop so frame/stop/watchdog tasks remain receivable.
        await asyncio.to_thread(_validate_ready_path,start_q,goal,self.settings,self.model)
        self._require_connected()
        if epoch!=getattr(self,'_ready_epoch',0) or self._q!=start_q or self._motion is not None:
            raise ValueError('demo_ready_revoked')
        distance=max(abs(x-y) for x,y in zip(self._q,goal))
        # Quintic peak speed=1.875*d/T; peak acceleration=10/sqrt(3)*d/T².
        # Preserve requested/configured limits while making the animation clear.
        duration=max(2.,1.875*distance/min(v,self.settings.control.max_joint_speed_radps,self.model.max_joint_speed_radps),
            math.sqrt(10/math.sqrt(3)*distance/min(a,self.settings.control.max_joint_acceleration_radps2,self.model.max_joint_accel_radps2)))
        return await super().movej(p,a,v,duration,r)

    async def stop_move(self):
        self._ready_epoch=getattr(self,'_ready_epoch',0)+1
        await super().stop_move()

    async def stop_sys(self):
        raise ValueError('demo_system_stop_not_supported')

    def _advance(self):
        motion=self._motion
        if motion is None:return
        u=min(1.,max(0.,(self._clock()-motion.started_ns)/motion.duration_ns))
        if u>=1.:
            self._q=motion.target_q
            self._qd=self._qdd=(0.,)*6
            self._motion=None
            self._running_motion_id=None
            return
        duration=motion.duration_ns/1e9
        start=np.array(motion.start_q)
        delta=np.array(motion.target_q)-start
        self._q=_joint_tuple(start+(10*u**3-15*u**4+6*u**5)*delta)
        self._qd=_joint_tuple((30*u**2-60*u**3+30*u**4)*delta/duration)
        self._qdd=_joint_tuple((60*u-180*u**2+120*u**3)*delta/duration**2)


def create_segmented_demo_app(web_directory:Path):
    validate_demo_environment(os.environ)
    web_directory=Path(web_directory).resolve()
    if not (web_directory/'index.html').is_file():
        raise ValueError('delivery_web_missing')
    from app.main import create_app
    settings=demo_settings()
    client=SegmentedDemoClient.idle(settings.lebai)
    async def factory(ip):
        if ip!='digital-twin':raise ValueError('demo_factory_target_invalid')
        return client
    app=create_app(settings=settings,client_factory=factory,backend_label='LEBAI_FAKE',
        offline_unsent_expiry_hold=True,offline_parallel_segment_fk=True)
    @app.get('/demo/identity')
    async def identity():
        return dict(runtime='LEBAI_FAKE',target_strategy='segmented_experimental',
            hardware_verified=False,native_sdk_loaded='lebai_sdk' in sys.modules,
            physics_engine=False,headset_verified=False,
            log_session_dir=str(app.state.log_session_dir) if app.state.log_session_dir else None)
    app.mount('/',StaticFiles(directory=web_directory,html=True),name='offline_demo')
    return app
