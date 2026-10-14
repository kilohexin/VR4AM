"""Offline delivery app: local LM3 model, explicit fake factory, no native SDK."""
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from fastapi.staticfiles import StaticFiles
from scipy.spatial.transform import Rotation

from app.digital_twin.lebai_client import DigitalTwinLebaiClient, _joint_tuple
from app.robots.lebai_codec import joint_vector, pose_from_lebai, pose_to_lebai
from app.sim.kinematics import forward_pose, geometric_jacobian
from app.sim.self_collision import is_self_colliding


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
    control=replace(settings.lebai.control,pvat_horizon_s=.08,pvat_send_hz=25,
        max_tcp_speed_mps=.005,max_tcp_acceleration_mps2=.02,
        max_tcp_step_m=.0002,translation_scale=.2)
    return replace(settings,lebai=replace(settings.lebai,control=control),
        segmented_target=SegmentedSettings(PursuitSettings(500,60,60,.001,.002,3)))


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

    async def movej(self,*args,**kwargs):
        raise ValueError('demo_home_not_supported')

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
    app=create_app(settings=settings,client_factory=factory,backend_label='LEBAI_FAKE')
    @app.get('/demo/identity')
    async def identity():
        return dict(runtime='LEBAI_FAKE',target_strategy='segmented_experimental',
            hardware_verified=False,native_sdk_loaded='lebai_sdk' in sys.modules,
            physics_engine=False,headset_verified=False,
            log_session_dir=str(app.state.log_session_dir) if app.state.log_session_dir else None)
    app.mount('/',StaticFiles(directory=web_directory,html=True),name='offline_demo')
    return app
