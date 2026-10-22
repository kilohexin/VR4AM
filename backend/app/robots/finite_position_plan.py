"""Sampled model estimates for offline finite goals; no hardware certificate."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from app.control.finite_position_types import (MAX_STEP_M,MIN_PROGRESS_M,MAX_ENDPOINT_ERROR_M,
    JOINT_ENDPOINT_TOLERANCE_RAD,PathAudit)

FRACTIONS=(0.,.25,.5,.75,1.)


def build_position_target(sample,intent):
    d=np.asarray(intent.vector)*MAX_STEP_M
    if np.linalg.norm(d)<1e-7:raise ValueError('position_unresolvable_step')
    return tuple(float(x) for x in np.asarray(sample.tcp_p)+d),sample.tcp_q


def angle(a,b):
    return (Rotation.from_quat(a)*Rotation.from_quat(b).inv()).magnitude()


def audit_position_path(start,goal_joints,*,fk_samples,settings,model):
    if len(fk_samples)!=5:raise ValueError('position_fk_samples_required')
    q0=np.asarray(start.actual_q);dq=np.asarray(goal_joints)-q0
    distance=float(np.max(np.abs(dq)))
    if not np.isfinite(dq).all() or distance>settings.control.max_joint_step_rad:
        raise ValueError('position_joint_jump')
    if distance<1e-12:raise ValueError('position_no_joint_progress')
    validate_path=getattr(model,'validate_path',None)
    if callable(validate_path):validate_path(start.actual_q,goal_joints,settings)
    direction=dq/distance
    lo=np.array(settings.soft_joint_min_rad)+settings.joint_limit_margin_rad
    hi=np.array(settings.soft_joint_max_rad)-settings.joint_limit_margin_rad
    for f,(p,r) in zip(FRACTIONS,fk_samples,strict=True):
        q=q0+f*dq
        if np.any(q<lo) or np.any(q>hi):raise ValueError('position_joint_limit')
        if any(not a<=x<=b for a,x,b in zip(settings.startup_tcp_min_m,p,settings.startup_tcp_max_m)):
            raise ValueError('position_workspace')
        mp,mr=model.forward(tuple(q))
        if math.dist(mp,p)>1e-5 or angle(mr,r)>1e-4:raise ValueError('position_model_mismatch')
        if angle(start.tcp_q,r)>math.radians(settings.tcp_rotation_tolerance_deg):
            raise ValueError('position_path_rotation')
    v=min(.05,settings.control.max_joint_speed_radps)
    a=min(.2,settings.control.max_joint_acceleration_radps2)
    for _ in range(11):
        speeds=[];accels=[];angular=[];angular_acc=[]
        for f in FRACTIONS:
            q=tuple(q0+f*dq);qd=direction*v
            j=np.asarray(model.jacobian(q));jd=np.asarray(model.jdot(q,tuple(qd)))
            if j.shape!=(6,6) or jd.shape!=(6,6) or not np.isfinite(j).all() or not np.isfinite(jd).all():
                raise ValueError('position_invalid_model')
            velocity=j@qd
            acceleration=np.abs(j@(direction*a))+np.abs(jd@qd)
            speeds.append(float(np.linalg.norm(velocity[:3])))
            accels.append(float(np.linalg.norm(acceleration[:3])))
            angular.append(float(np.linalg.norm(velocity[3:])))
            angular_acc.append(float(np.linalg.norm(acceleration[3:])))
        duration=max(1.875*distance/v,math.sqrt(10/math.sqrt(3)*distance/a))
        if (max(speeds)<=settings.control.max_tcp_speed_mps
                and max(accels)<=settings.control.max_tcp_acceleration_mps2
                and max(angular)<=settings.control.max_tcp_rotation_radps
                and max(angular_acc)<=settings.control.max_tcp_angular_acceleration_radps2):
            if duration>1.8:raise ValueError('position_estimated_duration')
            return PathAudit(model.model_sha256,FRACTIONS,True,True,True,
                model.factory_calibrated,v,a,max(speeds),max(accels))
        v/=2;a/=4
    raise ValueError('position_model_rates')


def evaluate_position_arrival(plan,sample,*,motion_id,tail=False):
    if sample.binding is not plan.binding: return 'position_binding_changed'
    if sample.fault:return 'position_feedback_fault'
    if sample.running_id not in (None,motion_id):return 'position_foreign_task'
    if str(sample.robot_state)!='IDLE' or sample.task_state!='FINISHED':return 'position_not_finished'
    delta=np.array(plan.target_p)-plan.start.tcp_p;length=float(np.linalg.norm(delta))
    actual=np.array(sample.tcp_p)-plan.start.tcp_p
    progress=float(actual@delta/length)
    cross=float(np.linalg.norm(actual-progress*delta/length))
    if progress<MIN_PROGRESS_M-1e-12:return 'position_no_actual_progress'
    if cross>length*.2:return 'position_lateral_deviation'
    if angle(sample.tcp_q,plan.target_q)>plan.arrival_policy.orientation_error_rad:return 'position_orientation_error'
    if math.dist(sample.tcp_p,plan.target_p)>plan.arrival_policy.endpoint_error_m+1e-12:return 'position_endpoint_error'
    if max(abs(x-y) for x,y in zip(sample.actual_q,plan.goal_joints))>JOINT_ENDPOINT_TOLERANCE_RAD:return 'position_joint_endpoint'
    velocity_limit=plan.arrival_policy.tail_velocity_radps if tail else .002
    if any(abs(x)>velocity_limit for x in sample.actual_qd):return 'position_not_stable'
    return None


async def plan_position_step(adapter,intent,*,authorize):
    return await adapter.plan_step(intent,authorize=authorize)
