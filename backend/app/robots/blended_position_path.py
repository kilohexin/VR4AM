"""Sampled Cartesian continuation audit; nominal-model evidence, not certification."""
from dataclasses import dataclass
import math
import numpy as np

from app.robots.finite_position_plan import angle


@dataclass(frozen=True)
class BlendPathAudit:
    speed: float
    acceleration: float
    model_sha256: str
    samples: int


def audited_rates(q, matrices, direction, *, model, limits, speed, acceleration):
    """Reduce tangential acceleration independently of feasible speed."""
    for _ in range(10):
        candidate=acceleration;valid=True
        for configuration,j in zip(q,matrices):
            gain=np.linalg.solve(j,direction)
            qd=gain*speed
            jd=np.asarray(model.jdot(tuple(configuration),tuple(qd)))
            if jd.shape!=(6,6) or not np.isfinite(jd).all():raise ValueError('blend_invalid_model')
            curvature=np.abs(np.linalg.solve(j,jd@qd))
            room=limits.max_joint_acceleration_radps2-curvature
            if np.max(np.abs(qd))>limits.max_joint_speed_radps or np.any(room<=0):
                valid=False;break
            nonzero=np.abs(gain)>1e-12
            if np.any(nonzero):candidate=min(candidate,float(np.min(room[nonzero]/np.abs(gain[nonzero])))*.999999)
        if valid and candidate>1e-6:
            return speed,candidate
        speed/=2
    raise ValueError('blend_model_rates')


def audit_blended_path(positions, joints, orientation, *, model, settings):
    if len(positions) != 5 or len(joints) != 5:
        raise ValueError('blend_path_samples')
    p = np.asarray(positions, dtype=float)
    q = np.asarray(joints, dtype=float)
    if p.shape != (5, 3) or q.shape != (5, 6) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('blend_invalid_path')
    delta = p[-1]-p[0]
    distance = float(np.linalg.norm(delta))
    if not 1e-6 < distance <= .008000001:
        raise ValueError('blend_path_length')
    if not np.allclose(p, p[0]+np.linspace(0, 1, 5)[:, None]*delta, atol=1e-9, rtol=0):
        raise ValueError('blend_path_not_linear')
    lo = np.asarray(settings.soft_joint_min_rad)+settings.joint_limit_margin_rad
    hi = np.asarray(settings.soft_joint_max_rad)-settings.joint_limit_margin_rad
    if np.any(q < lo) or np.any(q > hi):
        raise ValueError('blend_joint_limit')
    if np.max(np.abs(q-q[0])) > settings.control.max_joint_step_rad:
        raise ValueError('blend_joint_jump')
    if np.any(p < settings.startup_tcp_min_m) or np.any(p > settings.startup_tcp_max_m):
        raise ValueError('blend_workspace')
    direction = np.r_[delta/distance, np.zeros(3)]
    matrices = []
    for point, configuration in zip(p, q):
        actual, rotation = model.forward(tuple(configuration))
        if math.dist(actual, point) > 1e-5 or angle(rotation, orientation) > 1e-4:
            raise ValueError('blend_model_mismatch')
        j = np.asarray(model.jacobian(tuple(configuration)))
        if j.shape != (6, 6) or not np.isfinite(j).all() or np.linalg.cond(j) > 1e6:
            raise ValueError('blend_singular_path')
        matrices.append(j)
    limits = settings.control
    speed = min(.02, limits.max_tcp_speed_mps)
    acceleration = min(.1, limits.max_tcp_acceleration_mps2)
    speed,acceleration=audited_rates(q,matrices,direction,model=model,limits=limits,speed=speed,acceleration=acceleration)
    return BlendPathAudit(speed,acceleration,model.model_sha256,len(q))
