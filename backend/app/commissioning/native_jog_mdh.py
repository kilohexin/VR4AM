"""Saved-state mathematical diagnostics using the published theoretical LM3 MDH.

Source: https://help.lebai.ltd/guide/pose.html (literal alpha=1.5708).
This is not the visual model, individual factory calibration, a workspace bound,
a braking model or a motion admission path. No SDK/network/control imports.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

_D = (.21583, 0., 0., .12063, .09833, .08343)
_A = (0., 0., -.28, -.26, 0., 0.)
_ALPHA = (0., 1.5708, 0., 0., 1.5708, -1.5708)


def _finite(value):
    if type(value) not in (int, float):
        raise ValueError('finite numeric value required')
    try:
        number = float(value)
    except OverflowError:
        raise ValueError('finite numeric value required') from None
    if not math.isfinite(number):
        raise ValueError('finite numeric value required')
    return number


def _components(value, count):
    if type(value) not in (tuple, list) or len(value) != count:
        raise ValueError(f'{count} finite components required')
    return tuple(_finite(x) for x in value)


@dataclass(frozen=True)
class MdhTool:
    translation_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        p = _components(self.translation_m, 3)
        q = _components(self.rotation_xyzw, 4)
        if abs(math.hypot(*q) - 1.) > 1e-9:
            raise ValueError('unit tool quaternion required')
        object.__setattr__(self, 'translation_m', p)
        object.__setattr__(self, 'rotation_xyzw', q)


def mdh_tcp_jacobian(q, tool: MdhTool):
    """Return fresh base-frame TCP transform and geometric J (linear, angular).

    Encoder angles are used directly, without the GLB shoulder offset. Linear
    rows use m/rad; angular rows map joint rad/s to angular velocity rad/s,
    NOT to ZYX Euler angle derivatives. The tool is explicitly supplied.
    """
    q = _components(q, 6)
    if type(tool) is not MdhTool:
        raise ValueError('explicit immutable MDH tool required')
    transform = np.eye(4)
    origins, axes = [], []
    for theta, d, a, alpha in zip(q, _D, _A, _ALPHA, strict=True):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        transform = transform @ np.array([
            [ct, -st, 0., a], [st*ca, ct*ca, -sa, -d*sa],
            [st*sa, ct*sa, ca, d*ca], [0., 0., 0., 1.],
        ])
        # This point is displaced by d along the joint's own axis; it lies
        # on the same rotation axis as the origin before Rz(theta) Tz(d).
        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, 2].copy())
    tcp = np.eye(4)
    tcp[:3, 3] = tool.translation_m
    tcp[:3, :3] = Rotation.from_quat(tool.rotation_xyzw).as_matrix()
    transform = transform @ tcp
    jacobian = np.array([
        np.r_[np.cross(axis, transform[:3, 3]-origin), axis]
        for origin, axis in zip(origins, axes, strict=True)
    ]).T
    if not np.isfinite(transform).all() or not np.isfinite(jacobian).all():
        raise ValueError('nonfinite theoretical MDH result')
    return transform, jacobian


def assess_translation_reference(q, tool: MdhTool, *, cartesian_speed_mps,
        cartesian_acceleration_mps2, joint_speed_limit_radps,
        joint_acceleration_limit_radps2):
    """Pointwise worst direction, zero angular velocity, starting from rest.

    ||row(J^-1[:, :3])|| bounds each joint's linear response at THIS q for
    every unit translation direction. Acceleration omits Jdot*qdot because
    its sole scope is the initial instant at rest. It proves neither a
    neighborhood bound nor acceleration during motion or braking.
    """
    values = tuple(_finite(x) for x in (cartesian_speed_mps,
        cartesian_acceleration_mps2, joint_speed_limit_radps,
        joint_acceleration_limit_radps2))
    if any(x <= 0 for x in values):
        raise ValueError('positive diagnostic parameters required')
    speed, acceleration, joint_speed, joint_acceleration = values
    _, jacobian = mdh_tcp_jacobian(q, tool)
    if np.linalg.matrix_rank(jacobian) < 6:
        raise ValueError('mdh_singular_reference')
    gain = np.linalg.solve(jacobian, np.vstack([np.eye(3), np.zeros((3, 3))]))
    max_gain = max(math.hypot(*row) for row in gain)
    numbers = (max_gain*speed, max_gain*acceleration,
        joint_speed/max_gain, joint_acceleration/max_gain)
    if not all(math.isfinite(x) for x in numbers):
        raise ValueError('nonfinite theoretical joint response')
    return dict(
        model='manufacturer_theoretical_mdh_not_factory_calibration',
        offline_only=True, hardware_verified=False, joint_bounds_verified=False,
        braking_verified=False, continuous_control_admitted=False,
        admission_error='native_jog_joint_bounds_unverified',
        scope='single_reference_not_workspace_or_trajectory',
        acceleration_scope='instant_from_rest_only_no_jdot_bounds',
        peak_joint_speed_radps=numbers[0], peak_start_acceleration_radps2=numbers[1],
        cartesian_speed_at_joint_limit_mps=numbers[2],
        cartesian_start_acceleration_at_joint_limit_mps2=numbers[3],
        limits_satisfied_at_reference=(numbers[0] <= joint_speed
            and numbers[1] <= joint_acceleration),
    )
