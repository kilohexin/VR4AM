"""Opt-in synthetic moving-state checks, exclusively for loopback streams.

Conditional theoretical-region checks only: not calibrated tracking, braking,
controller acceleration constraints, or permission to move any real robot.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from app.commissioning.native_jog_mdh import _components, mdh_tcp_jacobian_rate
from app.commissioning.native_jog_model_guard import ModelJogRegionGuard, _value_matcher


@dataclass(frozen=True)
class MovingModelJogRegionGuard(ModelJogRegionGuard):
    _velocity_bound: tuple = field(init=False, repr=False)
    _acceleration_bound: tuple = field(init=False, repr=False)
    _motion_match: object = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        super().__post_init__()  # Includes exact literal loopback and fixed-box checks.
        assessment = self.assessment
        object.__setattr__(self, '_velocity_bound', tuple(assessment['joint_velocity_bound_radps']))
        object.__setattr__(self, '_acceleration_bound', tuple(assessment['joint_acceleration_bound_radps2']))
        object.__setattr__(self, '_motion_match', _value_matcher(self._model_values()))

    def _model_values(self):
        return (self.reference_q, self.tool, self.limits, self.joint_radius_rad, self.initial_offset_rad,
            self.duration_s, self._assessment_json, self._velocity_bound, self._acceleration_bound)

    def require_binding(self, settings, limits, config_sha256):
        super().require_binding(settings, limits, config_sha256)
        if not self._motion_match(self._model_values()):
            raise ValueError('native_jog_region_binding_changed')

    def _validate_start_state(self, actual, target, snapshot):
        for prefix, q in (('actual', actual), ('target', target)):
            qd = _components(getattr(snapshot, prefix+'_qd'), 6)
            qdd = _components(getattr(snapshot, prefix+'_qdd'), 6)
            # The fixed centre/radius never follow the feedback. V*t is only
            # the conditional reference excursion, never a braking bound.
            if any(abs(v-ref)+rate*self.duration_s >= radius for v, ref, rate, radius in
                    zip(q, self.reference_q, self._velocity_bound, self.joint_radius_rad, strict=True)):
                raise ValueError('native_jog_moving_reference_leaves_region')
            if any(abs(v) > cap for v, cap in zip(qd, self._velocity_bound, strict=True)):
                raise ValueError('native_jog_moving_joint_velocity_exceeded')
            if any(abs(v) > cap for v, cap in zip(qdd, self._acceleration_bound, strict=True)):
                raise ValueError('native_jog_moving_joint_acceleration_exceeded')
            with np.errstate(over='raise', invalid='raise'):
                _, j, jdot = mdh_tcp_jacobian_rate(q, qd, self.tool)
                twist = j @ np.array(qd)
                acceleration = j @ np.array(qdd)+jdot @ np.array(qd)
            if not np.isfinite(twist).all() or not np.isfinite(acceleration).all():
                raise ValueError('native_jog_moving_nonfinite_kinematics')
            # Absolute tolerances cover only synthetic floating-point residuals;
            # they are not robot calibration or field angular-motion allowances.
            if math.hypot(*twist[3:]) > 1e-10 or math.hypot(*acceleration[3:]) > 1e-9:
                raise ValueError('native_jog_moving_not_pure_translation')
            if (math.hypot(*twist[:3]) > self.limits.max_speed_mps
                    or math.hypot(*acceleration[:3]) > self.limits.max_acceleration_mps2):
                raise ValueError('native_jog_moving_cartesian_rates_exceeded')
