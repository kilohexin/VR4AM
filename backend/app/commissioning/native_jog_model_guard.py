"""Immutable model binding for injected loopback diagnostics only.

This is not evidence of factory calibration, physical braking or real-robot
joint limits. The public constructor rejects every non-loopback address.
"""
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math

from scipy.spatial.transform import Rotation

from app.commissioning.native_jog_mdh import MdhTool, _components, _finite, mdh_tcp_jacobian
from app.commissioning.native_jog_packet import NativeJogLimits, NativeJogPacket, _sha
from app.commissioning.native_jog_region import assess_translation_region
from app.config import LebaiSettings


def _identity(settings):
    return hashlib.sha256(json.dumps(asdict(settings), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def _pose_matches(p, q, expected_p, expected_q, position_m, rotation_deg):
    p, q = _components(p, 3), _components(q, 4)
    if abs(math.hypot(*q)-1.) > 1e-9:
        raise ValueError('native_jog_region_invalid_pose')
    angle = (Rotation.from_quat(q)*Rotation.from_quat(expected_q).inv()).magnitude()
    return math.dist(p, expected_p) <= position_m and math.degrees(angle) <= rotation_deg


@dataclass(frozen=True)
class ModelJogRegionGuard:
    settings: LebaiSettings
    limits: NativeJogLimits
    config_sha256: str
    reference_q: tuple[float, ...]
    tool: MdhTool
    joint_radius_rad: tuple[float, ...]
    initial_offset_rad: tuple[float, ...]
    duration_s: float
    _settings_sha256: str = field(init=False, repr=False)
    _assessment_json: str = field(init=False, repr=False)

    def __post_init__(self):
        if (type(self.settings) is not LebaiSettings or self.settings.ip != '127.0.0.1'
                or self.settings.mode != 'control' or type(self.limits) is not NativeJogLimits
                or type(self.tool) is not MdhTool):
            raise ValueError('native_jog_region_injected_loopback_required')
        _sha(self.config_sha256)
        for name in ('reference_q', 'joint_radius_rad', 'initial_offset_rad'):
            object.__setattr__(self, name, _components(getattr(self, name), 6))
        object.__setattr__(self, 'duration_s', _finite(self.duration_s))
        s, control = self.settings, self.settings.control
        if (self.reference_q != _components(s.teleop_ready_q, 6)
                or self.limits.max_speed_mps > _finite(control.max_tcp_speed_mps)
                or self.limits.max_acceleration_mps2 > _finite(control.max_tcp_acceleration_mps2)):
            raise ValueError('native_jog_region_configuration_mismatch')
        margin = _finite(s.joint_limit_margin_rad)
        lo, hi = _components(s.soft_joint_min_rad, 6), _components(s.soft_joint_max_rad, 6)
        if margin < 0 or any(q-r < lower+margin or q+r > upper-margin
                for q, r, lower, upper in zip(self.reference_q, self.joint_radius_rad, lo, hi, strict=True)):
            raise ValueError('native_jog_region_outside_soft_limits')
        expected = s.expected_tcp
        expected_q = tuple(Rotation.from_euler('ZYX', [expected.rz, expected.ry, expected.rx]).as_quat())
        if not _pose_matches(self.tool.translation_m, self.tool.rotation_xyzw,
                (expected.x, expected.y, expected.z), expected_q, 1e-9, 1e-6):
            raise ValueError('native_jog_region_tool_mismatch')
        result = assess_translation_region(self.reference_q, self.tool,
            joint_radius_rad=self.joint_radius_rad, initial_offset_rad=self.initial_offset_rad,
            duration_s=self.duration_s, cartesian_speed_mps=self.limits.max_speed_mps,
            cartesian_acceleration_mps2=self.limits.max_acceleration_mps2,
            joint_speed_limit_radps=control.max_joint_speed_radps,
            joint_acceleration_limit_radps2=control.max_joint_acceleration_radps2)
        if not result['model_reference_eligible']:
            raise ValueError('native_jog_region_model_reference_rejected')
        object.__setattr__(self, '_settings_sha256', _identity(s))
        object.__setattr__(self, '_assessment_json', json.dumps(result, allow_nan=False))

    @property
    def assessment(self):
        return json.loads(self._assessment_json)  # Never expose mutable cached evidence.

    def require_binding(self, settings, limits, config_sha256):
        if (settings is not self.settings or limits is not self.limits
                or config_sha256 != self.config_sha256 or _identity(settings) != self._settings_sha256):
            raise ValueError('native_jog_region_binding_changed')

    def validate(self, packet, snapshot, *, settings, limits, config_sha256):
        self.require_binding(settings, limits, config_sha256)
        if (type(packet) is not NativeJogPacket or packet.duration_s != self.duration_s
                or packet.config_sha256 != self.config_sha256
                or packet.acceleration_mps2 > self.limits.max_acceleration_mps2
                or math.hypot(*packet.velocity_mps) > self.limits.max_speed_mps):
            raise ValueError('native_jog_region_packet_mismatch')
        actual, target = _components(snapshot.actual_q, 6), _components(snapshot.target_q, 6)
        for q in (actual, target):
            if any(abs(v-ref) > cap for v, ref, cap in zip(q, self.reference_q, self.initial_offset_rad, strict=True)):
                raise ValueError('native_jog_region_reference_changed')
        for field_name in ('actual_qd', 'actual_qdd', 'target_qd', 'target_qdd'):
            if any(v != 0 for v in _components(getattr(snapshot, field_name), 6)):
                raise ValueError('native_jog_region_initial_motion_reported')
        setting = snapshot.tcp_setting
        if not _pose_matches(setting.p, setting.q, self.tool.translation_m,
                self.tool.rotation_xyzw, 1e-9, 1e-6):
            raise ValueError('native_jog_region_tool_changed')
        for q, pose in ((actual, snapshot.actual_tcp), (target, snapshot.target_tcp)):
            model, _ = mdh_tcp_jacobian(q, self.tool)
            if not _pose_matches(pose.p, pose.q, model[:3, 3],
                    Rotation.from_matrix(model[:3, :3]).as_quat(), .0001, .1):
                raise ValueError('native_jog_region_feedback_mismatch')
