"""Immutable model binding for injected loopback diagnostics only.

This is not evidence of factory calibration, physical braking or real-robot
joint limits. The public constructor rejects every non-loopback address.
"""
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from functools import lru_cache
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


def _value_matcher(value):
    """Capture independent immutable values once; compare all fields at gates."""
    cls=type(value)
    if is_dataclass(value):
        checks=tuple((f.name,_value_matcher(getattr(value,f.name))) for f in fields(value))
        return lambda current:type(current) is cls and all(check(getattr(current,name)) for name,check in checks)
    if cls in (tuple,list):
        checks=tuple(_value_matcher(v) for v in value)
        return lambda current:type(current) is cls and len(current)==len(checks) and all(
            check(v) for check,v in zip(checks,current,strict=True))
    if cls is float:
        expected=value.hex()  # Preserve type and signed-zero differences too.
        return lambda current:type(current) is float and current.hex()==expected
    if cls not in (str,int,bool,type(None)):
        raise ValueError('native_jog_region_unsupported_configuration_value')
    return lambda current:type(current) is cls and current==value


def _model_pose_cache():
    # A bounded cache of pure math, never of admission or SDK results. Keys
    # contain all normalized FK inputs; results cannot expose mutable arrays.
    @lru_cache(maxsize=4)
    def model_pose(joints,translation,rotation):
        model,_=mdh_tcp_jacobian(joints,MdhTool(translation,rotation))
        return (tuple(float(v) for v in model[:3,3]),
            tuple(float(v) for v in Rotation.from_matrix(model[:3,:3]).as_quat()))
    return model_pose


def _pose_matches(p, q, expected_p, expected_q, position_m, rotation_deg):
    p, q = _components(p, 3), _components(q, 4)
    if abs(math.hypot(*q)-1.) > 1e-9:
        raise ValueError('native_jog_region_invalid_pose')
    angle = (Rotation.from_quat(q)*Rotation.from_quat(expected_q).inv()).magnitude()
    return math.dist(p, expected_p) <= position_m and math.degrees(angle) <= rotation_deg


@dataclass(frozen=True)
class _ModelRegionGuardCore:
    settings: LebaiSettings
    limits: NativeJogLimits
    config_sha256: str
    reference_q: tuple[float, ...]
    tool: MdhTool
    joint_radius_rad: tuple[float, ...]
    initial_offset_rad: tuple[float, ...]
    duration_s: float
    _settings_sha256: str = field(init=False, repr=False)
    _settings_match: object = field(init=False, repr=False, compare=False)
    _model_pose: object = field(init=False, repr=False, compare=False)
    _assessment_json: str = field(init=False, repr=False)

    def __post_init__(self):
        if (type(self.settings) is not LebaiSettings
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
        object.__setattr__(self, '_settings_match', _value_matcher(s))
        object.__setattr__(self, '_model_pose', _model_pose_cache())
        object.__setattr__(self, '_assessment_json', json.dumps(result, allow_nan=False))

    @property
    def assessment(self):
        return json.loads(self._assessment_json)  # Never expose mutable cached evidence.

    def require_binding(self, settings, limits, config_sha256):
        if (settings is not self.settings or limits is not self.limits
                or config_sha256 != self.config_sha256 or not self._settings_match(settings)):
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
            model_p,model_q=self._model_pose(q,_components(self.tool.translation_m,3),
                _components(self.tool.rotation_xyzw,4))
            if not _pose_matches(pose.p, pose.q,model_p,model_q,.0001,.1):
                raise ValueError('native_jog_region_feedback_mismatch')


@dataclass(frozen=True)
class ModelJogRegionGuard(_ModelRegionGuardCore):
    def __post_init__(self):
        if self.settings.ip != '127.0.0.1':
            raise ValueError('native_jog_region_injected_loopback_required')
        super().__post_init__()
