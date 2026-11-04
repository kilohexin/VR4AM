"""Bounded full-IK Docker pose profile; not used by real/velocity executors."""
from dataclasses import dataclass


@dataclass(frozen=True)
class DockerPoseProfile:
    quarantined: bool = True  # Cartesian path deviation observed during pitch reversal.
    duration_ms: int = 300_000
    maximum_points: int = 3750
    target_tcp_span_m: float = .300
    feedback_tcp_span_m: float = .350
    mailbox_ttl_ms: int = 500
    reference_speed_radps: float = .2
    reference_acceleration_radps2: float = .2
    reference_jerk_radps3: float = 1.
    response_frequency: float = 4.
    translation_scale: float = 1.


POSE_PROFILE = DockerPoseProfile()
