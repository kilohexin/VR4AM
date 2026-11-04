"""Bounded full-IK Docker pose profile; not used by real/velocity executors."""
from dataclasses import dataclass


@dataclass(frozen=True)
class DockerPoseProfile:
    duration_ms: int = 300_000
    maximum_points: int = 3750
    target_tcp_span_m: float = .100
    feedback_tcp_span_m: float = .120
    mailbox_ttl_ms: int = 250
    reference_speed_radps: float = .2
    reference_acceleration_radps2: float = .2
    reference_jerk_radps3: float = 1.
    response_frequency: float = 4.


POSE_PROFILE = DockerPoseProfile()
