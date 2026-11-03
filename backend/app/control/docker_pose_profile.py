"""Bounded full-IK Docker pose profile; not used by real/velocity executors."""
from dataclasses import dataclass


@dataclass(frozen=True)
class DockerPoseProfile:
    duration_ms: int = 300_000
    maximum_points: int = 3750
    target_tcp_span_m: float = .100
    feedback_tcp_span_m: float = .120


POSE_PROFILE = DockerPoseProfile()
