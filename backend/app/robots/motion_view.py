"""Immutable local observations, scoped to one adapter/pump lifetime.

These are cached facts, not motion permission, freshness or physical stop proof.
Reading a view never refreshes the SDK. Consumers must check source generation,
timestamps and their own authority; submission retains the adapter's hard gates.
"""

from dataclasses import dataclass

from app.robots.command_admission import CommandRef
from app.schemas.messages import JointVector, Pose


@dataclass(frozen=True)
class AcceptedTarget:
    command: CommandRef
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    solution_q: JointVector

    @property
    def tcp(self) -> Pose:
        # Pose is mutable. Never expose an alias into the adapter or this view.
        return Pose(p=self.position, q=self.orientation)


@dataclass(frozen=True)
class JointObservation:
    # Captured before waiting for the SDK lock. A read begun before revocation
    # cannot become evidence for a new generation even if it acquired the lock late.
    generation: int
    captured_ns: int
    read_completed_ns: int
    read_purpose: str
    actual_q: JointVector
    actual_qd: JointVector


@dataclass(frozen=True)
class MotionView:
    observed_ns: int
    generation: int
    client_attached: bool  # Local reference exists; does not probe connectivity.
    pump_running: bool
    pump_fault: str | None
    latched_fault: str | None
    preflight_ready: bool
    stop_in_progress: bool
    accepted: AcceptedTarget | None
    sample: JointObservation | None

    def is_current(self, generation: int) -> bool:
        """Pump currency only, not approval to move or proof of fresh telemetry."""
        return self.pump_running and self.pump_fault is None and generation == self.generation
