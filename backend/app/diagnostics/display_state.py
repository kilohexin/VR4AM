"""Offline display composition prototype; not a robot_state wire message.

Read-only composition deliberately performs none of RobotControl.state_message's
fault latching, recording, or stop-completion transitions. Runtime integration
must retain those authoritative responsibilities elsewhere before replacement.
"""

from dataclasses import dataclass
from typing import Literal

from app.diagnostics.display_snapshot import DisplaySnapshotView
from app.schemas.messages import (
    BackendState, ConstraintKind, JointVector, Pose, RecoveryPhase, TeleopMode,
)


@dataclass(frozen=True)
class LiveDisplayControl:
    server_mono_ns: int
    ack_seq: int | None
    mode: TeleopMode
    vr_input_age_ms: float | None
    fault: str | None
    constraint: ConstraintKind | None
    recovery_phase: RecoveryPhase | None
    translation_scale: float
    control_generation: int


@dataclass(frozen=True)
class DisplayRobotMeasurement:
    # Whitelist measurement fields only. Never leak cached ack/mode/VR age or
    # readiness into the current control metadata, even if present in a sample.
    actual_tcp: Pose
    actual_q: JointVector
    gripper: float
    robot_state: BackendState
    backend: str | None
    real_robot_mode: str | None


@dataclass(frozen=True)
class ComposedDisplayState:
    control: LiveDisplayControl
    robot: DisplayRobotMeasurement | None
    status: Literal["available", "stale", "missing", "disconnected", "invalid_clock"]
    connection_generation: int
    fault: str | None
    local_read_age_ms: float | None
    local_completion_age_ms: float | None
    read_started_ns: int | None
    read_completed_ns: int | None
    read_purpose: str | None
    source_freshness: Literal["unknown"]


def compose_display_state(
    view: DisplaySnapshotView, current: LiveDisplayControl,
) -> ComposedDisplayState:
    sample = view.read(fault=current.fault)
    state = sample.state
    robot = None if state is None else DisplayRobotMeasurement(
        actual_tcp=state.actual_tcp, actual_q=state.actual_q,
        gripper=state.gripper, robot_state=state.robot_state,
        backend=state.backend, real_robot_mode=state.real_robot_mode,
    )
    return ComposedDisplayState(
        control=current, robot=robot, status=sample.status,
        connection_generation=sample.generation, fault=sample.fault,
        local_read_age_ms=sample.local_read_age_ms,
        local_completion_age_ms=sample.local_completion_age_ms,
        read_started_ns=sample.read_started_ns, read_completed_ns=sample.read_completed_ns,
        read_purpose=sample.read_purpose, source_freshness=sample.source_freshness,
    )
