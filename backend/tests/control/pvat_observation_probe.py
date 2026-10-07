"""TEST ONLY: qualify cached SDK PVA, not motion permission/physical arrival."""
from dataclasses import dataclass

import numpy as np

from app.control.target_strategy import TargetContext
from app.robots.command_admission import CommandRef
from app.robots.motion_view import AcceptedTarget,JointObservation,MotionView


@dataclass(frozen=True)
class ObservationBinding:
    owner_generation: int
    command: CommandRef
    not_before_ns: int


@dataclass(frozen=True)
class QualifiedObservation:
    owner_generation: int
    command: CommandRef
    sample: JointObservation
    checked_ns: int
    age_ns: int
    read_span_ns: int


def qualify_observation(view,context,binding,*,max_age_ns):
    """Pure cached-data check. Read span is uncertainty, not robot sample time."""
    if (type(max_age_ns) is not int or max_age_ns<=0
            or not isinstance(binding,ObservationBinding)
            or type(binding.owner_generation) is not int or binding.owner_generation<0
            or not isinstance(binding.command,CommandRef)
            or any(type(value) is not int or value<0 for value in (
                binding.command.command_id,binding.command.generation,binding.command.submission_id,
                binding.not_before_ns))):
        raise ValueError('invalid_observation_limits')
    if not isinstance(view,MotionView):raise ValueError('pvat_observation_source')
    if (not isinstance(context,TargetContext) or type(context.generation) is not int
            or context.generation!=binding.owner_generation
            or not context.active or not context.authorized):
        raise ValueError('pvat_observation_authority')
    if (not view.client_attached or not view.pump_running or view.pump_fault is not None
            or view.latched_fault is not None or not view.preflight_ready or view.stop_in_progress):
        raise ValueError('pvat_observation_backend')
    sample=view.sample
    if not isinstance(sample,JointObservation) or sample.read_purpose not in {'state','motion'}:
        raise ValueError('pvat_observation_source')
    if (type(view.generation) is not int or type(sample.generation) is not int
            or view.generation!=binding.command.generation or sample.generation!=view.generation):
        raise ValueError('pvat_observation_generation')
    if not isinstance(view.accepted,AcceptedTarget) or not isinstance(view.accepted.command,CommandRef):
        raise ValueError('pvat_observation_accepted_changed')
    accepted=view.accepted.command
    if type(accepted.generation) is not int or accepted.generation!=view.generation:
        raise ValueError('pvat_observation_generation')
    if (type(accepted.command_id) is not int or type(accepted.submission_id) is not int
            or accepted!=binding.command):
        raise ValueError('pvat_observation_accepted_changed')
    timestamps=(sample.captured_ns,sample.read_completed_ns,view.observed_ns,context.now_ns)
    if (any(type(value) is not int or value<0 for value in timestamps)
            or not sample.captured_ns<=sample.read_completed_ns<=view.observed_ns<=context.now_ns):
        raise ValueError('pvat_observation_clock')
    age=context.now_ns-sample.captured_ns
    if age>max_age_ns:raise ValueError('pvat_observation_stale')
    if sample.captured_ns<binding.not_before_ns:raise ValueError('pvat_observation_before_binding')
    if sample.actual_qdd is None:raise ValueError('pvat_observation_acceleration_missing')
    try:
        vectors=np.asarray((sample.actual_q,sample.actual_qd,sample.actual_qdd),dtype=float)
        if vectors.shape!=(3,6) or not np.all(np.isfinite(vectors)):raise ValueError
    except (ValueError,TypeError):
        raise ValueError('pvat_observation_invalid') from None
    detached=JointObservation(sample.generation,sample.captured_ns,sample.read_completed_ns,
                              sample.read_purpose,*(tuple(float(x) for x in row) for row in vectors))
    return QualifiedObservation(binding.owner_generation,binding.command,detached,context.now_ns,
                                age,sample.read_completed_ns-sample.captured_ns)
