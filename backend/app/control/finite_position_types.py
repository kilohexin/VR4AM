"""Immutable contracts for the explicitly injected offline position candidate."""
from dataclasses import dataclass
import math

MAX_STEP_M=.002
MAX_PITCH_STEP_RAD=math.radians(.5)
MAX_PITCH_RANGE_RAD=math.radians(20.)
POSITION_PLANNING_BOUNDARIES=frozenset({'position_workspace','position_joint_limit',
    'position_joint_jump','position_ik_unreachable','position_estimated_duration',
    'position_path_rotation','position_pitch_limit','position_unresolvable_step'})
INPUT_DEADZONE=.15
MIN_PROGRESS_M=.001
MAX_ENDPOINT_ERROR_M=.0005
JOINT_ENDPOINT_TOLERANCE_RAD=math.radians(.1)
INPUT_NS=100_000_000
PLAN_NS=1_000_000_000
SESSION_NS=10_000_000_000
STEP_NS=2_000_000_000
QUIET_OBSERVATION_NS=1_000_000_000
QUIET_OBSERVATION_READS=5
QUIET_OBSERVATION_INTERVAL_S=.02


@dataclass(frozen=True)
class PositionArrivalPolicy:
    name: str
    endpoint_error_m: float
    orientation_error_rad: float
    stable_tcp_drift_m: float
    stable_joint_drift_rad: float
    stable_span_ns: int
    completion_ns: int
    tail_tcp_drift_m: float
    tail_joint_drift_rad: float
    tail_velocity_radps: float


STRICT_ARRIVAL=PositionArrivalPolicy('strict',MAX_ENDPOINT_ERROR_M,math.radians(.1),
    .00001,.00001,100_000_000,STEP_NS,.00001,.00001,.002)
COARSE_GRASP_ARRIVAL=PositionArrivalPolicy('coarse_grasp',.001,math.radians(.2),
    .0001,math.radians(.02),200_000_000,5_000_000_000,.001,JOINT_ENDPOINT_TOLERANCE_RAD,.02)


def position_arrival_policy(name):
    if type(name) is not str or name not in ('strict','coarse_grasp'):
        raise ValueError('position_invalid_arrival_policy')
    return STRICT_ARRIVAL if name=='strict' else COARSE_GRASP_ARRIVAL


def integer(value):
    if type(value) is not int or not 0<=value<=2**53-1:raise ValueError('position_invalid_integer')
    return value


def vector(value,n):
    if type(value) not in (tuple,list) or len(value)!=n:raise ValueError('position_invalid_vector')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in value):
        raise ValueError('position_invalid_vector')
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class PositionBinding:
    connection: object
    generation: int
    config_sha256: str
    tool_sha256: str

    def __post_init__(self):
        integer(self.generation)
        for v in (self.config_sha256,self.tool_sha256):
            if type(v) is not str or len(v)!=64 or any(c not in '0123456789abcdef' for c in v):
                raise ValueError('position_invalid_identity')


@dataclass(frozen=True)
class PositionIntent:
    session_id: str
    generation: int
    seq: int
    received_ns: int
    kind: str
    vector: tuple
    pitch: float=0.

    def __post_init__(self):
        if type(self.session_id) is not str or not 1<=len(self.session_id)<=128:
            raise ValueError('position_invalid_session')
        for x in (self.generation,self.seq,self.received_ns):integer(x)
        if self.kind not in ('held','release'):raise ValueError('position_invalid_kind')
        v=vector(self.vector,3);length=math.hypot(*v)
        if type(self.pitch) not in (int,float) or not math.isfinite(self.pitch) or abs(self.pitch)>1:
            raise ValueError('position_invalid_pitch')
        pitch=0. if abs(self.pitch)<=INPUT_DEADZONE else math.copysign(1.,self.pitch)
        if self.kind=='release' and (length or self.pitch):raise ValueError('position_release_vector')
        if length<=INPUT_DEADZONE:v=(0.,0.,0.);length=0.
        else:v=tuple(x/length for x in v)
        object.__setattr__(self,'vector',v)
        object.__setattr__(self,'pitch',pitch)
        if not length and not pitch:object.__setattr__(self,'kind','release')


@dataclass(frozen=True)
class PositionSample:
    binding: PositionBinding
    sequence: int
    captured_ns: int
    actual_q: tuple
    target_q: tuple
    actual_qd: tuple
    actual_qdd: tuple
    target_qd: tuple
    target_qdd: tuple
    tcp_p: tuple
    tcp_q: tuple
    robot_state: object
    running_id: object
    task_state: str|None
    fault: str|None

    def __post_init__(self):
        integer(self.sequence);integer(self.captured_ns)
        for name in ('actual_q','target_q','actual_qd','actual_qdd','target_qd','target_qdd'):
            object.__setattr__(self,name,vector(getattr(self,name),6))
        object.__setattr__(self,'tcp_p',vector(self.tcp_p,3))
        object.__setattr__(self,'tcp_q',vector(self.tcp_q,4))
        if abs(math.hypot(*self.tcp_q)-1)>1e-6:raise ValueError('position_invalid_quaternion')


@dataclass(frozen=True)
class PathAudit:
    model_sha256: str
    sample_fractions: tuple
    workspace_ok: bool
    joint_limits_ok: bool
    model_rates_ok: bool
    factory_calibrated: bool
    joint_speed: float
    joint_acceleration: float
    tcp_speed_estimate: float
    tcp_acceleration_estimate: float


@dataclass(frozen=True)
class PositionPlan:
    binding: PositionBinding
    intent_seq: int
    start: PositionSample
    target_p: tuple
    target_q: tuple
    goal_joints: tuple
    joint_speed: float
    joint_acceleration: float
    created_ns: int
    evidence: PathAudit
    arrival_policy: PositionArrivalPolicy=STRICT_ARRIVAL

    def __post_init__(self):
        integer(self.intent_seq);integer(self.created_ns)
        object.__setattr__(self,'target_p',vector(self.target_p,3))
        object.__setattr__(self,'target_q',vector(self.target_q,4))
        object.__setattr__(self,'goal_joints',vector(self.goal_joints,6))
        if (self.arrival_policy is not STRICT_ARRIVAL and self.arrival_policy is not COARSE_GRASP_ARRIVAL):
            raise ValueError('position_invalid_arrival_policy')
        if (self.binding is not self.start.binding
                or self.joint_speed!=self.evidence.joint_speed
                or self.joint_acceleration!=self.evidence.joint_acceleration):
            raise ValueError('position_invalid_plan_binding')


@dataclass(frozen=True)
class PositionResult:
    complete: bool
    motion_id: int|None
    entered: bool
    remote_unknown: bool
    final_sample: PositionSample|None
    reason: str|None


@dataclass(frozen=True)
class PositionView:
    phase: str='inactive'
    generation: int=0
    entered_steps: int=0
    motion_id: int|None=None
    reason: str|None=None
    hardware_verified: bool=False
    blocked_input_seq: int|None=None
