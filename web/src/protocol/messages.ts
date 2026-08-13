export const PROTOCOL_VERSION = 1 as const;

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];
export type JointVector = [number, number, number, number, number, number];

export interface Pose {
  p: Vec3;
  q: Quat;
}

export interface ControllerState extends Pose {
  grip: boolean;
  trigger: number;
}

export type VisibilityState = 'visible' | 'visible-blurred' | 'hidden';
export type ConstraintKind =
  | 'workspace_boundary'
  | 'ik_boundary'
  | 'joint_boundary'
  | 'motion_continuity_boundary'
  | 'self_collision';
export type RecoveryPhase = 'stopping' | 'homing' | 'stabilizing';

export interface VRFrame {
  v: typeof PROTOCOL_VERSION;
  type: 'vr_frame';
  session_id: string;
  seq: number;
  client_mono_ms: number;
  tracking_valid: boolean;
  visibility: VisibilityState;
  right: ControllerState;
  head_q?: Quat | null;
}

export type TeleopMode =
  | 'DISCONNECTED'
  | 'READY'
  | 'ARMED'
  | 'ACTIVE'
  | 'HOLD'
  | 'STALE'
  | 'FAULT'
  | 'DISARMED';

export type BackendState = 'DISCONNECTED' | 'IDLE' | 'MOVING' | 'HOLD' | 'FAULT';
export type RuntimeBackend = 'SIMULATOR' | 'LEBAI' | 'LEBAI_FAKE';
export type RealRobotMode = 'readonly' | 'control';

export interface RobotStateMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'robot_state';
  server_mono_ns: number;
  ack_seq?: number | null;
  mode: TeleopMode;
  robot_state: BackendState;
  actual_tcp: Pose;
  actual_q: JointVector;
  gripper: number;
  sample_age_ms?: number | null;
  fault?: string | null;
  constraint?: ConstraintKind | null;
  recovery_phase?: RecoveryPhase | null;
  backend?: RuntimeBackend | null;
  real_robot_mode?: RealRobotMode | null;
  preflight_ready?: boolean | null;
  preflight_reason?: string | null;
  translation_scale?: number | null;
}

export type DiagnosticPayloadValue =
  | string
  | number
  | boolean
  | null
  | DiagnosticPayloadValue[]
  | {readonly [key: string]: DiagnosticPayloadValue};

export interface DiagnosticEvent {
  event_id: number;
  server_mono_ns: number;
  kind: string;
  critical: boolean;
  payload: Record<string, DiagnosticPayloadValue>;
}

export interface DiagnosticsMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'diagnostics';
  server_mono_ns: number;
  runtime: RuntimeBackend;
  hardware_verified: false;
  control_generation: number;
  actual_qd: JointVector | null;
  actual_qdd: JointVector | null;
  target_q: JointVector | null;
  target_qd: JointVector | null;
  target_qdd: JointVector | null;
  target_tcp: Pose | null;
  sdk_latencies_ms: Record<string, number>;
  pvat_send_hz: number | null;
  log_session_dir: string | null;
  dropped_events: number;
  recent_events: DiagnosticEvent[];
}

export type ClientControlType =
  | 'hello'
  | 'arm_request'
  | 'disarm'
  | 'reset_fault'
  | 'home_request'
  | 'ping';

export interface ClientControlMessage {
  v: typeof PROTOCOL_VERSION;
  type: ClientControlType;
  request_id: string;
  client_mono_ms?: number | null;
}

export interface ArmAckMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'arm_ack';
  request_id: string;
}

export interface ArmRejectedMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'arm_rejected';
  request_id: string;
  message: string;
}

export type ArmFeedbackMessage = ArmAckMessage | ArmRejectedMessage;

export interface ConnectionRejectedMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'connection_rejected';
  reason: 'controller_occupied';
  message: string;
}

export type FaultResetRejectReason =
  | 'no_fault'
  | 'stop_incomplete'
  | 'backend_moving'
  | 'unrecoverable_fault'
  | 'control_loop_unavailable';

export type FaultResetResultMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'fault_reset_result';
      request_id: string;
      accepted: true;
      mode: 'DISARMED';
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'fault_reset_result';
      request_id: string;
      accepted: false;
      reason: FaultResetRejectReason;
      message: string;
    };

export type HomeRejectReason =
  | 'fault_present'
  | 'grip_pressed'
  | 'not_stopped'
  | 'control_loop_unavailable'
  | 'home_failed';

export type HomeResultMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'home_result';
      request_id: string;
      accepted: true;
      mode: 'DISARMED';
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'home_result';
      request_id: string;
      accepted: false;
      reason: HomeRejectReason;
      message: string;
    };

export type OfflineRehearsalPhase =
  | 'identity_preflight'
  | 'home'
  | 'arm_and_anchor'
  | 'translate'
  | 'rotate'
  | 'gripper'
  | 'pick_place'
  | 'soft_boundary'
  | 'tracking_loss'
  | 'recovery_and_home'
  | 'final_stop'
  | 'finalize';

export type OfflineRehearsalOutcome = 'passed' | 'failed' | 'aborted';

export const OFFLINE_REHEARSAL_HARDWARE_PENDING = [
  'sdk_connection',
  'tcp_home_joint_limits',
  'translation_direction',
  'rotation_direction',
  'gripper_direction_force',
  'pvat_tracking_latency',
  'stop_distance_estop',
  'lightweight_grasp_release',
] as const;

export type OfflineRehearsalHardwarePending = typeof OFFLINE_REHEARSAL_HARDWARE_PENDING;

export interface OfflineRehearsalBeginMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'offline_rehearsal_begin';
  request_id: string;
  plan_version: 1;
}

export interface SetSimulationScaleMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'set_simulation_scale';
  request_id: string;
  translation_scale: number;
}

export type SimulationScaleRejectReason =
  | 'not_simulation'
  | 'not_stopped'
  | 'invalid_scale'
  | 'automation_active';

export type SimulationScaleResultMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'simulation_scale_result';
      request_id: string;
      accepted: true;
      translation_scale: number;
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'simulation_scale_result';
      request_id: string;
      accepted: false;
      translation_scale: number;
      reason: SimulationScaleRejectReason;
    };

export interface OfflineRehearsalPhaseMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'offline_rehearsal_phase';
  request_id: string;
  run_id: string;
  phase: OfflineRehearsalPhase;
  status: 'passed' | 'failed';
  started_client_ms: number;
  completed_client_ms: number;
  target: Record<string, DiagnosticPayloadValue>;
  measurements: Record<string, DiagnosticPayloadValue>;
  failure: Record<string, DiagnosticPayloadValue> | null;
}

export interface OfflineRehearsalFinishMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'offline_rehearsal_finish';
  request_id: string;
  run_id: string;
  outcome: OfflineRehearsalOutcome;
  failure: Record<string, DiagnosticPayloadValue> | null;
}

export type OfflineRehearsalClientMessage =
  | OfflineRehearsalBeginMessage
  | OfflineRehearsalPhaseMessage
  | OfflineRehearsalFinishMessage;

export type OfflineRehearsalBeginResultMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'offline_rehearsal_begin_result';
      request_id: string;
      accepted: true;
      run_id: string;
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'offline_rehearsal_begin_result';
      request_id: string;
      accepted: false;
      reason: string;
    };

export type OfflineRehearsalPhaseAckMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'offline_rehearsal_phase_ack';
      request_id: string;
      accepted: true;
      run_id: string;
      phase: OfflineRehearsalPhase;
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'offline_rehearsal_phase_ack';
      request_id: string;
      accepted: false;
      run_id: string;
      phase: OfflineRehearsalPhase;
      reason: string;
    };

export type OfflineRehearsalFinishResultMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'offline_rehearsal_finish_result';
      request_id: string;
      accepted: true;
      run_id: string;
      outcome: OfflineRehearsalOutcome;
      json_path: string;
      markdown_path: string;
      hardware_verified: false;
      hardware_pending: OfflineRehearsalHardwarePending;
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'offline_rehearsal_finish_result';
      request_id: string;
      accepted: false;
      run_id: string;
      reason: string;
    };

export type OfflineRehearsalFeedbackMessage =
  | OfflineRehearsalBeginResultMessage
  | OfflineRehearsalPhaseAckMessage
  | OfflineRehearsalFinishResultMessage;

type UnknownRecord = Record<string, unknown>;

const TELEOP_MODES: readonly TeleopMode[] = [
  'DISCONNECTED',
  'READY',
  'ARMED',
  'ACTIVE',
  'HOLD',
  'STALE',
  'FAULT',
  'DISARMED',
];
const BACKEND_STATES: readonly BackendState[] = [
  'DISCONNECTED',
  'IDLE',
  'MOVING',
  'HOLD',
  'FAULT',
];
const VISIBILITY_STATES: readonly VisibilityState[] = [
  'visible',
  'visible-blurred',
  'hidden',
];
const CONSTRAINT_KINDS: readonly ConstraintKind[] = [
  'workspace_boundary',
  'ik_boundary',
  'joint_boundary',
  'motion_continuity_boundary',
  'self_collision',
];
const RECOVERY_PHASES: readonly RecoveryPhase[] = ['stopping', 'homing', 'stabilizing'];
const REAL_ROBOT_MODES: readonly RealRobotMode[] = ['readonly', 'control'];
const CONTROL_TYPES: readonly ClientControlType[] = [
  'hello',
  'arm_request',
  'disarm',
  'reset_fault',
  'home_request',
  'ping',
];
const FAULT_RESET_REJECT_REASONS: readonly FaultResetRejectReason[] = [
  'no_fault',
  'stop_incomplete',
  'backend_moving',
  'unrecoverable_fault',
  'control_loop_unavailable',
];
const HOME_REJECT_REASONS: readonly HomeRejectReason[] = [
  'fault_present',
  'grip_pressed',
  'not_stopped',
  'control_loop_unavailable',
  'home_failed',
];
const OFFLINE_REHEARSAL_PHASES: readonly OfflineRehearsalPhase[] = [
  'identity_preflight',
  'home',
  'arm_and_anchor',
  'translate',
  'rotate',
  'gripper',
  'pick_place',
  'soft_boundary',
  'tracking_loss',
  'recovery_and_home',
  'final_stop',
  'finalize',
];
const OFFLINE_REHEARSAL_OUTCOMES: readonly OfflineRehearsalOutcome[] = [
  'passed',
  'failed',
  'aborted',
];

function isRecord(value: unknown): value is UnknownRecord {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function hasExactKeys(
  value: UnknownRecord,
  required: readonly string[],
  optional: readonly string[] = [],
): boolean {
  const keys = Object.keys(value);
  const allowed = new Set([...required, ...optional]);
  return (
    required.every((key) => Object.hasOwn(value, key)) && keys.every((key) => allowed.has(key))
  );
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}

function isNonNegativeNumber(value: unknown): value is number {
  return isFiniteNumber(value) && value >= 0;
}

function isNonNegativeInteger(value: unknown): value is number {
  return isNonNegativeNumber(value) && Number.isInteger(value);
}

function isUnitInterval(value: unknown): value is number {
  return isFiniteNumber(value) && value >= 0 && value <= 1;
}

function isSimulationScale(value: unknown): value is number {
  if (!isFiniteNumber(value)) return false;
  const scaled = Math.round(value * 10);
  return scaled >= 5 && scaled <= 100 && Math.abs(value * 10 - scaled) <= 1e-9;
}

function isPositiveNumber(value: unknown): value is number {
  return isFiniteNumber(value) && value > 0;
}

function codePointLength(value: string): number {
  return Array.from(value).length;
}

function isTupleOfFiniteNumbers(value: unknown, length: number): value is number[] {
  return Array.isArray(value) && value.length === length && value.every(isFiniteNumber);
}

function isVec3(value: unknown): value is Vec3 {
  return isTupleOfFiniteNumbers(value, 3);
}

function isQuat(value: unknown): value is Quat {
  if (!isTupleOfFiniteNumbers(value, 4)) return false;
  const norm = Math.sqrt(value.reduce((sum, component) => sum + component * component, 0));
  return norm >= 0.9 && norm <= 1.1;
}

function isJointVector(value: unknown): value is JointVector {
  return isTupleOfFiniteNumbers(value, 6);
}

function isPose(value: unknown): value is Pose {
  return (
    isRecord(value) &&
    hasExactKeys(value, ['p', 'q']) &&
    isVec3(value.p) &&
    isQuat(value.q)
  );
}

function isControllerState(value: unknown): value is ControllerState {
  return (
    isRecord(value) &&
    hasExactKeys(value, ['p', 'q', 'grip', 'trigger']) &&
    isVec3(value.p) &&
    isQuat(value.q) &&
    typeof value.grip === 'boolean' &&
    isUnitInterval(value.trigger)
  );
}

function isEnumValue<T extends string>(values: readonly T[], value: unknown): value is T {
  return typeof value === 'string' && values.includes(value as T);
}

function isNullableNonNegativeInteger(value: unknown): boolean {
  return value === null || isNonNegativeInteger(value);
}

function isNullableNonNegativeNumber(value: unknown): boolean {
  return value === null || isNonNegativeNumber(value);
}

function isIdentifier(value: unknown): value is string {
  return (
    typeof value === 'string' &&
    codePointLength(value) >= 1 &&
    codePointLength(value) <= 64
  );
}

function isNonEmptyString(value: unknown): value is string {
  return typeof value === 'string' && codePointLength(value) >= 1;
}

function isNullableJointVector(value: unknown): boolean {
  return value === null || isJointVector(value);
}

function isSdkLatencies(value: unknown): value is Record<string, number> {
  return isRecord(value) && Object.values(value).every(isNonNegativeNumber);
}

function isDiagnosticPayloadValue(
  value: unknown,
  ancestors: ReadonlySet<object> = new Set(),
): value is DiagnosticPayloadValue {
  if (
    value === null ||
    typeof value === 'string' ||
    typeof value === 'boolean' ||
    isFiniteNumber(value)
  ) {
    return true;
  }
  if (typeof value !== 'object' || value === null || ancestors.has(value)) return false;
  const nextAncestors = new Set(ancestors).add(value);
  if (Array.isArray(value)) return value.every((item) => isDiagnosticPayloadValue(item, nextAncestors));
  if (Object.getPrototypeOf(value) !== Object.prototype && Object.getPrototypeOf(value) !== null) {
    return false;
  }
  return isRecord(value) && Object.values(value).every((item) => isDiagnosticPayloadValue(item, nextAncestors));
}

function isDiagnosticPayloadRecord(value: unknown): value is Record<string, DiagnosticPayloadValue> {
  return isRecord(value) && isDiagnosticPayloadValue(value);
}

function isDiagnosticEvent(value: unknown): value is DiagnosticEvent {
  return (
    isRecord(value) &&
    hasExactKeys(value, ['event_id', 'server_mono_ns', 'kind', 'critical', 'payload']) &&
    isNonNegativeInteger(value.event_id) &&
    value.event_id >= 1 &&
    isNonNegativeInteger(value.server_mono_ns) &&
    typeof value.kind === 'string' &&
    codePointLength(value.kind) >= 1 &&
    codePointLength(value.kind) <= 64 &&
    typeof value.critical === 'boolean' &&
    isRecord(value.payload) &&
    Object.values(value.payload).every((item) => isDiagnosticPayloadValue(item))
  );
}

export function isVRFrame(value: unknown): value is VRFrame {
  if (
    !isRecord(value) ||
    !hasExactKeys(value, [
      'v', 'type', 'session_id', 'seq', 'client_mono_ms', 'tracking_valid', 'visibility', 'right',
    ], ['head_q']) ||
    value.v !== PROTOCOL_VERSION ||
    value.type !== 'vr_frame' ||
    typeof value.session_id !== 'string' ||
    codePointLength(value.session_id) < 1 ||
    codePointLength(value.session_id) > 64 ||
    !isNonNegativeInteger(value.seq) ||
    !isNonNegativeNumber(value.client_mono_ms) ||
    typeof value.tracking_valid !== 'boolean' ||
    !isEnumValue(VISIBILITY_STATES, value.visibility) ||
    !isControllerState(value.right)
  ) {
    return false;
  }
  return (
    !Object.hasOwn(value, 'head_q') || value.head_q === null || isQuat(value.head_q)
  );
}

export function isRobotStateMessage(value: unknown): value is RobotStateMessage {
  if (
    !isRecord(value) ||
    !hasExactKeys(
      value,
      ['v', 'type', 'server_mono_ns', 'mode', 'robot_state', 'actual_tcp', 'actual_q', 'gripper'],
      [
        'ack_seq', 'sample_age_ms', 'fault', 'constraint', 'recovery_phase', 'backend',
        'real_robot_mode', 'preflight_ready', 'preflight_reason',
        'translation_scale',
      ],
    ) ||
    value.v !== PROTOCOL_VERSION ||
    value.type !== 'robot_state' ||
    !isNonNegativeInteger(value.server_mono_ns) ||
    !isEnumValue(TELEOP_MODES, value.mode) ||
    !isEnumValue(BACKEND_STATES, value.robot_state) ||
    !isPose(value.actual_tcp) ||
    !isJointVector(value.actual_q) ||
    !isUnitInterval(value.gripper)
  ) {
    return false;
  }

  if (Object.hasOwn(value, 'ack_seq') && !isNullableNonNegativeInteger(value.ack_seq)) return false;
  if (
    Object.hasOwn(value, 'sample_age_ms') &&
    !isNullableNonNegativeNumber(value.sample_age_ms)
  ) {
    return false;
  }
  if (Object.hasOwn(value, 'fault') && value.fault !== null && typeof value.fault !== 'string') {
    return false;
  }
  if (
    Object.hasOwn(value, 'constraint') &&
    value.constraint !== null &&
    !isEnumValue(CONSTRAINT_KINDS, value.constraint)
  ) {
    return false;
  }
  if (
    Object.hasOwn(value, 'backend') &&
    value.backend !== null &&
    !isEnumValue(['SIMULATOR', 'LEBAI', 'LEBAI_FAKE'], value.backend)
  ) {
    return false;
  }
  if (
    Object.hasOwn(value, 'real_robot_mode') &&
    value.real_robot_mode !== null &&
    !isEnumValue(REAL_ROBOT_MODES, value.real_robot_mode)
  ) {
    return false;
  }
  if (
    Object.hasOwn(value, 'preflight_ready') &&
    value.preflight_ready !== null &&
    typeof value.preflight_ready !== 'boolean'
  ) {
    return false;
  }
  if (
    Object.hasOwn(value, 'preflight_reason') &&
    value.preflight_reason !== null &&
    typeof value.preflight_reason !== 'string'
  ) {
    return false;
  }
  if (
    Object.hasOwn(value, 'translation_scale') &&
    value.translation_scale !== null &&
    !isPositiveNumber(value.translation_scale)
  ) {
    return false;
  }
  return (
    !Object.hasOwn(value, 'recovery_phase') ||
    value.recovery_phase === null ||
    isEnumValue(RECOVERY_PHASES, value.recovery_phase)
  );
}

export function isDiagnosticsMessage(value: unknown): value is DiagnosticsMessage {
  if (
    !isRecord(value) ||
    !hasExactKeys(value, [
      'v', 'type', 'server_mono_ns', 'runtime', 'hardware_verified', 'control_generation',
      'actual_qd', 'actual_qdd', 'target_q', 'target_qd', 'target_qdd', 'target_tcp',
      'sdk_latencies_ms', 'pvat_send_hz', 'log_session_dir', 'dropped_events', 'recent_events',
    ]) ||
    value.v !== PROTOCOL_VERSION ||
    value.type !== 'diagnostics' ||
    !isNonNegativeInteger(value.server_mono_ns) ||
    !isEnumValue(['SIMULATOR', 'LEBAI', 'LEBAI_FAKE'], value.runtime) ||
    value.hardware_verified !== false ||
    !isNonNegativeInteger(value.control_generation) ||
    !isNullableJointVector(value.actual_qd) ||
    !isNullableJointVector(value.actual_qdd) ||
    !isNullableJointVector(value.target_q) ||
    !isNullableJointVector(value.target_qd) ||
    !isNullableJointVector(value.target_qdd) ||
    !(value.target_tcp === null || isPose(value.target_tcp)) ||
    !isSdkLatencies(value.sdk_latencies_ms) ||
    !isNullableNonNegativeNumber(value.pvat_send_hz) ||
    !(value.log_session_dir === null || typeof value.log_session_dir === 'string') ||
    !isNonNegativeInteger(value.dropped_events) ||
    !Array.isArray(value.recent_events) ||
    !value.recent_events.every(isDiagnosticEvent)
  ) {
    return false;
  }
  return true;
}

export function isClientControlMessage(value: unknown): value is ClientControlMessage {
  if (
    !isRecord(value) ||
    !hasExactKeys(value, ['v', 'type', 'request_id'], ['client_mono_ms']) ||
    value.v !== PROTOCOL_VERSION ||
    !isEnumValue(CONTROL_TYPES, value.type) ||
    typeof value.request_id !== 'string' ||
    codePointLength(value.request_id) < 1 ||
    codePointLength(value.request_id) > 64
  ) {
    return false;
  }
  return (
    !Object.hasOwn(value, 'client_mono_ms') || isNullableNonNegativeNumber(value.client_mono_ms)
  );
}

export function isArmFeedbackMessage(value: unknown): value is ArmFeedbackMessage {
  if (
    !isRecord(value) ||
    value.v !== PROTOCOL_VERSION ||
    (value.type !== 'arm_ack' && value.type !== 'arm_rejected') ||
    typeof value.request_id !== 'string' ||
    codePointLength(value.request_id) < 1 ||
    codePointLength(value.request_id) > 64
  ) {
    return false;
  }
  if (value.type === 'arm_ack') return hasExactKeys(value, ['v', 'type', 'request_id']);
  return (
    hasExactKeys(value, ['v', 'type', 'request_id', 'message']) &&
    typeof value.message === 'string'
  );
}

export function isConnectionRejectedMessage(value: unknown): value is ConnectionRejectedMessage {
  return (
    isRecord(value) &&
    hasExactKeys(value, ['v', 'type', 'reason', 'message']) &&
    value.v === PROTOCOL_VERSION &&
    value.type === 'connection_rejected' &&
    value.reason === 'controller_occupied' &&
    typeof value.message === 'string'
  );
}

export function isFaultResetResultMessage(value: unknown): value is FaultResetResultMessage {
  if (
    !isRecord(value) ||
    value.v !== PROTOCOL_VERSION ||
    value.type !== 'fault_reset_result' ||
    typeof value.request_id !== 'string' ||
    codePointLength(value.request_id) < 1 ||
    codePointLength(value.request_id) > 64
  ) {
    return false;
  }

  if (value.accepted === true) {
    return (
      hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'mode']) &&
      value.mode === 'DISARMED'
    );
  }

  return (
    value.accepted === false &&
    hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'reason', 'message']) &&
    isEnumValue(FAULT_RESET_REJECT_REASONS, value.reason) &&
    typeof value.message === 'string'
  );
}

export function isSetSimulationScaleMessage(
  value: unknown,
): value is SetSimulationScaleMessage {
  return (
    isRecord(value) &&
    hasExactKeys(value, ['v', 'type', 'request_id', 'translation_scale']) &&
    value.v === PROTOCOL_VERSION &&
    value.type === 'set_simulation_scale' &&
    isIdentifier(value.request_id) &&
    isSimulationScale(value.translation_scale)
  );
}

export function isSimulationScaleResultMessage(
  value: unknown,
): value is SimulationScaleResultMessage {
  if (
    !isRecord(value) ||
    value.v !== PROTOCOL_VERSION ||
    value.type !== 'simulation_scale_result' ||
    !isIdentifier(value.request_id) ||
    !isSimulationScale(value.translation_scale)
  ) return false;
  if (value.accepted === true) {
    return hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'translation_scale']);
  }
  return (
    value.accepted === false &&
    hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'translation_scale', 'reason']) &&
    isEnumValue(['not_simulation', 'not_stopped', 'invalid_scale', 'automation_active'], value.reason)
  );
}

export function isOfflineRehearsalClientMessage(
  value: unknown,
): value is OfflineRehearsalClientMessage {
  if (
    !isRecord(value) ||
    value.v !== PROTOCOL_VERSION ||
    !isIdentifier(value.request_id)
  ) {
    return false;
  }

  if (value.type === 'offline_rehearsal_begin') {
    return (
      hasExactKeys(value, ['v', 'type', 'request_id', 'plan_version']) &&
      value.plan_version === 1
    );
  }

  if (value.type === 'offline_rehearsal_phase') {
    return (
      hasExactKeys(value, [
        'v', 'type', 'request_id', 'run_id', 'phase', 'status', 'started_client_ms',
        'completed_client_ms', 'target', 'measurements', 'failure',
      ]) &&
      isIdentifier(value.run_id) &&
      isEnumValue(OFFLINE_REHEARSAL_PHASES, value.phase) &&
      (value.status === 'passed' || value.status === 'failed') &&
      isNonNegativeNumber(value.started_client_ms) &&
      isNonNegativeNumber(value.completed_client_ms) &&
      isDiagnosticPayloadRecord(value.target) &&
      isDiagnosticPayloadRecord(value.measurements) &&
      (value.failure === null || isDiagnosticPayloadRecord(value.failure))
    );
  }

  return (
    value.type === 'offline_rehearsal_finish' &&
    hasExactKeys(value, ['v', 'type', 'request_id', 'run_id', 'outcome', 'failure']) &&
    isIdentifier(value.run_id) &&
    isEnumValue(OFFLINE_REHEARSAL_OUTCOMES, value.outcome) &&
    (value.failure === null || isDiagnosticPayloadRecord(value.failure))
  );
}

export function isHomeResultMessage(value: unknown): value is HomeResultMessage {
  if (
    !isRecord(value) ||
    value.v !== PROTOCOL_VERSION ||
    value.type !== 'home_result' ||
    typeof value.request_id !== 'string' ||
    codePointLength(value.request_id) < 1 ||
    codePointLength(value.request_id) > 64
  ) {
    return false;
  }

  if (value.accepted === true) {
    return (
      hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'mode']) &&
      value.mode === 'DISARMED'
    );
  }

  return (
    value.accepted === false &&
    hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'reason', 'message']) &&
    isEnumValue(HOME_REJECT_REASONS, value.reason) &&
    typeof value.message === 'string'
  );
}

export function isOfflineRehearsalFeedbackMessage(
  value: unknown,
): value is OfflineRehearsalFeedbackMessage {
  if (
    !isRecord(value) ||
    value.v !== PROTOCOL_VERSION ||
    !isIdentifier(value.request_id)
  ) {
    return false;
  }

  if (value.type === 'offline_rehearsal_begin_result') {
    if (value.accepted === true) {
      return (
        hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'run_id']) &&
        isIdentifier(value.run_id)
      );
    }
    return (
      value.accepted === false &&
      hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'reason']) &&
      isNonEmptyString(value.reason)
    );
  }

  if (value.type === 'offline_rehearsal_phase_ack') {
    if (
      !isIdentifier(value.run_id) ||
      !isEnumValue(OFFLINE_REHEARSAL_PHASES, value.phase)
    ) {
      return false;
    }
    if (value.accepted === true) {
      return hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'run_id', 'phase']);
    }
    return (
      value.accepted === false &&
      hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'run_id', 'phase', 'reason']) &&
      isNonEmptyString(value.reason)
    );
  }

  if (value.type !== 'offline_rehearsal_finish_result' || !isIdentifier(value.run_id)) {
    return false;
  }
  if (value.accepted === true) {
    return (
      hasExactKeys(value, [
        'v', 'type', 'request_id', 'accepted', 'run_id', 'outcome', 'json_path', 'markdown_path',
        'hardware_verified', 'hardware_pending',
      ]) &&
      isEnumValue(OFFLINE_REHEARSAL_OUTCOMES, value.outcome) &&
      isNonEmptyString(value.json_path) &&
      isNonEmptyString(value.markdown_path) &&
      value.hardware_verified === false &&
      Array.isArray(value.hardware_pending) &&
      value.hardware_pending.length === OFFLINE_REHEARSAL_HARDWARE_PENDING.length &&
      value.hardware_pending.every(
        (item, index) => item === OFFLINE_REHEARSAL_HARDWARE_PENDING[index],
      )
    );
  }
  return (
    value.accepted === false &&
    hasExactKeys(value, ['v', 'type', 'request_id', 'accepted', 'run_id', 'reason']) &&
    isNonEmptyString(value.reason)
  );
}
