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

export interface VRFrame {
  v: typeof PROTOCOL_VERSION;
  type: 'vr_frame';
  session_id: string;
  seq: number;
  client_mono_ms: number;
  tracking_valid: boolean;
  visibility: VisibilityState;
  right: ControllerState;
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
}

export type ClientControlType = 'hello' | 'arm_request' | 'disarm' | 'ping';

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
const CONTROL_TYPES: readonly ClientControlType[] = ['hello', 'arm_request', 'disarm', 'ping'];

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

export function isVRFrame(value: unknown): value is VRFrame {
  return (
    isRecord(value) &&
    hasExactKeys(value, [
      'v',
      'type',
      'session_id',
      'seq',
      'client_mono_ms',
      'tracking_valid',
      'visibility',
      'right',
    ]) &&
    value.v === PROTOCOL_VERSION &&
    value.type === 'vr_frame' &&
    typeof value.session_id === 'string' &&
    codePointLength(value.session_id) >= 1 &&
    codePointLength(value.session_id) <= 64 &&
    isNonNegativeInteger(value.seq) &&
    isNonNegativeNumber(value.client_mono_ms) &&
    typeof value.tracking_valid === 'boolean' &&
    isEnumValue(VISIBILITY_STATES, value.visibility) &&
    isControllerState(value.right)
  );
}

export function isRobotStateMessage(value: unknown): value is RobotStateMessage {
  if (
    !isRecord(value) ||
    !hasExactKeys(
      value,
      ['v', 'type', 'server_mono_ns', 'mode', 'robot_state', 'actual_tcp', 'actual_q', 'gripper'],
      ['ack_seq', 'sample_age_ms', 'fault'],
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
  return !Object.hasOwn(value, 'fault') || value.fault === null || typeof value.fault === 'string';
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
