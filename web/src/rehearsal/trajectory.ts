import type {Quat, Vec3} from '../protocol/messages';
import type {TcpPose} from '../scenes/kinematicGraspController';
import {
  copyOfflineControllerSample,
  type OfflineControllerSample,
} from './types';

export type CartesianAxis = 'x' | 'y' | 'z';
export type SignedDirection = -1 | 1;

export interface NextControllerSampleInput {
  sample: OfflineControllerSample;
  controllerAnchor: OfflineControllerSample;
  tcpAnchor: TcpPose;
  targetTcp: TcpPose;
  translationScale: number;
  maxPositionStepM: number;
  maxRotationStepRad: number;
}

interface CopiedTcpPose {
  p: Vec3;
  q: Quat;
}

export function translationTarget(
  anchor: TcpPose,
  axis: CartesianAxis,
  direction: SignedDirection,
  distanceM: number,
): TcpPose {
  const pose = copyPose(anchor);
  if (!isAxis(axis) || !isDirection(direction) || !isPositiveFinite(distanceM)) {
    throw new Error('invalid_translation');
  }
  const index = axis === 'x' ? 0 : axis === 'y' ? 1 : 2;
  const position = [...pose.p] as Vec3;
  position[index] += direction * distanceM;
  return {p: position, q: pose.q};
}

export function rotationTarget(
  anchor: TcpPose,
  axis: CartesianAxis,
  direction: SignedDirection,
  angleRad: number,
): TcpPose {
  const pose = copyPose(anchor);
  if (!isAxis(axis) || !isDirection(direction) || !isPositiveFinite(angleRad)) {
    throw new Error('invalid_rotation');
  }
  const half = direction * angleRad / 2;
  const sine = Math.sin(half);
  const worldRotation: Quat = axis === 'x'
    ? [sine, 0, 0, Math.cos(half)]
    : axis === 'y'
      ? [0, sine, 0, Math.cos(half)]
      : [0, 0, sine, Math.cos(half)];
  return {p: pose.p, q: multiplyQuaternion(worldRotation, pose.q)};
}

export function quaternionAngularError(
  first: readonly number[],
  second: readonly number[],
): number {
  const a = copyQuaternion(first);
  const b = copyQuaternion(second);
  const dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(clamp(dot, -1, 1));
}

export function nextControllerSample(input: NextControllerSampleInput): OfflineControllerSample {
  const sample = copyOfflineControllerSample(input.sample);
  const controllerAnchor = copyOfflineControllerSample(input.controllerAnchor);
  const tcpAnchor = copyPose(input.tcpAnchor);
  const target = copyPose(input.targetTcp);
  if (
    !isPositiveFinite(input.translationScale)
    || !isPositiveFinite(input.maxPositionStepM)
    || !isPositiveFinite(input.maxRotationStepRad)
  ) {
    throw new Error('invalid_controller_step');
  }

  const mappedPosition: Vec3 = [
    controllerAnchor.position[0] + (target.p[0] - tcpAnchor.p[0]) / input.translationScale,
    controllerAnchor.position[1] + (target.p[1] - tcpAnchor.p[1]) / input.translationScale,
    controllerAnchor.position[2] + (target.p[2] - tcpAnchor.p[2]) / input.translationScale,
  ];
  const difference: Vec3 = [
    mappedPosition[0] - sample.position[0],
    mappedPosition[1] - sample.position[1],
    mappedPosition[2] - sample.position[2],
  ];
  const distance = Math.hypot(...difference);
  const positionScale = distance === 0 ? 0 : Math.min(1, input.maxPositionStepM / distance);
  const position: Vec3 = [
    sample.position[0] + difference[0] * positionScale,
    sample.position[1] + difference[1] * positionScale,
    sample.position[2] + difference[2] * positionScale,
  ];
  const targetDelta = multiplyQuaternion(target.q, invertQuaternion(tcpAnchor.q));
  const mappedQuaternion = multiplyQuaternion(targetDelta, controllerAnchor.quaternion);
  const error = shortestRotation(multiplyQuaternion(mappedQuaternion, invertQuaternion(sample.quaternion)));
  const errorAngle = 2 * Math.acos(clamp(error[3], -1, 1));
  const rotationStep = Math.min(errorAngle, input.maxRotationStepRad);
  const quaternion = rotationStep === 0
    ? sample.quaternion
    : multiplyQuaternion(rotationStepQuaternion(error, errorAngle, rotationStep), sample.quaternion);
  return {
    position,
    quaternion,
    grip: sample.grip,
    trigger: sample.trigger,
    trackingValid: sample.trackingValid,
  };
}

function copyPose(value: TcpPose): CopiedTcpPose {
  if (!isFiniteVector(value.p, 3) || !isFiniteVector(value.q, 4)) {
    throw new Error('invalid_tcp_pose');
  }
  return {p: [...value.p] as Vec3, q: copyQuaternion(value.q, 'invalid_tcp_pose')};
}

function copyQuaternion(value: readonly number[], message = 'invalid_quaternion'): Quat {
  if (!isFiniteVector(value, 4)) throw new Error(message);
  const norm = Math.hypot(...value);
  if (norm < 0.9 || norm > 1.1) throw new Error(message);
  return [value[0] / norm, value[1] / norm, value[2] / norm, value[3] / norm];
}

function multiplyQuaternion(first: Quat, second: Quat): Quat {
  const [ax, ay, az, aw] = first;
  const [bx, by, bz, bw] = second;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

function invertQuaternion(value: Quat): Quat {
  return [-value[0], -value[1], -value[2], value[3]];
}

function shortestRotation(value: Quat): Quat {
  return value[3] < 0 ? [-value[0], -value[1], -value[2], -value[3]] : value;
}

function rotationStepQuaternion(error: Quat, angle: number, step: number): Quat {
  if (angle === 0) return [0, 0, 0, 1];
  const scale = Math.sin(step / 2) / Math.sin(angle / 2);
  return [error[0] * scale, error[1] * scale, error[2] * scale, Math.cos(step / 2)];
}

function isFiniteVector(value: readonly number[], length: number): boolean {
  return Array.isArray(value) && value.length === length && value.every(Number.isFinite);
}

function isPositiveFinite(value: number): boolean {
  return Number.isFinite(value) && value > 0;
}

function isAxis(value: string): value is CartesianAxis {
  return value === 'x' || value === 'y' || value === 'z';
}

function isDirection(value: number): value is SignedDirection {
  return value === -1 || value === 1;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}
