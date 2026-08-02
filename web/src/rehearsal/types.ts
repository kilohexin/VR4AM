import type {Quat, Vec3} from '../protocol/messages';

export const REHEARSAL_PHASES = [
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
] as const;

export type RehearsalPhase = typeof REHEARSAL_PHASES[number];

export const REHEARSAL_CONFIG = {
  planVersion: 1,
  translationDistanceM: 0.020,
  rotationAngleRad: 8 * Math.PI / 180,
  translationToleranceM: 0.003,
  rotationToleranceRad: 2 * Math.PI / 180,
  motionTimeoutMs: 8_000,
  maxPositionStepM: 0.002,
  maxRotationStepRad: Math.PI / 180,
  completionSamples: 3,
} as const;

export interface OfflineControllerSample {
  position: Vec3;
  quaternion: Quat;
  grip: boolean;
  trigger: number;
  trackingValid: boolean;
}

export interface OfflineSceneBlock {
  id: string;
  position: Vec3;
  sizeM: number;
}

export interface GraspSceneSnapshot {
  carriedBlockId: string | null;
  blocks: ReadonlyArray<OfflineSceneBlock>;
  invalidOverlap: boolean;
}

export interface OfflineSceneSnapshot extends GraspSceneSnapshot {
  controller: OfflineControllerSample | null;
}

export function copyOfflineControllerSample(
  sample: OfflineControllerSample,
): OfflineControllerSample {
  if (
    !isFiniteVector(sample.position, 3)
    || !isValidQuaternion(sample.quaternion)
    || typeof sample.grip !== 'boolean'
    || !Number.isFinite(sample.trigger)
    || sample.trigger < 0
    || sample.trigger > 1
    || typeof sample.trackingValid !== 'boolean'
  ) {
    throw new Error('invalid_controller_sample');
  }
  return {
    position: [...sample.position] as Vec3,
    quaternion: [...sample.quaternion] as Quat,
    grip: sample.grip,
    trigger: sample.trigger,
    trackingValid: sample.trackingValid,
  };
}

export function copyOfflineSceneSnapshot(
  snapshot: OfflineSceneSnapshot,
): OfflineSceneSnapshot {
  if (
    (snapshot.controller !== null && !isValidControllerSample(snapshot.controller))
    || typeof snapshot.carriedBlockId !== 'string' && snapshot.carriedBlockId !== null
    || typeof snapshot.invalidOverlap !== 'boolean'
    || !Array.isArray(snapshot.blocks)
  ) {
    throw new Error('invalid_offline_scene_snapshot');
  }
  return {
    controller: snapshot.controller === null ? null : copyOfflineControllerSample(snapshot.controller),
    carriedBlockId: snapshot.carriedBlockId,
    blocks: snapshot.blocks.map((block) => copyBlock(block)),
    invalidOverlap: snapshot.invalidOverlap,
  };
}

function copyBlock(block: OfflineSceneBlock): OfflineSceneBlock {
  if (
    typeof block.id !== 'string'
    || block.id.length === 0
    || !isFiniteVector(block.position, 3)
    || !Number.isFinite(block.sizeM)
    || block.sizeM <= 0
  ) {
    throw new Error('invalid_offline_scene_snapshot');
  }
  return {id: block.id, position: [...block.position] as Vec3, sizeM: block.sizeM};
}

function isValidControllerSample(sample: OfflineControllerSample): boolean {
  try {
    copyOfflineControllerSample(sample);
    return true;
  } catch {
    return false;
  }
}

function isFiniteVector(value: readonly number[], length: number): boolean {
  return Array.isArray(value) && value.length === length && value.every(Number.isFinite);
}

function isValidQuaternion(value: readonly number[]): boolean {
  if (!isFiniteVector(value, 4)) return false;
  const norm = Math.hypot(...value);
  return norm >= 0.9 && norm <= 1.1;
}
