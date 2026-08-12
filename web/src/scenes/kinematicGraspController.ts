import * as THREE from 'three';
import type {GraspSceneSnapshot} from '../rehearsal/types';

export interface GraspBlock {
  id: string;
  object: THREE.Object3D;
  sizeM: number;
}

export interface TcpPose {
  p: readonly [number, number, number];
  q: readonly [number, number, number, number];
}

export interface KinematicGraspOptions {
  visualRoot: THREE.Object3D;
  blocks: readonly GraspBlock[];
  tableTopY: number;
  tableHalfWidth: number;
  tableHalfDepth: number;
  captureHalfExtents?: THREE.Vector3;
  closeThreshold?: number;
  releaseThreshold?: number;
  supportCenterTolerance?: number;
}

type InitialTransform = Readonly<{
  block: GraspBlock;
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  scale: THREE.Vector3;
}>;

const DEFAULT_CAPTURE_HALF_EXTENTS = new THREE.Vector3(0.055, 0.045, 0.055);
const DEFAULT_CLOSE_THRESHOLD = 0.65;
const DEFAULT_RELEASE_THRESHOLD = 0.35;
const DEFAULT_SUPPORT_CENTER_TOLERANCE = 0.039;
const OVERLAP_EPSILON = 1e-9;
const COORDINATE_PRECISION = 1e12;

export class KinematicGraspController {
  private readonly captureHalfExtents: THREE.Vector3;
  private readonly closeThreshold: number;
  private readonly releaseThreshold: number;
  private readonly supportCenterTolerance: number;
  private readonly initialTransforms: readonly InitialTransform[];
  private carried: GraspBlock | null = null;
  private closeArmed = true;
  private tableTopY: number;

  constructor(private readonly options: KinematicGraspOptions) {
    this.tableTopY = options.tableTopY;
    this.captureHalfExtents = (
      options.captureHalfExtents ?? DEFAULT_CAPTURE_HALF_EXTENTS
    ).clone();
    this.closeThreshold = options.closeThreshold ?? DEFAULT_CLOSE_THRESHOLD;
    this.releaseThreshold = options.releaseThreshold ?? DEFAULT_RELEASE_THRESHOLD;
    this.supportCenterTolerance = (
      options.supportCenterTolerance ?? DEFAULT_SUPPORT_CENTER_TOLERANCE
    );
    this.validateOptions();
    this.initialTransforms = options.blocks.map((block) => ({
      block,
      position: block.object.position.clone(),
      quaternion: block.object.quaternion.clone(),
      scale: block.object.scale.clone(),
    }));
  }

  update(actualTcp: TcpPose, gripper: number): void {
    if (!Number.isFinite(gripper)) return;
    const {position, quaternion} = validatedTcp(actualTcp);

    if (this.carried) {
      this.writeCarriedPose(position, quaternion);
      if (gripper <= this.releaseThreshold) {
        this.release();
        this.closeArmed = true;
      }
      return;
    }

    if (gripper <= this.releaseThreshold) {
      this.closeArmed = true;
      return;
    }
    if (gripper < this.closeThreshold || !this.closeArmed) return;

    this.closeArmed = false;
    this.carried = this.closestCandidate(position, quaternion);
    if (this.carried) {
      this.writeCarriedPose(position, quaternion);
    }
  }

  reset(): void {
    for (const initial of this.initialTransforms) {
      if (initial.block.object.parent !== this.options.visualRoot) {
        this.options.visualRoot.add(initial.block.object);
      }
      initial.block.object.position.copy(initial.position);
      initial.block.object.quaternion.copy(initial.quaternion);
      initial.block.object.scale.copy(initial.scale);
      initial.block.object.updateMatrix();
      initial.block.object.updateMatrixWorld(true);
    }
    this.carried = null;
    this.closeArmed = true;
    this.tableTopY = this.options.tableTopY;
  }

  setTableTopY(tableTopY: number): void {
    if (!Number.isFinite(tableTopY)) throw new Error('invalid_grasp_options');
    this.tableTopY = tableTopY;
  }

  snapshot(): GraspSceneSnapshot {
    const blocks = this.options.blocks.map((block) => {
      const position = block.object.position.toArray();
      if (
        !position.every(Number.isFinite)
        || !Number.isFinite(block.sizeM)
        || block.sizeM <= 0
      ) {
        throw new Error('invalid_grasp_snapshot');
      }
      return {
        id: block.id,
        position: [position[0], position[1], position[2]] as [number, number, number],
        sizeM: block.sizeM,
      };
    });
    return {
      carriedBlockId: this.carried?.id ?? null,
      blocks,
      invalidOverlap: this.options.blocks.some((first, firstIndex) => (
        this.options.blocks.slice(firstIndex + 1).some((second) => strictlyOverlaps(first, second))
      )),
    };
  }

  private validateOptions(): void {
    const {
      blocks,
      tableTopY,
      tableHalfWidth,
      tableHalfDepth,
      visualRoot,
    } = this.options;
    if (
      !Number.isFinite(tableTopY)
      || !isPositiveFinite(tableHalfWidth)
      || !isPositiveFinite(tableHalfDepth)
      || !isPositiveVector(this.captureHalfExtents)
      || !isUnitInterval(this.closeThreshold)
      || !isUnitInterval(this.releaseThreshold)
      || this.closeThreshold <= this.releaseThreshold
      || !isPositiveFinite(this.supportCenterTolerance)
    ) {
      throw new Error('invalid_grasp_options');
    }

    const ids = new Set<string>();
    const objects = new Set<THREE.Object3D>();
    for (const block of blocks) {
      if (
        typeof block.id !== 'string'
        || block.id.length === 0
        || ids.has(block.id)
        || objects.has(block.object)
        || !isPositiveFinite(block.sizeM)
        || block.sizeM / 2 > tableHalfWidth
        || block.sizeM / 2 > tableHalfDepth
        || block.object.parent !== visualRoot
      ) {
        throw new Error('invalid_grasp_block');
      }
      ids.add(block.id);
      objects.add(block.object);
    }
  }

  private closestCandidate(
    tcpPosition: THREE.Vector3,
    tcpQuaternion: THREE.Quaternion,
  ): GraspBlock | null {
    const inverse = tcpQuaternion.clone().invert();
    let closest: GraspBlock | null = null;
    let closestDistanceSq = Number.POSITIVE_INFINITY;
    for (const block of this.options.blocks) {
      const local = block.object.position
        .clone()
        .sub(tcpPosition)
        .applyQuaternion(inverse);
      if (
        Math.abs(local.x) > this.captureHalfExtents.x
        || Math.abs(local.y) > this.captureHalfExtents.y
        || Math.abs(local.z) > this.captureHalfExtents.z
      ) {
        continue;
      }
      const distanceSq = block.object.position.distanceToSquared(tcpPosition);
      if (distanceSq < closestDistanceSq) {
        closest = block;
        closestDistanceSq = distanceSq;
      }
    }
    return closest;
  }

  private writeCarriedPose(
    tcpPosition: THREE.Vector3,
    tcpQuaternion: THREE.Quaternion,
  ): void {
    if (!this.carried) return;
    this.carried.object.position.copy(tcpPosition);
    this.carried.object.quaternion.copy(tcpQuaternion);
    this.carried.object.scale.set(1, 1, 1);
    this.carried.object.updateMatrix();
    this.carried.object.updateMatrixWorld(true);
  }

  private release(): void {
    if (!this.carried) return;
    const released = this.carried;
    const half = released.sizeM / 2;
    let x = THREE.MathUtils.clamp(
      released.object.position.x,
      -this.options.tableHalfWidth + half,
      this.options.tableHalfWidth - half,
    );
    let z = THREE.MathUtils.clamp(
      released.object.position.z,
      -this.options.tableHalfDepth + half,
      this.options.tableHalfDepth - half,
    );
    const support = this.highestSupport(released, x, z);
    let supportTop = this.tableTopY;
    if (support) {
      x = support.object.position.x;
      z = support.object.position.z;
      supportTop = support.object.position.y + support.sizeM / 2;
    }

    released.object.position.set(
      rounded(x),
      rounded(supportTop + half),
      rounded(z),
    );
    released.object.quaternion.identity();
    released.object.scale.set(1, 1, 1);

    for (let attempt = 0; attempt <= this.options.blocks.length; attempt += 1) {
      const overlaps = this.options.blocks.filter(
        (other) => other !== released && strictlyOverlaps(released, other),
      );
      if (overlaps.length === 0) {
        released.object.updateMatrix();
        released.object.updateMatrixWorld(true);
        this.carried = null;
        return;
      }
      const horizontal = this.options.blocks.filter(
        (other) => other !== released && horizontallyOverlaps(released, other),
      );
      if (horizontal.length === 0) break;
      const highestTop = Math.max(
        ...horizontal.map(
          (other) => other.object.position.y + other.sizeM / 2,
        ),
      );
      released.object.position.y = rounded(highestTop + half);
    }
    throw new Error('invalid_block_stack');
  }

  private highestSupport(
    released: GraspBlock,
    x: number,
    z: number,
  ): GraspBlock | null {
    let support: GraspBlock | null = null;
    let highestTop = Number.NEGATIVE_INFINITY;
    for (const other of this.options.blocks) {
      if (
        other === released
        || Math.abs(other.object.position.x - x) > this.supportCenterTolerance
        || Math.abs(other.object.position.z - z) > this.supportCenterTolerance
      ) {
        continue;
      }
      const top = other.object.position.y + other.sizeM / 2;
      if (top > highestTop) {
        support = other;
        highestTop = top;
      }
    }
    return support;
  }
}

function validatedTcp(actualTcp: TcpPose): {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
} {
  if (
    !Array.isArray(actualTcp.p)
    || actualTcp.p.length !== 3
    || !actualTcp.p.every(Number.isFinite)
    || !Array.isArray(actualTcp.q)
    || actualTcp.q.length !== 4
    || !actualTcp.q.every(Number.isFinite)
  ) {
    throw new Error('invalid_tcp_pose');
  }
  const quaternion = new THREE.Quaternion(...actualTcp.q);
  const norm = quaternion.length();
  if (norm < 0.9 || norm > 1.1) {
    throw new Error('invalid_tcp_pose');
  }
  quaternion.normalize();
  return {
    position: new THREE.Vector3(...actualTcp.p),
    quaternion,
  };
}

function strictlyOverlaps(first: GraspBlock, second: GraspBlock): boolean {
  const threshold = (first.sizeM + second.sizeM) / 2 - OVERLAP_EPSILON;
  return (
    Math.abs(first.object.position.x - second.object.position.x) < threshold
    && Math.abs(first.object.position.y - second.object.position.y) < threshold
    && Math.abs(first.object.position.z - second.object.position.z) < threshold
  );
}

function horizontallyOverlaps(first: GraspBlock, second: GraspBlock): boolean {
  const threshold = (first.sizeM + second.sizeM) / 2 - OVERLAP_EPSILON;
  return (
    Math.abs(first.object.position.x - second.object.position.x) < threshold
    && Math.abs(first.object.position.z - second.object.position.z) < threshold
  );
}

function isPositiveFinite(value: number): boolean {
  return Number.isFinite(value) && value > 0;
}

function isUnitInterval(value: number): boolean {
  return Number.isFinite(value) && value >= 0 && value <= 1;
}

function isPositiveVector(value: THREE.Vector3): boolean {
  return (
    isPositiveFinite(value.x)
    && isPositiveFinite(value.y)
    && isPositiveFinite(value.z)
  );
}

function rounded(value: number): number {
  return Math.round(value * COORDINATE_PRECISION) / COORDINATE_PRECISION;
}
