import * as THREE from 'three';

export interface KinematicGraspOptions {
  visualRoot: THREE.Object3D;
  tool: THREE.Object3D;
  cube: THREE.Object3D;
  tcpOffset: THREE.Vector3;
  tableRestY: number;
  tableHalfWidth: number;
  tableHalfDepth: number;
  captureDistance?: number;
  closeThreshold?: number;
  releaseThreshold?: number;
}

export class KinematicGraspController {
  private readonly initialPosition: THREE.Vector3;
  private readonly initialQuaternion: THREE.Quaternion;
  private attached = false;

  constructor(private readonly options: KinematicGraspOptions) {
    this.initialPosition = options.cube.position.clone();
    this.initialQuaternion = options.cube.quaternion.clone();
  }

  update(gripper: number): void {
    if (!Number.isFinite(gripper)) return;
    const closeThreshold = this.options.closeThreshold ?? 0.65;
    const releaseThreshold = this.options.releaseThreshold ?? 0.35;
    if (!this.attached && gripper >= closeThreshold && this.isCubeNearTcp()) {
      this.attach();
    } else if (this.attached && gripper <= releaseThreshold) {
      this.release();
    }
  }

  reset(): void {
    if (this.options.cube.parent !== this.options.visualRoot) {
      this.options.visualRoot.attach(this.options.cube);
    }
    this.attached = false;
    this.options.cube.position.copy(this.initialPosition);
    this.options.cube.quaternion.copy(this.initialQuaternion);
    this.options.cube.updateMatrixWorld(true);
  }

  private isCubeNearTcp(): boolean {
    this.options.visualRoot.updateMatrixWorld(true);
    const tcpWorld = this.options.tool.localToWorld(this.options.tcpOffset.clone());
    const cubeWorld = this.options.cube.getWorldPosition(new THREE.Vector3());
    return tcpWorld.distanceTo(cubeWorld) <= (this.options.captureDistance ?? 0.07);
  }

  private attach(): void {
    this.options.tool.add(this.options.cube);
    this.options.cube.position.copy(this.options.tcpOffset);
    this.options.cube.quaternion.identity();
    this.options.cube.updateMatrixWorld(true);
    this.attached = true;
  }

  private release(): void {
    this.options.visualRoot.updateMatrixWorld(true);
    this.options.visualRoot.attach(this.options.cube);
    this.options.cube.position.x = THREE.MathUtils.clamp(
      this.options.cube.position.x,
      -this.options.tableHalfWidth,
      this.options.tableHalfWidth,
    );
    this.options.cube.position.y = this.options.tableRestY;
    this.options.cube.position.z = THREE.MathUtils.clamp(
      this.options.cube.position.z,
      -this.options.tableHalfDepth,
      this.options.tableHalfDepth,
    );
    this.options.cube.quaternion.identity();
    this.options.cube.updateMatrixWorld(true);
    this.attached = false;
  }
}
