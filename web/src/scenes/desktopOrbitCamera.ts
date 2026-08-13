import * as THREE from 'three';

const DEFAULT_POSITION = new THREE.Vector3(1.55, 1.06, 1.9);
const DEFAULT_TARGET = new THREE.Vector3(0.03, 0.42, 0);
const ROTATION_PER_PIXEL = 0.006;

export class DesktopOrbitCamera {
  private activePointer: number | null = null;
  private azimuth = 0;
  private polar = 0;
  private radius = 1;
  private attached = false;
  private readonly ownerWindow: Window;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly camera: THREE.PerspectiveCamera,
  ) {
    this.ownerWindow = canvas.ownerDocument.defaultView ?? window;
    this.reset();
  }

  attach(): void {
    if (this.attached) return;
    this.attached = true;
    this.canvas.addEventListener('pointerdown', this.onPointerDown);
    this.canvas.addEventListener('pointermove', this.onPointerMove);
    this.canvas.addEventListener('pointerup', this.onPointerUp);
    this.canvas.addEventListener('pointercancel', this.onPointerUp);
    this.canvas.addEventListener('lostpointercapture', this.onPointerUp);
    this.ownerWindow.addEventListener('blur', this.onBlur);
  }

  dispose(): void {
    if (!this.attached) return;
    this.release();
    this.attached = false;
    this.canvas.removeEventListener('pointerdown', this.onPointerDown);
    this.canvas.removeEventListener('pointermove', this.onPointerMove);
    this.canvas.removeEventListener('pointerup', this.onPointerUp);
    this.canvas.removeEventListener('pointercancel', this.onPointerUp);
    this.canvas.removeEventListener('lostpointercapture', this.onPointerUp);
    this.ownerWindow.removeEventListener('blur', this.onBlur);
  }

  reset(): void {
    const offset = DEFAULT_POSITION.clone().sub(DEFAULT_TARGET);
    this.radius = offset.length();
    this.polar = Math.acos(offset.y / this.radius);
    this.azimuth = Math.atan2(offset.x, offset.z);
    this.updateCamera();
  }

  private readonly onPointerDown = (event: PointerEvent): void => {
    if (event.button !== 2 || this.activePointer !== null) return;
    event.preventDefault();
    this.activePointer = event.pointerId;
    try {
      this.canvas.setPointerCapture(event.pointerId);
    } catch {
      this.activePointer = null;
    }
  };

  private readonly onPointerMove = (event: PointerEvent): void => {
    if (event.pointerId !== this.activePointer) return;
    event.preventDefault();
    this.azimuth -= event.movementX * ROTATION_PER_PIXEL;
    this.polar = THREE.MathUtils.clamp(
      this.polar + event.movementY * ROTATION_PER_PIXEL,
      0.18,
      Math.PI - 0.18,
    );
    this.updateCamera();
  };

  private readonly onPointerUp = (event: PointerEvent): void => {
    if (event.pointerId === this.activePointer) this.release();
  };

  private readonly onBlur = (): void => this.release();

  private release(): void {
    const pointer = this.activePointer;
    this.activePointer = null;
    if (pointer === null) return;
    try {
      if (this.canvas.hasPointerCapture(pointer)) this.canvas.releasePointerCapture(pointer);
    } catch {
      // Browser may already have released capture.
    }
  }

  private updateCamera(): void {
    const sinPolar = Math.sin(this.polar);
    this.camera.position.set(
      DEFAULT_TARGET.x + this.radius * sinPolar * Math.sin(this.azimuth),
      DEFAULT_TARGET.y + this.radius * Math.cos(this.polar),
      DEFAULT_TARGET.z + this.radius * sinPolar * Math.cos(this.azimuth),
    );
    this.camera.lookAt(DEFAULT_TARGET);
  }
}
