import * as THREE from 'three';
import type {TrackedPoseSample} from '../xr/controllerInput';

const LEFT_COLOR = 0x75648f;
const RIGHT_COLOR = 0x32d7ff;

type DisposableResource = THREE.BufferGeometry | THREE.Material | THREE.Texture;

export class ControllerHints {
  readonly left: THREE.Group;
  readonly right: THREE.Group;

  private readonly root = new THREE.Group();
  private readonly sharedArrowGeometries: ReadonlySet<THREE.BufferGeometry>;
  private visible = true;
  private leftTrackingValid = false;
  private rightTrackingValid = false;
  private disposed = false;

  constructor(parent: THREE.Object3D) {
    this.root.name = 'controller-hints';
    this.left = createHandHint('left', LEFT_COLOR, 'L');
    this.right = createHandHint('right', RIGHT_COLOR, 'R');
    this.root.add(this.left, this.right);
    this.sharedArrowGeometries = collectArrowGeometries(this.root);
    parent.add(this.root);
  }

  update(left: TrackedPoseSample, right: TrackedPoseSample): void {
    this.leftTrackingValid = updateHand(this.left, left);
    this.rightTrackingValid = updateHand(this.right, right);
    this.applyVisibility();
  }

  setVisible(visible: boolean): void {
    this.visible = visible;
    this.applyVisibility();
  }

  dispose(): void {
    if (this.disposed) return;
    this.disposed = true;

    const resources = collectResources(this.root, this.sharedArrowGeometries);
    this.root.removeFromParent();
    for (const resource of resources) resource.dispose();
  }

  private applyVisibility(): void {
    this.left.visible = this.visible && this.leftTrackingValid;
    this.right.visible = this.visible && this.rightTrackingValid;
  }
}

function createHandHint(
  handedness: 'left' | 'right',
  color: number,
  label: 'L' | 'R',
): THREE.Group {
  const hand = new THREE.Group();
  hand.name = `${handedness}-controller-hint`;
  hand.userData.handedness = handedness;
  hand.visible = false;

  const grip = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.025, 0.08, 6, 12),
    new THREE.MeshStandardMaterial({color, roughness: 0.5, metalness: 0.1}),
  );
  grip.rotation.x = Math.PI / 10;
  hand.add(grip);

  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, -1),
    new THREE.Vector3(0, 0.045, -0.015),
    0.09,
    color,
    0.025,
    0.015,
  );
  hand.add(arrow);

  const sprite = createLabel(label, color);
  sprite.position.set(0, 0.085, 0);
  sprite.scale.set(0.045, 0.045, 1);
  hand.add(sprite);

  return hand;
}

function createLabel(label: 'L' | 'R', color: number): THREE.Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 128;
  const context = canvas.getContext('2d');
  if (!context) throw new Error('Unable to create controller hint label');

  context.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
  context.font = '700 96px sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(label, 64, 68);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
  });
  return new THREE.Sprite(material);
}

function updateHand(group: THREE.Group, sample: TrackedPoseSample): boolean {
  if (!sample.trackingValid) return false;
  group.position.fromArray(sample.p);
  group.quaternion.fromArray(sample.q);
  return true;
}

function collectArrowGeometries(root: THREE.Object3D): Set<THREE.BufferGeometry> {
  const geometries = new Set<THREE.BufferGeometry>();
  root.traverse((object) => {
    if (!(object instanceof THREE.ArrowHelper)) return;
    geometries.add(object.line.geometry);
    geometries.add(object.cone.geometry);
  });
  return geometries;
}

function collectResources(
  root: THREE.Object3D,
  excludedGeometries: ReadonlySet<THREE.BufferGeometry>,
): Set<DisposableResource> {
  const resources = new Set<DisposableResource>();
  root.traverse((object) => {
    if (
      'geometry' in object &&
      object.geometry instanceof THREE.BufferGeometry &&
      !excludedGeometries.has(object.geometry)
    ) {
      resources.add(object.geometry);
    }
    if (!('material' in object)) return;
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) {
      if (!(material instanceof THREE.Material)) continue;
      resources.add(material);
      if ('map' in material && material.map instanceof THREE.Texture) {
        resources.add(material.map);
      }
    }
  });
  return resources;
}
