// @vitest-environment jsdom
import * as THREE from 'three';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {ControllerHints} from '../src/scenes/controllerHints';
import type {TrackedPoseSample} from '../src/xr/controllerInput';

describe('ControllerHints', () => {
  let context: CanvasRenderingContext2D;

  beforeEach(() => {
    context = canvasContext();
    vi.spyOn(HTMLCanvasElement.prototype, 'getContext').mockReturnValue(context);
  });

  afterEach(() => vi.restoreAllMocks());

  it('creates distinct left and right hints with labels and local -Z arrows', () => {
    const parent = new THREE.Group();
    const hints = new ControllerHints(parent);

    expect(hints.left.userData.handedness).toBe('left');
    expect(hints.right.userData.handedness).toBe('right');
    expect(materialColors(hints.left)).toContain(0x75648f);
    expect(materialColors(hints.right)).toContain(0x32d7ff);
    expect(vi.mocked(context.fillText).mock.calls.map(([text]) => text)).toEqual(['L', 'R']);
    expect(labelFor(hints.left).material.map).toBeInstanceOf(THREE.CanvasTexture);
    expect(labelFor(hints.right).material.map).toBeInstanceOf(THREE.CanvasTexture);

    for (const hand of [hints.left, hints.right]) {
      const arrow = hand.children.find((child): child is THREE.ArrowHelper =>
        child instanceof THREE.ArrowHelper,
      );
      expect(arrow).toBeDefined();
      const direction = new THREE.Vector3(0, 1, 0).applyQuaternion(arrow!.quaternion);
      expect(direction.distanceTo(new THREE.Vector3(0, 0, -1))).toBeLessThan(1e-12);
    }
  });

  it('copies tracked poses exactly and hides tracking loss independently', () => {
    const hints = new ControllerHints(new THREE.Group());
    const left = validPose([-0.2, 1.1, -0.4], [0, 0.5, 0, Math.sqrt(0.75)]);
    const right = validPose([0.2, 1.1, -0.4], [0.1, 0.2, 0.3, 0.9]);

    hints.update(left, right);

    expect(hints.left.visible).toBe(true);
    expect(hints.right.visible).toBe(true);
    expect(hints.left.position.toArray()).toEqual(left.p);
    expect(hints.left.quaternion.toArray()).toEqual(left.q);
    expect(hints.right.position.toArray()).toEqual(right.p);
    expect(hints.right.quaternion.toArray()).toEqual(right.q);

    hints.update(invalidPose(), validPose([0.3, 1, -0.5]));
    expect(hints.left.visible).toBe(false);
    expect(hints.right.visible).toBe(true);
    expect(hints.right.position.toArray()).toEqual([0.3, 1, -0.5]);
  });

  it('gates visibility without losing poses or independent tracking state', () => {
    const hints = new ControllerHints(new THREE.Group());
    const leftPose = validPose([-0.1, 1, -0.3]);
    const rightPose = validPose([0.1, 1, -0.3]);
    hints.update(leftPose, rightPose);

    hints.setVisible(false);
    expect(hints.left.visible).toBe(false);
    expect(hints.right.visible).toBe(false);
    expect(hints.left.position.toArray()).toEqual(leftPose.p);

    const hiddenPose = validPose([-0.4, 0.9, -0.6], [0, 0, 1, 0]);
    hints.update(hiddenPose, invalidPose());
    expect(hints.left.visible).toBe(false);
    expect(hints.left.position.toArray()).toEqual(hiddenPose.p);
    expect(hints.left.quaternion.toArray()).toEqual(hiddenPose.q);

    hints.setVisible(true);
    expect(hints.left.visible).toBe(true);
    expect(hints.right.visible).toBe(false);
  });

  it('removes its root and disposes every owned resource exactly once', () => {
    const externalArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1));
    const parent = new THREE.Group();
    const hints = new ControllerHints(parent);
    const root = hints.left.parent;
    expect(root?.parent).toBe(parent);

    const hintArrow = arrowFor(hints.left);
    expect(hintArrow.line.geometry).toBe(externalArrow.line.geometry);
    expect(hintArrow.cone.geometry).toBe(externalArrow.cone.geometry);
    const sharedGeometries = new Set([
      externalArrow.line.geometry,
      externalArrow.cone.geometry,
    ]);
    const sharedDisposals = [...sharedGeometries]
      .map((geometry) => vi.spyOn(geometry, 'dispose'));
    const resources = ownedResources(root!, sharedGeometries);
    const disposals = resources.map((resource) => vi.spyOn(resource, 'dispose'));

    hints.dispose();
    hints.dispose();

    expect(root?.parent).toBeNull();
    expect(resources.length).toBeGreaterThan(0);
    for (const dispose of sharedDisposals) expect(dispose).not.toHaveBeenCalled();
    for (const dispose of disposals) expect(dispose).toHaveBeenCalledOnce();
  });
});

function validPose(
  p: TrackedPoseSample['p'],
  q: TrackedPoseSample['q'] = [0, 0, 0, 1],
): TrackedPoseSample {
  return {p, q, trackingValid: true};
}

function invalidPose(): TrackedPoseSample {
  return {p: [0, 0, 0], q: [0, 0, 0, 1], trackingValid: false};
}

function materialColors(root: THREE.Object3D): number[] {
  const colors: number[] = [];
  root.traverse((object) => {
    if (!('material' in object)) return;
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    for (const material of materials) {
      if (material instanceof THREE.Material && 'color' in material) {
        colors.push((material as THREE.Material & {color: THREE.Color}).color.getHex());
      }
    }
  });
  return colors;
}

function labelFor(root: THREE.Object3D): THREE.Sprite {
  let label: THREE.Sprite | undefined;
  root.traverse((object) => {
    if (object instanceof THREE.Sprite) label = object;
  });
  if (!label) throw new Error('controller label sprite not found');
  return label;
}

function arrowFor(root: THREE.Object3D): THREE.ArrowHelper {
  const arrow = root.children.find((child): child is THREE.ArrowHelper =>
    child instanceof THREE.ArrowHelper,
  );
  if (!arrow) throw new Error('controller direction arrow not found');
  return arrow;
}

function ownedResources(
  root: THREE.Object3D,
  excludedGeometries: ReadonlySet<THREE.BufferGeometry>,
): Array<THREE.BufferGeometry | THREE.Material | THREE.Texture> {
  const resources = new Set<THREE.BufferGeometry | THREE.Material | THREE.Texture>();
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
      if ('map' in material && material.map instanceof THREE.Texture) resources.add(material.map);
    }
  });
  return [...resources];
}

function canvasContext(): CanvasRenderingContext2D {
  return {
    clearRect: vi.fn(),
    fillText: vi.fn(),
  } as unknown as CanvasRenderingContext2D;
}
