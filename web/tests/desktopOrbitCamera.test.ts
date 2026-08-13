// @vitest-environment jsdom
import * as THREE from 'three';
import {describe, expect, it, vi} from 'vitest';
import {DesktopOrbitCamera} from '../src/scenes/desktopOrbitCamera';

describe('DesktopOrbitCamera', () => {
  it('orbits only with the right pointer and resets to the exact default view', () => {
    const canvas = document.createElement('canvas');
    canvas.setPointerCapture = vi.fn();
    canvas.hasPointerCapture = vi.fn(() => false);
    const camera = new THREE.PerspectiveCamera();
    const orbit = new DesktopOrbitCamera(canvas, camera);
    orbit.attach();
    const initial = camera.position.clone();

    canvas.dispatchEvent(pointerEvent('pointerdown', 1, 0));
    canvas.dispatchEvent(pointerEvent('pointermove', 1, 0, 40, 20));
    expect(camera.position.toArray()).toEqual(initial.toArray());

    canvas.dispatchEvent(pointerEvent('pointerdown', 2, 2));
    canvas.dispatchEvent(pointerEvent('pointermove', 2, 2, 40, 20));
    expect(camera.position.distanceTo(initial)).toBeGreaterThan(0.05);

    orbit.reset();
    expect(camera.position.toArray()).toEqual(initial.toArray());
    orbit.dispose();
  });
});

function pointerEvent(
  type: string,
  pointerId: number,
  button: number,
  movementX = 0,
  movementY = 0,
): Event {
  const event = new Event(type, {cancelable: true});
  Object.defineProperties(event, {
    pointerId: {value: pointerId},
    button: {value: button},
    movementX: {value: movementX},
    movementY: {value: movementY},
  });
  return event;
}
