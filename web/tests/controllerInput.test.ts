import {expect, it} from 'vitest';
import {readRightController} from '../src/xr/controllerInput';

const source = (handedness: 'left' | 'right', trigger = 0.3, grip = 0.6) => ({
  handedness,
  gripSpace: {},
  gamepad: {buttons: [{value: trigger}, {value: grip}]},
}) as unknown as XRInputSource;

const frame = (hasPose = true) => ({
  getPose: () => hasPose ? {
    transform: {
      position: {x: 1, y: 2, z: 3},
      orientation: {x: 0, y: 0, z: 0, w: 1},
    },
  } : null,
}) as unknown as XRFrame;

it('selects right gripSpace and maps buttons', () => {
  const sample = readRightController(
    frame(),
    {} as XRReferenceSpace,
    [source('left'), source('right')],
  );

  expect(sample?.p).toEqual([1, 2, 3]);
  expect(sample?.grip).toBe(true);
  expect(sample?.trigger).toBeCloseTo(0.3);
  expect(sample?.trackingValid).toBe(true);
});

it('reports tracking loss when pose is absent', () => {
  expect(
    readRightController(frame(false), {} as XRReferenceSpace, [source('right')])?.trackingValid,
  ).toBe(false);
});

it('does not use left controller', () => {
  expect(readRightController(frame(), {} as XRReferenceSpace, [source('left')])).toBeNull();
});

it('reports tracking loss instead of falling back when right gripSpace is absent', () => {
  const right = {
    handedness: 'right',
    targetRaySpace: {},
    gamepad: {buttons: [{value: 1}, {value: 1}]},
  } as unknown as XRInputSource;

  expect(readRightController(frame(), {} as XRReferenceSpace, [right])).toMatchObject({
    trackingValid: false,
    grip: false,
    trigger: 0,
  });
});

it('clamps trigger values and treats button pressed as Grip', () => {
  const right = {
    handedness: 'right',
    gripSpace: {},
    gamepad: {buttons: [{value: 1.4}, {value: 0.1, pressed: true}]},
  } as unknown as XRInputSource;

  const sample = readRightController(frame(), {} as XRReferenceSpace, [right]);

  expect(sample?.trigger).toBe(1);
  expect(sample?.grip).toBe(true);
});

it.each([
  ['absent', {}],
  ['null', {buttons: null}],
] as const)('returns tracking-invalid neutral input when gamepad buttons are %s', (_name, gamepad) => {
  const right = {
    handedness: 'right',
    gripSpace: {},
    gamepad,
  } as unknown as XRInputSource;

  expect(() => readRightController(frame(), {} as XRReferenceSpace, [right])).not.toThrow();
  expect(readRightController(frame(), {} as XRReferenceSpace, [right])).toEqual({
    p: [0, 0, 0],
    q: [0, 0, 0, 1],
    grip: false,
    trigger: 0,
    trackingValid: false,
  });
});
