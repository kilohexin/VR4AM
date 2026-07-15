import {expect, it} from 'vitest';
import {readRightController} from '../src/xr/controllerInput';

const source = (handedness: 'left' | 'right', trigger = 0.3, grip = 0.6) => ({
  handedness,
  profiles: ['generic-trigger-squeeze'],
  gripSpace: {},
  gamepad: {buttons: [{value: trigger}, {value: grip}]},
}) as unknown as XRInputSource;

function questSource(options: {
  profile?: string;
  buttonCount?: number;
  a?: boolean;
  b?: boolean;
  aValue?: number;
  bValue?: number;
  buttonsMissing?: boolean;
} = {}): XRInputSource {
  const buttons = Array.from({length: options.buttonCount ?? 7}, () => ({value: 0, pressed: false}));
  if (buttons[0]) buttons[0] = {value: 0.25, pressed: false};
  if (buttons[1]) buttons[1] = {value: 0.6, pressed: false};
  if (buttons[4]) buttons[4] = {value: options.aValue ?? 0, pressed: options.a ?? false};
  if (buttons[5]) buttons[5] = {value: options.bValue ?? 0, pressed: options.b ?? false};
  return {
    handedness: 'right',
    profiles: [options.profile ?? 'meta-quest-touch-plus'],
    gripSpace: {},
    gamepad: options.buttonsMissing ? {} : {buttons},
  } as unknown as XRInputSource;
}

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

it('returns null when no controller is present', () => {
  expect(readRightController(frame(), {} as XRReferenceSpace, [])).toBeNull();
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
    armButton: false,
    stopButton: false,
    questFaceButtonsSupported: false,
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

it('reads Quest right-controller A and B from indices 4 and 5', () => {
  const aPressed = readRightController(
    frame(),
    {} as XRReferenceSpace,
    [questSource({a: true})],
  );
  const bByValue = readRightController(
    frame(),
    {} as XRReferenceSpace,
    [questSource({profile: 'oculus-touch-v3', bValue: 0.5})],
  );

  expect(aPressed).toMatchObject({
    armButton: true,
    stopButton: false,
    questFaceButtonsSupported: true,
  });
  expect(bByValue).toMatchObject({
    armButton: false,
    stopButton: true,
    questFaceButtonsSupported: true,
  });
});

it.each([
  ['unknown profile', questSource({profile: 'generic-trigger-squeeze'})],
  ['short button array', questSource({buttonCount: 5, a: true})],
] as const)('keeps face actions disabled for %s', (_name, right) => {
  expect(readRightController(frame(), {} as XRReferenceSpace, [right])).toMatchObject({
    trackingValid: true,
    armButton: false,
    stopButton: false,
    questFaceButtonsSupported: false,
  });
});

it.each([
  ['absent', {}],
  ['null', {buttons: null}],
] as const)('keeps pose tracking valid with neutral input when gamepad buttons are %s', (_name, gamepad) => {
  const right = {
    handedness: 'right',
    profiles: ['meta-quest-touch-plus'],
    gripSpace: {},
    gamepad,
  } as unknown as XRInputSource;

  expect(() => readRightController(frame(), {} as XRReferenceSpace, [right])).not.toThrow();
  expect(readRightController(frame(), {} as XRReferenceSpace, [right])).toEqual({
    p: [1, 2, 3],
    q: [0, 0, 0, 1],
    grip: false,
    trigger: 0,
    trackingValid: true,
    armButton: false,
    stopButton: false,
    questFaceButtonsSupported: false,
  });
});
