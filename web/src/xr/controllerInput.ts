import type {Quat, Vec3} from '../protocol/messages';

const GRIP_BUTTON_INDEX = 1;
const TRIGGER_BUTTON_INDEX = 0;
const GRIP_THRESHOLD = 0.5;

export interface ControllerSample {
  p: Vec3;
  q: Quat;
  grip: boolean;
  trigger: number;
  trackingValid: boolean;
}

export function readRightController(
  frame: XRFrame,
  referenceSpace: XRReferenceSpace,
  inputSources: Iterable<XRInputSource>,
): ControllerSample | null {
  const source = Array.from(inputSources).find(({handedness}) => handedness === 'right');
  if (!source) return null;
  if (!source.gripSpace) return invalidControllerSample();

  const pose = frame.getPose(source.gripSpace, referenceSpace);
  if (!pose) return invalidControllerSample();

  const {position, orientation} = pose.transform;
  const buttons = source.gamepad?.buttons;
  if (!buttons) return invalidControllerSample();
  const triggerButton = buttons[TRIGGER_BUTTON_INDEX];
  const gripButton = buttons[GRIP_BUTTON_INDEX];
  return {
    p: [position.x, position.y, position.z],
    q: [orientation.x, orientation.y, orientation.z, orientation.w],
    grip: gripButton?.pressed === true || normalizeButton(gripButton?.value) >= GRIP_THRESHOLD,
    trigger: normalizeButton(triggerButton?.value),
    trackingValid: true,
  };
}

export function invalidControllerSample(): ControllerSample {
  return {
    p: [0, 0, 0],
    q: [0, 0, 0, 1],
    grip: false,
    trigger: 0,
    trackingValid: false,
  };
}

function normalizeButton(value: number | undefined): number {
  if (value === undefined || !Number.isFinite(value)) return 0;
  return Math.min(1, Math.max(0, value));
}
