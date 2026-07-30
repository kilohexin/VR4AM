import type {Quat, Vec3} from '../protocol/messages';

const GRIP_BUTTON_INDEX = 1;
const TRIGGER_BUTTON_INDEX = 0;
const ARM_BUTTON_INDEX = 4;
const STOP_BUTTON_INDEX = 5;
const THUMBSTICK_BUTTON_INDEX = 3;
const THUMBSTICK_X_AXIS_INDEX = 2;
const THUMBSTICK_Y_AXIS_INDEX = 3;
const GRIP_THRESHOLD = 0.5;
const QUEST_PROFILE_PREFIXES = ['meta-quest-touch', 'oculus-touch'] as const;

export interface TrackedPoseSample {
  p: Vec3;
  q: Quat;
  trackingValid: boolean;
}

export interface LeftControllerSample extends TrackedPoseSample {
  thumbstickX: number;
  thumbstickY: number;
  grip: boolean;
  thumbstickPressed: boolean;
}

export interface ControllerSample extends TrackedPoseSample {
  grip: boolean;
  trigger: number;
  armButton: boolean;
  stopButton: boolean;
  questFaceButtonsSupported: boolean;
}

export interface ControllerPairSample {
  left: LeftControllerSample;
  right: ControllerSample;
}

export function readControllers(
  frame: XRFrame,
  referenceSpace: XRReferenceSpace,
  inputSources: Iterable<XRInputSource>,
): ControllerPairSample {
  const sources = Array.from(inputSources);
  const leftSource = sources.find(({handedness}) => handedness === 'left');
  const rightSource = sources.find(({handedness}) => handedness === 'right');
  return {
    left: readLeftController(frame, referenceSpace, leftSource),
    right: readRightControllerSource(frame, referenceSpace, rightSource),
  };
}

export function readRightController(
  frame: XRFrame,
  referenceSpace: XRReferenceSpace,
  inputSources: Iterable<XRInputSource>,
): ControllerSample | null {
  const source = Array.from(inputSources).find(({handedness}) => handedness === 'right');
  if (!source) return null;

  return readRightControllerSource(frame, referenceSpace, source);
}

function readLeftController(
  frame: XRFrame,
  referenceSpace: XRReferenceSpace,
  source: XRInputSource | undefined,
): LeftControllerSample {
  const pose = readPose(frame, referenceSpace, source);
  if (!pose.trackingValid || !source) return invalidLeftControllerSample();

  return {
    ...pose,
    thumbstickX: normalizeAxis(source.gamepad?.axes?.[THUMBSTICK_X_AXIS_INDEX]),
    thumbstickY: normalizeAxis(source.gamepad?.axes?.[THUMBSTICK_Y_AXIS_INDEX]),
    grip: pressed(source.gamepad?.buttons?.[GRIP_BUTTON_INDEX]),
    thumbstickPressed: pressed(source.gamepad?.buttons?.[THUMBSTICK_BUTTON_INDEX]),
  };
}

function readRightControllerSource(
  frame: XRFrame,
  referenceSpace: XRReferenceSpace,
  source: XRInputSource | undefined,
): ControllerSample {
  const pose = readPose(frame, referenceSpace, source);
  if (!pose.trackingValid || !source) return invalidControllerSample();

  const buttons = source.gamepad?.buttons ?? [];
  const triggerButton = buttons[TRIGGER_BUTTON_INDEX];
  const gripButton = buttons[GRIP_BUTTON_INDEX];
  const questFaceButtonsSupported = supportsQuestFaceButtons(source, buttons);
  return {
    ...pose,
    grip: gripButton?.pressed === true || normalizeButton(gripButton?.value) >= GRIP_THRESHOLD,
    trigger: normalizeButton(triggerButton?.value),
    armButton: questFaceButtonsSupported && pressed(buttons[ARM_BUTTON_INDEX]),
    stopButton: questFaceButtonsSupported && pressed(buttons[STOP_BUTTON_INDEX]),
    questFaceButtonsSupported,
  };
}

function readPose(
  frame: XRFrame,
  referenceSpace: XRReferenceSpace,
  source: XRInputSource | undefined,
): TrackedPoseSample {
  if (!source?.gripSpace) return invalidTrackedPoseSample();

  const pose = frame.getPose(source.gripSpace, referenceSpace);
  if (!pose) return invalidTrackedPoseSample();

  const {position, orientation} = pose.transform;
  return {
    p: [position.x, position.y, position.z],
    q: [orientation.x, orientation.y, orientation.z, orientation.w],
    trackingValid: true,
  };
}

export function invalidControllerSample(): ControllerSample {
  return {
    p: [0, 0, 0],
    q: [0, 0, 0, 1],
    grip: false,
    trigger: 0,
    armButton: false,
    stopButton: false,
    questFaceButtonsSupported: false,
    trackingValid: false,
  };
}

function invalidLeftControllerSample(): LeftControllerSample {
  return {
    ...invalidTrackedPoseSample(),
    thumbstickX: 0,
    thumbstickY: 0,
    grip: false,
    thumbstickPressed: false,
  };
}

function invalidTrackedPoseSample(): TrackedPoseSample {
  return {
    p: [0, 0, 0],
    q: [0, 0, 0, 1],
    trackingValid: false,
  };
}

function pressed(button: GamepadButton | undefined): boolean {
  return button?.pressed === true || normalizeButton(button?.value) >= GRIP_THRESHOLD;
}

function supportsQuestFaceButtons(
  source: XRInputSource,
  buttons: readonly GamepadButton[],
): boolean {
  return buttons.length > STOP_BUTTON_INDEX && source.profiles.some((profile) =>
    QUEST_PROFILE_PREFIXES.some((prefix) => profile.startsWith(prefix)),
  );
}

function normalizeButton(value: number | undefined): number {
  if (value === undefined || !Number.isFinite(value)) return 0;
  return Math.min(1, Math.max(0, value));
}

function normalizeAxis(value: number | undefined): number {
  if (value === undefined || !Number.isFinite(value)) return 0;
  return Math.min(1, Math.max(-1, value));
}
