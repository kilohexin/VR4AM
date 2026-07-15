import type {Quat, Vec3} from '../protocol/messages';

const GRIP_BUTTON_INDEX = 1;
const TRIGGER_BUTTON_INDEX = 0;
const ARM_BUTTON_INDEX = 4;
const STOP_BUTTON_INDEX = 5;
const GRIP_THRESHOLD = 0.5;
const QUEST_PROFILE_PREFIXES = ['meta-quest-touch', 'oculus-touch'] as const;

export interface ControllerSample {
  p: Vec3;
  q: Quat;
  grip: boolean;
  trigger: number;
  armButton: boolean;
  stopButton: boolean;
  questFaceButtonsSupported: boolean;
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
  const buttons = source.gamepad?.buttons ?? [];
  const triggerButton = buttons[TRIGGER_BUTTON_INDEX];
  const gripButton = buttons[GRIP_BUTTON_INDEX];
  const questFaceButtonsSupported = supportsQuestFaceButtons(source, buttons);
  return {
    p: [position.x, position.y, position.z],
    q: [orientation.x, orientation.y, orientation.z, orientation.w],
    grip: gripButton?.pressed === true || normalizeButton(gripButton?.value) >= GRIP_THRESHOLD,
    trigger: normalizeButton(triggerButton?.value),
    armButton: questFaceButtonsSupported && pressed(buttons[ARM_BUTTON_INDEX]),
    stopButton: questFaceButtonsSupported && pressed(buttons[STOP_BUTTON_INDEX]),
    questFaceButtonsSupported,
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
