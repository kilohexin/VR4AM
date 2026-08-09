import type {VRFrame} from './protocol/messages';

export type TeleopFrameSource = 'manual' | 'xr' | 'offline_rehearsal';

export function forwardTeleopFrame(
  frame: VRFrame,
  source: TeleopFrameSource,
  rehearsalActive: boolean,
  sink: (frame: VRFrame, source: TeleopFrameSource) => void,
): boolean {
  if (rehearsalActive && source !== 'offline_rehearsal') return false;
  sink(frame, source);
  return true;
}
