import type {VRFrame} from './protocol/messages';
import type {XRSessionStatus} from './xr/session';

/** Failed XR entry/teardown leaves no live input owner; authority is separate. */
export function desktopOwnsInput(state:XRSessionStatus['state']):boolean {
  return state==='idle'||state==='error';
}

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
