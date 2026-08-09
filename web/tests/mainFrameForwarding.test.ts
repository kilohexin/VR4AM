import {describe, expect, it, vi} from 'vitest';

import type {VRFrame} from '../src/protocol/messages';
import {
  forwardTeleopFrame,
  type TeleopFrameSource,
} from '../src/appFrameForwarding';

const frame = {seq: 17} as VRFrame;

describe('main teleop frame forwarding', () => {
  it('blocks manual and XR frames during rehearsal but forwards rehearsal frames through one sink', () => {
    const sent: Array<{frame: VRFrame; source: TeleopFrameSource}> = [];
    const sink = vi.fn((next: VRFrame, source: TeleopFrameSource) => sent.push({frame: next, source}));

    expect(forwardTeleopFrame(frame, 'manual', true, sink)).toBe(false);
    expect(forwardTeleopFrame(frame, 'xr', true, sink)).toBe(false);
    expect(forwardTeleopFrame(frame, 'offline_rehearsal', true, sink)).toBe(true);

    expect(sink).toHaveBeenCalledTimes(1);
    expect(sent).toEqual([{frame, source: 'offline_rehearsal'}]);
  });

  it('forwards ordinary manual and XR frames when rehearsal is inactive', () => {
    const sink = vi.fn();

    expect(forwardTeleopFrame(frame, 'manual', false, sink)).toBe(true);
    expect(forwardTeleopFrame(frame, 'xr', false, sink)).toBe(true);

    expect(sink).toHaveBeenCalledTimes(2);
  });
});
