import {describe, expect, it} from 'vitest';
import {createVRFrame} from '../src/scenes/simulationScene';

describe('desktop VR frame path', () => {
  it('uses the shared VRFrame shape with metre poses and xyzw quaternion order', () => {
    const frame = createVRFrame({
      sessionId: 'desktop-test',
      sequence: 7,
      nowMs: 123.5,
      trackingValid: true,
      position: [0.4, 0.2, -0.1],
      quaternion: [0, 0.5, 0, 0.866],
      grip: false,
      trigger: 0.25,
    });

    expect(frame).toMatchObject({
      v: 1,
      type: 'vr_frame',
      session_id: 'desktop-test',
      seq: 7,
      client_mono_ms: 123.5,
      tracking_valid: true,
      visibility: 'visible',
      right: {
        p: [0.4, 0.2, -0.1],
        q: [0, 0.5, 0, 0.866],
        grip: false,
        trigger: 0.25,
      },
    });
  });
});
