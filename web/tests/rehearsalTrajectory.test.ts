import {describe, expect, it} from 'vitest';

import {
  nextControllerSample,
  quaternionAngularError,
  rotationTarget,
  translationTarget,
} from '../src/rehearsal/trajectory';
import type {OfflineControllerSample} from '../src/rehearsal/types';

const anchor = {p: [0.3, 0.4, -0.2], q: [0, 0, 0, 1]} as const;
const neutralSample: OfflineControllerSample = {
  position: [0.56, 0.42, 0.18],
  quaternion: [0, 0, 0, 1],
  grip: false,
  trigger: 0,
  trackingValid: true,
};

describe('offline rehearsal trajectory', () => {
  it('derives exact signed 20 mm targets along every world translation axis', () => {
    expect(translationTarget(anchor, 'x', 1, 0.020).p)
      .toEqual([anchor.p[0] + 0.020, anchor.p[1], anchor.p[2]]);
    expect(translationTarget(anchor, 'y', -1, 0.020).p)
      .toEqual([anchor.p[0], anchor.p[1] - 0.020, anchor.p[2]]);
    expect(translationTarget(anchor, 'z', 1, 0.020).p)
      .toEqual([anchor.p[0], anchor.p[1], anchor.p[2] + 0.020]);
  });

  it('derives exact signed 8 degree world-axis quaternion targets', () => {
    const eightDegrees = 8 * Math.PI / 180;
    const half = eightDegrees / 2;

    expect(rotationTarget(anchor, 'x', 1, eightDegrees).q).toEqual([
      Math.sin(half), 0, 0, Math.cos(half),
    ]);
    expect(rotationTarget(anchor, 'y', -1, eightDegrees).q).toEqual([
      0, -Math.sin(half), 0, Math.cos(half),
    ]);
    expect(rotationTarget(anchor, 'z', 1, eightDegrees).q).toEqual([
      0, 0, Math.sin(half), Math.cos(half),
    ]);
  });

  it('measures the shortest path for equivalent quaternion signs', () => {
    expect(quaternionAngularError([0, 0, 0, 1], [0, 0, 0, -1])).toBe(0);
    expect(quaternionAngularError([0, 0, 0, 1], [0, 1, 0, 0])).toBeCloseTo(Math.PI);
  });

  it('moves the synthetic controller by finite bounded position and rotation steps', () => {
    const target = translationTarget(anchor, 'x', 1, 0.020);
    const update = nextControllerSample({
      sample: neutralSample,
      actualTcp: anchor,
      targetTcp: target,
      maxPositionStepM: 0.002,
      maxRotationStepRad: Math.PI / 180,
    });

    expect(update.position).toEqual([0.562, 0.42, 0.18]);
    expect(update.quaternion).toEqual([0, 0, 0, 1]);
    expect(update).not.toBe(neutralSample);
    expect(update.position).not.toBe(neutralSample.position);
    expect(update.quaternion).not.toBe(neutralSample.quaternion);
  });

  it('takes a bounded shortest rotation step without changing the source sample', () => {
    const update = nextControllerSample({
      sample: neutralSample,
      actualTcp: anchor,
      targetTcp: rotationTarget(anchor, 'z', -1, 8 * Math.PI / 180),
      maxPositionStepM: 0.002,
      maxRotationStepRad: Math.PI / 180,
    });

    expect(quaternionAngularError(neutralSample.quaternion, update.quaternion))
      .toBeCloseTo(Math.PI / 180);
    expect(neutralSample.quaternion).toEqual([0, 0, 0, 1]);
  });

  it('rejects non-finite poses, non-unit quaternions, and invalid bounded steps', () => {
    expect(() => translationTarget({p: [NaN, 0, 0], q: [0, 0, 0, 1]}, 'x', 1, 0.02))
      .toThrow('invalid_tcp_pose');
    expect(() => rotationTarget(anchor, 'x', 1, Number.NaN)).toThrow('invalid_rotation');
    expect(() => quaternionAngularError([0, 0, 0, 0], [0, 0, 0, 1]))
      .toThrow('invalid_quaternion');
    expect(() => nextControllerSample({
      sample: {...neutralSample, quaternion: [0, 0, 0, 0]},
      actualTcp: anchor,
      targetTcp: anchor,
      maxPositionStepM: 0.002,
      maxRotationStepRad: Math.PI / 180,
    })).toThrow('invalid_controller_sample');
  });
});
