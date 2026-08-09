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

    expect(update.position).toEqual([anchor.p[0] + 0.002, anchor.p[1], anchor.p[2]]);
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

  it('does not accumulate translation error into an overshooting command when actual TCP lags', () => {
    const target = translationTarget(anchor, 'x', 1, 0.020);
    let sample: OfflineControllerSample = {
      ...neutralSample,
      position: [...anchor.p],
    };

    for (let index = 0; index < 20; index += 1) {
      sample = nextControllerSample({
        sample,
        actualTcp: anchor,
        targetTcp: target,
        maxPositionStepM: 0.002,
        maxRotationStepRad: Math.PI / 180,
      });
      expect(sample.position[0]).toBeLessThanOrEqual(anchor.p[0] + 0.020);
    }

    expect(sample.position).toEqual([anchor.p[0] + 0.002, anchor.p[1], anchor.p[2]]);
  });

  it('does not accumulate rotation error into an overshooting command when actual TCP lags', () => {
    const target = rotationTarget(anchor, 'z', 1, 8 * Math.PI / 180);
    let sample: OfflineControllerSample = {
      ...neutralSample,
      position: [...anchor.p],
      quaternion: [...anchor.q],
    };

    for (let index = 0; index < 20; index += 1) {
      sample = nextControllerSample({
        sample,
        actualTcp: anchor,
        targetTcp: target,
        maxPositionStepM: 0.002,
        maxRotationStepRad: Math.PI / 180,
      });
      expect(quaternionAngularError(anchor.q, sample.quaternion))
        .toBeLessThanOrEqual(8 * Math.PI / 180 + 1e-12);
    }

    expect(quaternionAngularError(anchor.q, sample.quaternion)).toBeCloseTo(Math.PI / 180);
  });

  it('keeps commands bounded and converges as lagging authoritative translation catches up', () => {
    const target = translationTarget(anchor, 'x', 1, 0.020);
    let actual = {p: [...anchor.p], q: [...anchor.q]} as typeof target;
    let sample: OfflineControllerSample = {
      ...neutralSample,
      position: [...anchor.p],
    };

    for (let index = 0; index < 20; index += 1) {
      sample = nextControllerSample({
        sample,
        actualTcp: actual,
        targetTcp: target,
        maxPositionStepM: 0.002,
        maxRotationStepRad: Math.PI / 180,
      });
      expect(sample.position[0]).toBeGreaterThanOrEqual(actual.p[0]);
      expect(sample.position[0]).toBeLessThanOrEqual(target.p[0]);
      actual = {
        p: [Math.min(target.p[0], actual.p[0] + 0.001), actual.p[1], actual.p[2]],
        q: [...actual.q],
      };
    }

    expect(sample.position).toEqual([...target.p]);
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
