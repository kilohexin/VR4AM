import {describe, expect, it} from 'vitest';

import type {JointVector, RobotStateMessage} from '../src/protocol/messages';
import {RobotStateBuffer} from '../src/robot/robotState';

function state(time: number, joint: number, gripper: number): RobotStateMessage {
  return {
    v: 1,
    type: 'robot_state',
    server_mono_ns: time,
    ack_seq: 1,
    mode: 'ACTIVE',
    robot_state: 'MOVING',
    actual_tcp: {p: [0, 0, 0], q: [0, 0, 0, 1]},
    actual_q: [joint, joint, joint, joint, joint, joint],
    gripper,
    sample_age_ms: 1,
    fault: null,
  };
}

describe('RobotStateBuffer', () => {
  it('returns null when empty', () => {
    expect(new RobotStateBuffer().sample(0)).toBeNull();
  });

  it('returns and ages a single sample without extrapolation', () => {
    const buffer = new RobotStateBuffer();
    const only = state(1_000_000, 0.25, 0.4);
    buffer.push(only);

    expect(buffer.sample(50_000_000)).toEqual({state: only, stale: false});
    expect(buffer.sample(101_000_001)).toEqual({state: only, stale: true});
  });

  it('interpolates joint and gripper values between the latest samples', () => {
    const buffer = new RobotStateBuffer();
    buffer.push(state(0, 0, 0));
    buffer.push(state(50_000_000, 1, 1));

    const sample = buffer.sample(25_000_000);

    expect(sample?.state.actual_q[0]).toBeCloseTo(0.5);
    expect(sample?.state.gripper).toBeCloseTo(0.5);
    expect(sample?.stale).toBe(false);
  });

  it('keeps metadata and actual TCP from the newest authoritative state', () => {
    const buffer = new RobotStateBuffer();
    const first = state(0, 0, 0);
    const newest: RobotStateMessage = {
      ...state(50_000_000, 1, 1),
      ack_seq: 9,
      actual_tcp: {p: [1, 2, 3], q: [0, 0, 1, 0]},
    };
    buffer.push(first);
    buffer.push(newest);

    const sample = buffer.sample(25_000_000);

    expect(sample?.state.ack_seq).toBe(9);
    expect(sample?.state.actual_tcp).toBe(newest.actual_tcp);
  });

  it('ignores duplicate and out-of-order states', () => {
    const buffer = new RobotStateBuffer();
    buffer.push(state(100, 1, 0.1));
    buffer.push(state(200, 2, 0.2));
    buffer.push(state(200, 99, 0.9));
    buffer.push(state(150, 88, 0.8));

    const sample = buffer.sample(200);

    expect(sample?.state.actual_q[0]).toBe(2);
    expect(sample?.state.gripper).toBe(0.2);
  });

  it('keeps only the latest two monotonic states', () => {
    const buffer = new RobotStateBuffer();
    buffer.push(state(0, 0, 0));
    buffer.push(state(10, 1, 0.1));
    buffer.push(state(20, 2, 0.2));

    expect(buffer.sample(10)?.state.actual_q[0]).toBe(1);
  });

  it('is fresh at exactly 100ms and stale immediately after it', () => {
    const buffer = new RobotStateBuffer();
    buffer.push(state(0, 0, 0));
    buffer.push(state(50_000_000, 1, 1));

    expect(buffer.sample(150_000_000)?.stale).toBe(false);
    const stale = buffer.sample(150_000_001);
    expect(stale?.stale).toBe(true);
    expect(stale?.state.actual_q[0]).toBe(1);
    expect(stale?.state.gripper).toBe(1);
  });

  it('clamps interpolated gripper values to the normalized range', () => {
    const buffer = new RobotStateBuffer();
    buffer.push(state(0, 0, -1));
    buffer.push(state(10, 1, 3));

    expect(buffer.sample(0)?.state.gripper).toBe(0);
    expect(buffer.sample(10)?.state.gripper).toBe(1);
  });

  it('returns a six-value joint tuple after interpolation', () => {
    const buffer = new RobotStateBuffer();
    buffer.push(state(0, 0, 0));
    buffer.push(state(10, 1, 1));

    const q: JointVector | undefined = buffer.sample(5)?.state.actual_q;
    expect(q).toHaveLength(6);
  });
});
