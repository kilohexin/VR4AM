import {describe, expect, it} from 'vitest';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import vrFixture from '../../schemas/fixtures/vr-frame-valid.json';
import {
  isClientControlMessage,
  isRobotStateMessage,
  isVRFrame,
} from '../src/protocol/messages';

describe('protocol guards', () => {
  it('accepts the shared protocol fixtures', () => {
    expect(isVRFrame(vrFixture)).toBe(true);
    expect(isRobotStateMessage(robotFixture)).toBe(true);
  });

  it('accepts every valid control type and optional nullable fields', () => {
    for (const type of ['hello', 'arm_request', 'disarm', 'ping']) {
      expect(isClientControlMessage({v: 1, type, request_id: 'request'})).toBe(true);
      expect(
        isClientControlMessage({v: 1, type, request_id: 'request', client_mono_ms: null}),
      ).toBe(true);
    }
  });

  it('rejects unknown versions and message types', () => {
    expect(isVRFrame({...vrFixture, v: 2})).toBe(false);
    expect(isVRFrame({...vrFixture, type: 'controller_frame'})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, v: 2})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, type: 'state'})).toBe(false);
    expect(isClientControlMessage({v: 1, type: 'resume', request_id: 'request'})).toBe(false);
  });

  it('rejects extra fields at every object level', () => {
    expect(isVRFrame({...vrFixture, extra: true})).toBe(false);
    expect(isVRFrame({...vrFixture, right: {...vrFixture.right, extra: true}})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, extra: true})).toBe(false);
    expect(
      isRobotStateMessage({
        ...robotFixture,
        actual_tcp: {...robotFixture.actual_tcp, extra: true},
      }),
    ).toBe(false);
    expect(isClientControlMessage({v: 1, type: 'hello', request_id: 'request', extra: true})).toBe(
      false,
    );
  });

  it('requires exact vector, quaternion, and joint tuple lengths', () => {
    expect(isVRFrame({...vrFixture, right: {...vrFixture.right, p: [0, 1]}})).toBe(false);
    expect(isVRFrame({...vrFixture, right: {...vrFixture.right, q: [0, 0, 0, 1, 0]}})).toBe(
      false,
    );
    expect(isRobotStateMessage({...robotFixture, actual_q: [0, 0, 0, 0, 0]})).toBe(false);
  });

  it('rejects non-finite numbers, including nested pose values', () => {
    expect(
      isVRFrame({...vrFixture, right: {...vrFixture.right, p: [0, Number.NaN, 0]}}),
    ).toBe(false);
    expect(isVRFrame({...vrFixture, client_mono_ms: Number.POSITIVE_INFINITY})).toBe(false);
    expect(
      isRobotStateMessage({
        ...robotFixture,
        actual_tcp: {...robotFixture.actual_tcp, q: [0, 0, Number.NEGATIVE_INFINITY, 1]},
      }),
    ).toBe(false);
    expect(
      isRobotStateMessage({...robotFixture, actual_q: [0, 0, 0, Number.NaN, 0, 0]}),
    ).toBe(false);
  });

  it('rejects out-of-range trigger and gripper values', () => {
    expect(isVRFrame({...vrFixture, right: {...vrFixture.right, trigger: 1.01}})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, gripper: -0.01})).toBe(false);
  });

  it('rejects unknown visibility, mode, and backend state enums', () => {
    expect(isVRFrame({...vrFixture, visibility: 'occluded'})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, mode: 'CONNECTED'})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, robot_state: 'STOPPED'})).toBe(false);
  });

  it('rejects malformed scalar constraints and invalid quaternion norms', () => {
    expect(isVRFrame({...vrFixture, session_id: ''})).toBe(false);
    expect(isVRFrame({...vrFixture, seq: 1.5})).toBe(false);
    expect(isVRFrame({...vrFixture, right: {...vrFixture.right, q: [0, 0, 0, 2]}})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, server_mono_ns: -1})).toBe(false);
    expect(isClientControlMessage({v: 1, type: 'hello', request_id: ''})).toBe(false);
  });
});
