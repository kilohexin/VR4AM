import {describe, expect, it} from 'vitest';
import validDiagnostics from '../../schemas/fixtures/diagnostics-valid.json';
import validOfflineRehearsalFinish from '../../schemas/fixtures/offline-rehearsal-finish-valid.json';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import vrFixture from '../../schemas/fixtures/vr-frame-valid.json';
import {
  isClientControlMessage,
  isConnectionRejectedMessage,
  isDiagnosticsMessage,
  isFaultResetResultMessage,
  isHomeResultMessage,
  isOfflineRehearsalFeedbackMessage,
  isRobotStateMessage,
  isSimulationScaleResultMessage,
  isVRFrame,
} from '../src/protocol/messages';

describe('protocol guards', () => {
  it('accepts only exact correlated offline rehearsal feedback variants', () => {
    const beginAccepted = {
      v: 1,
      type: 'offline_rehearsal_begin_result',
      request_id: 'begin-1',
      accepted: true,
      run_id: 'run-1',
    };
    const beginRejected = {
      v: 1,
      type: 'offline_rehearsal_begin_result',
      request_id: 'begin-2',
      accepted: false,
      reason: 'not_fake_runtime',
    };
    const phaseAccepted = {
      v: 1,
      type: 'offline_rehearsal_phase_ack',
      request_id: 'phase-1',
      accepted: true,
      run_id: 'run-1',
      phase: 'identity_preflight',
    };
    const phaseRejected = {
      v: 1,
      type: 'offline_rehearsal_phase_ack',
      request_id: 'phase-2',
      accepted: false,
      run_id: 'run-1',
      phase: 'identity_preflight',
      reason: 'phase_out_of_order',
    };
    const finishRejected = {
      v: 1,
      type: 'offline_rehearsal_finish_result',
      request_id: 'finish-2',
      accepted: false,
      run_id: 'run-1',
      reason: 'run_incomplete',
    };

    expect(isOfflineRehearsalFeedbackMessage(beginAccepted)).toBe(true);
    expect(isOfflineRehearsalFeedbackMessage(beginRejected)).toBe(true);
    expect(isOfflineRehearsalFeedbackMessage(phaseAccepted)).toBe(true);
    expect(isOfflineRehearsalFeedbackMessage(phaseRejected)).toBe(true);
    expect(isOfflineRehearsalFeedbackMessage(validOfflineRehearsalFinish)).toBe(true);
    expect(isOfflineRehearsalFeedbackMessage(finishRejected)).toBe(true);

    expect(isOfflineRehearsalFeedbackMessage({...beginAccepted, extra: true})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...beginRejected, request_id: ''})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...phaseAccepted, phase: 'move'})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...phaseRejected, extra: true})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...validOfflineRehearsalFinish, hardware_verified: true})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...validOfflineRehearsalFinish, extra: true})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...validOfflineRehearsalFinish, hardware_pending: []})).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({
      ...validOfflineRehearsalFinish,
      hardware_pending: [...validOfflineRehearsalFinish.hardware_pending.slice(0, 7), ''],
    })).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({
      ...validOfflineRehearsalFinish,
      json_path: '',
    })).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({
      ...validOfflineRehearsalFinish,
      markdown_path: undefined,
    })).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({
      ...validOfflineRehearsalFinish,
      hardware_pending: [
        'sdk_connection',
        'tcp_home_joint_limits',
        'translation_direction',
        'rotation_direction',
        'gripper_direction_force',
        'pvat_tracking_latency',
        'stop_distance_estop',
        'unexpected_check',
      ],
    })).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({
      ...validOfflineRehearsalFinish,
      hardware_pending: [
        'sdk_connection',
        'sdk_connection',
        'translation_direction',
        'rotation_direction',
        'gripper_direction_force',
        'pvat_tracking_latency',
        'stop_distance_estop',
        'lightweight_grasp_release',
      ],
    })).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({
      ...validOfflineRehearsalFinish,
      hardware_pending: [
        'tcp_home_joint_limits',
        'sdk_connection',
        'translation_direction',
        'rotation_direction',
        'gripper_direction_force',
        'pvat_tracking_latency',
        'stop_distance_estop',
        'lightweight_grasp_release',
      ],
    })).toBe(false);
    expect(isOfflineRehearsalFeedbackMessage({...finishRejected, run_id: ''})).toBe(false);
  });

  it('accepts only strict diagnostics messages with unverified hardware', () => {
    expect(isDiagnosticsMessage(validDiagnostics)).toBe(true);
    expect(isDiagnosticsMessage({...validDiagnostics, hardware_verified: true})).toBe(false);
    expect(isDiagnosticsMessage({...validDiagnostics, runtime: 'LEBAI_MOCK'})).toBe(false);
    expect(isDiagnosticsMessage({
      ...validDiagnostics,
      sdk_latencies_ms: {get_kin_data: Number.NaN},
    })).toBe(false);
  });

  it('accepts only the exact controller-occupied rejection contract', () => {
    const valid = {
      v: 1,
      type: 'connection_rejected',
      reason: 'controller_occupied',
      message: '已有控制页面占用，请关闭电脑端网页后重试。',
    };
    expect(isConnectionRejectedMessage(valid)).toBe(true);
    expect(isConnectionRejectedMessage({...valid, reason: 'busy'})).toBe(false);
    expect(isConnectionRejectedMessage({...valid, extra: true})).toBe(false);
  });

  it('accepts the shared protocol fixtures', () => {
    expect(isVRFrame(vrFixture)).toBe(true);
    expect(isRobotStateMessage(robotFixture)).toBe(true);
    expect(isRobotStateMessage({...robotFixture, constraint: 'self_collision'})).toBe(true);
    expect(isRobotStateMessage({...robotFixture, constraint: 'motion_continuity_boundary'})).toBe(true);
    expect(isRobotStateMessage({...robotFixture, backend: 'LEBAI_FAKE'})).toBe(true);
    expect(isRobotStateMessage({...robotFixture, translation_scale: 1.5})).toBe(true);
    expect(isRobotStateMessage({...robotFixture, translation_scale: 1.55})).toBe(false);
    expect(isRobotStateMessage({
      ...robotFixture,
      backend: 'LEBAI',
      real_robot_mode: 'readonly',
      preflight_ready: false,
      preflight_reason: 'commissioning_not_ready',
    })).toBe(true);
  });

  it('accepts only exact simulation scale results', () => {
    expect(isSimulationScaleResultMessage({
      v: 1,
      type: 'simulation_scale_result',
      request_id: 'scale-1',
      accepted: true,
      translation_scale: 1.5,
    })).toBe(true);
    expect(isSimulationScaleResultMessage({
      v: 1,
      type: 'simulation_scale_result',
      request_id: 'scale-2',
      accepted: false,
      translation_scale: 1.5,
      reason: 'not_stopped',
    })).toBe(true);
    expect(isSimulationScaleResultMessage({
      v: 1,
      type: 'simulation_scale_result',
      request_id: 'scale-3',
      accepted: true,
      translation_scale: 1.55,
    })).toBe(false);
  });

  it('accepts every valid control type and optional nullable fields', () => {
    for (const type of ['hello', 'arm_request', 'disarm', 'reset_fault', 'home_request', 'ping']) {
      expect(isClientControlMessage({v: 1, type, request_id: 'request'})).toBe(true);
      expect(
        isClientControlMessage({v: 1, type, request_id: 'request', client_mono_ms: null}),
      ).toBe(true);
    }
  });

  it('accepts only exact discriminated fault reset results', () => {
    const accepted = {
      v: 1,
      type: 'fault_reset_result',
      request_id: 'reset-1',
      accepted: true,
      mode: 'DISARMED',
    };
    const rejected = {
      v: 1,
      type: 'fault_reset_result',
      request_id: 'reset-2',
      accepted: false,
      reason: 'unrecoverable_fault',
      message: '该故障无法在线复位，请重启后端并重新检查。',
    };

    expect(isFaultResetResultMessage(accepted)).toBe(true);
    expect(isFaultResetResultMessage(rejected)).toBe(true);
    expect(isFaultResetResultMessage({...accepted, mode: 'READY'})).toBe(false);
    expect(isFaultResetResultMessage({...accepted, accepted: false})).toBe(false);
    expect(isFaultResetResultMessage({...rejected, accepted: true})).toBe(false);
    expect(isFaultResetResultMessage({...rejected, reason: 'unknown'})).toBe(false);
    expect(isFaultResetResultMessage({...rejected, extra: true})).toBe(false);
  });

  it('enforces request identifier code-point limits on fault reset results', () => {
    const accepted = {
      v: 1,
      type: 'fault_reset_result',
      accepted: true,
      mode: 'DISARMED',
    };
    expect(isFaultResetResultMessage({...accepted, request_id: '😀'.repeat(64)})).toBe(true);
    expect(isFaultResetResultMessage({...accepted, request_id: ''})).toBe(false);
    expect(isFaultResetResultMessage({...accepted, request_id: '😀'.repeat(65)})).toBe(false);
  });

  it('accepts only exact discriminated Home results', () => {
    const accepted = {
      v: 1,
      type: 'home_result',
      request_id: 'home-1',
      accepted: true,
      mode: 'DISARMED',
    };
    const rejected = {
      v: 1,
      type: 'home_result',
      request_id: 'home-2',
      accepted: false,
      reason: 'home_failed',
      message: '仿真无法返回初始姿态，请稍后重试。',
    };

    expect(isClientControlMessage({v: 1, type: 'home_request', request_id: 'home-1'})).toBe(true);
    expect(isHomeResultMessage(accepted)).toBe(true);
    expect(isHomeResultMessage(rejected)).toBe(true);
    expect(isHomeResultMessage({...accepted, mode: 'READY'})).toBe(false);
    expect(isHomeResultMessage({...accepted, accepted: false})).toBe(false);
    expect(isHomeResultMessage({...rejected, accepted: true})).toBe(false);
    expect(isHomeResultMessage({...rejected, reason: 'unknown'})).toBe(false);
    expect(isHomeResultMessage({...rejected, extra: true})).toBe(false);
  });

  it('enforces request identifier code-point limits on Home results', () => {
    const accepted = {
      v: 1,
      type: 'home_result',
      accepted: true,
      mode: 'DISARMED',
    };
    expect(isHomeResultMessage({...accepted, request_id: '😀'.repeat(64)})).toBe(true);
    expect(isHomeResultMessage({...accepted, request_id: ''})).toBe(false);
    expect(isHomeResultMessage({...accepted, request_id: '😀'.repeat(65)})).toBe(false);
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
    expect(isRobotStateMessage({...robotFixture, backend: 'LEBAI_MOCK'})).toBe(false);
  });

  it('rejects malformed scalar constraints and invalid quaternion norms', () => {
    expect(isVRFrame({...vrFixture, session_id: ''})).toBe(false);
    expect(isVRFrame({...vrFixture, seq: 1.5})).toBe(false);
    expect(isVRFrame({...vrFixture, right: {...vrFixture.right, q: [0, 0, 0, 2]}})).toBe(false);
    expect(isRobotStateMessage({...robotFixture, server_mono_ns: -1})).toBe(false);
    expect(isClientControlMessage({v: 1, type: 'hello', request_id: ''})).toBe(false);
  });

  it('counts session and request identifier limits by Unicode code points', () => {
    const fortyEmoji = '😀'.repeat(40);
    const sixtyFiveEmoji = '😀'.repeat(65);

    expect(isVRFrame({...vrFixture, session_id: fortyEmoji})).toBe(true);
    expect(isVRFrame({...vrFixture, session_id: sixtyFiveEmoji})).toBe(false);
    expect(isVRFrame({...vrFixture, session_id: ''})).toBe(false);

    expect(isClientControlMessage({v: 1, type: 'hello', request_id: fortyEmoji})).toBe(true);
    expect(isClientControlMessage({v: 1, type: 'hello', request_id: sixtyFiveEmoji})).toBe(false);
    expect(isClientControlMessage({v: 1, type: 'hello', request_id: ''})).toBe(false);
  });
});
