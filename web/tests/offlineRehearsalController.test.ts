import {describe, expect, it} from 'vitest';

import type {
  ArmFeedbackMessage,
  ClientControlMessage,
  DiagnosticsMessage,
  FaultResetResultMessage,
  HomeResultMessage,
  OfflineRehearsalClientMessage,
  OfflineRehearsalFeedbackMessage,
  Pose,
  RobotStateMessage,
} from '../src/protocol/messages';
import {
  OfflineRehearsalController,
  type OfflineRehearsalPorts,
} from '../src/rehearsal/offlineRehearsalController';
import {
  REHEARSAL_PHASES,
  type OfflineControllerSample,
  type OfflineSceneSnapshot,
} from '../src/rehearsal/types';

const ANCHOR: Pose = {p: [0.30, 0.20, -0.20], q: [0, 0, 0, 1]};

function robotState(overrides: Partial<RobotStateMessage> = {}): RobotStateMessage {
  return {
    v: 1,
    type: 'robot_state',
    server_mono_ns: 1_000_000,
    mode: 'READY',
    robot_state: 'IDLE',
    actual_tcp: structuredClone(ANCHOR),
    actual_q: [0, 0, 0, 0, 0, 0],
    gripper: 0,
    fault: null,
    constraint: null,
    recovery_phase: null,
    backend: 'LEBAI_FAKE',
    // RealLebaiAdapter.get_state(), as used by create_digital_twin_app(),
    // leaves the health-only fields null on the WebSocket state contract.
    real_robot_mode: null,
    preflight_ready: null,
    preflight_reason: null,
    ...overrides,
  };
}

function diagnostics(overrides: Partial<DiagnosticsMessage> = {}): DiagnosticsMessage {
  return {
    v: 1,
    type: 'diagnostics',
    server_mono_ns: 1_000_000,
    runtime: 'LEBAI_FAKE',
    hardware_verified: false,
    control_generation: 1,
    actual_qd: [0, 0, 0, 0, 0, 0],
    actual_qdd: [0, 0, 0, 0, 0, 0],
    target_q: null,
    target_qd: null,
    target_qdd: null,
    target_tcp: null,
    sdk_latencies_ms: {},
    pvat_send_hz: 25,
    log_session_dir: null,
    dropped_events: 0,
    recent_events: [],
    ...overrides,
  };
}

function harness(options: {throwOnBegin?: boolean} = {}) {
  let nowMs = 1000;
  let serverMonoNs = 1_000_000;
  let ackSeq = 0;
  const controls: ClientControlMessage[] = [];
  const reports: OfflineRehearsalClientMessage[] = [];
  const samples: Array<OfflineControllerSample | null> = [];
  const scene: OfflineSceneSnapshot = {
    controller: null,
    carriedBlockId: null,
    blocks: [{id: 'block-orange', position: [0.18, 0.030, -0.32], sizeM: 0.06}],
    invalidOverlap: false,
  };
  let resets = 0;
  let connectionAborts = 0;
  const serverRun = {active: false};
  const ports: OfflineRehearsalPorts = {
    nowMs: () => nowMs,
    sendControl: (message) => controls.push(structuredClone(message)),
    sendRehearsal: (message) => {
      if (options.throwOnBegin && message.type === 'offline_rehearsal_begin') {
        throw new Error('transport send failed');
      }
      reports.push(structuredClone(message));
    },
    setOfflineController: (sample) => {
      const copy = sample === null ? null : structuredClone(sample);
      samples.push(copy);
      scene.controller = copy;
    },
    readScene: () => structuredClone(scene),
    resetScene: () => { resets += 1; },
    closeConnection: () => {
      connectionAborts += 1;
      serverRun.active = false;
    },
  };
  const controller = new OfflineRehearsalController(ports);
  return {
    controller,
    ports,
    advance: (milliseconds: number) => {
      nowMs += milliseconds;
      serverMonoNs += milliseconds * 1_000_000;
    },
    emitDiagnostics: (overrides: Partial<DiagnosticsMessage> = {}) => {
      controller.onDiagnostics(diagnostics({server_mono_ns: serverMonoNs, ...overrides}));
    },
    emitState: (
      overrides: Partial<RobotStateMessage> = {},
      options: {advanceTimestamp?: boolean; advanceAck?: boolean} = {},
    ): RobotStateMessage => {
      if (options.advanceTimestamp !== false) serverMonoNs += 1;
      if (options.advanceAck !== false) ackSeq += 1;
      const state = robotState({server_mono_ns: serverMonoNs, ack_seq: ackSeq, ...overrides});
      controller.onRobotState(state);
      return state;
    },
    controls,
    reports,
    samples,
    scene,
    resets: () => resets,
    connectionAborts: () => connectionAborts,
    serverRun,
  };
}

type Harness = ReturnType<typeof harness>;

function connectReady(test: Harness): void {
  test.controller.onConnection({state: 'connected'});
  test.emitDiagnostics();
  test.emitState();
}

function latestReport<T extends OfflineRehearsalClientMessage['type']>(
  test: Harness,
  type: T,
): Extract<OfflineRehearsalClientMessage, {type: T}> {
  const result = [...test.reports].reverse().find((message) => message.type === type);
  if (!result || result.type !== type) throw new Error(`missing ${type}`);
  return result as Extract<OfflineRehearsalClientMessage, {type: T}>;
}

function acceptBegin(test: Harness): void {
  const begin = latestReport(test, 'offline_rehearsal_begin');
  test.controller.onFeedback({
    v: 1,
    type: 'offline_rehearsal_begin_result',
    request_id: begin.request_id,
    accepted: true,
    run_id: 'run-1',
  });
  test.serverRun.active = true;
}

function acknowledgePhase(test: Harness): void {
  const phase = latestReport(test, 'offline_rehearsal_phase');
  test.controller.onFeedback({
    v: 1,
    type: 'offline_rehearsal_phase_ack',
    request_id: phase.request_id,
    accepted: true,
    run_id: phase.run_id,
    phase: phase.phase,
  });
}

function acknowledgeFinish(
  test: Harness,
  outcome?: 'passed' | 'failed' | 'aborted',
): void {
  const finish = latestReport(test, 'offline_rehearsal_finish');
  test.controller.onFeedback({
    v: 1,
    type: 'offline_rehearsal_finish_result',
    request_id: finish.request_id,
    accepted: true,
    run_id: finish.run_id,
    outcome: outcome ?? finish.outcome,
    json_path: 'artifacts/run-1.json',
    markdown_path: 'artifacts/run-1.md',
    hardware_verified: false,
    hardware_pending: [
      'sdk_connection', 'tcp_home_joint_limits', 'translation_direction',
      'rotation_direction', 'gripper_direction_force', 'pvat_tracking_latency',
      'stop_distance_estop', 'lightweight_grasp_release',
    ],
  });
  test.serverRun.active = false;
}

function acceptLatestControl(test: Harness): void {
  const control = test.controls.at(-1);
  if (!control) throw new Error('missing control');
  let feedback: ArmFeedbackMessage | HomeResultMessage | FaultResetResultMessage;
  if (control.type === 'arm_request') {
    feedback = {v: 1, type: 'arm_ack', request_id: control.request_id};
  } else if (control.type === 'home_request') {
    feedback = {
      v: 1,
      type: 'home_result',
      request_id: control.request_id,
      accepted: true,
      mode: 'DISARMED',
    };
  } else if (control.type === 'reset_fault') {
    feedback = {
      v: 1,
      type: 'fault_reset_result',
      request_id: control.request_id,
      accepted: true,
      mode: 'DISARMED',
    };
  } else {
    throw new Error(`cannot accept ${control.type}`);
  }
  test.controller.onFeedback(feedback);
}

function confirmState(
  test: Harness,
  overrides: Partial<RobotStateMessage>,
  count = 3,
): void {
  for (let index = 0; index < count; index += 1) {
    test.emitState(overrides);
  }
}

function beginThroughAnchor(test: Harness): void {
  connectReady(test);
  expect(test.controller.start()).toBe(true);
  acceptBegin(test);
  acknowledgePhase(test);
  expect(test.controls.at(-1)?.type).toBe('home_request');
  acceptLatestControl(test);
  confirmState(test, {mode: 'DISARMED'});
  acknowledgePhase(test);
  expect(test.controls.at(-1)?.type).toBe('arm_request');
  acceptLatestControl(test);
  expect(test.samples.at(-1)).toMatchObject({grip: true, trigger: 0, trackingValid: true});
  confirmState(test, {mode: 'ARMED', robot_state: 'IDLE'});
  expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('home');
  confirmState(test, {mode: 'ACTIVE', robot_state: 'MOVING'});
  expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('arm_and_anchor');
}

function driveCurrentMotionTarget(test: Harness): void {
  const target = test.controller.snapshot.targetTcp;
  if (target === null) throw new Error(`missing target in ${test.controller.snapshot.step}`);
  confirmState(test, {
    mode: 'ACTIVE',
    robot_state: 'MOVING',
    actual_tcp: structuredClone(target),
  });
}

function driveMotionPhase(test: Harness, phase: 'translate' | 'rotate'): string[] {
  let targets = 0;
  const steps: string[] = [];
  while (test.controller.snapshot.phase === phase) {
    const step = test.controller.snapshot.step;
    if (step === null) throw new Error(`missing ${phase} step`);
    steps.push(step);
    driveCurrentMotionTarget(test);
    targets += 1;
    if (targets > 12) throw new Error(`${phase} did not terminate`);
  }
  expect(targets).toBe(12);
  expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe(phase);
  return steps;
}

function driveGripper(test: Harness): void {
  let confirmations = 0;
  while (test.controller.snapshot.phase === 'gripper') {
    const trigger = test.controller.snapshot.targetTrigger;
    if (trigger === null) throw new Error('missing gripper target');
    expect(test.samples.at(-1)?.grip).toBe(true);
    confirmState(test, {
      mode: 'ACTIVE',
      robot_state: 'MOVING',
      gripper: trigger,
    });
    confirmations += 1;
    if (confirmations > 4) throw new Error('gripper did not terminate');
  }
  expect(confirmations).toBe(4);
}

function setCarriedBlockAtTcp(test: Harness, tcp: Pose, offset: readonly number[] = [0, -0.03, 0]): void {
  test.scene.carriedBlockId = 'block-orange';
  test.scene.blocks[0].position = [
    tcp.p[0] + offset[0],
    tcp.p[1] + offset[1],
    tcp.p[2] + offset[2],
  ];
}

function drivePickPlace(test: Harness): void {
  while (test.controller.snapshot.phase === 'pick_place') {
    const {step, targetTcp, targetTrigger, placementTarget} = test.controller.snapshot;
    if (step === 'pick_approach') {
      driveCurrentMotionTarget(test);
    } else if (step === 'pick_attach') {
      if (targetTcp === null || targetTrigger === null) throw new Error('missing attach target');
      setCarriedBlockAtTcp(test, targetTcp);
      confirmState(test, {
        mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: targetTcp, gripper: targetTrigger,
      }, 2);
      test.scene.carriedBlockId = null;
      confirmState(test, {
        mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: targetTcp, gripper: targetTrigger,
      }, 1);
      expect(test.controller.snapshot.step).toBe('pick_attach');
      setCarriedBlockAtTcp(test, targetTcp);
      confirmState(test, {
        mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: targetTcp, gripper: targetTrigger,
      });
    } else if (step === 'pick_lift' || step === 'pick_transfer' || step === 'pick_lower') {
      if (targetTcp === null) throw new Error('missing carried target');
      setCarriedBlockAtTcp(test, targetTcp);
      confirmState(test, {
        mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: targetTcp, gripper: 1,
      });
    } else if (step === 'pick_release') {
      if (targetTcp === null || placementTarget === null) throw new Error('missing release target');
      test.scene.carriedBlockId = null;
      test.scene.invalidOverlap = false;
      test.scene.blocks[0].position = structuredClone(placementTarget);
      confirmState(test, {
        mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: targetTcp, gripper: 0,
      });
    } else {
      throw new Error(`unknown pick step ${step}`);
    }
  }
}

function rejectLatestPhase(test: Harness): void {
  const phase = latestReport(test, 'offline_rehearsal_phase');
  const rejected: OfflineRehearsalFeedbackMessage = {
    v: 1,
    type: 'offline_rehearsal_phase_ack',
    request_id: phase.request_id,
    accepted: false,
    run_id: phase.run_id,
    phase: phase.phase,
    reason: 'store_rejected',
  };
  test.controller.onFeedback(rejected);
}

function expectFailed(test: Harness, reason: string): void {
  expect(test.samples.at(-1)).toBeNull();
  expect(test.controls.at(-1)?.type).toBe('disarm');
  expect(test.controller.snapshot.phase).toBe('failed');
  expect(test.controller.snapshot.failure).toBe(reason);
  expect(test.controller.snapshot.hardwareVerified).toBe(false);
}

function driveFromAnchorToFinish(test: Harness, recoverFault = false): void {
  acknowledgePhase(test);
  expect(driveMotionPhase(test, 'translate')).toEqual([
    '+x', '+x_return', '-x', '-x_return',
    '+y', '+y_return', '-y', '-y_return',
    '+z', '+z_return', '-z', '-z_return',
  ]);
  acknowledgePhase(test);
  expect(driveMotionPhase(test, 'rotate')).toEqual([
    '+roll', '+roll_return', '-roll', '-roll_return',
    '+pitch', '+pitch_return', '-pitch', '-pitch_return',
    '+yaw', '+yaw_return', '-yaw', '-yaw_return',
  ]);
  acknowledgePhase(test);
  driveGripper(test);
  acknowledgePhase(test);
  drivePickPlace(test);
  acknowledgePhase(test);

  expect(test.controller.snapshot.phase).toBe('soft_boundary');
  confirmState(test, {
    mode: 'ACTIVE', robot_state: 'HOLD', constraint: 'workspace_boundary',
  }, 2);
  confirmState(test, {
    mode: 'ACTIVE', robot_state: 'MOVING', constraint: null,
  }, 1);
  expect(test.controller.snapshot.step).toBe('boundary_outward');
  confirmState(test, {
    mode: 'ACTIVE', robot_state: 'HOLD', constraint: 'workspace_boundary',
  });
  expect(test.controller.snapshot.step).toBe('boundary_retreat');
  driveCurrentMotionTarget(test);
  acknowledgePhase(test);

  expect(test.controller.snapshot.phase).toBe('tracking_loss');
  expect(test.samples.at(-1)?.trackingValid).toBe(false);
  confirmState(test, {mode: 'STALE', robot_state: 'IDLE'});
  const staleAck = test.emitState({mode: 'STALE', robot_state: 'IDLE'}).ack_seq;
  acknowledgePhase(test);

  expect(test.controller.snapshot.phase).toBe('recovery_and_home');
  expect(test.samples.at(-1)).toMatchObject({trackingValid: true, grip: false, trigger: 0});
  expect(test.controls.at(-1)?.type).toBe('arm_request');
  test.emitState({mode: 'STALE', robot_state: 'IDLE', ack_seq: staleAck});
  expect(test.controls.at(-1)?.type).toBe('arm_request');
  test.emitState({
    mode: 'DISARMED', robot_state: 'IDLE',
    fault: recoverFault ? 'workspace_violation' : null,
  });
  expect(test.controls.at(-1)?.type).toBe(recoverFault ? 'reset_fault' : 'home_request');
  if (recoverFault) {
    test.emitState({mode: 'DISARMED', robot_state: 'IDLE', fault: null});
    expect(test.controls.at(-1)?.type).toBe('reset_fault');
    acceptLatestControl(test);
    expect(test.controls.at(-1)?.type).toBe('home_request');
  }
  acceptLatestControl(test);
  confirmState(test, {mode: 'DISARMED', robot_state: 'IDLE'});
  acknowledgePhase(test);

  expect(test.controller.snapshot.phase).toBe('final_stop');
  expect(test.controls.at(-1)?.type).toBe('disarm');
  confirmState(test, {mode: 'DISARMED', robot_state: 'IDLE'});
  acknowledgePhase(test);
  expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('finalize');
  acknowledgePhase(test);
  expect(latestReport(test, 'offline_rehearsal_finish').outcome).toBe('passed');
}

describe('OfflineRehearsalController happy path', () => {
  it('accepts the real create_digital_twin_app WebSocket identity shape', () => {
    const actualDigitalTwin = harness();
    connectReady(actualDigitalTwin);

    expect(actualDigitalTwin.controller.start()).toBe(true);
    expect(latestReport(actualDigitalTwin, 'offline_rehearsal_begin')).toBeDefined();

    for (const backend of [null, 'SIMULATOR', 'LEBAI'] as const) {
      const unsafe = harness();
      unsafe.controller.onConnection({state: 'connected'});
      unsafe.emitDiagnostics();
      unsafe.emitState({backend});
      expect(unsafe.controller.start()).toBe(false);
      expect(unsafe.controller.snapshot.failure).toBe('identity_preflight_failed');
    }

    const wrongDiagnostics = harness();
    wrongDiagnostics.controller.onConnection({state: 'connected'});
    wrongDiagnostics.emitDiagnostics({runtime: 'SIMULATOR'});
    wrongDiagnostics.emitState();
    expect(wrongDiagnostics.controller.start()).toBe(false);
  });

  it('runs the exact state-confirmed phase sequence and keeps lifecycle reports report-only', () => {
    const test = harness();
    beginThroughAnchor(test);
    expect(test.resets()).toBe(1);
    expect(test.controller.start()).toBe(false);

    driveFromAnchorToFinish(test);
    acknowledgeFinish(test);

    const phaseOrder = test.reports
      .filter((message) => message.type === 'offline_rehearsal_phase')
      .map((message) => message.phase);
    expect(phaseOrder).toEqual(REHEARSAL_PHASES);
    expect(test.controller.snapshot.phase).toBe('passed');
    expect(test.controller.snapshot.reportPaths).toEqual({
      json: 'artifacts/run-1.json', markdown: 'artifacts/run-1.md',
    });
    expect(test.samples.at(-1)).toBeNull();

    for (const report of test.reports) {
      expect(report).not.toHaveProperty('tracking_valid');
      expect(report).not.toHaveProperty('right');
      expect(report).not.toHaveProperty('target_tcp');
      expect(report).not.toHaveProperty('gripper');
    }
    expect(test.controls.map((message) => message.type)).toEqual([
      'home_request', 'arm_request', 'home_request', 'disarm',
    ]);
  });

  it('fails closed when accepted finish outcome contradicts the requested result', () => {
    const test = harness();
    beginThroughAnchor(test);
    driveFromAnchorToFinish(test);
    acknowledgeFinish(test, 'failed');

    expectFailed(test, 'finish_outcome_mismatch');
  });

  it('keeps safety cleanup active when a stale normal finish is accepted', () => {
    const test = harness();
    beginThroughAnchor(test);
    driveFromAnchorToFinish(test);

    test.advance(1_001);
    acknowledgeFinish(test);

    expect(test.controls.at(-1)?.type).toBe('disarm');
    expect(test.serverRun.active).toBe(false);
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'state_timeout', active: true, stopVerified: false,
    });

    confirmState(test, {mode: 'DISARMED', robot_state: 'IDLE'});
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'state_timeout', active: false, stopVerified: true,
    });
  });

  it('orders valid tracking, conditional fault reset, authoritative stop, then Home', () => {
    const test = harness();
    beginThroughAnchor(test);
    driveFromAnchorToFinish(test, true);
    acknowledgeFinish(test);

    expect(test.controls.map((message) => message.type)).toEqual([
      'home_request', 'arm_request', 'reset_fault', 'home_request', 'disarm',
    ]);
    expect(test.controller.snapshot.phase).toBe('passed');
  });

  it('ignores stale request IDs and requires three consecutive authoritative confirmations', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    const begin = latestReport(test, 'offline_rehearsal_begin');
    test.controller.onFeedback({
      v: 1, type: 'offline_rehearsal_begin_result', request_id: 'stale',
      accepted: true, run_id: 'wrong-run',
    });
    expect(test.reports).toHaveLength(1);
    acceptBegin(test);
    const identity = latestReport(test, 'offline_rehearsal_phase');
    test.controller.onFeedback({
      v: 1, type: 'offline_rehearsal_phase_ack', request_id: identity.request_id,
      accepted: true, run_id: 'wrong-run', phase: 'identity_preflight',
    });
    expect(test.controls).toHaveLength(0);
    acknowledgePhase(test);
    acceptLatestControl(test);
    confirmState(test, {mode: 'DISARMED'}, 2);
    confirmState(test, {mode: 'ACTIVE', robot_state: 'MOVING'}, 1);
    confirmState(test, {mode: 'DISARMED'}, 2);
    expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('identity_preflight');
    confirmState(test, {mode: 'DISARMED'}, 1);
    expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('home');
    expect(begin.request_id).not.toBe(identity.request_id);
  });

  it('does not count replayed or out-of-order robot-state timestamps as confirmations', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    acknowledgePhase(test);
    acceptLatestControl(test);

    const first = test.emitState({mode: 'DISARMED'});
    test.controller.onRobotState(first);
    test.controller.onRobotState({...first, server_mono_ns: first.server_mono_ns - 1});
    expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('identity_preflight');
    confirmState(test, {mode: 'DISARMED'}, 2);
    expect(latestReport(test, 'offline_rehearsal_phase').phase).toBe('home');
  });

  it('renews the 8 second deadline only after each motion target has three advancing confirmations', () => {
    const test = harness();
    beginThroughAnchor(test);
    acknowledgePhase(test);
    expect(test.controller.snapshot).toMatchObject({phase: 'translate', step: '+x'});

    const target = test.controller.snapshot.targetTcp;
    if (target === null) throw new Error('missing first translation target');
    test.advance(500);
    test.emitDiagnostics();
    const first = test.emitState({mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: target});
    expect(test.controller.snapshot.remainingTimeoutMs).toBe(7_500);
    expect(test.samples.at(-1)?.grip).toBe(true);

    test.advance(500);
    test.emitDiagnostics();
    test.emitState({mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: target});
    test.controller.onRobotState(first);
    expect(test.controller.snapshot).toMatchObject({step: '+x', remainingTimeoutMs: 7_000});

    test.advance(500);
    test.emitDiagnostics();
    test.emitState({mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: target});
    expect(test.controller.snapshot).toMatchObject({
      phase: 'translate', step: '+x_return', remainingTimeoutMs: 8_000,
    });
    expect(test.samples.at(-1)?.grip).toBe(true);
  });
});

describe('OfflineRehearsalController fail-closed cleanup', () => {
  it('hard-stops a full rehearsal after 300 seconds through the verified cleanup path', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    acknowledgePhase(test);

    test.advance(300_001);
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'full_rehearsal_timeout', active: true, stopVerified: false,
    });
    expect(test.samples.at(-1)).toBeNull();
    expect(test.controls.at(-1)?.type).toBe('disarm');

    confirmState(test, {mode: 'DISARMED', robot_state: 'IDLE'});
    expect(test.controller.snapshot.stopVerified).toBe(true);
    expect(latestReport(test, 'offline_rehearsal_phase')).toMatchObject({
      phase: 'home', status: 'failed', failure: {reason: 'full_rehearsal_timeout'},
    });
  });

  it.each([
    ['connection_lost', (test: Harness) => test.controller.onConnection({state: 'disconnected'})],
    ['page_hidden', (test: Harness) => test.controller.requestStop('page_hidden')],
    ['controller_occupied', (test: Harness) => (
      test.controller.onConnection({state: 'occupied', message: 'busy'})
    )],
    ['robot_fault', (test: Harness) => (
      test.emitState({mode: 'FAULT', robot_state: 'FAULT', fault: 'fault'})
    )],
    ['unexpected_stale', (test: Harness) => (
      test.emitState({mode: 'STALE'})
    )],
    ['non_finite_state', (test: Harness) => (
      test.emitState({actual_tcp: {p: [NaN, 0, 0], q: [0, 0, 0, 1]}})
    )],
    ['phase_timeout', (test: Harness) => {
      for (let index = 0; index < 8; index += 1) {
        test.advance(900);
        test.emitState();
        test.emitDiagnostics();
      }
      test.advance(801);
      test.emitState();
    }],
  ])('fails closed for %s', (reason, trigger) => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    trigger(test);
    expectFailed(test, reason);
  });

  it('fails closed when a phase report or Home request is rejected', () => {
    const reportFailure = harness();
    connectReady(reportFailure);
    reportFailure.controller.start();
    acceptBegin(reportFailure);
    rejectLatestPhase(reportFailure);
    expectFailed(reportFailure, 'report_rejected');

    const homeFailure = harness();
    connectReady(homeFailure);
    homeFailure.controller.start();
    acceptBegin(homeFailure);
    acknowledgePhase(homeFailure);
    const home = homeFailure.controls.at(-1);
    if (!home) throw new Error('missing Home');
    homeFailure.controller.onFeedback({
      v: 1, type: 'home_result', request_id: home.request_id, accepted: false,
      reason: 'home_failed', message: 'failed',
    });
    expectFailed(homeFailure, 'home_rejected');
  });

  it('rejects invalid released placement after consecutive scene observations', () => {
    const test = harness();
    beginThroughAnchor(test);
    acknowledgePhase(test);
    driveMotionPhase(test, 'translate');
    acknowledgePhase(test);
    driveMotionPhase(test, 'rotate');
    acknowledgePhase(test);
    driveGripper(test);
    acknowledgePhase(test);

    while (test.controller.snapshot.step !== 'pick_release') {
      const {step, targetTcp} = test.controller.snapshot;
      if (targetTcp === null) throw new Error(`missing target for ${step}`);
      if (step !== 'pick_approach') setCarriedBlockAtTcp(test, targetTcp);
      confirmState(test, {
        mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: targetTcp,
        gripper: step === 'pick_approach' ? 0 : 1,
      });
    }
    const target = test.controller.snapshot.targetTcp;
    if (target === null) throw new Error('missing release target');
    test.scene.carriedBlockId = null;
    test.scene.invalidOverlap = true;
    test.scene.blocks[0].position = [0.8, 0.8, 0.8];
    confirmState(test, {
      mode: 'ACTIVE', robot_state: 'MOVING', actual_tcp: target, gripper: 0,
    });
    expectFailed(test, 'invalid_block_placement');
  });

  it('dispose during begin is terminal and late feedback cannot resume the run', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    const begin = latestReport(test, 'offline_rehearsal_begin');
    test.controller.dispose();
    expectFailed(test, 'disposed');
    expect(test.connectionAborts()).toBe(1);
    expect(test.controller.snapshot.active).toBe(false);
    test.controller.onFeedback({
      v: 1, type: 'offline_rehearsal_begin_result', request_id: begin.request_id,
      accepted: true, run_id: 'late-run',
    });
    expect(test.reports).toHaveLength(1);
    test.advance(8_001);
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'disposed', active: false, stopVerified: false,
    });
    expect(test.controller.start()).toBe(false);
    expectFailed(test, 'disposed');
  });

  it('bounds pending-begin stop, begin rejection, and disconnect cleanup', () => {
    const pending = harness();
    connectReady(pending);
    pending.controller.start();
    pending.controller.requestStop('operator_stop');
    confirmState(pending, {mode: 'DISARMED'});
    pending.advance(8_001);
    expect(pending.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'operator_stop', active: false, stopVerified: true,
    });

    const rejected = harness();
    connectReady(rejected);
    rejected.controller.start();
    const begin = latestReport(rejected, 'offline_rehearsal_begin');
    rejected.controller.onFeedback({
      v: 1, type: 'offline_rehearsal_begin_result', request_id: begin.request_id,
      accepted: false, reason: 'not_ready',
    });
    confirmState(rejected, {mode: 'DISARMED'});
    expect(rejected.controller.snapshot.active).toBe(false);

    const disconnected = harness();
    connectReady(disconnected);
    disconnected.controller.start();
    acceptBegin(disconnected);
    disconnected.controller.onConnection({state: 'disconnected'});
    expect(disconnected.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'connection_lost', active: false,
    });

    const disconnectedDuringStop = harness();
    connectReady(disconnectedDuringStop);
    disconnectedDuringStop.controller.start();
    acceptBegin(disconnectedDuringStop);
    disconnectedDuringStop.controller.requestStop('operator_stop');
    disconnectedDuringStop.controller.onConnection({state: 'disconnected'});
    expect(disconnectedDuringStop.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'operator_stop', active: false,
    });

  });

  it('contains begin send failures in a terminal fail-closed state', () => {
    const test = harness({throwOnBegin: true});
    connectReady(test);

    expect(() => test.controller.start()).not.toThrow();
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'send_failed', active: false,
    });
    expect(test.samples.at(-1)).toBeNull();
    expect(test.controls.at(-1)?.type).toBe('disarm');
  });

  it('retains failure and marks an unverified stop when FAULT never clears', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    test.controller.requestStop('page_hidden');
    test.advance(8_001);
    test.emitState({mode: 'FAULT', robot_state: 'FAULT', fault: 'latched'});
    expectFailed(test, 'stop_unverified');
    expect(test.controller.snapshot.stopVerified).toBe(false);
  });

  it('drains a known backend run after the stop-confirmation deadline', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    acknowledgePhase(test);
    test.controller.requestStop('page_hidden');

    test.advance(8_001);
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'stop_unverified', active: true, stopVerified: false,
    });
    expect(test.serverRun.active).toBe(true);

    for (const expectedPhase of REHEARSAL_PHASES.slice(1)) {
      const phase = latestReport(test, 'offline_rehearsal_phase');
      expect(phase).toMatchObject({phase: expectedPhase, status: 'failed'});
      expect(phase.measurements).toMatchObject({stop_verified: false});
      acknowledgePhase(test);
    }
    expect(latestReport(test, 'offline_rehearsal_finish').outcome).toBe('aborted');
    expect(test.serverRun.active).toBe(true);

    acknowledgeFinish(test);
    expect(test.serverRun.active).toBe(false);
    expect(test.controller.snapshot).toMatchObject({
      phase: 'failed', failure: 'stop_unverified', active: false, stopVerified: false,
    });
  });

  it('retries an exact cleanup report then closes the owning socket at the bound', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    acknowledgePhase(test);
    test.controller.requestStop('operator_stop');
    confirmState(test, {mode: 'DISARMED', robot_state: 'IDLE'});

    const first = latestReport(test, 'offline_rehearsal_phase');
    expect(first).toMatchObject({phase: 'home', status: 'failed'});
    for (let expectedAttempts = 2; expectedAttempts <= 3; expectedAttempts += 1) {
      test.advance(8_001);
      expect(test.controller.snapshot.active).toBe(true);
      const attempts = test.reports.filter((message) => (
        message.type === 'offline_rehearsal_phase' && message.phase === 'home'
      ));
      expect(attempts).toHaveLength(expectedAttempts);
      expect(attempts.at(-1)).toEqual(first);
      expect(test.connectionAborts()).toBe(0);
      expect(test.serverRun.active).toBe(true);
    }

    test.advance(8_001);
    expect(test.controller.snapshot.active).toBe(false);
    expect(test.connectionAborts()).toBe(1);
    expect(test.serverRun.active).toBe(false);
  });

  it('submits every remaining failed phase before an aborted finish becomes terminal', () => {
    const test = harness();
    connectReady(test);
    test.controller.start();
    acceptBegin(test);
    test.controller.requestStop('page_hidden');
    confirmState(test, {mode: 'DISARMED', robot_state: 'IDLE'});

    expect(latestReport(test, 'offline_rehearsal_phase')).toMatchObject({
      phase: 'identity_preflight', status: 'passed',
    });
    acknowledgePhase(test);
    for (const expectedPhase of REHEARSAL_PHASES.slice(1)) {
      const phase = latestReport(test, 'offline_rehearsal_phase');
      expect(phase.phase).toBe(expectedPhase);
      expect(phase.status).toBe('failed');
      acknowledgePhase(test);
    }
    expect(latestReport(test, 'offline_rehearsal_finish').outcome).toBe('aborted');
    acknowledgeFinish(test);

    expect(test.controller.snapshot.phase).toBe('failed');
    expect(test.controller.snapshot.failure).toBe('page_hidden');
    expect(test.controller.snapshot.active).toBe(false);
    expect(test.controller.snapshot.stopVerified).toBe(true);
  });

  it('requires fresh correlated Fake diagnostics and state identity throughout the run', () => {
    const stale = harness();
    connectReady(stale);
    stale.advance(1_001);
    expect(stale.controller.start()).toBe(false);
    expect(stale.controller.snapshot.failure).toBe('identity_preflight_failed');

    const uncorrelated = harness();
    uncorrelated.controller.onConnection({state: 'connected'});
    uncorrelated.emitDiagnostics({server_mono_ns: 0});
    uncorrelated.emitState({server_mono_ns: 2_000_000_000});
    expect(uncorrelated.controller.start()).toBe(false);

    const silence = harness();
    connectReady(silence);
    silence.controller.start();
    acceptBegin(silence);
    silence.advance(1_001);
    silence.emitState();
    expectFailed(silence, 'diagnostics_timeout');

    const drift = harness();
    connectReady(drift);
    drift.controller.start();
    acceptBegin(drift);
    drift.emitState({backend: 'LEBAI'});
    expectFailed(drift, 'identity_lost');
  });
});
