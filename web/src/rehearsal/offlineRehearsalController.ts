import {
  OFFLINE_REHEARSAL_HARDWARE_PENDING,
  PROTOCOL_VERSION,
  type ArmFeedbackMessage,
  type ClientControlMessage,
  type DiagnosticPayloadValue,
  type DiagnosticsMessage,
  type FaultResetResultMessage,
  type HomeResultMessage,
  type OfflineRehearsalClientMessage,
  type OfflineRehearsalFeedbackMessage,
  type Pose,
  type RobotStateMessage,
  type Vec3,
} from '../protocol/messages';
import type {TcpPose} from '../scenes/kinematicGraspController';
import type {TeleopConnectionStatus} from '../transport/teleopSocket';
import {
  nextControllerSample,
  quaternionAngularError,
  rotationTarget,
  translationTarget,
  type CartesianAxis,
} from './trajectory';
import {
  FAKE_REHEARSAL_PREP,
  REHEARSAL_CONFIG,
  REHEARSAL_PHASES,
  copyOfflineControllerSample,
  copyOfflineSceneSnapshot,
  type OfflineControllerSample,
  type OfflineSceneSnapshot,
  type RehearsalPhase,
} from './types';

export type OfflineRehearsalDisplayPhase =
  | 'idle'
  | RehearsalPhase
  | 'reporting'
  | 'failed'
  | 'passed';

export interface OfflineRehearsalSnapshot {
  phase: OfflineRehearsalDisplayPhase;
  currentPhase: RehearsalPhase | null;
  step: string | null;
  active: boolean;
  failure: string | null;
  runId: string | null;
  completedPhases: readonly RehearsalPhase[];
  targetTcp: Pose | null;
  targetTrigger: 0 | 1 | null;
  placementTarget: Vec3 | null;
  remainingTimeoutMs: number | null;
  stopVerified: boolean;
  reportPaths: {json: string; markdown: string} | null;
  hardwareVerified: false;
  hardwarePending: typeof OFFLINE_REHEARSAL_HARDWARE_PENDING;
}

export interface OfflineRehearsalPorts {
  nowMs(): number;
  sendControl(message: ClientControlMessage): void;
  sendRehearsal(message: OfflineRehearsalClientMessage): void;
  closeConnection(): void;
  setOfflineController(sample: OfflineControllerSample | null): void;
  readScene(): OfflineSceneSnapshot;
  resetScene?(): void;
  onSnapshot?(snapshot: OfflineRehearsalSnapshot): void;
}

type ControllerFeedback =
  | ArmFeedbackMessage
  | FaultResetResultMessage
  | HomeResultMessage
  | OfflineRehearsalFeedbackMessage;

type PhaseReportMessage = Extract<
  OfflineRehearsalClientMessage,
  {type: 'offline_rehearsal_phase'}
>;

type FinishReportMessage = Extract<
  OfflineRehearsalClientMessage,
  {type: 'offline_rehearsal_finish'}
>;

type PendingReport = Readonly<{
  kind: 'phase';
  requestId: string;
  phase: RehearsalPhase;
  cleanupGeneration: number | null;
  attempts: number;
  message: PhaseReportMessage;
}> | Readonly<{
  kind: 'finish';
  requestId: string;
  outcome: 'passed' | 'failed' | 'aborted';
  cleanupGeneration: number | null;
  attempts: number;
  message: FinishReportMessage;
}>;

type MotionTarget = Readonly<{
  step: string;
  target: TcpPose;
}>;

const SAFE_CONFIRMATIONS = REHEARSAL_CONFIG.completionSamples;
const GRIPPER_CLOSE_THRESHOLD = 0.65;
const GRIPPER_OPEN_THRESHOLD = 0.35;
const PICK_LIFT_M = 0.10;
const CARRIED_OFFSET_TOLERANCE_M = 0.005;
const RELEASE_SUPPORT_TOLERANCE_M = 0.015;
const BOUNDARY_PROBE_DISTANCE_M = 0.30;
const AUTHORITATIVE_FRESHNESS_MS = 1_000;
const AUTHORITATIVE_CORRELATION_NS = AUTHORITATIVE_FRESHNESS_MS * 1_000_000;
const MAX_CLEANUP_REPORT_ATTEMPTS = 3;

export class OfflineRehearsalController {
  private displayPhase: OfflineRehearsalDisplayPhase = 'idle';
  private currentPhase: RehearsalPhase | null = null;
  private step: string | null = null;
  private failure: string | null = null;
  private runId: string | null = null;
  private completedPhases: RehearsalPhase[] = [];
  private reportPaths: {json: string; markdown: string} | null = null;
  private stopVerified = false;
  private latestState: RobotStateMessage | null = null;
  private latestDiagnostics: DiagnosticsMessage | null = null;
  private stateReceivedMs: number | null = null;
  private diagnosticsReceivedMs: number | null = null;
  private lastRobotServerMonoNs = -1;
  private lastDiagnosticsServerMonoNs = -1;
  private connection: TeleopConnectionStatus['state'] = 'disconnected';
  private disposed = false;
  private cleanupActive = false;
  private cleanupReadyToReport = false;
  private cleanupOutcome: 'failed' | 'aborted' = 'failed';
  private cleanupStopConfirmations = 0;
  private cleanupGeneration = 0;
  private cleanupClosingConnection = false;
  private phaseStartedMs = 0;
  private deadlineMs: number | null = null;
  private fullRehearsalDeadlineMs: number | null = null;
  private requestSequence = 0;
  private beginRequestId: string | null = null;
  private controlRequestId: string | null = null;
  private controlRequestType: 'home_request' | 'arm_request' | 'reset_fault' | null = null;
  private pendingReport: PendingReport | null = null;
  private homeAccepted = false;
  private armAccepted = false;
  private confirmations = 0;
  private invalidConfirmations = 0;
  private sample: OfflineControllerSample | null = null;
  private controllerAnchor: OfflineControllerSample | null = null;
  private anchor: Pose | null = null;
  private prepControllerAnchor: OfflineControllerSample | null = null;
  private prepTcpAnchor: Pose | null = null;
  private targetTcp: Pose | null = null;
  private targetTrigger: 0 | 1 | null = null;
  private placementTarget: Vec3 | null = null;
  private motionTargets: MotionTarget[] = [];
  private motionIndex = 0;
  private gripperIndex = 0;
  private carriedOffset: Vec3 | null = null;
  private trackingLossAckSeq: number | null = null;
  private recoverySafeAfterResetRequest = false;

  constructor(private readonly ports: OfflineRehearsalPorts) {}

  get snapshot(): OfflineRehearsalSnapshot {
    this.checkDeadline();
    return this.buildSnapshot();
  }

  start(): boolean {
    if (this.disposed || this.isActive()) return false;
    if (!this.isEligible()) {
      this.beginCleanup('identity_preflight_failed');
      return false;
    }

    this.resetRun();
    this.displayPhase = 'identity_preflight';
    this.currentPhase = 'identity_preflight';
    this.step = 'begin';
    this.phaseStartedMs = this.ports.nowMs();
    this.deadlineMs = this.phaseStartedMs + REHEARSAL_CONFIG.phaseTimeoutMs;
    this.fullRehearsalDeadlineMs = this.phaseStartedMs + REHEARSAL_CONFIG.fullRehearsalTimeoutMs;
    this.ports.setOfflineController(null);
    this.ports.resetScene?.();
    this.beginRequestId = this.nextRequestId('begin');
    const begin = {
      v: PROTOCOL_VERSION,
      type: 'offline_rehearsal_begin' as const,
      request_id: this.beginRequestId,
      plan_version: REHEARSAL_CONFIG.planVersion,
    };
    if (!this.trySendRehearsal(begin)) {
      this.beginRequestId = null;
      this.beginCleanup('send_failed');
      this.finishLocalCleanup();
      return false;
    }
    this.notify();
    return true;
  }

  requestStop(reason: string): void {
    if (!this.isActive() || this.cleanupActive) return;
    this.beginCleanup(isNonEmptyString(reason) ? reason : 'operator_stop');
  }

  onRobotState(state: RobotStateMessage): void {
    if (!isFiniteRobotState(state)) {
      if (this.isActive()) this.beginCleanup('non_finite_state');
      return;
    }
    if (state.server_mono_ns <= this.lastRobotServerMonoNs) return;
    this.lastRobotServerMonoNs = state.server_mono_ns;
    this.stateReceivedMs = this.ports.nowMs();
    this.latestState = structuredClone(state);
    if (!this.isActive()) return;
    this.checkDeadline();
    if (!this.isActive()) return;
    if (this.cleanupActive) {
      this.confirmCleanupStop(state);
      return;
    }
    if (!hasFakeControlIdentity(state)) {
      this.beginCleanup('identity_lost');
      return;
    }
    const authoritativeFailure = this.authoritativeFreshnessFailure();
    if (authoritativeFailure !== null) {
      this.beginCleanup(authoritativeFailure);
      return;
    }
    if (
      this.currentPhase !== 'recovery_and_home'
      && (state.mode === 'FAULT' || state.robot_state === 'FAULT' || state.fault !== null)
    ) {
      this.beginCleanup('robot_fault');
      return;
    }
    if (
      state.mode === 'STALE'
      && this.currentPhase !== 'tracking_loss'
      && this.currentPhase !== 'recovery_and_home'
    ) {
      this.beginCleanup('unexpected_stale');
      return;
    }

    switch (this.currentPhase) {
      case 'home':
        this.processHomeState(state);
        break;
      case 'arm_and_anchor':
        this.processArmState(state);
        break;
      case 'translate':
      case 'rotate':
        this.processMotionState(state);
        break;
      case 'gripper':
        this.processGripperState(state);
        break;
      case 'pick_place':
        this.processPickPlaceState(state);
        break;
      case 'soft_boundary':
        this.processBoundaryState(state);
        break;
      case 'tracking_loss':
        this.processTrackingLossState(state);
        break;
      case 'recovery_and_home':
        this.processRecoveryState(state);
        break;
      case 'final_stop':
        this.processFinalStopState(state);
        break;
      default:
        break;
    }
  }

  onDiagnostics(message: DiagnosticsMessage): void {
    if (!isFiniteJson(message)) {
      if (this.isActive()) this.beginCleanup('non_finite_diagnostics');
      return;
    }
    if (message.server_mono_ns <= this.lastDiagnosticsServerMonoNs) return;
    this.lastDiagnosticsServerMonoNs = message.server_mono_ns;
    this.diagnosticsReceivedMs = this.ports.nowMs();
    this.latestDiagnostics = structuredClone(message);
    if (
      this.isActive()
      && !this.cleanupActive
      && (message.runtime !== 'LEBAI_FAKE' || message.hardware_verified !== false)
    ) {
      this.beginCleanup('identity_lost');
      return;
    }
    if (this.isActive() && !this.cleanupActive) {
      const authoritativeFailure = this.authoritativeFreshnessFailure();
      if (authoritativeFailure !== null) this.beginCleanup(authoritativeFailure);
    }
  }

  onFeedback(message: ControllerFeedback): void {
    this.checkDeadline();
    if (message.type === 'offline_rehearsal_begin_result') {
      this.processBeginFeedback(message);
      return;
    }
    if (message.type === 'offline_rehearsal_phase_ack') {
      this.processPhaseFeedback(message);
      return;
    }
    if (message.type === 'offline_rehearsal_finish_result') {
      this.processFinishFeedback(message);
      return;
    }
    if (message.type === 'arm_ack' || message.type === 'arm_rejected') {
      this.processArmFeedback(message);
      return;
    }
    if (message.type === 'fault_reset_result') {
      this.processFaultResetFeedback(message);
      return;
    }
    this.processHomeFeedback(message);
  }

  onConnection(status: TeleopConnectionStatus): void {
    const previous = this.connection;
    this.connection = status.state;
    if (status.state === 'connected') {
      if (previous !== 'connected' && !this.isActive()) this.clearAuthoritativeSnapshots();
      return;
    }
    if (!this.isActive()) return;
    if (this.cleanupActive) {
      this.finishLocalCleanup();
      return;
    }
    this.beginCleanup(status.state === 'occupied' ? 'controller_occupied' : 'connection_lost');
    this.finishLocalCleanup();
  }

  dispose(): void {
    if (this.disposed) return;
    if (this.isActive()) {
      this.beginCleanup('disposed');
      this.abortBackendRunByClosingConnection();
    }
    this.disposed = true;
  }

  private processBeginFeedback(
    message: Extract<OfflineRehearsalFeedbackMessage, {type: 'offline_rehearsal_begin_result'}>,
  ): void {
    if (this.cleanupClosingConnection) return;
    if (
      this.beginRequestId === null
      || message.request_id !== this.beginRequestId
    ) return;
    this.beginRequestId = null;
    if (!message.accepted) {
      if (this.cleanupActive) {
        if (this.stopVerified || this.cleanupReadyToReport) this.finishLocalCleanup();
        return;
      }
      this.beginCleanup(message.reason === 'controller_occupied'
        ? 'controller_occupied'
        : 'begin_rejected');
      return;
    }
    this.runId = message.run_id;
    if (this.cleanupActive) {
      if (this.cleanupReadyToReport) this.sendNextCleanupReport();
      return;
    }
    this.step = 'identity_confirmed';
    this.reportCurrentPhase('passed', {
      runtime: 'LEBAI_FAKE',
      hardware_verified: false,
      control_ready: true,
    });
  }

  private processPhaseFeedback(
    message: Extract<OfflineRehearsalFeedbackMessage, {type: 'offline_rehearsal_phase_ack'}>,
  ): void {
    if (this.cleanupClosingConnection) return;
    const pending = this.pendingReport;
    if (
      pending?.kind !== 'phase'
      || message.request_id !== pending.requestId
      || message.run_id !== this.runId
      || message.phase !== pending.phase
    ) return;
    this.pendingReport = null;
    if (!message.accepted) {
      if (this.cleanupActive) {
        this.failure = 'report_rejected';
        this.abortBackendRunByClosingConnection();
      } else {
        this.beginCleanup('report_rejected');
      }
      return;
    }
    if (!this.completedPhases.includes(pending.phase)) this.completedPhases.push(pending.phase);
    if (this.cleanupActive) {
      if (this.cleanupReadyToReport) this.sendNextCleanupReport();
      return;
    }
    const index = REHEARSAL_PHASES.indexOf(pending.phase);
    if (index < 0 || index === REHEARSAL_PHASES.length - 1) {
      this.sendFinish('passed');
      return;
    }
    this.enterPhase(REHEARSAL_PHASES[index + 1]);
  }

  private processFinishFeedback(
    message: Extract<OfflineRehearsalFeedbackMessage, {type: 'offline_rehearsal_finish_result'}>,
  ): void {
    if (this.cleanupClosingConnection) return;
    const pending = this.pendingReport;
    if (
      pending?.kind !== 'finish'
      || message.request_id !== pending.requestId
      || message.run_id !== this.runId
    ) return;
    this.pendingReport = null;
    const belongsToCurrentCleanup = pending.cleanupGeneration !== null
      && pending.cleanupGeneration === this.cleanupGeneration;
    if (!message.accepted) {
      if (this.cleanupActive) {
        this.failure = 'finish_rejected';
        this.abortBackendRunByClosingConnection();
      } else {
        this.beginCleanup('finish_rejected');
      }
      return;
    }
    if (message.outcome !== pending.outcome) {
      this.runId = null;
      if (this.cleanupActive) {
        this.failure = 'finish_outcome_mismatch';
        this.finishAcceptedDuringCleanup(belongsToCurrentCleanup);
      } else {
        this.beginCleanup('finish_outcome_mismatch');
      }
      return;
    }
    if (message.hardware_verified !== false) {
      this.runId = null;
      if (this.cleanupActive) {
        this.failure = 'invalid_finish_identity';
        this.finishAcceptedDuringCleanup(belongsToCurrentCleanup);
      } else {
        this.beginCleanup('invalid_finish_identity');
      }
      return;
    }
    this.reportPaths = {json: message.json_path, markdown: message.markdown_path};
    this.runId = null;
    if (this.cleanupActive) {
      this.finishAcceptedDuringCleanup(belongsToCurrentCleanup);
      return;
    }
    this.displayPhase = 'passed';
    this.currentPhase = null;
    this.step = null;
    this.deadlineMs = null;
    this.ports.setOfflineController(null);
    this.notify();
  }

  private finishAcceptedDuringCleanup(belongsToCurrentCleanup: boolean): void {
    if (belongsToCurrentCleanup || this.stopVerified) {
      this.finishLocalCleanup();
      return;
    }
    this.step = 'cleanup_stop';
    this.notify();
  }

  private processArmFeedback(message: ArmFeedbackMessage): void {
    if (
      this.cleanupActive
      || this.currentPhase !== 'arm_and_anchor'
      || this.controlRequestType !== 'arm_request'
      || message.request_id !== this.controlRequestId
    ) return;
    this.controlRequestId = null;
    this.controlRequestType = null;
    if (message.type === 'arm_rejected') {
      this.beginCleanup('arm_rejected');
      return;
    }
    this.armAccepted = true;
    this.step = 'active_confirmation';
    this.confirmations = 0;
    if (this.sample !== null) {
      this.sample = {...copyOfflineControllerSample(this.sample), grip: true};
      this.publishSample();
    }
    this.notify();
  }

  private processFaultResetFeedback(message: FaultResetResultMessage): void {
    if (
      this.cleanupActive
      || this.currentPhase !== 'recovery_and_home'
      || this.controlRequestType !== 'reset_fault'
      || message.request_id !== this.controlRequestId
    ) return;
    this.controlRequestId = null;
    this.controlRequestType = null;
    if (!message.accepted) {
      this.beginCleanup('fault_reset_rejected');
      return;
    }
    if (!this.recoverySafeAfterResetRequest) {
      this.step = 'recovery_post_reset_confirmation';
      return;
    }
    this.sendRecoveryHome();
  }

  private processHomeFeedback(message: HomeResultMessage): void {
    if (
      this.cleanupActive
      || (this.currentPhase !== 'home' && this.currentPhase !== 'recovery_and_home')
      || this.controlRequestType !== 'home_request'
      || message.request_id !== this.controlRequestId
    ) return;
    this.controlRequestId = null;
    this.controlRequestType = null;
    if (!message.accepted) {
      this.beginCleanup('home_rejected');
      return;
    }
    this.homeAccepted = true;
    this.step = this.currentPhase === 'home' ? 'home_confirmation' : 'recovery_confirmation';
    this.confirmations = 0;
    this.notify();
  }

  private processHomeState(state: RobotStateMessage): void {
    if (!this.homeAccepted || this.pendingReport !== null) return;
    if (this.confirm(isSafeStoppedState(state))) {
      this.reportCurrentPhase('passed', {mode: state.mode, robot_state: state.robot_state});
    }
  }

  private processArmState(state: RobotStateMessage): void {
    if (!this.armAccepted || this.pendingReport !== null) return;
    if (this.step === 'active_confirmation') {
      if (!this.confirm(state.mode === 'ACTIVE')) return;
      if (this.sample === null) {
        this.beginCleanup('missing_controller_sample');
        return;
      }
      this.prepTcpAnchor = copyPose(state.actual_tcp);
      this.prepControllerAnchor = copyOfflineControllerSample(this.sample);
      this.targetTcp = copyPose(FAKE_REHEARSAL_PREP.tcp);
      this.step = 'fake_prep';
      this.confirmations = 0;
      this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.fakePrepTimeoutMs;
      this.notify();
      return;
    }
    if (
      this.step !== 'fake_prep'
      || this.targetTcp === null
      || this.sample === null
      || this.prepControllerAnchor === null
      || this.prepTcpAnchor === null
    ) return;
    this.sample = nextControllerSample({
      sample: this.sample,
      controllerAnchor: this.prepControllerAnchor,
      tcpAnchor: this.prepTcpAnchor,
      targetTcp: this.targetTcp,
      translationScale: REHEARSAL_CONFIG.fakeTranslationScale,
      maxPositionStepM: REHEARSAL_CONFIG.maxPositionStepM,
      maxRotationStepRad: REHEARSAL_CONFIG.maxRotationStepRad,
    });
    this.publishSample();
    if (!this.confirm(
      state.mode === 'ACTIVE' && isPoseWithinTolerance(state.actual_tcp, this.targetTcp),
    )) return;
    this.anchor = copyPose(state.actual_tcp);
    this.controllerAnchor = copyOfflineControllerSample(this.sample);
    this.targetTcp = null;
    this.reportCurrentPhase('passed', {
      anchor_confirmations: SAFE_CONFIRMATIONS,
      fake_prep_joint_q: [...FAKE_REHEARSAL_PREP.joints],
      fake_prep_tcp_p: [...FAKE_REHEARSAL_PREP.tcp.p],
      fake_prep_tcp_q: [...FAKE_REHEARSAL_PREP.tcp.q],
      minimum_jacobian_singular_value: FAKE_REHEARSAL_PREP.minimumJacobianSingularValue,
    });
  }

  private processMotionState(state: RobotStateMessage): void {
    if (
      this.targetTcp === null
      || this.sample === null
      || this.controllerAnchor === null
      || this.anchor === null
      || this.pendingReport !== null
    ) return;
    this.sample = nextControllerSample({
      sample: this.sample,
      controllerAnchor: this.controllerAnchor,
      tcpAnchor: this.anchor,
      targetTcp: this.targetTcp,
      translationScale: REHEARSAL_CONFIG.fakeTranslationScale,
      maxPositionStepM: REHEARSAL_CONFIG.maxPositionStepM,
      maxRotationStepRad: REHEARSAL_CONFIG.maxRotationStepRad,
    });
    this.publishSample();
    if (!this.confirm(isPoseWithinTolerance(state.actual_tcp, this.targetTcp))) return;
    this.motionIndex += 1;
    this.confirmations = 0;
    if (this.motionIndex >= this.motionTargets.length) {
      this.targetTcp = null;
      this.reportCurrentPhase('passed', {
        target_count: this.motionTargets.length,
        consecutive_confirmations: SAFE_CONFIRMATIONS,
      });
      return;
    }
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.motionStepTimeoutMs;
    this.applyMotionTarget();
  }

  private processGripperState(state: RobotStateMessage): void {
    if (this.targetTrigger === null || this.pendingReport !== null) return;
    const reached = this.targetTrigger === 1
      ? state.gripper >= GRIPPER_CLOSE_THRESHOLD
      : state.gripper <= GRIPPER_OPEN_THRESHOLD;
    if (!this.confirm(reached)) return;
    this.gripperIndex += 1;
    this.confirmations = 0;
    if (this.gripperIndex >= 4) {
      this.reportCurrentPhase('passed', {
        open_threshold: GRIPPER_OPEN_THRESHOLD,
        close_threshold: GRIPPER_CLOSE_THRESHOLD,
        repeated_close_confirmed: true,
      });
      return;
    }
    const settings: ReadonlyArray<readonly [string, 0 | 1]> = [
      ['gripper_open', 0],
      ['gripper_close', 1],
      ['gripper_repeat_close', 1],
      ['gripper_reopen', 0],
    ];
    [this.step, this.targetTrigger] = settings[this.gripperIndex];
    this.setSampleTrigger(this.targetTrigger);
    this.notify();
  }

  private processPickPlaceState(state: RobotStateMessage): void {
    if (
      this.targetTcp === null
      || this.targetTrigger === null
      || this.sample === null
      || this.controllerAnchor === null
      || this.anchor === null
    ) return;
    const scene = this.readValidScene();
    if (scene === null) return;
    this.sample = nextControllerSample({
      sample: this.sample,
      controllerAnchor: this.controllerAnchor,
      tcpAnchor: this.anchor,
      targetTcp: this.targetTcp,
      translationScale: REHEARSAL_CONFIG.fakeTranslationScale,
      maxPositionStepM: REHEARSAL_CONFIG.maxPositionStepM,
      maxRotationStepRad: REHEARSAL_CONFIG.maxRotationStepRad,
    });
    this.publishSample();
    const poseReached = isPoseWithinTolerance(state.actual_tcp, this.targetTcp);
    const block = scene.blocks.find((candidate) => candidate.id === 'block-orange');
    if (!block) {
      this.beginCleanup('invalid_block_placement');
      return;
    }

    if (this.step === 'pick_approach') {
      if (this.confirm(poseReached && state.gripper <= GRIPPER_OPEN_THRESHOLD)) {
        this.step = 'pick_attach';
        this.targetTrigger = 1;
        this.setSampleTrigger(1);
        this.confirmations = 0;
      }
      return;
    }

    if (this.step === 'pick_attach') {
      const attached = poseReached
        && state.gripper >= GRIPPER_CLOSE_THRESHOLD
        && scene.carriedBlockId === 'block-orange';
      if (!attached) {
        this.confirmations = 0;
        this.carriedOffset = null;
        return;
      }
      const offset = vectorDifference(block.position, state.actual_tcp.p);
      if (this.carriedOffset === null) this.carriedOffset = offset;
      const stable = vectorDistance(offset, this.carriedOffset) <= CARRIED_OFFSET_TOLERANCE_M;
      if (this.confirm(stable)) this.enterPickMotion('pick_lift');
      return;
    }

    if (this.step === 'pick_release') {
      if (scene.carriedBlockId !== null || state.gripper > GRIPPER_OPEN_THRESHOLD) {
        this.confirmations = 0;
        this.invalidConfirmations = 0;
        return;
      }
      const valid = poseReached && this.isValidPlacement(scene, block.position, block.sizeM);
      this.invalidConfirmations = valid ? 0 : this.invalidConfirmations + 1;
      if (this.invalidConfirmations >= SAFE_CONFIRMATIONS) {
        this.beginCleanup('invalid_block_placement');
        return;
      }
      if (this.confirm(valid)) {
        this.reportCurrentPhase('passed', {
          block_id: 'block-orange',
          attached_confirmations: SAFE_CONFIRMATIONS,
          released_confirmations: SAFE_CONFIRMATIONS,
          invalid_overlap: false,
        });
      }
      return;
    }

    const attached = scene.carriedBlockId === 'block-orange'
      && state.gripper >= GRIPPER_CLOSE_THRESHOLD;
    if (!attached || this.carriedOffset === null) {
      this.confirmations = 0;
      return;
    }
    const offset = vectorDifference(block.position, state.actual_tcp.p);
    const stable = vectorDistance(offset, this.carriedOffset) <= CARRIED_OFFSET_TOLERANCE_M;
    if (!this.confirm(poseReached && stable)) return;
    if (this.step === 'pick_lift') this.enterPickMotion('pick_transfer');
    else if (this.step === 'pick_transfer') this.enterPickMotion('pick_lower');
    else if (this.step === 'pick_lower') {
      this.step = 'pick_release';
      this.targetTrigger = 0;
      this.confirmations = 0;
      this.invalidConfirmations = 0;
      this.setSampleTrigger(0);
    }
  }

  private processBoundaryState(state: RobotStateMessage): void {
    if (
      this.targetTcp === null
      || this.sample === null
      || this.controllerAnchor === null
      || this.anchor === null
    ) return;
    this.sample = nextControllerSample({
      sample: this.sample,
      controllerAnchor: this.controllerAnchor,
      tcpAnchor: this.anchor,
      targetTcp: this.targetTcp,
      translationScale: REHEARSAL_CONFIG.fakeTranslationScale,
      maxPositionStepM: REHEARSAL_CONFIG.maxPositionStepM,
      maxRotationStepRad: REHEARSAL_CONFIG.maxRotationStepRad,
    });
    this.publishSample();
    if (this.step === 'boundary_outward') {
      if (!this.confirm(state.constraint === 'workspace_boundary')) return;
      if (this.anchor === null) {
        this.beginCleanup('missing_anchor');
        return;
      }
      this.step = 'boundary_retreat';
      this.targetTcp = copyPose(this.anchor);
      this.confirmations = 0;
      return;
    }
    const clearAndHome = state.constraint === null
      && isPoseWithinTolerance(state.actual_tcp, this.targetTcp);
    if (this.confirm(clearAndHome)) {
      this.reportCurrentPhase('passed', {
        constraint: 'workspace_boundary',
        retreated_to_anchor: true,
      });
    }
  }

  private processTrackingLossState(state: RobotStateMessage): void {
    if (state.ack_seq != null) {
      this.trackingLossAckSeq = Math.max(this.trackingLossAckSeq ?? -1, state.ack_seq);
    }
    if (this.confirm(state.mode === 'STALE' && isStoppedBackendState(state))) {
      this.reportCurrentPhase('passed', {
        tracking_valid: false,
        authoritative_mode: 'STALE',
      });
    }
  }

  private processRecoveryState(state: RobotStateMessage): void {
    if (this.homeAccepted) {
      if (!this.confirm(isSafeStoppedState(state))) return;
      this.reportCurrentPhase('passed', {home_after_tracking_loss: true});
      return;
    }
    const validFrameAcknowledged = state.ack_seq != null
      && this.trackingLossAckSeq != null
      && state.ack_seq > this.trackingLossAckSeq;
    const stoppedMode = state.mode === 'READY' || state.mode === 'DISARMED';
    if (this.controlRequestType === 'reset_fault') {
      if (validFrameAcknowledged && stoppedMode && isStoppedBackendState(state) && state.fault === null) {
        this.recoverySafeAfterResetRequest = true;
      }
      return;
    }
    if (this.controlRequestType === 'home_request' || !validFrameAcknowledged) return;
    if (!stoppedMode || !isStoppedBackendState(state)) return;
    if (state.fault !== null || state.robot_state === 'FAULT') {
      this.step = 'recovery_reset_fault';
      this.recoverySafeAfterResetRequest = false;
      this.sendControl('reset_fault');
      return;
    }
    this.sendRecoveryHome();
  }

  private processFinalStopState(state: RobotStateMessage): void {
    if (!this.confirm(isSafeStoppedState(state))) return;
    this.stopVerified = true;
    this.ports.setOfflineController(null);
    this.sample = null;
    this.reportCurrentPhase('passed', {
      stop_verified: true,
      hardware_verified: false,
    });
  }

  private enterPhase(phase: RehearsalPhase): void {
    this.currentPhase = phase;
    this.displayPhase = phase;
    this.step = null;
    this.phaseStartedMs = this.ports.nowMs();
    this.deadlineMs = this.phaseStartedMs + (
      phase === 'translate' || phase === 'rotate'
        ? REHEARSAL_CONFIG.motionStepTimeoutMs
        : REHEARSAL_CONFIG.phaseTimeoutMs
    );
    this.confirmations = 0;
    this.invalidConfirmations = 0;
    this.homeAccepted = false;
    this.armAccepted = false;
    this.pendingReport = null;
    this.targetTcp = null;
    this.targetTrigger = null;
    this.placementTarget = null;

    switch (phase) {
      case 'home':
        this.step = 'home_request';
        this.sendControl('home_request');
        break;
      case 'arm_and_anchor':
        this.step = 'arm_request';
        if (this.latestState !== null) {
          this.sample = sampleFromPose(this.latestState.actual_tcp, 0, true);
          this.publishSample();
        }
        this.sendControl('arm_request');
        break;
      case 'translate':
        this.prepareAxisMotion(false);
        break;
      case 'rotate':
        this.prepareAxisMotion(true);
        break;
      case 'gripper':
        this.gripperIndex = 0;
        this.step = 'gripper_open';
        this.targetTrigger = 0;
        this.setSampleTrigger(0);
        break;
      case 'pick_place':
        this.preparePickPlace();
        break;
      case 'soft_boundary':
        this.prepareBoundary();
        break;
      case 'tracking_loss':
        this.step = 'tracking_invalid';
        this.trackingLossAckSeq = this.latestState?.ack_seq ?? null;
        if (this.sample === null && this.anchor !== null) this.sample = sampleFromPose(this.anchor, 0, false);
        if (this.sample !== null) {
          this.sample = {...copyOfflineControllerSample(this.sample), trackingValid: false};
          this.publishSample();
        }
        break;
      case 'recovery_and_home':
        this.step = 'recovery_valid_tracking';
        this.recoverySafeAfterResetRequest = false;
        if (this.sample !== null) {
          this.sample = {...copyOfflineControllerSample(this.sample), trackingValid: true, grip: false, trigger: 0};
          this.publishSample();
        }
        break;
      case 'final_stop':
        this.step = 'final_disarm';
        if (this.sample !== null) {
          this.sample = {...copyOfflineControllerSample(this.sample), grip: false, trigger: 0};
          this.publishSample();
        }
        this.sendControl('disarm');
        break;
      case 'finalize':
        this.step = 'finalize_report';
        this.reportCurrentPhase('passed', {
          hardware_verified: false,
          hardware_pending_count: OFFLINE_REHEARSAL_HARDWARE_PENDING.length,
        });
        break;
      default:
        break;
    }
    this.notify();
  }

  private prepareAxisMotion(rotation: boolean): void {
    if (this.anchor === null || this.sample === null) {
      this.beginCleanup('missing_anchor');
      return;
    }
    const axes: CartesianAxis[] = ['x', 'y', 'z'];
    this.motionTargets = [];
    for (const axis of axes) {
      for (const direction of [1, -1] as const) {
        const axisLabel = rotation
          ? axis === 'x' ? 'roll' : axis === 'y' ? 'pitch' : 'yaw'
          : axis;
        const label = `${direction === 1 ? '+' : '-'}${axisLabel}`;
        const target = rotation
          ? rotationTarget(this.anchor, axis, direction, REHEARSAL_CONFIG.rotationAngleRad)
          : translationTarget(this.anchor, axis, direction, REHEARSAL_CONFIG.translationDistanceM);
        this.motionTargets.push({step: label, target});
        this.motionTargets.push({step: `${label}_return`, target: copyPose(this.anchor)});
      }
    }
    this.motionIndex = 0;
    this.applyMotionTarget();
  }

  private applyMotionTarget(): void {
    const motion = this.motionTargets[this.motionIndex];
    this.step = motion.step;
    this.targetTcp = copyPose(motion.target);
    this.notify();
  }

  private preparePickPlace(): void {
    if (this.anchor === null) {
      this.beginCleanup('missing_anchor');
      return;
    }
    const scene = this.readValidScene();
    const block = scene?.blocks.find((candidate) => candidate.id === 'block-orange');
    if (!scene || !block || scene.carriedBlockId !== null || scene.invalidOverlap) {
      this.beginCleanup('invalid_block_placement');
      return;
    }
    this.placementTarget = [-block.position[0], block.position[1], block.position[2]];
    this.targetTcp = {
      p: [block.position[0], block.position[1] + block.sizeM / 2, block.position[2]],
      q: [...this.anchor.q],
    };
    this.targetTrigger = 0;
    this.step = 'pick_approach';
    this.carriedOffset = null;
    this.setSampleTrigger(0);
  }

  private enterPickMotion(step: 'pick_lift' | 'pick_transfer' | 'pick_lower'): void {
    if (this.targetTcp === null || this.placementTarget === null) {
      this.beginCleanup('invalid_block_placement');
      return;
    }
    const current = this.targetTcp;
    if (step === 'pick_lift') {
      this.targetTcp = {p: [current.p[0], current.p[1] + PICK_LIFT_M, current.p[2]], q: [...current.q]};
    } else if (step === 'pick_transfer') {
      this.targetTcp = {
        p: [this.placementTarget[0], current.p[1], this.placementTarget[2]],
        q: [...current.q],
      };
    } else {
      this.targetTcp = {
        p: [this.placementTarget[0], this.placementTarget[1] + 0.03, this.placementTarget[2]],
        q: [...current.q],
      };
    }
    this.step = step;
    this.confirmations = 0;
    this.notify();
  }

  private prepareBoundary(): void {
    if (this.anchor === null) {
      this.beginCleanup('missing_anchor');
      return;
    }
    this.step = 'boundary_outward';
    this.targetTcp = copyPose(
      translationTarget(this.anchor, 'x', 1, BOUNDARY_PROBE_DISTANCE_M),
    );
  }

  private reportCurrentPhase(
    status: 'passed' | 'failed',
    measurements: Record<string, DiagnosticPayloadValue>,
  ): void {
    if (this.runId === null || this.currentPhase === null || this.pendingReport !== null) return;
    const requestId = this.nextRequestId('phase');
    const phase = this.currentPhase;
    this.displayPhase = this.cleanupActive ? 'failed' : 'reporting';
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.phaseTimeoutMs;
    this.targetTcp = null;
    const message: PhaseReportMessage = {
      v: PROTOCOL_VERSION,
      type: 'offline_rehearsal_phase',
      request_id: requestId,
      run_id: this.runId,
      phase,
      status,
      started_client_ms: this.phaseStartedMs,
      completed_client_ms: this.ports.nowMs(),
      target: {
        evidence_only: true,
        step: this.step ?? phase,
      },
      measurements,
      failure: status === 'failed' ? {reason: this.failure ?? 'failed'} : null,
    };
    this.pendingReport = {
      kind: 'phase',
      requestId,
      phase,
      cleanupGeneration: this.cleanupActive ? this.cleanupGeneration : null,
      attempts: 1,
      message,
    };
    if (!this.trySendRehearsal(message)) {
      this.pendingReport = null;
      if (this.cleanupActive) {
        this.failure = 'send_failed';
        this.abortBackendRunByClosingConnection();
      } else {
        this.beginCleanup('send_failed');
      }
      return;
    }
    this.notify();
  }

  private sendFinish(outcome: 'passed' | 'failed' | 'aborted'): void {
    if (this.runId === null || this.pendingReport !== null) return;
    const requestId = this.nextRequestId('finish');
    const message: FinishReportMessage = {
      v: PROTOCOL_VERSION,
      type: 'offline_rehearsal_finish',
      request_id: requestId,
      run_id: this.runId,
      outcome,
      failure: outcome === 'passed' ? null : {reason: this.failure ?? outcome},
    };
    this.pendingReport = {
      kind: 'finish',
      requestId,
      outcome,
      cleanupGeneration: this.cleanupActive ? this.cleanupGeneration : null,
      attempts: 1,
      message,
    };
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.phaseTimeoutMs;
    if (!this.trySendRehearsal(message)) {
      this.pendingReport = null;
      if (!this.cleanupActive) this.beginCleanup('send_failed');
      else {
        this.failure = 'send_failed';
        this.abortBackendRunByClosingConnection();
      }
      return;
    }
    this.notify();
  }

  private beginCleanup(reason: string): void {
    if (this.cleanupActive) return;
    this.cleanupActive = true;
    this.cleanupGeneration += 1;
    this.cleanupClosingConnection = false;
    this.cleanupOutcome = isAbortReason(reason) ? 'aborted' : 'failed';
    this.failure = reason;
    this.displayPhase = 'failed';
    this.stopVerified = false;
    this.step = 'cleanup_stop';
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.phaseTimeoutMs;
    this.cleanupStopConfirmations = 0;
    this.cleanupReadyToReport = false;
    this.sample = null;
    this.targetTcp = null;
    this.targetTrigger = null;
    this.ports.setOfflineController(null);
    const disarmSent = this.trySendControl({
      v: PROTOCOL_VERSION,
      type: 'disarm',
      request_id: this.nextRequestId('cleanup-disarm'),
      client_mono_ms: this.ports.nowMs(),
    });
    if (!disarmSent) {
      this.failure = 'send_failed';
      if (this.runId !== null || this.beginRequestId !== null) {
        this.abortBackendRunByClosingConnection();
      } else {
        this.finishLocalCleanup();
      }
      return;
    }
    this.notify();
  }

  private confirmCleanupStop(state: RobotStateMessage): void {
    this.cleanupStopConfirmations = isSafeStoppedState(state)
      ? this.cleanupStopConfirmations + 1
      : 0;
    if (this.cleanupStopConfirmations < SAFE_CONFIRMATIONS) return;
    if (this.stopVerified) return;
    this.stopVerified = true;
    this.cleanupReadyToReport = true;
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.phaseTimeoutMs;
    if (this.runId === null) {
      if (this.beginRequestId === null) this.finishLocalCleanup();
      return;
    }
    this.sendNextCleanupReport();
  }

  private sendNextCleanupReport(): void {
    if (!this.cleanupReadyToReport || this.runId === null || this.pendingReport !== null) return;
    const next = REHEARSAL_PHASES.find((phase) => !this.completedPhases.includes(phase));
    if (!next) {
      this.sendCleanupFinish();
      return;
    }
    this.currentPhase = next;
    this.phaseStartedMs = Math.min(this.phaseStartedMs, this.ports.nowMs());
    this.step = 'cleanup_report';
    this.reportCurrentPhase('failed', {
      stop_verified: this.stopVerified,
      hardware_verified: false,
    });
  }

  private sendCleanupFinish(): void {
    if (this.pendingReport !== null) return;
    this.sendFinish(this.cleanupOutcome);
  }

  private finishLocalCleanup(): void {
    this.cleanupActive = false;
    this.cleanupReadyToReport = false;
    this.cleanupClosingConnection = false;
    this.currentPhase = null;
    this.step = null;
    this.deadlineMs = null;
    this.fullRehearsalDeadlineMs = null;
    this.beginRequestId = null;
    this.controlRequestId = null;
    this.controlRequestType = null;
    this.pendingReport = null;
    this.sample = null;
    this.targetTcp = null;
    this.targetTrigger = null;
    this.ports.setOfflineController(null);
    this.notify();
  }

  private checkDeadline(): void {
    if (!this.isActive()) return;
    if (!this.cleanupActive) {
      if (
        this.fullRehearsalDeadlineMs !== null
        && this.ports.nowMs() > this.fullRehearsalDeadlineMs
      ) {
        this.beginCleanup('full_rehearsal_timeout');
        return;
      }
      const authoritativeFailure = this.authoritativeFreshnessFailure();
      if (authoritativeFailure !== null) {
        this.beginCleanup(authoritativeFailure);
        return;
      }
    }
    if (this.deadlineMs === null || this.ports.nowMs() <= this.deadlineMs) return;
    if (this.cleanupActive) {
      if (this.cleanupClosingConnection) {
        this.abortBackendRunByClosingConnection();
        return;
      }
      if (!this.cleanupReadyToReport) {
        if (!this.stopVerified) this.failure = 'stop_unverified';
        this.cleanupReadyToReport = true;
        if (this.runId === null) {
          this.finishLocalCleanup();
          return;
        }
      }
      if (this.runId === null) {
        this.finishLocalCleanup();
        return;
      }
      if (this.pendingReport !== null) {
        this.retryPendingCleanupReport();
      } else {
        this.sendNextCleanupReport();
      }
      return;
    }
    this.beginCleanup('phase_timeout');
  }

  private retryPendingCleanupReport(): void {
    const pending = this.pendingReport;
    if (pending === null) return;
    if (pending.attempts >= MAX_CLEANUP_REPORT_ATTEMPTS) {
      this.abortBackendRunByClosingConnection();
      return;
    }
    this.pendingReport = {...pending, attempts: pending.attempts + 1};
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.phaseTimeoutMs;
    if (!this.trySendRehearsal(pending.message)) {
      this.failure = 'send_failed';
      this.abortBackendRunByClosingConnection();
      return;
    }
    this.notify();
  }

  private abortBackendRunByClosingConnection(): void {
    this.cleanupClosingConnection = true;
    this.pendingReport = null;
    this.deadlineMs = this.ports.nowMs() + REHEARSAL_CONFIG.phaseTimeoutMs;
    try {
      this.ports.closeConnection();
      this.finishLocalCleanup();
    } catch {
      // Keep cleanup active and retry socket closure at the next deadline.
      this.notify();
    }
  }

  private sendControl(type: 'home_request' | 'arm_request' | 'reset_fault' | 'disarm'): void {
    const requestId = this.nextRequestId(type);
    if (type !== 'disarm') {
      this.controlRequestId = requestId;
      this.controlRequestType = type;
    }
    const sent = this.trySendControl({
      v: PROTOCOL_VERSION,
      type,
      request_id: requestId,
      client_mono_ms: this.ports.nowMs(),
    });
    if (!sent) {
      this.controlRequestId = null;
      this.controlRequestType = null;
      this.beginCleanup('send_failed');
    }
  }

  private sendRecoveryHome(): void {
    if (this.controlRequestType !== null) return;
    this.step = 'recovery_home_request';
    this.sendControl('home_request');
    this.notify();
  }

  private trySendControl(message: ClientControlMessage): boolean {
    try {
      this.ports.sendControl(message);
      return true;
    } catch {
      return false;
    }
  }

  private trySendRehearsal(message: OfflineRehearsalClientMessage): boolean {
    try {
      this.ports.sendRehearsal(message);
      return true;
    } catch {
      return false;
    }
  }

  private setSampleTrigger(trigger: 0 | 1): void {
    if (this.sample === null && this.anchor !== null) this.sample = sampleFromPose(this.anchor, trigger, true);
    if (this.sample === null) {
      this.beginCleanup('missing_controller_sample');
      return;
    }
    this.sample = {
      ...copyOfflineControllerSample(this.sample),
      grip: true,
      trigger,
      trackingValid: true,
    };
    this.publishSample();
  }

  private publishSample(): void {
    if (this.sample !== null && !this.cleanupActive) {
      this.ports.setOfflineController(copyOfflineControllerSample(this.sample));
    }
  }

  private readValidScene(): OfflineSceneSnapshot | null {
    try {
      return copyOfflineSceneSnapshot(this.ports.readScene());
    } catch {
      this.beginCleanup('invalid_scene_snapshot');
      return null;
    }
  }

  private isValidPlacement(
    scene: OfflineSceneSnapshot,
    position: readonly number[],
    sizeM: number,
  ): boolean {
    if (this.placementTarget === null || scene.invalidOverlap) return false;
    const horizontalTolerance = sizeM;
    return Math.abs(position[0] - this.placementTarget[0]) <= horizontalTolerance
      && Math.abs(position[2] - this.placementTarget[2]) <= horizontalTolerance
      && Math.abs(position[1] - this.placementTarget[1]) <= RELEASE_SUPPORT_TOLERANCE_M;
  }

  private confirm(condition: boolean): boolean {
    this.confirmations = condition ? this.confirmations + 1 : 0;
    return this.confirmations >= SAFE_CONFIRMATIONS;
  }

  private isEligible(): boolean {
    const state = this.latestState;
    const diagnostics = this.latestDiagnostics;
    return this.connection === 'connected'
      && diagnostics?.runtime === 'LEBAI_FAKE'
      && diagnostics.hardware_verified === false
      && state?.backend === 'LEBAI_FAKE'
      && state.mode === 'READY'
      && state.robot_state === 'IDLE'
      && state.fault === null
      && state.constraint === null
      && this.authoritativeFreshnessFailure() === null;
  }

  private authoritativeFreshnessFailure(): string | null {
    if (this.latestState === null || this.stateReceivedMs === null) return 'state_timeout';
    if (this.latestDiagnostics === null || this.diagnosticsReceivedMs === null) {
      return 'diagnostics_timeout';
    }
    const nowMs = this.ports.nowMs();
    if (nowMs - this.stateReceivedMs > AUTHORITATIVE_FRESHNESS_MS) return 'state_timeout';
    if (nowMs - this.diagnosticsReceivedMs > AUTHORITATIVE_FRESHNESS_MS) {
      return 'diagnostics_timeout';
    }
    if (
      Math.abs(this.latestState.server_mono_ns - this.latestDiagnostics.server_mono_ns)
      > AUTHORITATIVE_CORRELATION_NS
    ) return 'identity_uncorrelated';
    return null;
  }

  private clearAuthoritativeSnapshots(): void {
    this.latestState = null;
    this.latestDiagnostics = null;
    this.stateReceivedMs = null;
    this.diagnosticsReceivedMs = null;
    this.lastRobotServerMonoNs = -1;
    this.lastDiagnosticsServerMonoNs = -1;
  }

  private isActive(): boolean {
    return this.displayPhase !== 'idle'
      && this.displayPhase !== 'passed'
      && !(this.displayPhase === 'failed' && !this.cleanupActive);
  }

  private resetRun(): void {
    this.displayPhase = 'idle';
    this.currentPhase = null;
    this.step = null;
    this.failure = null;
    this.runId = null;
    this.completedPhases = [];
    this.reportPaths = null;
    this.stopVerified = false;
    this.cleanupActive = false;
    this.cleanupReadyToReport = false;
    this.cleanupStopConfirmations = 0;
    this.cleanupGeneration = 0;
    this.cleanupClosingConnection = false;
    this.fullRehearsalDeadlineMs = null;
    this.beginRequestId = null;
    this.controlRequestId = null;
    this.controlRequestType = null;
    this.pendingReport = null;
    this.homeAccepted = false;
    this.armAccepted = false;
    this.confirmations = 0;
    this.invalidConfirmations = 0;
    this.sample = null;
    this.controllerAnchor = null;
    this.anchor = null;
    this.prepControllerAnchor = null;
    this.prepTcpAnchor = null;
    this.targetTcp = null;
    this.targetTrigger = null;
    this.placementTarget = null;
    this.motionTargets = [];
    this.motionIndex = 0;
    this.gripperIndex = 0;
    this.carriedOffset = null;
    this.trackingLossAckSeq = null;
    this.recoverySafeAfterResetRequest = false;
  }

  private nextRequestId(kind: string): string {
    this.requestSequence += 1;
    return `offline-${kind}-${this.requestSequence}`;
  }

  private buildSnapshot(): OfflineRehearsalSnapshot {
    return {
      phase: this.displayPhase,
      currentPhase: this.currentPhase,
      step: this.step,
      active: this.isActive(),
      failure: this.failure,
      runId: this.runId,
      completedPhases: [...this.completedPhases],
      targetTcp: this.targetTcp === null ? null : copyPose(this.targetTcp),
      targetTrigger: this.targetTrigger,
      placementTarget: this.placementTarget === null ? null : [...this.placementTarget] as Vec3,
      remainingTimeoutMs: this.deadlineMs === null
        ? null
        : Math.max(0, this.deadlineMs - this.ports.nowMs()),
      stopVerified: this.stopVerified,
      reportPaths: this.reportPaths === null ? null : {...this.reportPaths},
      hardwareVerified: false,
      hardwarePending: OFFLINE_REHEARSAL_HARDWARE_PENDING,
    };
  }

  private notify(): void {
    this.ports.onSnapshot?.(this.buildSnapshot());
  }
}

function sampleFromPose(
  pose: TcpPose,
  trigger: 0 | 1,
  trackingValid: boolean,
  grip = false,
): OfflineControllerSample {
  return {
    position: [...pose.p],
    quaternion: [...pose.q],
    grip,
    trigger,
    trackingValid,
  };
}

function copyPose(pose: TcpPose): Pose {
  return {p: [...pose.p], q: [...pose.q]};
}

function isPoseWithinTolerance(actual: TcpPose, target: TcpPose): boolean {
  return vectorDistance(actual.p, target.p) <= REHEARSAL_CONFIG.translationToleranceM
    && quaternionAngularError(actual.q, target.q) <= REHEARSAL_CONFIG.rotationToleranceRad;
}

function vectorDifference(first: readonly number[], second: readonly number[]): Vec3 {
  return [first[0] - second[0], first[1] - second[1], first[2] - second[2]];
}

function vectorDistance(first: readonly number[], second: readonly number[]): number {
  return Math.hypot(first[0] - second[0], first[1] - second[1], first[2] - second[2]);
}

function isSafeStoppedState(state: RobotStateMessage): boolean {
  return (state.mode === 'DISARMED' || state.mode === 'READY')
    && isStoppedBackendState(state)
    && state.fault === null
    && state.recovery_phase == null;
}

function hasFakeControlIdentity(state: RobotStateMessage): boolean {
  return state.backend === 'LEBAI_FAKE';
}

function isStoppedBackendState(state: RobotStateMessage): boolean {
  return state.robot_state === 'IDLE' || state.robot_state === 'HOLD';
}

function isFiniteRobotState(state: RobotStateMessage): boolean {
  return isFiniteJson(state)
    && isFiniteVector(state.actual_tcp.p, 3)
    && isFiniteVector(state.actual_tcp.q, 4)
    && isFiniteVector(state.actual_q, 6)
    && isUnitQuaternion(state.actual_tcp.q)
    && Number.isFinite(state.gripper)
    && state.gripper >= 0
    && state.gripper <= 1;
}

function isFiniteJson(value: unknown): boolean {
  if (value === null || typeof value === 'string' || typeof value === 'boolean') return true;
  if (typeof value === 'number') return Number.isFinite(value);
  if (Array.isArray(value)) return value.every(isFiniteJson);
  if (typeof value !== 'object') return false;
  return Object.values(value).every(isFiniteJson);
}

function isFiniteVector(value: readonly number[], length: number): boolean {
  return Array.isArray(value) && value.length === length && value.every(Number.isFinite);
}

function isUnitQuaternion(value: readonly number[]): boolean {
  const norm = Math.hypot(...value);
  return norm >= 0.9 && norm <= 1.1;
}

function isAbortReason(reason: string): boolean {
  return reason === 'operator_stop'
    || reason === 'page_hidden'
    || reason === 'connection_lost'
    || reason === 'disposed';
}

function isNonEmptyString(value: string): boolean {
  return value.trim().length > 0;
}
