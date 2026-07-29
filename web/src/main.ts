import './styles.css';
import {disposeAppForUnload} from './appDisposal';
import {
  PROTOCOL_VERSION,
  type DiagnosticsMessage,
  type RobotStateMessage,
  type VRFrame,
} from './protocol/messages';
import {RobotStateBuffer} from './robot/robotState';
import {SimulationScene} from './scenes/simulationScene';
import type {RobotRuntimeSummary} from './scenes/vrSafetyPanel';
import {resolveTeleopSocketUrl} from './transport/socketUrl';
import {TeleopSocket} from './transport/teleopSocket';
import {ArmPanel} from './ui/armPanel';
import {Hud} from './ui/hud';
import {LatencyTracker} from './ui/latency';
import {DiagnosticsPanel} from './ui/diagnosticsPanel';
import {XRSessionController, type XRSessionStatus} from './xr/session';

const app = document.querySelector<HTMLElement>('#app');
if (!app) throw new Error('页面缺少仿真界面容器');

const hud = new Hud(app);
const diagnosticsPanel = new DiagnosticsPanel(hud.diagnosticsContainer);
const stateBuffer = new RobotStateBuffer();
const latency = new LatencyTracker(512);
const pendingFrames = new Map<number, number>();
let latestRobotState: RobotStateMessage | null = null;
let latestDiagnostics: DiagnosticsMessage | null = null;

let scene: SimulationScene;
let armPanel: ArmPanel;
let xrController: XRSessionController;
let vrControlSequence = 0;

const socket = new TeleopSocket(
  resolveTeleopSocketUrl(window.location, import.meta.env.VITE_TELEOP_WS_URL),
  (state) => onRobotState(state),
  undefined,
  (status) => {
    scene?.resetConnection();
    pendingFrames.clear();
    latency.reset();
    hud.setConnectionStatus(status);
    hud.setLatency(null, null);
    armPanel?.setConnectionStatus(status);
    xrController?.setConstraint(null);
    if (status.state !== 'connected') {
      latestRobotState = null;
      latestDiagnostics = null;
      diagnosticsPanel.clear();
      scene?.setRuntimeSummary(emptyRuntimeSummary());
    }
  },
  (message) => armPanel?.handleArmFeedback(message),
  (message) => armPanel?.handleFaultResetResult(message),
  (message) => armPanel?.handleHomeResult(message),
  (message) => onDiagnostics(message),
);

armPanel = new ArmPanel(
  hud.actionContainer,
  (message) => socket.sendControl(message),
  () => void (xrController.isActive ? xrController.exitVR() : xrController.enterVR()),
  (snapshot) => scene?.setArmSafetyState(snapshot),
);
armPanel.setConnectionStatus({state: 'disconnected'});

scene = new SimulationScene(hud.sceneContainer, {
  stateBuffer,
  onFrame: sendFrame,
  onController: updateController,
  onError: (message) => hud.showSceneError(message),
});
scene.setArmSafetyState(armPanel.safetyState);

xrController = new XRSessionController({
  xr: navigator.xr,
  host: scene,
  onFrame: sendFrame,
  onController: updateController,
  onArmRequest: () => { armPanel.requestArm('xr'); },
  onStopOrResetRequest: () => { armPanel.requestStopOrReset('xr'); },
  onControllerSupport: (supported) => scene.setQuestControllerSupport(supported),
  onDisarm: sendVRDisarm,
  onLockReset: () => armPanel.resetToLocked(),
  onStatus: updateXRStatus,
});

scene.start();
socket.connect();

window.addEventListener('beforeunload', () => {
  disposeAppForUnload(xrController, scene, socket);
});

function updateController(controller: {tracking: boolean; grip: boolean; trigger: number}): void {
  hud.setController(controller);
  if (controller.tracking) armPanel.observeGrip(controller.grip);
}

function sendVRDisarm(): void {
  vrControlSequence += 1;
  socket.sendControl({
    v: PROTOCOL_VERSION,
    type: 'disarm',
    request_id: `vr-disarm-${vrControlSequence}`,
    client_mono_ms: performance.now(),
  });
}

function updateXRStatus(status: XRSessionStatus): void {
  armPanel.setVRStatus(status);
  if (status.state === 'starting') {
    pendingFrames.clear();
    latency.reset();
    hud.setLatency(null, null);
  } else if (status.state === 'error') {
    hud.showSceneError(status.message);
  } else if (status.state === 'active') {
    hud.clearSceneError();
  }
}

function sendFrame(frame: VRFrame): void {
  pendingFrames.set(frame.seq, frame.client_mono_ms);
  while (pendingFrames.size > 512) pendingFrames.delete(pendingFrames.keys().next().value!);
  socket.sendFrame(frame);
}

function onRobotState(state: RobotStateMessage): void {
  latestRobotState = state;
  const constraint = state.constraint ?? null;
  const recoveryPhase = state.recovery_phase ?? null;
  scene.applyRobotState(state);
  armPanel.setFault(state.fault ?? null);
  armPanel.setConstraint(constraint);
  armPanel.setRecoveryPhase(recoveryPhase);
  armPanel.setMode(state.mode);
  xrController.setConstraint(constraint);
  hud.setRobotState({
    mode: state.mode,
    backendState: state.robot_state,
    sampleAgeMs: state.sample_age_ms ?? null,
    fault: state.fault ?? null,
    constraint,
    recoveryPhase,
  });
  recordAcknowledgement(state.ack_seq ?? null);
  updateDiagnostics();
}

function onDiagnostics(diagnostics: DiagnosticsMessage): void {
  latestDiagnostics = diagnostics;
  updateDiagnostics();
}

function recordAcknowledgement(sequence: number | null): void {
  if (sequence === null) return;
  const sentAt = pendingFrames.get(sequence);
  if (sentAt !== undefined) latency.add(Math.max(0, performance.now() - sentAt));
  for (const pendingSequence of pendingFrames.keys()) {
    if (pendingSequence <= sequence) pendingFrames.delete(pendingSequence);
  }
  hud.setLatency(latency.current, latency.p95());
  updateDiagnostics();
}

function updateDiagnostics(): void {
  const state = latestRobotState;
  if (!state) return;
  const backend = latestDiagnostics?.runtime ?? state.backend ?? null;
  const realRobotMode = state.real_robot_mode ?? null;
  hud.setRuntimeIdentity(backend, realRobotMode);
  armPanel?.setRuntimeIdentity(backend);
  diagnosticsPanel.update(state, latestDiagnostics, {
    currentMs: latency.current,
    p95Ms: latency.p95(),
  });
  scene?.setRuntimeSummary({
    backend,
    realRobotMode,
    actualTcp: state.actual_tcp,
    gripper: state.gripper,
    latencyMs: latency.current,
    hardwareVerified: false,
  });
}

function emptyRuntimeSummary(): RobotRuntimeSummary {
  return {
    backend: null,
    realRobotMode: null,
    actualTcp: null,
    gripper: null,
    latencyMs: null,
    hardwareVerified: false,
  };
}
