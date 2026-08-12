import './styles.css';
import {disposeAppForUnload, installRehearsalVisibilityStop} from './appDisposal';
import {forwardTeleopFrame, type TeleopFrameSource} from './appFrameForwarding';
import {
  PROTOCOL_VERSION,
  type DiagnosticsMessage,
  type RobotStateMessage,
  type VRFrame,
} from './protocol/messages';
import {
  OfflineRehearsalController,
  type OfflineRehearsalSnapshot,
} from './rehearsal/offlineRehearsalController';
import {RobotStateBuffer} from './robot/robotState';
import {SimulationScene} from './scenes/simulationScene';
import type {RobotRuntimeSummary} from './scenes/vrSafetyPanel';
import {resolveTeleopSocketUrl} from './transport/socketUrl';
import {TeleopSocket} from './transport/teleopSocket';
import {ArmPanel} from './ui/armPanel';
import {Hud} from './ui/hud';
import {LatencyTracker} from './ui/latency';
import {DiagnosticsPanel, DiagnosticsUpdateCoordinator} from './ui/diagnosticsPanel';
import {
  OfflineRehearsalPanel,
  type OfflineRehearsalEligibility,
} from './ui/offlineRehearsalPanel';
import {XRSessionController, type XRSessionStatus} from './xr/session';

const app = document.querySelector<HTMLElement>('#app');
if (!app) throw new Error('页面缺少仿真界面容器');

const hud = new Hud(app);
const diagnosticsPanel = new DiagnosticsPanel(hud.diagnosticsContainer);
const stateBuffer = new RobotStateBuffer();
const latency = new LatencyTracker(512);
const pendingFrames = new Map<number, number>();
const diagnosticsUpdates = new DiagnosticsUpdateCoordinator(
  diagnosticsPanel,
  applyDiagnosticsRuntime,
  () => ({currentMs: latency.current, p95Ms: latency.p95()}),
);

let scene: SimulationScene;
let armPanel: ArmPanel;
let xrController: XRSessionController;
let rehearsal: OfflineRehearsalController;
let rehearsalPanel: OfflineRehearsalPanel;
let rehearsalActive = false;
let connectionState: OfflineRehearsalEligibility['connected'] = false;
let latestRobotState: RobotStateMessage | null = null;
let latestDiagnostics: DiagnosticsMessage | null = null;
let xrState: XRSessionStatus['state'] = 'idle';
let vrControlSequence = 0;

const socket = new TeleopSocket(
  resolveTeleopSocketUrl(window.location, import.meta.env.VITE_TELEOP_WS_URL),
  (state) => onRobotState(state),
  undefined,
  (status) => {
    connectionState = status.state === 'connected';
    rehearsal?.onConnection(status);
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
      hud.setRuntimeIdentity(null, null);
      armPanel?.setRuntimeIdentity(null);
      diagnosticsUpdates.clear();
      scene?.setRuntimeSummary(emptyRuntimeSummary());
    }
    refreshRehearsalPanel();
  },
  (message) => {
    armPanel?.handleArmFeedback(message);
    rehearsal?.onFeedback(message);
  },
  (message) => {
    armPanel?.handleFaultResetResult(message);
    rehearsal?.onFeedback(message);
  },
  (message) => {
    armPanel?.handleHomeResult(message);
    rehearsal?.onFeedback(message);
  },
  (message) => onDiagnostics(message),
  (message) => rehearsal?.onFeedback(message),
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
  onFrame: (frame) => sendFrame(frame, 'xr'),
  onController: updateController,
  onArmRequest: () => { armPanel.requestArm('xr'); },
  onStopOrResetRequest: () => { armPanel.requestStopOrReset('xr'); },
  onControllerSupport: (supported) => scene.setQuestControllerSupport(supported),
  onDisarm: sendVRDisarm,
  onLockReset: () => armPanel.resetToLocked(),
  onStatus: updateXRStatus,
});

rehearsalPanel = new OfflineRehearsalPanel(
  hud.rehearsalContainer,
  startRehearsal,
  () => rehearsal.requestStop('operator_stop'),
);
rehearsal = new OfflineRehearsalController({
  nowMs: () => performance.now(),
  sendControl: (message) => socket.sendControl(message),
  sendRehearsal: (message) => socket.sendRehearsal(message),
  closeConnection: () => socket.close(),
  setOfflineController: (sample) => scene.setOfflineController(sample),
  readScene: () => scene.getOfflineSceneSnapshot(),
  configureFakeWorkspace: (limiterAnchor, taskAnchor) => (
    scene.configureFakeRehearsalWorkspace(limiterAnchor, taskAnchor)
  ),
  resetScene: () => scene.resetOfflineScene(),
  onSnapshot: renderRehearsalSnapshot,
});
const removeRehearsalVisibilityStop = installRehearsalVisibilityStop(document, rehearsal);
refreshRehearsalPanel();

scene.start();
socket.connect();

window.addEventListener('pagehide', disposeForPageExit);
window.addEventListener('beforeunload', disposeForPageExit);

function updateController(controller: {tracking: boolean; grip: boolean; trigger: number}): void {
  hud.setController(controller);
  if (controller.tracking) armPanel.observeGrip(controller.grip);
}

function sendVRDisarm(): void {
  if (rehearsalActive) return;
  vrControlSequence += 1;
  socket.sendControl({
    v: PROTOCOL_VERSION,
    type: 'disarm',
    request_id: `vr-disarm-${vrControlSequence}`,
    client_mono_ms: performance.now(),
  });
}

function updateXRStatus(status: XRSessionStatus): void {
  xrState = status.state;
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
  refreshRehearsalPanel();
}

function sendFrame(frame: VRFrame, source: TeleopFrameSource): void {
  forwardTeleopFrame(frame, source, rehearsalActive, sendFrameToSocket);
}

function sendFrameToSocket(frame: VRFrame): void {
  pendingFrames.set(frame.seq, frame.client_mono_ms);
  while (pendingFrames.size > 512) pendingFrames.delete(pendingFrames.keys().next().value!);
  socket.sendFrame(frame);
}

function onRobotState(state: RobotStateMessage): void {
  latestRobotState = structuredClone(state);
  rehearsal.onRobotState(state);
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
  diagnosticsUpdates.onRobotState(state, recordAcknowledgement);
  refreshRehearsalPanel();
}

function onDiagnostics(diagnostics: DiagnosticsMessage): void {
  latestDiagnostics = structuredClone(diagnostics);
  rehearsal.onDiagnostics(diagnostics);
  diagnosticsUpdates.onDiagnostics(diagnostics);
  refreshRehearsalPanel();
}

function renderRehearsalSnapshot(snapshot: OfflineRehearsalSnapshot): void {
  rehearsalActive = snapshot.active;
  scene.setAutomationActive(snapshot.active);
  armPanel.setAutomationActive(snapshot.active);
  rehearsalPanel.update(snapshot, rehearsalEligibility());
}

function refreshRehearsalPanel(): void {
  if (!rehearsal || !rehearsalPanel) return;
  renderRehearsalSnapshot(rehearsal.snapshot);
}

function rehearsalEligibility(): OfflineRehearsalEligibility {
  return {
    connected: connectionState,
    runtime: latestDiagnostics?.runtime ?? null,
    robotRuntime: latestRobotState?.backend ?? null,
    hardwareVerified: latestDiagnostics?.hardware_verified ?? null,
    mode: latestRobotState?.mode ?? null,
    backendState: latestRobotState?.robot_state ?? null,
    fault: latestRobotState?.fault ?? null,
    constraint: latestRobotState?.constraint ?? null,
    actualTcp: latestRobotState?.actual_tcp ?? null,
    xrState,
  };
}

function startRehearsal(): void {
  if (rehearsalEligibility().xrState !== 'idle') {
    refreshRehearsalPanel();
    return;
  }
  void rehearsal.start();
}

function disposeForPageExit(): void {
  window.removeEventListener('pagehide', disposeForPageExit);
  window.removeEventListener('beforeunload', disposeForPageExit);
  removeRehearsalVisibilityStop();
  disposeAppForUnload(rehearsal, xrController, scene, socket);
}

function recordAcknowledgement(sequence: number | null): void {
  if (sequence === null) return;
  const sentAt = pendingFrames.get(sequence);
  if (sentAt !== undefined) latency.add(Math.max(0, performance.now() - sentAt));
  for (const pendingSequence of pendingFrames.keys()) {
    if (pendingSequence <= sequence) pendingFrames.delete(pendingSequence);
  }
  hud.setLatency(latency.current, latency.p95());
}

function applyDiagnosticsRuntime(
  state: RobotStateMessage,
  diagnostics: DiagnosticsMessage | null,
  latencyView: Readonly<{currentMs: number | null; p95Ms: number | null}>,
): void {
  const backend = diagnostics?.runtime ?? state.backend ?? null;
  const realRobotMode = state.real_robot_mode ?? null;
  hud.setRuntimeIdentity(backend, realRobotMode);
  armPanel?.setRuntimeIdentity(backend);
  scene?.setRuntimeSummary({
    backend,
    realRobotMode,
    actualTcp: state.actual_tcp,
    gripper: state.gripper,
    latencyMs: latencyView.currentMs,
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
