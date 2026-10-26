import './styles.css';
import {type TeleopFrameSource} from './appFrameForwarding';
import {
  PROTOCOL_VERSION,
  type DiagnosticsMessage,
  type RobotStateMessage,
  type VRFrame,
  type ClientControlMessage,
} from './protocol/messages';
import {RobotStateBuffer} from './robot/robotState';
import {SimulationScene} from './scenes/simulationScene';
import type {RobotRuntimeSummary} from './scenes/vrSafetyPanel';
import {resolveTeleopSocketUrl} from './transport/socketUrl';
import {TeleopSocket} from './transport/teleopSocket';
import {ArmPanel} from './ui/armPanel';
import {Hud} from './ui/hud';
import {HoldControlPanel} from './ui/holdControlPanel';
import {FinitePositionInput} from './ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame} from './xr/finiteInput';
import {SimulationSettingsPanel} from './ui/simulationSettingsPanel';
import {LatencyTracker} from './ui/latency';
import {DiagnosticsPanel, DiagnosticsUpdateCoordinator} from './ui/diagnosticsPanel';
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
let settingsPanel: SimulationSettingsPanel;
let connectionState = false;
let latestRobotState: RobotStateMessage | null = null;
let latestDiagnostics: DiagnosticsMessage | null = null;
let xrState: XRSessionStatus['state'] = 'idle';
let vrControlSequence = 0;
let scaleRequestSequence = 0;
let pendingScaleRequestId: string | null = null;
let latestGrip = false;
let preferredScaleAttempted = false;
let holdPanel:HoldControlPanel;
let lastManualFrame:VRFrame|null=null;
let desktopSequence=0;
let lastRobotStateReceivedMs=Number.NEGATIVE_INFINITY;
let positionGeneration:number|null=null;
let positionStopSuppressed=false;

const socket = new TeleopSocket(
  resolveTeleopSocketUrl(window.location, import.meta.env.VITE_TELEOP_WS_URL),
  (state) => onRobotState(state),
  undefined,
  (status) => {
    connectionState = status.state === 'connected';
    scene?.resetConnection();
    pendingFrames.clear();
    latency.reset();
    hud.setConnectionStatus(status);
    hud.setLatency(null, null);
    armPanel?.setConnectionStatus(status);
    xrController?.setConstraint(null);
    if (status.state !== 'connected') {
      finiteXR?.reset('xr_disconnected');
      positionGeneration=null;
      latestGrip = false;
      pendingScaleRequestId = null;
      latestRobotState = null;
      latestDiagnostics = null;
      preferredScaleAttempted = false;
      hud.setRuntimeIdentity(null, null);
      armPanel?.setRuntimeIdentity(null);
      diagnosticsUpdates.clear();
      scene?.setRuntimeSummary(emptyRuntimeSummary());
    }
    refreshHoldPanel();
  },
  (message) => {
    armPanel?.handleArmFeedback(message);
  },
  (message) => {
    armPanel?.handleFaultResetResult(message);
  },
  (message) => {
    armPanel?.handleHomeResult(message);
  },
  (message) => onDiagnostics(message),
  undefined,
  (message) => {
    if (message.request_id !== pendingScaleRequestId) return;
    pendingScaleRequestId = null;
    if (latestRobotState) {
      latestRobotState = {...latestRobotState, translation_scale: message.translation_scale};
    }
    settingsPanel?.handleResult(message);
    refreshSettingsPanel();
  },
  (message)=>{holdPanel?.onGripperResult(message);finiteXR?.onGripperResult(message);},
  (message)=>armPanel?.handleDisarmAck(message),
);

const positionInput=new FinitePositionInput(message=>socket.sendPositionInput(message),(reason)=>{
 if(!positionStopSuppressed&&connectionState)sendControl({v:1,type:'disarm',request_id:`finite-abort-${reason??'cancel'}-${Date.now()}`});
});
const finiteXR=new FiniteXRInput(positionInput,m=>socket.sendGripper(m),id=>socket.cancelGripper(id));

armPanel = new ArmPanel(
  hud.actionContainer,
  sendControl,
  () => void (xrController.isActive ? xrController.exitVR() : xrController.enterVR()),
  (snapshot) => { scene?.setArmSafetyState(snapshot); refreshHoldPanel(); },
);
armPanel.setConnectionStatus({state: 'disconnected'});
holdPanel=new HoldControlPanel(hud.desktopControlContainer,(message)=>socket.sendGripper(message),
 ()=>performance.now(),()=>{
  if(connectionState&&lastManualFrame&&xrState!=='active')
   sendFrame({...lastManualFrame,client_mono_ms:performance.now(),
    visibility:document.visibilityState==='visible'?'visible':'hidden'},'manual');
 },requestId=>socket.cancelGripper(requestId),(vector,event,reason)=>{
  if(xrState!=='idle')return; // A disabled desktop panel cannot revoke the XR owner's gesture.
  if(event==='held')positionInput.offer(vector);
  else if(event==='release')positionInput.release();
  else positionInput.abort(reason);
 });

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
  onArmRequest: () => { armPanel.requestXRPrimaryAction(); },
  onStopOrResetRequest: () => { armPanel.requestStopOrReset('xr'); },
  onControllerSupport: (supported) => scene.setQuestControllerSupport(supported),
  onDisarm: sendVRDisarm,
  onLockReset: () => armPanel.resetToLocked(),
  onStatus: updateXRStatus,
});

settingsPanel = new SimulationSettingsPanel(
  hud.settingsContainer,
  requestSimulationScale,
  () => scene.resetCameraView(),
);

scene.start();
socket.connect();

window.addEventListener('pagehide', disposeForPageExit);
window.addEventListener('beforeunload', disposeForPageExit);

function updateController(controller: {tracking: boolean; grip: boolean; trigger: number}): void {
  latestGrip = controller.grip;
  hud.setController(controller);
  if (controller.tracking) armPanel.observeGrip(controller.grip);
  refreshSettingsPanel();
}

function sendVRDisarm(): void {
  vrControlSequence += 1;
  sendControl({
    v: PROTOCOL_VERSION,
    type: 'disarm',
    request_id: `vr-disarm-${vrControlSequence}`,
    client_mono_ms: performance.now(),
  });
}

function sendControl(message:ClientControlMessage):void {
  if(message.type!=='disarm'){socket.sendControl(message);return;}
  positionStopSuppressed=true;
  try {
    armPanel?.noteStopRequested(message.request_id);
    socket.sendControl(message);
    holdPanel?.cancel();
    finiteXR?.reset('operator_stop');
  } finally {positionStopSuppressed=false;}
}

function updateXRStatus(status: XRSessionStatus): void {
  if(status.state!==xrState){
    finiteXR.reset('xr_source_changed');
    positionInput.reset(`${status.state==='active'?'xr':'web'}-finite-${crypto.randomUUID()}`,positionGeneration??0);
  }
  xrState = status.state;
  hud.setXRInputState(status.state);
  refreshHoldPanel();
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
  refreshSettingsPanel();
  maybeRestorePreferredScale();
}

function sendFrame(frame: VRFrame, source: TeleopFrameSource): void {
  if(source==='offline_rehearsal')return;
  if(source==='xr'&&latestRobotState?.finite_position){
    finiteXR.apply(frame);
    frame=finiteXRTransportFrame(frame);
  }
  if(source==='manual'&&holdPanel){
   lastManualFrame=frame;
   frame=holdPanel.applyFrame(frame);
   frame={...frame,seq:desktopSequence++};
   updateController({tracking:frame.tracking_valid,grip:frame.right.grip,trigger:frame.right.trigger});
  }
  sendFrameToSocket(frame);
}

function sendFrameToSocket(frame: VRFrame): void {
  pendingFrames.set(frame.seq, frame.client_mono_ms);
  while (pendingFrames.size > 512) pendingFrames.delete(pendingFrames.keys().next().value!);
  socket.sendFrame(frame);
}

function onRobotState(state: RobotStateMessage): void {
  if(state.finite_position&&state.finite_position.generation!==positionGeneration){
    positionGeneration=state.finite_position.generation;
    positionInput.reset(`${xrState==='active'?'xr':'web'}-finite-${crypto.randomUUID()}`,positionGeneration);
  }
  lastRobotStateReceivedMs=performance.now();
  if(state.finite_position)positionInput.observe(state.finite_position.phase);
  latestRobotState = structuredClone(state);
  finiteXR.update(state,lastRobotStateReceivedMs);
  armPanel.setVRAllowed(!state.finite_position || state.finite_position.vr_enabled===true);
  const constraint = state.constraint ?? null;
  const recoveryPhase = state.recovery_phase ?? null;
  scene.applyRobotState(state);
  armPanel.setFault(state.fault ?? null);
  armPanel.setConstraint(constraint);
  armPanel.setRecoveryPhase(recoveryPhase);
  armPanel.setTargetControl(state.target_control ?? null);
  armPanel.setPreparationState(state.preparation_required ?? false, state.preparation_unlocked ?? false);
  armPanel.setMode(state.mode);
  xrController.setConstraint(constraint);
  hud.setRobotState({
    mode: state.mode,
    backendState: state.robot_state,
    sampleAgeMs: state.sample_age_ms ?? null,
    fault: state.fault ?? null,
    constraint,
    recoveryPhase,
    targetControl: state.target_control ?? null,
  });
  diagnosticsUpdates.onRobotState(state, recordAcknowledgement);
  refreshSettingsPanel();
  refreshHoldPanel();
}

function onDiagnostics(diagnostics: DiagnosticsMessage): void {
  latestDiagnostics = structuredClone(diagnostics);
  diagnosticsUpdates.onDiagnostics(diagnostics);
}

function refreshSettingsPanel(): void {
  if (!settingsPanel) return;
  settingsPanel.update({
    runtime: latestRobotState?.backend ?? latestDiagnostics?.runtime ?? null,
    connected: connectionState,
    mode: latestRobotState?.mode ?? null,
    backendState: latestRobotState?.robot_state ?? null,
    grip: latestGrip,
    automationActive: false,
    authoritativeScale: latestRobotState?.translation_scale ?? null,
  });
}

function requestSimulationScale(value: number): void {
  scaleRequestSequence += 1;
  pendingScaleRequestId = `simulation-scale-${scaleRequestSequence}`;
  socket.sendSimulationScale({
    v: PROTOCOL_VERSION,
    type: 'set_simulation_scale',
    request_id: pendingScaleRequestId,
    translation_scale: value,
  });
}

function maybeRestorePreferredScale(): void {
  if (preferredScaleAttempted || !latestRobotState) return;
  const preferred = settingsPanel.preferredScale();
  const stopped =
    connectionState &&
    latestRobotState.backend === 'LEBAI_FAKE' &&
    latestRobotState.robot_state === 'IDLE' &&
    ['READY', 'HOLD', 'DISARMED', 'ARMED'].includes(latestRobotState.mode) &&
    !latestGrip;
  if (!stopped || preferred === null || latestRobotState.translation_scale == null) return;
  preferredScaleAttempted = true;
  if (Math.abs(preferred - latestRobotState.translation_scale) > 1e-9) {
    requestSimulationScale(preferred);
  }
}

function disposeForPageExit(): void {
  window.removeEventListener('pagehide', disposeForPageExit);
  window.removeEventListener('beforeunload', disposeForPageExit);
  holdPanel.dispose();
  void xrController.dispose();
  scene.dispose();
  socket.close();
}

function refreshHoldPanel():void{
 holdPanel?.update(latestRobotState,connectionState&&xrState==='idle'&&Boolean(armPanel?.isArmed),lastRobotStateReceivedMs);
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
    targetControl: state.target_control ?? null,
    finitePosition: state.finite_position ?? null,
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
