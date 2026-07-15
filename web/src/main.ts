import './styles.css';
import type {RobotStateMessage, VRFrame} from './protocol/messages';
import {RobotStateBuffer} from './robot/robotState';
import {SimulationScene} from './scenes/simulationScene';
import {TeleopSocket} from './transport/teleopSocket';
import {ArmPanel} from './ui/armPanel';
import {Hud} from './ui/hud';
import {LatencyTracker} from './ui/latency';

const app = document.querySelector<HTMLElement>('#app');
if (!app) throw new Error('页面缺少仿真界面容器');

const hud = new Hud(app);
const stateBuffer = new RobotStateBuffer();
const latency = new LatencyTracker(512);
const pendingFrames = new Map<number, number>();

let scene: SimulationScene;
let armPanel: ArmPanel;

const socket = new TeleopSocket(
  resolveSocketUrl(),
  (state) => onRobotState(state),
  undefined,
  (connected) => {
    scene?.resetConnection();
    pendingFrames.clear();
    latency.reset();
    hud.setConnection(connected);
    hud.setLatency(null, null);
    armPanel?.setConnected(connected);
  },
  (message) => armPanel?.handleArmFeedback(message),
);

armPanel = new ArmPanel(hud.actionContainer, (message) => socket.sendControl(message), () => {
  const label = armPanel.vrButton.querySelector('span');
  if (!label) return;
  label.textContent = 'VR 暂不可用';
  armPanel.vrButton.dataset.unavailable = 'true';
  window.setTimeout(() => {
    label.textContent = '进入 VR';
    delete armPanel.vrButton.dataset.unavailable;
  }, 2_000);
});
armPanel.setConnected(false);

scene = new SimulationScene(hud.sceneContainer, {
  stateBuffer,
  onFrame: sendFrame,
  onController: (controller) => {
    hud.setController(controller);
    if (controller.tracking) armPanel.observeGrip(controller.grip);
  },
  onError: (message) => hud.showSceneError(message),
});

scene.start();
socket.connect();

window.addEventListener('beforeunload', () => {
  scene.dispose();
  socket.close();
});

function sendFrame(frame: VRFrame): void {
  pendingFrames.set(frame.seq, frame.client_mono_ms);
  while (pendingFrames.size > 512) pendingFrames.delete(pendingFrames.keys().next().value!);
  socket.sendFrame(frame);
}

function onRobotState(state: RobotStateMessage): void {
  scene.applyRobotState(state);
  armPanel.setFault(state.fault ?? null);
  armPanel.setMode(state.mode);
  hud.setRobotState({
    mode: state.mode,
    backendState: state.robot_state,
    sampleAgeMs: state.sample_age_ms ?? null,
    fault: state.fault ?? null,
  });
  recordAcknowledgement(state.ack_seq ?? null);
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

function resolveSocketUrl(): string {
  const configured = import.meta.env.VITE_TELEOP_WS_URL;
  if (configured) return configured;
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${protocol}//${window.location.hostname}:8000/ws/v1/teleop`;
}
