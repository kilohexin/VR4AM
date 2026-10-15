import * as THREE from 'three';
import type {TeleopFrameSource} from '../appFrameForwarding';
import {
  PROTOCOL_VERSION,
  type Pose,
  type Quat,
  type RobotStateMessage,
  type VisibilityState,
  type VRFrame,
  type Vec3,
} from '../protocol/messages';
import {
  LM3_HOME_Q,
  loadRobotModel,
  type RobotModel,
} from '../robot/robotModel';
import {RobotStateBuffer} from '../robot/robotState';
import type {ArmSafetySnapshot} from '../ui/armPanel';
import type {XRPresentationSample} from '../xr/session';
import {ControllerHints} from './controllerHints';
import {DesktopOrbitCamera} from './desktopOrbitCamera';
import type {GraspBlock} from './kinematicGraspController';
import {WorkspacePlacementController} from './workspacePlacementController';
import {type RobotRuntimeSummary, VrSafetyPanel} from './vrSafetyPanel';
import {
  copyOfflineControllerSample,
  copyOfflineSceneSnapshot,
  FAKE_REHEARSAL_WORKSPACE,
  type OfflineControllerSample,
  type OfflineSceneSnapshot,
} from '../rehearsal/types';

const FRAME_INTERVAL_MS = 1_000 / 60;
const DESKTOP_CONTROLLER_POSITION: Vec3 = [0.56, 0.42, 0.18];
const DESKTOP_CONTROLLER_QUATERNION: Quat = [0, 0, 0, 1];
const DESK_TOP_Y = -0.005;
// The visual worktop, initial block contact and kinematic release bounds share one surface.
export const GRASP_WORKBENCH_SURFACE = {
  tableTopY: 0.53,
  tableCenterX: -0.17,
  tableCenterZ: -0.12,
  tableHalfWidth: 0.19,
  tableHalfDepth: 0.20,
} as const;

export interface VRFrameInput {
  sessionId: string;
  sequence: number;
  nowMs: number;
  trackingValid: boolean;
  position: Vec3;
  quaternion: Quat;
  headQ?: Quat;
  grip: boolean;
  trigger: number;
  visibility?: VisibilityState;
}

export function createVRFrame(input: VRFrameInput): VRFrame {
  const frame: VRFrame = {
    v: PROTOCOL_VERSION,
    type: 'vr_frame',
    session_id: input.sessionId,
    seq: input.sequence,
    client_mono_ms: input.nowMs,
    tracking_valid: input.trackingValid,
    visibility: input.visibility ?? 'visible',
    right: {
      p: [...input.position],
      q: [...input.quaternion],
      grip: input.grip,
      trigger: clamp(input.trigger, 0, 1),
    },
  };
  if (input.headQ) frame.head_q = [...input.headQ];
  return frame;
}

export class ServerClockAnchor {
  private value: {serverNs: number; clientMs: number} | null = null;

  update(serverNs: number, clientMs: number): void {
    this.value = {serverNs, clientMs};
  }

  reset(): void {
    this.value = null;
  }

  estimate(clientMs: number): number | null {
    if (!this.value) return null;
    return this.value.serverNs + (clientMs - this.value.clientMs) * 1_000_000;
  }
}

export interface DesktopControllerState {
  tracking: boolean;
  grip: boolean;
  trigger: number;
}

export interface SimulationSceneOptions {
  stateBuffer: RobotStateBuffer;
  onFrame(frame: VRFrame, source: TeleopFrameSource): void;
  onController(state: DesktopControllerState): void;
  onError(message: string): void;
}

export class SimulationScene {
  readonly renderer: THREE.WebGLRenderer;
  readonly camera: THREE.PerspectiveCamera;

  private readonly scene = new THREE.Scene();
  private readonly robotVisualRoot = new THREE.Group();
  private readonly grid = new THREE.GridHelper(4, 40, 0x1f839f, 0x183245);
  private readonly targetMarker = new THREE.Group();
  private sessionId = createSessionId();
  private robotModel: RobotModel | null = null;
  private animationHandle: number | null = null;
  private sequence = 0;
  private lastFrameMs = Number.NEGATIVE_INFINITY;
  private readonly clockAnchor = new ServerClockAnchor();
  private readonly orbitCamera: DesktopOrbitCamera;
  private readonly controllerPosition = new THREE.Vector3(...DESKTOP_CONTROLLER_POSITION);
  private readonly controllerQuaternion = new THREE.Quaternion(...DESKTOP_CONTROLLER_QUATERNION);
  private offlineController: OfflineControllerSample | null = null;
  private readonly vrSafetyPanel: VrSafetyPanel;
  private readonly controllerHints: ControllerHints;
  private readonly workspacePlacement = new WorkspacePlacementController(window.localStorage);
  private armSafetyState: ArmSafetySnapshot = {
    phase: 'disconnected',
    connected: false,
    connectionState: 'disconnected',
    eligible: false,
    armed: false,
    pending: false,
    mode: 'DISCONNECTED',
    fault: null,
    faultRecoverable: false,
    faultResetPending: false,
    constraint: null,
    recoveryPhase: null,
  };
  private questControllerSupported: boolean | null = null;
  private runtimeSummary: RobotRuntimeSummary = {
    backend: null,
    realRobotMode: null,
    actualTcp: null,
    gripper: null,
    latencyMs: null,
    hardwareVerified: false,
  };
  private started = false;
  private automationActive = false;

  constructor(
    private readonly container: HTMLElement,
    private readonly options: SimulationSceneOptions,
  ) {
    this.renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: 'high-performance'});
    this.renderer.xr.enabled = true;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.1;
    this.renderer.shadowMap.enabled = true;
    this.renderer.domElement.className = 'simulation-canvas';
    this.renderer.domElement.style.touchAction = 'none';

    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
    this.orbitCamera = new DesktopOrbitCamera(this.renderer.domElement, this.camera);

    this.scene.background = new THREE.Color(0x07131e);
    this.robotVisualRoot.name = 'robot-visual-root';
    this.scene.add(this.robotVisualRoot);
    this.createEnvironment();
    this.createTargetMarker();
    this.controllerHints = new ControllerHints(this.scene);
    this.controllerHints.setVisible(false);
    this.vrSafetyPanel = new VrSafetyPanel(this.robotVisualRoot);
    this.vrSafetyPanel.update(this.armSafetyState, this.questControllerSupported, this.runtimeSummary);
  }

  start(): void {
    if (this.started) return;
    this.started = true;
    this.container.append(this.renderer.domElement);
    this.addListeners();
    this.resize();
    void this.loadModel();
    this.animationHandle = requestAnimationFrame(this.animate);
  }

  applyRobotState(state: RobotStateMessage): void {
    this.options.stateBuffer.push(state);
    this.clockAnchor.update(state.server_mono_ns, performance.now());
  }

  resetConnection(): void {
    this.options.stateBuffer.reset();
    this.clockAnchor.reset();
  }

  setOfflineController(sample: OfflineControllerSample | null): void {
    if (sample === null) {
      this.controllerPosition.fromArray(DESKTOP_CONTROLLER_POSITION);
      this.controllerQuaternion.fromArray(DESKTOP_CONTROLLER_QUATERNION);
      this.offlineController = null;
      return;
    }
    this.offlineController = copyOfflineControllerSample(sample);
    this.controllerPosition.fromArray(this.offlineController.position);
    this.controllerQuaternion.fromArray(this.offlineController.quaternion);
  }

  setAutomationActive(active: boolean): void {
    if (this.automationActive === active) return;
    this.automationActive = active;
  }

  getOfflineSceneSnapshot(): OfflineSceneSnapshot {
    return copyOfflineSceneSnapshot({
      controller: this.offlineController === null
        ? null
        : copyOfflineControllerSample(this.offlineController),
      carriedBlockId: null,
      blocks: [],
      invalidOverlap: false,
      workspace: null,
    });
  }

  configureFakeRehearsalWorkspace(_limiterAnchor: Pose, _taskAnchor: Pose): void {
    throw new Error('feature_removed');
  }

  resetOfflineScene(): void {
    // Compatibility for retired rehearsal callers; no grasp props exist.
  }

  setArmSafetyState(snapshot: ArmSafetySnapshot): void {
    this.armSafetyState = snapshot;
    this.vrSafetyPanel.update(snapshot, this.questControllerSupported, this.runtimeSummary);
  }

  setQuestControllerSupport(supported: boolean | null): void {
    this.questControllerSupported = supported;
    this.vrSafetyPanel.update(this.armSafetyState, supported, this.runtimeSummary);
  }

  setRuntimeSummary(summary: RobotRuntimeSummary): void {
    this.runtimeSummary = summary;
    this.vrSafetyPanel.update(this.armSafetyState, this.questControllerSupported, summary);
  }

  resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  resetCameraView(): void {
    this.orbitCamera.reset();
  }

  async startXR(session: XRSession, loop: XRFrameRequestCallback): Promise<void> {
    if (!this.started) throw new Error('Simulation scene is not running');
    if (this.animationHandle !== null) cancelAnimationFrame(this.animationHandle);
    this.animationHandle = null;
    this.vrSafetyPanel.setVisible(false);
    this.controllerHints.setVisible(false);
    await this.renderer.xr.setSession(session);
    this.workspacePlacement.beginSession();
    this.renderer.setAnimationLoop(loop);
    this.controllerHints.setVisible(true);
    this.vrSafetyPanel.setVisible(true);
    this.targetMarker.visible = true;
  }

  async stopXR(): Promise<void> {
    this.renderer.setAnimationLoop(null);
    try {
      await this.renderer.xr.setSession(null);
    } finally {
      this.workspacePlacement.endSession();
      this.robotVisualRoot.position.set(0, 0, 0);
      this.controllerHints.setVisible(false);
      this.vrSafetyPanel.setVisible(false);
      this.targetMarker.visible = false;
      this.sessionId = createSessionId();
      this.sequence = 0;
      this.lastFrameMs = Number.NEGATIVE_INFINITY;
      if (this.started && this.animationHandle === null) {
        this.animationHandle = requestAnimationFrame(this.animate);
      }
    }
  }

  updateXRPresentation(sample: XRPresentationSample, nowMs: number): void {
    this.controllerHints.update(sample.left, sample.right);
    const adjustable = (
      this.armSafetyState.phase === 'locked'
      || this.armSafetyState.phase === 'stopped'
      || this.armSafetyState.phase === 'fault'
      || this.armSafetyState.phase === 'armed'
    )
      && !this.armSafetyState.pending
      && !this.armSafetyState.faultResetPending
      && !sample.right.grip;
    this.robotVisualRoot.position.fromArray(this.workspacePlacement.update({
      headY: sample.headY,
      axisX: sample.left.thumbstickX,
      axisY: sample.left.thumbstickY,
      heightModifier: sample.left.grip,
      resetPressed: sample.left.thumbstickPressed,
      enabled: adjustable && sample.left.trackingValid,
      nowMs,
    }));
  }

  renderXR(nowMs: number): void {
    if (!this.started) return;
    this.updateScene(nowMs);
    this.renderer.render(this.scene, this.camera);
  }

  dispose(): void {
    if (!this.started) return;
    this.started = false;
    if (this.animationHandle !== null) cancelAnimationFrame(this.animationHandle);
    this.animationHandle = null;
    this.renderer.setAnimationLoop(null);
    this.removeListeners();
    this.vrSafetyPanel.setVisible(false);
    this.controllerHints.setVisible(false);
    this.controllerHints.dispose();
    this.vrSafetyPanel.dispose();
    disposeObjectResources(this.scene);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private readonly animate = (nowMs: number): void => {
    if (!this.started) return;
    this.animationHandle = requestAnimationFrame(this.animate);

    this.updateScene(nowMs);
    this.sendDesktopFrame(nowMs);
    this.renderer.render(this.scene, this.camera);
  };

  private updateScene(nowMs: number): void {
    const nowNs = this.clockAnchor.estimate(nowMs);
    if (this.robotModel && nowNs !== null) {
      const sample = this.options.stateBuffer.sample(nowNs);
      if (sample) {
        this.robotModel.setJointAngles(sample.state.actual_q, sample.state.backend);
        this.robotModel.setGripper(sample.state.gripper);
        this.robotVisualRoot.updateMatrixWorld(true);
      }
    }

    this.targetMarker.position.copy(this.controllerPosition);
    this.targetMarker.quaternion.copy(this.controllerQuaternion);
  }

  private sendDesktopFrame(nowMs: number): void {
    if (nowMs - this.lastFrameMs >= FRAME_INTERVAL_MS) {
      this.lastFrameMs = nowMs;
      const offline = this.offlineController;
      const frame = createVRFrame({
        sessionId: this.sessionId,
        sequence: this.sequence++,
        nowMs,
        trackingValid: document.visibilityState === 'visible' && (offline?.trackingValid ?? true),
        position: offline?.position ?? this.controllerPosition.toArray() as Vec3,
        quaternion: offline?.quaternion ?? this.controllerQuaternion.toArray() as Quat,
        grip: offline?.grip ?? false,
        trigger: offline?.trigger ?? 0,
      });
      this.options.onController({
        tracking: frame.tracking_valid,
        grip: frame.right.grip,
        trigger: frame.right.trigger,
      });
      this.options.onFrame(frame, this.automationActive ? 'offline_rehearsal' : 'manual');
    }
  }

  private async loadModel(): Promise<void> {
    try {
      const model = await loadRobotModel();
      if (!this.started) {
        disposeObjectResources(model.group);
        return;
      }
      this.robotModel = model;
      model.setJointAngles(LM3_HOME_Q);
      model.group.scale.setScalar(1);
      model.group.position.set(0, 0, 0);
      this.robotVisualRoot.add(model.group);
    } catch (error) {
      if (this.started) this.options.onError(modelLoadErrorMessage(error));
    }
  }

  private createEnvironment(): void {
    this.scene.add(new THREE.HemisphereLight(0x9edcff, 0x061018, 1.45));
    const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
    keyLight.position.set(1.4, 2.4, 1.2);
    keyLight.castShadow = true;
    this.scene.add(keyLight);
    const rimLight = new THREE.DirectionalLight(0x29cfff, 0.45);
    rimLight.position.set(-1.5, 1.2, -1.5);
    this.scene.add(rimLight);

    this.grid.position.y = -0.115;
    this.scene.add(this.grid);

    const tableMaterial = new THREE.MeshStandardMaterial({color: 0x111923, metalness: 0.35, roughness: 0.78});
    const edgeMaterial = new THREE.MeshStandardMaterial({color: 0x080d12, metalness: 0.9, roughness: 0.35});
    const top = new THREE.Mesh(new THREE.BoxGeometry(1.22, 0.1, 0.86), tableMaterial);
    top.name = 'desk-top';
    top.position.set(0, DESK_TOP_Y - 0.05, 0);
    top.receiveShadow = true;
    this.robotVisualRoot.add(top);
    for (const [x, z] of [[-0.52, -0.34], [0.52, -0.34], [-0.52, 0.34], [0.52, 0.34]] as const) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.82, 0.07), edgeMaterial);
      leg.position.set(x, -0.49, z);
      leg.castShadow = true;
      this.robotVisualRoot.add(leg);
    }

    const workspaceGeometry = new THREE.EdgesGeometry(new THREE.BoxGeometry(1.4, 1.05, 1.02));
    const workspaceMaterial = new THREE.LineDashedMaterial({
      color: 0x32d7ff,
      transparent: true,
      opacity: 0.55,
      dashSize: 0.035,
      gapSize: 0.025,
    });
    const workspace = new THREE.LineSegments(workspaceGeometry, workspaceMaterial);
    workspace.position.set(0, 0.5, 0);
    workspace.computeLineDistances();
    this.robotVisualRoot.add(workspace);
  }

  private createTargetMarker(): void {
    const axes = new THREE.AxesHelper(0.17);
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.075, 0.08, 48),
      new THREE.MeshBasicMaterial({color: 0x35d7ff, transparent: true, opacity: 0.78, side: THREE.DoubleSide}),
    );
    ring.rotation.x = -Math.PI / 2;
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 20, 12),
      new THREE.MeshBasicMaterial({color: 0x42dcff, transparent: true, opacity: 0.9}),
    );
    this.targetMarker.add(axes, ring, marker);
    this.targetMarker.position.copy(this.controllerPosition);
    this.targetMarker.visible = false;
    this.robotVisualRoot.add(this.targetMarker);
  }

  private addListeners(): void {
    const canvas = this.renderer.domElement;
    this.orbitCamera.attach();
    window.addEventListener('resize', this.resizeFromEvent);
    canvas.addEventListener('contextmenu', this.preventContextMenu);
  }

  private removeListeners(): void {
    const canvas = this.renderer.domElement;
    this.orbitCamera.dispose();
    window.removeEventListener('resize', this.resizeFromEvent);
    canvas.removeEventListener('contextmenu', this.preventContextMenu);
  }

  private readonly resizeFromEvent = (): void => this.resize();

  private readonly preventContextMenu = (event: MouseEvent): void => event.preventDefault();
}

function createSessionId(): string {
  const token = typeof crypto !== 'undefined' && 'randomUUID' in crypto ? crypto.randomUUID() : `${Date.now()}`;
  return `desktop-${token}`;
}

// Legacy geometry helpers remain for historical isolated algorithm tests only.
// The live SimulationScene never constructs these props or animated grasping.
export function createGraspBlocks(): GraspBlock[] {
  const definitions = [
    ['block-orange', 0xff8a3d, [-0.05, -0.23]],
    ['block-blue', 0x39a8ff, [-0.14, -0.26]],
    ['block-green', 0x58d68d, [-0.24, -0.25]],
    ['block-yellow', 0xffd84d, [-0.27, -0.14]],
    ['block-purple', 0xa77bff, [-0.16, -0.10]],
  ] as const;
  return definitions.map(([id, color, position]) => {
    const object = new THREE.Mesh(
      new THREE.BoxGeometry(0.06, 0.06, 0.06),
      new THREE.MeshStandardMaterial({
        color,
        metalness: 0.05,
        roughness: 0.58,
      }),
    );
    object.name = id;
    object.position.set(position[0], GRASP_WORKBENCH_SURFACE.tableTopY + 0.03, position[1]);
    object.castShadow = true;
    object.receiveShadow = true;
    return {id, object, sizeM: 0.06};
  });
}

export function createGraspWorkbench(): THREE.Group {
  const {tableTopY, tableCenterX, tableCenterZ, tableHalfWidth, tableHalfDepth} = GRASP_WORKBENCH_SURFACE;
  const workbench = new THREE.Group();
  workbench.name = 'grasp-workbench';
  const topThickness = 0.03;
  const undersideY = tableTopY - topThickness;
  const postHeight = undersideY - DESK_TOP_Y;
  const frameMaterial = new THREE.MeshStandardMaterial({color: 0x627987, metalness: 0.55, roughness: 0.52});
  const top = new THREE.Mesh(
    new THREE.BoxGeometry(tableHalfWidth * 2, topThickness, tableHalfDepth * 2),
    new THREE.MeshStandardMaterial({color: 0x264656, metalness: 0.18, roughness: 0.72}),
  );
  top.name = 'workbench-top';
  top.position.set(tableCenterX, tableTopY - topThickness / 2, tableCenterZ);
  top.castShadow = true;
  top.receiveShadow = true;
  workbench.add(top);

  const insetX = tableHalfWidth - 0.025;
  const insetZ = tableHalfDepth - 0.025;
  for (const x of [-insetX, insetX]) {
    for (const z of [-insetZ, insetZ]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.04, postHeight, 0.04), frameMaterial);
      post.name = `workbench-post-${x}-${z}`;
      post.position.set(tableCenterX + x, DESK_TOP_Y + postHeight / 2, tableCenterZ + z);
      post.castShadow = true;
      post.receiveShadow = true;
      workbench.add(post);
    }
    const stretcher = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.03, insetZ * 2), frameMaterial);
    stretcher.name = `workbench-stretcher-${x}`;
    stretcher.position.set(tableCenterX + x, DESK_TOP_Y + 0.09, tableCenterZ);
    stretcher.castShadow = true;
    workbench.add(stretcher);
  }
  return workbench;
}

export function fakeRehearsalWorkspaceLayout(
  limiterAnchor: Pose,
  taskAnchor: Pose,
): {
  pickBlockCenter: Vec3;
  placementTarget: Vec3;
  supportCenter: Vec3;
  supportTopY: number;
  supportWidthM: number;
} {
  const add = (offset: Vec3): Vec3 => [
    limiterAnchor.p[0] + offset[0],
    limiterAnchor.p[1] + offset[1],
    limiterAnchor.p[2] + offset[2],
  ];
  const pickBlockCenter = add(FAKE_REHEARSAL_WORKSPACE.pickBlockCenterFromHomeM);
  const placementTarget = add(FAKE_REHEARSAL_WORKSPACE.placeBlockCenterFromHomeM);
  if (![...limiterAnchor.p, ...taskAnchor.p].every(Number.isFinite)) {
    throw new Error('invalid_offline_workspace_anchor');
  }
  const supportTopY = pickBlockCenter[1] - FAKE_REHEARSAL_WORKSPACE.blockSizeM / 2;
  return {
    pickBlockCenter,
    placementTarget,
    supportCenter: [
      (pickBlockCenter[0] + placementTarget[0]) / 2,
      supportTopY - 0.01,
      (pickBlockCenter[2] + placementTarget[2]) / 2,
    ],
    supportTopY,
    supportWidthM: Math.abs(pickBlockCenter[0] - placementTarget[0]) + 0.10,
  };
}

export function modelLoadErrorMessage(_error: unknown): string {
  return 'LM3 模型加载失败，请检查仿真资源。';
}

export function disposeObjectResources(root: THREE.Object3D): void {
  const disposedTextures = new Set<THREE.Texture>();
  root.traverse((object) => {
    if (!(object instanceof THREE.Mesh) && !(object instanceof THREE.Line)) return;
    object.geometry.dispose();
    const materials = Array.isArray(object.material) ? object.material : [object.material];
    materials.forEach((material) => {
      for (const value of Object.values(material)) {
        if (value instanceof THREE.Texture && !disposedTextures.has(value)) {
          disposedTextures.add(value);
          value.dispose();
        }
      }
      material.dispose();
    });
  });
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}
