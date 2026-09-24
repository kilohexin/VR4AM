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
import {
  type GraspBlock,
  KinematicGraspController,
} from './kinematicGraspController';
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
const GRASP_PLATFORM_TOP_Y = 0.53;
const GRASP_PLATFORM_CENTER_X = -0.17;
const GRASP_PLATFORM_CENTER_Z = -0.12;
const GRASP_PLATFORM_HALF_WIDTH = 0.19;
const GRASP_PLATFORM_HALF_DEPTH = 0.20;

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

export interface DesktopInputSnapshot {
  activePointer: number | null;
  grip: boolean;
  trigger: number;
}

export class DesktopInputSafety {
  private activePointer: number | null = null;
  private grip = false;
  private trigger = 0;
  private attached = false;
  private enabled = true;
  private readonly ownerWindow: Window;
  private readonly ownerDocument: Document;

  constructor(private readonly canvas: HTMLCanvasElement) {
    this.ownerDocument = canvas.ownerDocument;
    this.ownerWindow = this.ownerDocument.defaultView ?? window;
  }

  attach(): void {
    if (this.attached) return;
    this.attached = true;
    this.ownerWindow.addEventListener('blur', this.onInterrupted);
    this.ownerWindow.addEventListener('keydown', this.onKeyDown);
    this.ownerWindow.addEventListener('keyup', this.onKeyUp);
    this.ownerDocument.addEventListener('visibilitychange', this.onVisibilityChange);
    this.canvas.addEventListener('pointerdown', this.onPointerDown);
    this.canvas.addEventListener('pointerup', this.onPointerUp);
    this.canvas.addEventListener('pointercancel', this.onPointerUp);
    this.canvas.addEventListener('lostpointercapture', this.onLostPointerCapture);
  }

  dispose(): void {
    this.reset();
    if (!this.attached) return;
    this.attached = false;
    this.ownerWindow.removeEventListener('blur', this.onInterrupted);
    this.ownerWindow.removeEventListener('keydown', this.onKeyDown);
    this.ownerWindow.removeEventListener('keyup', this.onKeyUp);
    this.ownerDocument.removeEventListener('visibilitychange', this.onVisibilityChange);
    this.canvas.removeEventListener('pointerdown', this.onPointerDown);
    this.canvas.removeEventListener('pointerup', this.onPointerUp);
    this.canvas.removeEventListener('pointercancel', this.onPointerUp);
    this.canvas.removeEventListener('lostpointercapture', this.onLostPointerCapture);
  }

  snapshot(): DesktopInputSnapshot {
    return {activePointer: this.activePointer, grip: this.grip, trigger: this.trigger};
  }

  isActivePointer(pointerId: number): boolean {
    return this.enabled && this.activePointer === pointerId;
  }

  setEnabled(enabled: boolean): void {
    if (this.enabled === enabled) return;
    this.enabled = enabled;
    this.reset();
  }

  reset(): void {
    const pointerId = this.activePointer;
    this.activePointer = null;
    this.grip = false;
    this.trigger = 0;
    if (pointerId === null) return;
    try {
      if (this.canvas.hasPointerCapture(pointerId)) this.canvas.releasePointerCapture(pointerId);
    } catch {
      // Capture may already belong to the browser after blur/visibility transitions.
    }
  }

  private readonly onPointerDown = (event: PointerEvent): void => {
    if (!this.enabled || event.button !== 0) return;
    this.activePointer = event.pointerId;
    this.grip = true;
    try {
      this.canvas.setPointerCapture(event.pointerId);
    } catch {
      this.reset();
    }
  };

  private readonly onPointerUp = (event: PointerEvent): void => {
    if (this.activePointer !== event.pointerId) return;
    this.activePointer = null;
    this.grip = false;
    try {
      if (this.canvas.hasPointerCapture(event.pointerId)) {
        this.canvas.releasePointerCapture(event.pointerId);
      }
    } catch {
      // The release still wins locally even when browser capture has already gone.
    }
  };

  private readonly onLostPointerCapture = (_event: PointerEvent): void => this.reset();

  private readonly onInterrupted = (): void => this.reset();

  private readonly onVisibilityChange = (): void => {
    if (this.ownerDocument.visibilityState !== 'visible') this.reset();
  };

  private readonly onKeyDown = (event: KeyboardEvent): void => {
    if (!this.enabled || event.code !== 'Space') return;
    event.preventDefault();
    this.trigger = 1;
  };

  private readonly onKeyUp = (event: KeyboardEvent): void => {
    if (event.code === 'Space') this.trigger = 0;
  };
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
  private readonly graspBlocks = createGraspBlocks();
  private readonly graspPlatform = createGraspPlatform();
  private readonly rehearsalSupport = createRehearsalSupport();
  private readonly rehearsalPlacementMarker = createRehearsalPlacementMarker();
  private sessionId = createSessionId();
  private robotModel: RobotModel | null = null;
  private graspController: KinematicGraspController | null = null;
  private animationHandle: number | null = null;
  private sequence = 0;
  private lastFrameMs = Number.NEGATIVE_INFINITY;
  private readonly clockAnchor = new ServerClockAnchor();
  private readonly inputSafety: DesktopInputSafety;
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
  private offlineWorkspace: OfflineSceneSnapshot['workspace'] = null;

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
    this.inputSafety = new DesktopInputSafety(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
    this.orbitCamera = new DesktopOrbitCamera(this.renderer.domElement, this.camera);

    this.scene.background = new THREE.Color(0x07131e);
    this.robotVisualRoot.name = 'robot-visual-root';
    this.scene.add(this.robotVisualRoot);
    this.createEnvironment();
    this.robotVisualRoot.add(
      ...this.graspBlocks.map(({object}) => object),
    );
    this.robotVisualRoot.add(this.graspPlatform);
    this.robotVisualRoot.add(this.rehearsalSupport);
    this.robotVisualRoot.add(this.rehearsalPlacementMarker);
    setGraspPropVisibility(
      this.graspBlocks, this.graspPlatform, this.rehearsalSupport,
      this.rehearsalPlacementMarker, null, this.offlineWorkspace,
    );
    this.graspController = new KinematicGraspController({
      visualRoot: this.robotVisualRoot,
      blocks: this.graspBlocks,
      tableTopY: GRASP_PLATFORM_TOP_Y,
      tableCenterX: GRASP_PLATFORM_CENTER_X,
      tableCenterZ: GRASP_PLATFORM_CENTER_Z,
      tableHalfWidth: GRASP_PLATFORM_HALF_WIDTH,
      tableHalfDepth: GRASP_PLATFORM_HALF_DEPTH,
    });
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
    setGraspPropVisibility(
      this.graspBlocks, this.graspPlatform, this.rehearsalSupport,
      this.rehearsalPlacementMarker, state.backend, this.offlineWorkspace,
    );
  }

  resetConnection(): void {
    this.options.stateBuffer.reset();
    this.clockAnchor.reset();
    this.inputSafety.reset();
    setGraspPropVisibility(
      this.graspBlocks, this.graspPlatform, this.rehearsalSupport,
      this.rehearsalPlacementMarker, null, this.offlineWorkspace,
    );
  }

  setOfflineController(sample: OfflineControllerSample | null): void {
    if (sample === null) {
      this.inputSafety.reset();
      this.controllerPosition.fromArray(DESKTOP_CONTROLLER_POSITION);
      this.controllerQuaternion.fromArray(DESKTOP_CONTROLLER_QUATERNION);
      this.offlineController = null;
      return;
    }
    this.inputSafety.reset();
    this.offlineController = copyOfflineControllerSample(sample);
    this.controllerPosition.fromArray(this.offlineController.position);
    this.controllerQuaternion.fromArray(this.offlineController.quaternion);
  }

  setAutomationActive(active: boolean): void {
    if (this.automationActive === active) return;
    this.automationActive = active;
    this.inputSafety.setEnabled(!active);
  }

  getOfflineSceneSnapshot(): OfflineSceneSnapshot {
    const grasp = this.graspController?.snapshot() ?? {
      carriedBlockId: null,
      blocks: this.graspBlocks.map((block) => ({
        id: block.id,
        position: block.object.position.toArray() as Vec3,
        sizeM: block.sizeM,
      })),
      invalidOverlap: false,
    };
    return copyOfflineSceneSnapshot({
      controller: this.offlineController === null
        ? null
        : copyOfflineControllerSample(this.offlineController),
      ...grasp,
      workspace: this.offlineWorkspace,
    });
  }

  configureFakeRehearsalWorkspace(limiterAnchor: Pose, taskAnchor: Pose): void {
    const layout = fakeRehearsalWorkspaceLayout(limiterAnchor, taskAnchor);
    const orange = this.graspBlocks.find(({id}) => id === 'block-orange');
    if (!orange || !this.graspController) throw new Error('offline_workspace_unavailable');
    orange.object.position.fromArray(layout.pickBlockCenter);
    orange.object.quaternion.identity();
    orange.object.updateMatrix();
    orange.object.updateMatrixWorld(true);
    this.rehearsalSupport.visible = true;
    this.rehearsalSupport.position.fromArray(layout.supportCenter);
    this.rehearsalSupport.scale.set(layout.supportWidthM, 0.02, 0.10);
    this.rehearsalPlacementMarker.visible = true;
    this.rehearsalPlacementMarker.position.set(
      layout.placementTarget[0],
      layout.supportTopY + 0.001,
      layout.placementTarget[2],
    );
    this.graspController.setTableTopY(layout.supportTopY);
    this.offlineWorkspace = {
      limiterAnchor: [...limiterAnchor.p],
      taskAnchor: [...taskAnchor.p],
      placementTarget: [...layout.placementTarget],
      liftM: FAKE_REHEARSAL_WORKSPACE.liftM,
    };
  }

  resetOfflineScene(): void {
    this.graspController?.reset();
    this.rehearsalSupport.visible = false;
    this.rehearsalPlacementMarker.visible = false;
    this.offlineWorkspace = null;
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
    this.inputSafety.reset();
    this.vrSafetyPanel.setVisible(false);
    this.controllerHints.setVisible(false);
    await this.renderer.xr.setSession(session);
    this.workspacePlacement.beginSession();
    this.renderer.setAnimationLoop(loop);
    this.controllerHints.setVisible(true);
    this.vrSafetyPanel.setVisible(true);
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
        if (sample.state.backend !== 'LEBAI') {
          this.graspController?.update(
            sample.state.actual_tcp,
            sample.state.gripper,
          );
        }
      }
    }

    this.targetMarker.position.copy(this.controllerPosition);
    this.targetMarker.quaternion.copy(this.controllerQuaternion);
  }

  private sendDesktopFrame(nowMs: number): void {
    if (nowMs - this.lastFrameMs >= FRAME_INTERVAL_MS) {
      this.lastFrameMs = nowMs;
      const offline = this.offlineController;
      const input = offline === null && !this.automationActive ? this.inputSafety.snapshot() : null;
      const frame = createVRFrame({
        sessionId: this.sessionId,
        sequence: this.sequence++,
        nowMs,
        trackingValid: document.visibilityState === 'visible' && (offline?.trackingValid ?? true),
        position: offline?.position ?? this.controllerPosition.toArray() as Vec3,
        quaternion: offline?.quaternion ?? this.controllerQuaternion.toArray() as Quat,
        grip: offline?.grip ?? input?.grip ?? false,
        trigger: offline?.trigger ?? input?.trigger ?? 0,
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
    top.position.set(0, -0.055, 0);
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
    this.robotVisualRoot.add(this.targetMarker);
  }

  private addListeners(): void {
    const canvas = this.renderer.domElement;
    this.inputSafety.attach();
    this.orbitCamera.attach();
    window.addEventListener('resize', this.resizeFromEvent);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('wheel', this.onWheel, {passive: false});
    canvas.addEventListener('contextmenu', this.preventContextMenu);
  }

  private removeListeners(): void {
    const canvas = this.renderer.domElement;
    this.inputSafety.dispose();
    this.orbitCamera.dispose();
    window.removeEventListener('resize', this.resizeFromEvent);
    canvas.removeEventListener('pointermove', this.onPointerMove);
    canvas.removeEventListener('wheel', this.onWheel);
    canvas.removeEventListener('contextmenu', this.preventContextMenu);
  }

  private readonly resizeFromEvent = (): void => this.resize();

  private readonly onPointerMove = (event: PointerEvent): void => this.applyPointerMove(event);

  private applyPointerMove(event: PointerEvent): void {
    if (this.automationActive || this.offlineController !== null) return;
    if (!this.inputSafety.isActivePointer(event.pointerId)) return;
    this.controllerPosition.x = clamp(this.controllerPosition.x + event.movementX * 0.0012, -0.2, 0.8);
    this.controllerPosition.y = clamp(this.controllerPosition.y - event.movementY * 0.0012, 0.05, 1.05);
  }

  private readonly onWheel = (event: WheelEvent): void => this.applyWheel(event);

  private applyWheel(event: WheelEvent): void {
    if (this.automationActive || this.offlineController !== null) return;
    event.preventDefault();
    this.controllerPosition.z = clamp(this.controllerPosition.z + event.deltaY * 0.0008, -0.62, 0.62);
  }

  private readonly preventContextMenu = (event: MouseEvent): void => event.preventDefault();
}

function createSessionId(): string {
  const token = typeof crypto !== 'undefined' && 'randomUUID' in crypto ? crypto.randomUUID() : `${Date.now()}`;
  return `desktop-${token}`;
}

function setGraspPropVisibility(
  blocks: readonly GraspBlock[],
  platform: THREE.Object3D,
  rehearsalSupport: THREE.Object3D,
  rehearsalPlacementMarker: THREE.Object3D,
  backend: RobotStateMessage['backend'] | null,
  offlineWorkspace: OfflineSceneSnapshot['workspace'],
): void {
  const showSimulationProps = backend !== null && backend !== 'LEBAI';
  for (const {object} of blocks) object.visible = showSimulationProps;
  platform.visible = showSimulationProps;
  rehearsalSupport.visible = showSimulationProps && offlineWorkspace !== null;
  rehearsalPlacementMarker.visible = showSimulationProps && offlineWorkspace !== null;
}

export function createGraspBlocks(): GraspBlock[] {
  const definitions = [
    ['block-orange', 0xff8a3d, [-0.05, 0.56, -0.23]],
    ['block-blue', 0x39a8ff, [-0.14, 0.56, -0.26]],
    ['block-green', 0x58d68d, [-0.24, 0.56, -0.25]],
    ['block-yellow', 0xffd84d, [-0.27, 0.56, -0.14]],
    ['block-purple', 0xa77bff, [-0.16, 0.56, -0.10]],
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
    object.position.set(position[0], position[1], position[2]);
    object.castShadow = true;
    object.receiveShadow = true;
    return {id, object, sizeM: 0.06};
  });
}

function createGraspPlatform(): THREE.Mesh {
  const platform = new THREE.Mesh(
    new THREE.BoxGeometry(
      GRASP_PLATFORM_HALF_WIDTH * 2,
      0.03,
      GRASP_PLATFORM_HALF_DEPTH * 2,
    ),
    new THREE.MeshStandardMaterial({color: 0x264656, metalness: 0.18, roughness: 0.72}),
  );
  platform.name = 'grasp-platform';
  platform.position.set(
    GRASP_PLATFORM_CENTER_X,
    GRASP_PLATFORM_TOP_Y - 0.015,
    GRASP_PLATFORM_CENTER_Z,
  );
  platform.receiveShadow = true;
  return platform;
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

function createRehearsalSupport(): THREE.Mesh {
  const support = new THREE.Mesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshStandardMaterial({color: 0x203746, metalness: 0.25, roughness: 0.65}),
  );
  support.name = 'offline-rehearsal-support';
  support.visible = false;
  support.receiveShadow = true;
  return support;
}

function createRehearsalPlacementMarker(): THREE.Mesh {
  const marker = new THREE.Mesh(
    new THREE.RingGeometry(0.038, 0.044, 32),
    new THREE.MeshBasicMaterial({color: 0x58d68d, side: THREE.DoubleSide}),
  );
  marker.name = 'offline-rehearsal-placement-target';
  marker.rotation.x = -Math.PI / 2;
  marker.visible = false;
  return marker;
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
