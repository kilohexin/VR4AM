import * as THREE from 'three';
import {
  PROTOCOL_VERSION,
  type Quat,
  type RobotStateMessage,
  type VisibilityState,
  type VRFrame,
  type Vec3,
} from '../protocol/messages';
import {loadRobotModel, type RobotModel} from '../robot/robotModel';
import {RobotStateBuffer} from '../robot/robotState';
import type {ArmSafetySnapshot} from '../ui/armPanel';
import {VrSafetyPanel} from './vrSafetyPanel';

const FRAME_INTERVAL_MS = 1_000 / 60;

export interface VRFrameInput {
  sessionId: string;
  sequence: number;
  nowMs: number;
  trackingValid: boolean;
  position: Vec3;
  quaternion: Quat;
  grip: boolean;
  trigger: number;
  visibility?: VisibilityState;
}

export function createVRFrame(input: VRFrameInput): VRFrame {
  return {
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
    return this.activePointer === pointerId;
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
    if (event.button !== 0) return;
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
    if (event.code !== 'Space') return;
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
  onFrame(frame: VRFrame): void;
  onController(state: DesktopControllerState): void;
  onError(message: string): void;
}

export class SimulationScene {
  readonly renderer: THREE.WebGLRenderer;
  readonly camera: THREE.PerspectiveCamera;

  private readonly scene = new THREE.Scene();
  private readonly targetMarker = new THREE.Group();
  private sessionId = createSessionId();
  private robotModel: RobotModel | null = null;
  private animationHandle: number | null = null;
  private sequence = 0;
  private lastFrameMs = Number.NEGATIVE_INFINITY;
  private readonly clockAnchor = new ServerClockAnchor();
  private readonly inputSafety: DesktopInputSafety;
  private readonly controllerPosition = new THREE.Vector3(0.56, 0.42, 0.18);
  private readonly controllerQuaternion = new THREE.Quaternion(0, 0, 0, 1);
  private readonly vrSafetyPanel: VrSafetyPanel;
  private armSafetyState: ArmSafetySnapshot = {
    phase: 'disconnected',
    connected: false,
    connectionState: 'disconnected',
    eligible: false,
    armed: false,
    pending: false,
    mode: 'DISCONNECTED',
    fault: null,
  };
  private questControllerSupported: boolean | null = null;
  private started = false;

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
    this.camera.position.set(1.55, 1.06, 1.9);
    this.camera.lookAt(0.03, 0.42, 0);

    this.scene.background = new THREE.Color(0x07131e);
    this.createEnvironment();
    this.createTargetMarker();
    this.vrSafetyPanel = new VrSafetyPanel(this.scene);
    this.vrSafetyPanel.update(this.armSafetyState, this.questControllerSupported);
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
    this.inputSafety.reset();
  }

  setArmSafetyState(snapshot: ArmSafetySnapshot): void {
    this.armSafetyState = snapshot;
    this.vrSafetyPanel.update(snapshot, this.questControllerSupported);
  }

  setQuestControllerSupport(supported: boolean | null): void {
    this.questControllerSupported = supported;
    this.vrSafetyPanel.update(this.armSafetyState, supported);
  }

  resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  async startXR(session: XRSession, loop: XRFrameRequestCallback): Promise<void> {
    if (!this.started) throw new Error('Simulation scene is not running');
    if (this.animationHandle !== null) cancelAnimationFrame(this.animationHandle);
    this.animationHandle = null;
    this.inputSafety.reset();
    this.vrSafetyPanel.setVisible(false);
    await this.renderer.xr.setSession(session);
    this.renderer.setAnimationLoop(loop);
    this.vrSafetyPanel.setVisible(true);
  }

  async stopXR(): Promise<void> {
    this.renderer.setAnimationLoop(null);
    try {
      await this.renderer.xr.setSession(null);
    } finally {
      this.vrSafetyPanel.setVisible(false);
      this.sessionId = createSessionId();
      this.sequence = 0;
      this.lastFrameMs = Number.NEGATIVE_INFINITY;
      if (this.started && this.animationHandle === null) {
        this.animationHandle = requestAnimationFrame(this.animate);
      }
    }
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
        this.robotModel.setJointAngles(sample.state.actual_q);
        this.robotModel.setGripper(sample.state.gripper);
      }
    }

    this.targetMarker.position.copy(this.controllerPosition);
    this.targetMarker.quaternion.copy(this.controllerQuaternion);
  }

  private sendDesktopFrame(nowMs: number): void {
    if (nowMs - this.lastFrameMs >= FRAME_INTERVAL_MS) {
      this.lastFrameMs = nowMs;
      const input = this.inputSafety.snapshot();
      const frame = createVRFrame({
        sessionId: this.sessionId,
        sequence: this.sequence++,
        nowMs,
        trackingValid: document.visibilityState === 'visible',
        position: this.controllerPosition.toArray() as Vec3,
        quaternion: this.controllerQuaternion.toArray() as Quat,
        grip: input.grip,
        trigger: input.trigger,
      });
      this.options.onController({
        tracking: frame.tracking_valid,
        grip: frame.right.grip,
        trigger: frame.right.trigger,
      });
      this.options.onFrame(frame);
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
      model.setJointAngles([0.2, -0.8, -0.8, -0.4, 0.6, 0]);
      fitRobotToWorkbench(model.group);
      this.scene.add(model.group);
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

    const grid = new THREE.GridHelper(4, 40, 0x1f839f, 0x183245);
    grid.position.y = -0.115;
    this.scene.add(grid);

    const tableMaterial = new THREE.MeshStandardMaterial({color: 0x111923, metalness: 0.35, roughness: 0.78});
    const edgeMaterial = new THREE.MeshStandardMaterial({color: 0x080d12, metalness: 0.9, roughness: 0.35});
    const top = new THREE.Mesh(new THREE.BoxGeometry(1.22, 0.1, 0.86), tableMaterial);
    top.position.set(0, -0.055, 0);
    top.receiveShadow = true;
    this.scene.add(top);
    for (const [x, z] of [[-0.52, -0.34], [0.52, -0.34], [-0.52, 0.34], [0.52, 0.34]] as const) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.82, 0.07), edgeMaterial);
      leg.position.set(x, -0.49, z);
      leg.castShadow = true;
      this.scene.add(leg);
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
    this.scene.add(workspace);
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
    this.scene.add(this.targetMarker);
  }

  private addListeners(): void {
    const canvas = this.renderer.domElement;
    this.inputSafety.attach();
    window.addEventListener('resize', this.resizeFromEvent);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('wheel', this.onWheel, {passive: false});
    canvas.addEventListener('contextmenu', this.preventContextMenu);
  }

  private removeListeners(): void {
    const canvas = this.renderer.domElement;
    this.inputSafety.dispose();
    window.removeEventListener('resize', this.resizeFromEvent);
    canvas.removeEventListener('pointermove', this.onPointerMove);
    canvas.removeEventListener('wheel', this.onWheel);
    canvas.removeEventListener('contextmenu', this.preventContextMenu);
  }

  private readonly resizeFromEvent = (): void => this.resize();

  private readonly onPointerMove = (event: PointerEvent): void => {
    if (!this.inputSafety.isActivePointer(event.pointerId)) return;
    this.controllerPosition.x = clamp(this.controllerPosition.x + event.movementX * 0.0012, -0.2, 0.8);
    this.controllerPosition.y = clamp(this.controllerPosition.y - event.movementY * 0.0012, 0.05, 1.05);
  };

  private readonly onWheel = (event: WheelEvent): void => {
    event.preventDefault();
    this.controllerPosition.z = clamp(this.controllerPosition.z + event.deltaY * 0.0008, -0.62, 0.62);
  };

  private readonly preventContextMenu = (event: MouseEvent): void => event.preventDefault();
}

function fitRobotToWorkbench(group: THREE.Group): void {
  group.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(group);
  const size = bounds.getSize(new THREE.Vector3());
  const scale = size.y > 0 ? 0.68 / size.y : 1;
  group.scale.setScalar(scale);
  group.updateMatrixWorld(true);
  const fitted = new THREE.Box3().setFromObject(group);
  const center = fitted.getCenter(new THREE.Vector3());
  group.position.x -= center.x;
  group.position.x += 0.04;
  group.position.z -= center.z;
  group.position.y -= fitted.min.y;
}

function createSessionId(): string {
  const token = typeof crypto !== 'undefined' && 'randomUUID' in crypto ? crypto.randomUUID() : `${Date.now()}`;
  return `desktop-${token}`;
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
