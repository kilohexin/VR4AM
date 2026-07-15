import * as THREE from 'three';
import {PROTOCOL_VERSION, type Quat, type RobotStateMessage, type VRFrame, type Vec3} from '../protocol/messages';
import {loadRobotModel, type RobotModel} from '../robot/robotModel';
import {RobotStateBuffer} from '../robot/robotState';

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
}

export function createVRFrame(input: VRFrameInput): VRFrame {
  return {
    v: PROTOCOL_VERSION,
    type: 'vr_frame',
    session_id: input.sessionId,
    seq: input.sequence,
    client_mono_ms: input.nowMs,
    tracking_valid: input.trackingValid,
    visibility: 'visible',
    right: {
      p: [...input.position],
      q: [...input.quaternion],
      grip: input.grip,
      trigger: clamp(input.trigger, 0, 1),
    },
  };
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
  private readonly sessionId = createSessionId();
  private robotModel: RobotModel | null = null;
  private animationHandle: number | null = null;
  private sequence = 0;
  private lastFrameMs = Number.NEGATIVE_INFINITY;
  private clockAnchor: {serverNs: number; clientMs: number} | null = null;
  private activePointer: number | null = null;
  private grip = false;
  private trigger = 0;
  private readonly controllerPosition = new THREE.Vector3(0.56, 0.42, 0.18);
  private readonly controllerQuaternion = new THREE.Quaternion(0, 0, 0, 1);
  private started = false;

  constructor(
    private readonly container: HTMLElement,
    private readonly options: SimulationSceneOptions,
  ) {
    this.renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: 'high-performance'});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.1;
    this.renderer.shadowMap.enabled = true;
    this.renderer.domElement.className = 'simulation-canvas';
    this.renderer.domElement.style.touchAction = 'none';

    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
    this.camera.position.set(1.55, 1.06, 1.9);
    this.camera.lookAt(0.03, 0.42, 0);

    this.scene.background = new THREE.Color(0x07131e);
    this.createEnvironment();
    this.createTargetMarker();
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
    this.clockAnchor = {serverNs: state.server_mono_ns, clientMs: performance.now()};
  }

  resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  dispose(): void {
    if (!this.started) return;
    this.started = false;
    if (this.animationHandle !== null) cancelAnimationFrame(this.animationHandle);
    this.animationHandle = null;
    this.removeListeners();
    this.scene.traverse((object) => {
      if (!(object instanceof THREE.Mesh) && !(object instanceof THREE.Line)) return;
      object.geometry.dispose();
      const materials = Array.isArray(object.material) ? object.material : [object.material];
      materials.forEach((material) => material.dispose());
    });
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private readonly animate = (nowMs: number): void => {
    if (!this.started) return;
    this.animationHandle = requestAnimationFrame(this.animate);

    if (this.robotModel && this.clockAnchor) {
      const nowNs = this.clockAnchor.serverNs + (nowMs - this.clockAnchor.clientMs) * 1_000_000;
      const sample = this.options.stateBuffer.sample(nowNs);
      if (sample) {
        this.robotModel.setJointAngles(sample.state.actual_q);
        this.robotModel.setGripper(sample.state.gripper);
      }
    }

    this.targetMarker.position.copy(this.controllerPosition);
    this.targetMarker.quaternion.copy(this.controllerQuaternion);
    if (nowMs - this.lastFrameMs >= FRAME_INTERVAL_MS) {
      this.lastFrameMs = nowMs;
      const frame = createVRFrame({
        sessionId: this.sessionId,
        sequence: this.sequence++,
        nowMs,
        trackingValid: document.visibilityState === 'visible',
        position: this.controllerPosition.toArray() as Vec3,
        quaternion: this.controllerQuaternion.toArray() as Quat,
        grip: this.grip,
        trigger: this.trigger,
      });
      this.options.onController({
        tracking: frame.tracking_valid,
        grip: frame.right.grip,
        trigger: frame.right.trigger,
      });
      this.options.onFrame(frame);
    }
    this.renderer.render(this.scene, this.camera);
  };

  private async loadModel(): Promise<void> {
    try {
      this.robotModel = await loadRobotModel();
      this.robotModel.setJointAngles([0.2, -0.8, -0.8, -0.4, 0.6, 0]);
      fitRobotToWorkbench(this.robotModel.group);
      this.scene.add(this.robotModel.group);
    } catch (error) {
      const detail = error instanceof Error ? error.message : '未知错误';
      this.options.onError(`LM3 模型载入失败：${detail}`);
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
    window.addEventListener('resize', this.resizeFromEvent);
    window.addEventListener('keydown', this.onKeyDown);
    window.addEventListener('keyup', this.onKeyUp);
    canvas.addEventListener('pointerdown', this.onPointerDown);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('pointerup', this.onPointerUp);
    canvas.addEventListener('pointercancel', this.onPointerUp);
    canvas.addEventListener('wheel', this.onWheel, {passive: false});
    canvas.addEventListener('contextmenu', this.preventContextMenu);
  }

  private removeListeners(): void {
    const canvas = this.renderer.domElement;
    window.removeEventListener('resize', this.resizeFromEvent);
    window.removeEventListener('keydown', this.onKeyDown);
    window.removeEventListener('keyup', this.onKeyUp);
    canvas.removeEventListener('pointerdown', this.onPointerDown);
    canvas.removeEventListener('pointermove', this.onPointerMove);
    canvas.removeEventListener('pointerup', this.onPointerUp);
    canvas.removeEventListener('pointercancel', this.onPointerUp);
    canvas.removeEventListener('wheel', this.onWheel);
    canvas.removeEventListener('contextmenu', this.preventContextMenu);
  }

  private readonly resizeFromEvent = (): void => this.resize();

  private readonly onPointerDown = (event: PointerEvent): void => {
    if (event.button !== 0) return;
    this.activePointer = event.pointerId;
    this.grip = true;
    this.renderer.domElement.setPointerCapture(event.pointerId);
  };

  private readonly onPointerMove = (event: PointerEvent): void => {
    if (this.activePointer !== event.pointerId) return;
    this.controllerPosition.x = clamp(this.controllerPosition.x + event.movementX * 0.0012, -0.2, 0.8);
    this.controllerPosition.y = clamp(this.controllerPosition.y - event.movementY * 0.0012, 0.05, 1.05);
  };

  private readonly onPointerUp = (event: PointerEvent): void => {
    if (this.activePointer !== event.pointerId) return;
    this.activePointer = null;
    this.grip = false;
    if (this.renderer.domElement.hasPointerCapture(event.pointerId)) {
      this.renderer.domElement.releasePointerCapture(event.pointerId);
    }
  };

  private readonly onWheel = (event: WheelEvent): void => {
    event.preventDefault();
    this.controllerPosition.z = clamp(this.controllerPosition.z + event.deltaY * 0.0008, -0.62, 0.62);
  };

  private readonly onKeyDown = (event: KeyboardEvent): void => {
    if (event.code === 'Space') {
      event.preventDefault();
      this.trigger = 1;
    }
  };

  private readonly onKeyUp = (event: KeyboardEvent): void => {
    if (event.code === 'Space') this.trigger = 0;
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

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}
