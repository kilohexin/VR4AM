import * as THREE from 'three';
import type {Pose, RealRobotMode, RuntimeBackend} from '../protocol/messages';
import type {ArmSafetySnapshot} from '../ui/armPanel';
import {readableConstraint, readableFault} from '../ui/hud';

const CONTROL_FOOTER = 'A解锁 B停止/Home · 右Grip末端 Trigger夹爪 · 左摇杆平台/按下归零 · 左Grip+摇杆升降' as const;

export interface RobotRuntimeSummary {
  backend: RuntimeBackend | null;
  realRobotMode: RealRobotMode | null;
  actualTcp: Pose | null;
  gripper: number | null;
  latencyMs: number | null;
  hardwareVerified: false;
}

const EMPTY_RUNTIME_SUMMARY: RobotRuntimeSummary = {
  backend: null,
  realRobotMode: null,
  actualTcp: null,
  gripper: null,
  latencyMs: null,
  hardwareVerified: false,
};

export interface VrSafetyPresentation {
  title: string;
  instruction: string;
  footer: typeof CONTROL_FOOTER;
  statusLine: string;
  tone: 'cyan' | 'amber' | 'red' | 'muted';
  shape: 'shield' | 'stop' | 'warning';
}

type VrSafetyCore = Omit<VrSafetyPresentation, 'statusLine'>;

export function describeVrSafety(
  state: ArmSafetySnapshot,
  controllerSupported: boolean | null,
  summary: RobotRuntimeSummary = EMPTY_RUNTIME_SUMMARY,
): VrSafetyPresentation {
  return {...describeSafety(state, controllerSupported), statusLine: formatRuntimeSummary(summary)};
}

function describeSafety(
  state: ArmSafetySnapshot,
  controllerSupported: boolean | null,
): VrSafetyCore {
  if (state.connectionState === 'occupied') {
    return presentation('控制端已被占用', '请关闭电脑端网页后重试', 'red', 'warning');
  }
  if (state.connectionState === 'unreachable') {
    return presentation('后端不可达', '保持 Grip 松开', 'red', 'warning');
  }
  if (state.connectionState === 'disconnected') {
    return presentation('连接已中断', '保持 Grip 松开', 'red', 'warning');
  }
  if (state.connectionState === 'reconnecting') {
    return presentation('正在重连', '保持 Grip 松开', 'red', 'warning');
  }
  if (!state.connected) {
    return presentation('连接已中断', '保持 Grip 松开', 'red', 'warning');
  }
  if (state.homePending || state.recoveryPhase) {
    return presentation('正在返回 Home', '请保持 Grip 松开', 'red', 'warning');
  }
  if (state.faultResetPending) {
    return presentation('复位中', '等待仿真确认', 'red', 'warning');
  }
  if (state.faultRecoverable && state.fault) {
    return presentation(readableFault(state.fault), '松开 Grip，按 B 复位并回 Home', 'red', 'warning');
  }
  if (state.fault) {
    return presentation('无法在线复位', '保持安全距离，检查 L Master/急停', 'red', 'warning');
  }
  if (state.mode === 'STALE') {
    return presentation('数据陈旧', '机械臂已停止，请检查网络', 'red', 'warning');
  }
  if (controllerSupported === false) {
    return presentation('手柄不受支持', '当前配置不支持 A/B 安全控制', 'red', 'warning');
  }
  if (state.phase === 'disconnected') {
    return presentation('连接已中断', '保持 Grip 松开', 'red', 'warning');
  }
  if (state.phase === 'fault') {
    return presentation('无法在线复位', '保持安全距离，检查 L Master/急停', 'red', 'warning');
  }
  if (state.phase === 'pending') {
    return presentation('解锁中', '等待仿真确认', 'cyan', 'shield');
  }
  if (state.constraint) {
    return presentation(
      readableConstraint(state.constraint),
      constraintInstruction(state.constraint),
      'amber',
      'warning',
    );
  }
  if (state.phase === 'active') {
    return presentation('运动中', '松开 Grip 停止 · B 紧急停止', 'cyan', 'shield');
  }
  if (state.phase === 'armed') {
    return presentation('已解锁', '按住 Grip 移动', 'cyan', 'shield');
  }
  if (state.phase === 'stopped') {
    return presentation('已停止', '保持 Grip 松开：A 解锁，B 回 Home', 'red', 'stop');
  }
  return presentation('未解锁', '松开 Grip 后按 A', 'muted', 'shield');
}

function presentation(
  title: string,
  instruction: string,
  tone: VrSafetyCore['tone'],
  shape: VrSafetyCore['shape'],
): VrSafetyCore {
  return {
    title,
    instruction,
    footer: CONTROL_FOOTER,
    tone,
    shape,
  };
}

function constraintInstruction(constraint: NonNullable<ArmSafetySnapshot['constraint']>): string {
  if (constraint === 'workspace_boundary') return '保持 Grip 向反方向退回';
  if (constraint === 'ik_boundary') return '保持 Grip 退回上一个位置';
  if (constraint === 'joint_boundary') return '保持 Grip 反向退回';
  if (constraint === 'motion_continuity_boundary') return '运动变化过快，保持 Grip 放慢或反向退回';
  return '将手柄移回可达区域，无需复位';
}

function formatRuntimeSummary(summary: RobotRuntimeSummary): string {
  const tcp = summary.actualTcp
    ? summary.actualTcp.p.map((value) => value.toFixed(3).replace('-', '−')).join('/')
    : '—';
  const latency = summary.latencyMs === null ? '—' : `${Math.round(summary.latencyMs)} ms`;
  if (summary.backend === 'LEBAI') {
    const mode = summary.realRobotMode === 'control' ? 'CONTROL' : 'READONLY';
    return `真机已连接 · ${mode} · TCP ${tcp} · ${latency}`;
  }
  if (summary.backend === 'LEBAI_FAKE') return `仿真已连接 · TCP ${tcp} · ${latency}`;
  return `SIMULATOR · TCP ${tcp} · ${latency}`;
}

export class VrSafetyPanel {
  readonly sprite: THREE.Sprite;
  private readonly canvas: HTMLCanvasElement;
  private readonly context: CanvasRenderingContext2D;
  private readonly texture: THREE.CanvasTexture;
  private readonly material: THREE.SpriteMaterial;
  private lastKey = '';

  constructor(parent: THREE.Object3D) {
    this.canvas = document.createElement('canvas');
    this.canvas.width = 1024;
    this.canvas.height = 256;
    const context = this.canvas.getContext('2d');
    if (!context) throw new Error('无法创建 VR 安全状态牌');
    this.context = context;
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.material = new THREE.SpriteMaterial({
      map: this.texture,
      transparent: true,
      depthTest: false,
    });
    this.sprite = new THREE.Sprite(this.material);
    this.sprite.position.set(-0.82, 1.52, -0.72);
    this.sprite.scale.set(1.15, 0.2875, 1);
    this.sprite.renderOrder = 1000;
    this.sprite.visible = false;
    parent.add(this.sprite);
  }

  update(
    state: ArmSafetySnapshot,
    controllerSupported: boolean | null,
    summary: RobotRuntimeSummary = EMPTY_RUNTIME_SUMMARY,
  ): void {
    const view = describeVrSafety(state, controllerSupported, summary);
    const key = `${view.title}|${view.statusLine}|${view.instruction}|${view.tone}|${view.shape}`;
    if (key === this.lastKey) return;
    this.lastKey = key;
    this.draw(view);
    this.texture.needsUpdate = true;
  }

  setVisible(visible: boolean): void {
    this.sprite.visible = visible;
  }

  dispose(): void {
    this.sprite.removeFromParent();
    this.texture.dispose();
    this.material.dispose();
  }

  private draw(view: VrSafetyPresentation): void {
    const context = this.context;
    const accent = view.tone === 'cyan'
      ? '#32d7ff'
      : view.tone === 'amber'
        ? '#ffb84a'
      : view.tone === 'red'
        ? '#ff3347'
        : '#8ca5b6';
    context.clearRect(0, 0, 1024, 256);
    context.fillStyle = 'rgba(7, 19, 30, 0.96)';
    context.fillRect(0, 0, 1024, 256);
    context.strokeStyle = '#315064';
    context.lineWidth = 4;
    context.strokeRect(2, 2, 1020, 252);
    this.drawShape(view.shape, accent);
    context.fillStyle = '#ffffff';
    context.font = "700 50px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    context.fillText(view.title, 150, 62);
    context.fillStyle = '#c9d8e2';
    context.font = "500 26px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    context.fillText(view.statusLine, 150, 102);
    context.fillStyle = accent;
    context.font = "500 32px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    context.fillText(view.instruction, 150, 150);
    context.strokeStyle = '#315064';
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(40, 174);
    context.lineTo(984, 174);
    context.stroke();
    context.fillStyle = '#c9d8e2';
    context.font = "500 20px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    context.fillText(view.footer, 52, 220);
  }

  private drawShape(shape: VrSafetyPresentation['shape'], accent: string): void {
    const context = this.context;
    context.save();
    context.translate(82, 82);
    context.strokeStyle = accent;
    context.fillStyle = accent;
    context.lineWidth = 8;
    if (shape === 'stop') {
      context.strokeRect(-32, -32, 64, 64);
    } else if (shape === 'warning') {
      context.beginPath();
      context.moveTo(0, -38);
      context.lineTo(40, 34);
      context.lineTo(-40, 34);
      context.closePath();
      context.stroke();
      context.fillRect(-4, -14, 8, 26);
      context.fillRect(-4, 20, 8, 8);
    } else {
      context.beginPath();
      context.moveTo(0, -40);
      context.lineTo(36, -25);
      context.lineTo(30, 18);
      context.quadraticCurveTo(0, 44, -30, 18);
      context.lineTo(-36, -25);
      context.closePath();
      context.stroke();
    }
    context.restore();
  }
}
