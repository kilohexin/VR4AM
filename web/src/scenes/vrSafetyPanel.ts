import * as THREE from 'three';
import type {ArmSafetySnapshot} from '../ui/armPanel';

export interface VrSafetyPresentation {
  title: string;
  instruction: string;
  footer: 'A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪';
  tone: 'cyan' | 'red' | 'muted';
  shape: 'shield' | 'stop' | 'warning';
}

export function describeVrSafety(
  state: ArmSafetySnapshot,
  controllerSupported: boolean | null,
): VrSafetyPresentation {
  if (controllerSupported === false) {
    return presentation('手柄不受支持', '当前配置不支持 A/B 安全控制', 'red', 'warning');
  }
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
  if (state.phase === 'disconnected' || state.phase === 'fault') {
    return presentation('故障/失联', '保持 Grip 松开', 'red', 'warning');
  }
  if (state.phase === 'pending') {
    return presentation('解锁中', '等待仿真确认', 'cyan', 'shield');
  }
  if (state.phase === 'active') {
    return presentation('运动中', '松开 Grip 停止', 'cyan', 'shield');
  }
  if (state.phase === 'armed') {
    return presentation('已解锁', '按住 Grip 移动', 'cyan', 'shield');
  }
  if (state.phase === 'stopped') {
    return presentation('已停止', '松开 Grip 后按 A', 'red', 'stop');
  }
  return presentation('未解锁', '松开 Grip 后按 A', 'muted', 'shield');
}

function presentation(
  title: string,
  instruction: string,
  tone: VrSafetyPresentation['tone'],
  shape: VrSafetyPresentation['shape'],
): VrSafetyPresentation {
  return {
    title,
    instruction,
    footer: 'A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪',
    tone,
    shape,
  };
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
    this.sprite.position.set(-0.82, 1.18, -0.58);
    this.sprite.scale.set(1.15, 0.2875, 1);
    this.sprite.renderOrder = 1000;
    this.sprite.visible = false;
    parent.add(this.sprite);
  }

  update(state: ArmSafetySnapshot, controllerSupported: boolean | null): void {
    const view = describeVrSafety(state, controllerSupported);
    const key = `${view.title}|${view.instruction}|${view.tone}|${view.shape}`;
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
    context.font = "700 58px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    context.fillText(view.title, 150, 82);
    context.fillStyle = accent;
    context.font = "500 36px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
    context.fillText(view.instruction, 150, 137);
    context.strokeStyle = '#315064';
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(40, 174);
    context.lineTo(984, 174);
    context.stroke();
    context.fillStyle = '#c9d8e2';
    context.font = "500 28px 'Microsoft YaHei', 'Noto Sans SC', sans-serif";
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
