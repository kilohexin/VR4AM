import {Quaternion, Vector3} from 'three';
import type {GripperRequestMessage, GripperResultMessage, RobotStateMessage, VRFrame, Vec3, Quat} from '../protocol/messages';
import {FINITE_POSITION_DEADZONE} from './finitePositionInput';
import {finiteHoldTitle,enteredFiniteHold} from './finitePositionFeedback';

type Action = 'joystick' | 'z+' | 'z-' | 'pitch+' | 'pitch-' | 'spin+' | 'spin-' | 'open' | 'close';
const CLAW_RATE = .15;
const CLAW_INTERVAL_MS = 100;

export class HoldControlPanel {
  private state: RobotStateMessage | null = null;
  private enabled = false;
  private receivedAt = Number.NEGATIVE_INFINITY;
  private action: Action | null = null;
  private pointer: number | null = null;
  private capture: HTMLButtonElement | null = null;
  private position: Vector3 | null = null;
  private rotation: Quaternion | null = null;
  private spinAxis = new Vector3(0, 0, 1);
  private xy = {x: 0, y: 0};
  private previousTime: number | null = null;
  private pending: string | null = null;
  private pendingCancelled = false;
  private clawTarget = 0;
  private clawLastSent = 0;
  private clawLastSendMs = Number.NEGATIVE_INFINITY;
  private disposed = false;
  private positionCandidate=false;
  private feedbackPaused=false;
  private readonly status: HTMLElement;
  private readonly feedback: HTMLElement;
  private readonly opening: HTMLElement;
  private readonly stick: HTMLButtonElement;
  private readonly knob: HTMLElement;

  constructor(private readonly root: HTMLElement,
    private readonly send: (m: GripperRequestMessage) => void,
    private readonly now: () => number = () => performance.now(),
    private readonly onRelease: () => void = () => {},
    private readonly cancelRequest: (requestId: string) => void = () => {},
    private readonly positionIntent:(vector:Vec3,event:'held'|'release'|'abort',reason?:string,pitch?:number)=>void=()=>{}) {
    root.classList.add('hold-control-panel');
    root.innerHTML = `<h2>手动操纵</h2>
      <div class="joystick-layout">
        <div class="joystick-section"><span class="joystick-axis axis-north">前 +Y</span>
          <span class="joystick-axis axis-west">−X</span>
          <button type="button" class="xy-joystick" data-joystick aria-label="平面摇杆：拖动控制左右前后，松开停止" disabled>
            <span class="joystick-cross" aria-hidden="true"></span><span class="joystick-knob" aria-hidden="true"></span>
          </button><span class="joystick-axis axis-east">+X</span><span class="joystick-axis axis-south">后 −Y</span>
        </div>
        <div class="axis-controls"><span>高度</span>
          <button type="button" data-hold="z+" disabled>上升 +Z</button>
          <button type="button" data-hold="z-" disabled>下降 −Z</button>
          <span>工具俯仰</span>
          <button type="button" data-hold="pitch+" disabled>抬头 ＋</button>
          <button type="button" data-hold="pitch-" disabled>低头 −</button>
          <span>工具自转</span><div class="spin-buttons">
            <button type="button" data-hold="spin-" disabled aria-label="逆向自转">↶</button>
            <button type="button" data-hold="spin+" disabled aria-label="顺向自转">↷</button>
          </div>
        </div>
      </div>
      <section class="progressive-claw"><h3>夹爪开口 <strong data-field="claw-opening">—</strong></h3>
        <div class="claw-opening-track"><span data-field="claw-opening-fill"></span></div>
        <div class="claw-buttons"><button type="button" data-claw="open" disabled>长按打开</button>
          <button type="button" data-claw="close" disabled>长按闭合</button></div>
        <p>按住渐变，松开保持当前目标开口</p>
      </section>
      <p data-field="hold-status" role="status">先解锁，再点击进入准备姿态</p>
      <p data-field="hold-feedback">等待实际反馈</p>`;
    this.status = root.querySelector('[data-field="hold-status"]')!;
    this.feedback = root.querySelector('[data-field="hold-feedback"]')!;
    this.opening = root.querySelector('[data-field="claw-opening"]')!;
    this.stick = root.querySelector('[data-joystick]')!;
    this.knob = root.querySelector('.joystick-knob')!;
    root.addEventListener('pointerdown', this.onDown);
    root.addEventListener('pointermove', this.onMove);
    root.addEventListener('pointerup', this.onUp);
    root.addEventListener('pointercancel', this.onUp);
    root.addEventListener('lostpointercapture', this.onUp);
    window.addEventListener('blur', this.cancel);
    document.addEventListener('visibilitychange', this.onVisibility);
    document.addEventListener('focusin', this.onFocus);
  }

  update(state: RobotStateMessage | null, available: boolean, receivedAt = this.now()): void {
    const enteredHold=enteredFiniteHold(this.state?.finite_position,state?.finite_position);
    this.positionCandidate=(state?.backend==='LEBAI_FAKE'&&state.finite_position?.policy==='finite_position_offline')
      ||(state?.backend==='LEBAI'&&state.finite_position?.policy==='finite_position_web')
      ||(state?.backend==='LEBAI_DOCKER'&&state.finite_position?.policy==='finite_position_docker');
    this.stick.setAttribute('aria-label',this.positionCandidate?'平面摇杆：拖动控制左右前后，松开后完成当前小步':'平面摇杆：拖动控制左右前后，松开停止');
    this.state = state;
    this.receivedAt = receivedAt;
    this.enabled = available && state !== null && state.manual_input_ready === true && !state.fault
      && !state.recovery_phase && ['ARMED', 'ACTIVE', 'HOLD'].includes(state.mode)
      && ['IDLE', 'MOVING'].includes(state.robot_state);
    if (!this.enabled) {
      this.cancel('unavailable'); this.pending = null;
      this.position = this.rotation = null; this.previousTime = null;
    } else if(this.positionCandidate&&enteredHold&&this.action!==null&&this.action!=='open'&&this.action!=='close') {
      this.finishGesture('release');
    }
    if (state) {
      const percent = Math.max(0, Math.min(100, (1 - state.gripper) * 100));
      this.opening.textContent = `${percent.toFixed(0)}%`;
      this.root.querySelector<HTMLElement>('[data-field="claw-opening-fill"]')!.style.width = `${percent}%`;
      this.feedback.textContent = `实际 XYZ：${state.actual_tcp.p.map(n => (n * 1000).toFixed(2)).join(' / ')} mm`;
    }
    this.refresh();
  }

  applyFrame(frame: VRFrame): VRFrame {
    const time = frame.client_mono_ms;
    if (!this.position) {
      this.position = new Vector3(...frame.right.p); this.rotation = new Quaternion(...frame.right.q);
    }
    const delta = this.previousTime === null ? 0 : time - this.previousTime;
    this.previousTime = time;
    const fresh = this.fresh() && frame.tracking_valid && frame.visibility === 'visible';
    if (!fresh || delta < 0 || delta > 100) {
      const awaitingSelectedClaw=!fresh&&this.enabled&&this.pending!==null&&!this.motionHeld()
        &&frame.tracking_valid&&frame.visibility==='visible'&&delta>=0&&delta<=100
        &&this.now()-this.receivedAt>100;
      const finiteFeedbackGap=this.positionCandidate&&!fresh&&this.enabled
        &&frame.tracking_valid&&frame.visibility==='visible'&&delta>=0&&delta<=100
        &&this.now()-this.receivedAt>100;
      // A selected claw RPC can hold the SDK lock past the telemetry period.
      // Stop generating targets, but do not cancel its already-owned reply.
      if(finiteFeedbackGap){this.feedbackPaused=true;this.finishGesture('release');}
      else if(awaitingSelectedClaw)this.finishGesture('release');
      else this.cancel(!fresh?'feedback_or_tracking':'frame_gap');
    }
    if(this.positionCandidate&&this.motionHeld()&&fresh){
      if(this.action?.startsWith('pitch'))this.positionIntent([0,0,0],'held',undefined,this.action==='pitch+'?1:-1);
      else {
        const v:Vec3=this.action==='joystick'?[this.xy.x,this.xy.y,0]:[0,0,this.action==='z+'?1:-1];
        this.positionIntent(v,'held');
      }
    }
    if (!this.positionCandidate&&this.motionHeld() && this.state?.mode === 'ACTIVE') {
      // delta is milliseconds; bounded 20-ms integration, as for VR frames.
      const seconds = Math.min(20, Math.max(0, delta)) / 1000;
      if (this.action === 'joystick') {
        this.position.x += this.xy.x * seconds * .02;
        this.position.y += this.xy.y * seconds * .02;
      } else if (this.action?.startsWith('spin')) {
        const angle = (this.action.endsWith('+') ? 1 : -1) * seconds * Math.PI / 60;
        this.rotation!.premultiply(new Quaternion().setFromAxisAngle(this.spinAxis, angle));
      } else this.position.z += (this.action === 'z+' ? 1 : -1) * seconds * .02;
    }
    if (fresh && (this.action === 'open' || this.action === 'close') && !this.pending && this.clawReady()) {
      const seconds = Math.min(50, Math.max(0, delta)) / 1000;
      this.clawTarget = Math.max(0, Math.min(1, this.clawTarget + (this.action === 'close' ? 1 : -1) * seconds * CLAW_RATE));
      if (Math.abs(this.clawTarget - this.clawLastSent) >= .005 && this.now() - this.clawLastSendMs >= CLAW_INTERVAL_MS) {
        this.pending = `web-claw-${Date.now()}-${Math.random().toString(36).slice(2, 10)}`;
        this.pendingCancelled = false;
        this.clawLastSent = this.clawTarget; this.clawLastSendMs = this.now();
        this.send({v: 1, type: 'gripper_request', request_id: this.pending, value: this.clawTarget});
      }
    }
    this.refresh();
    return {...frame, right: {...frame.right, p: this.position.toArray() as Vec3,
      q: this.rotation!.toArray() as Quat, grip: !this.positionCandidate&&this.motionHeld() && fresh, trigger: 0}};
  }

  onGripperResult(result: GripperResultMessage): void {
    if (result.request_id !== this.pending) return;
    this.pending = null; this.pendingCancelled = false;
    if (!result.accepted) this.cancel();
    this.refresh();
  }

  cancel = (reason:unknown='cancel'): void => {
    this.finishGesture('abort',typeof reason==='string'?reason:reason instanceof Event?reason.type:'cancel');
  };

  private finishGesture(event:'release'|'abort',reason?:string):void {
    const held = this.motionHeld();
    const button = this.capture, id = this.pointer;
    this.action = null; this.pointer = null; this.capture = null; this.xy = {x: 0, y: 0};
    this.knob.style.transform = 'translate(-50%, -50%)';
    // A normal release keeps the already selected target. Cancelling its
    // in-flight RPC cannot undo a native claw command and makes it unknown.
    if (event==='abort' && this.pending && !this.pendingCancelled) {this.pendingCancelled = true; this.cancelRequest(this.pending);}
    if (button && id !== null) {try {if (button.hasPointerCapture(id)) button.releasePointerCapture(id);} catch {}}
    this.refresh();
    if (held || (event==='abort'&&this.positionCandidate)) {
      if(this.positionCandidate)this.positionIntent([0,0,0],event,reason);else this.onRelease();
    }
  }

  dispose(): void {
    if (this.disposed) return;
    this.cancel(); this.disposed = true;
    this.root.removeEventListener('pointerdown', this.onDown); this.root.removeEventListener('pointermove', this.onMove);
    this.root.removeEventListener('pointerup', this.onUp); this.root.removeEventListener('pointercancel', this.onUp);
    this.root.removeEventListener('lostpointercapture', this.onUp); window.removeEventListener('blur', this.cancel);
    document.removeEventListener('visibilitychange', this.onVisibility); document.removeEventListener('focusin', this.onFocus);
  }
  private fresh(maxAge=this.positionCandidate&&this.state?.backend==='LEBAI_DOCKER'
    &&this.action!=='open'&&this.action!=='close'?500:100): boolean {
    const time=this.now();return !this.disposed&&this.enabled&&time>=this.receivedAt&&time-this.receivedAt<=maxAge;
  }
  private motionHeld(): boolean {return this.action !== null && this.action !== 'open' && this.action !== 'close'
    && (this.action !== 'joystick' || Math.hypot(this.xy.x, this.xy.y) > 0);}
  private clawReady(): boolean {return this.fresh(100) && this.state?.robot_state === 'IDLE' && this.state.mode !== 'ACTIVE';}
  private refresh(): void {
    const boundary=this.positionCandidate?finiteHoldTitle(this.state?.finite_position):null;
    const finiteBusy=this.positionCandidate&&['planning','executing','draining'].includes(this.state?.finite_position?.phase??'');
    const movingWithoutGesture = !this.positionCandidate&&this.state?.mode === 'ACTIVE' && !this.motionHeld();
    for (const button of this.root.querySelectorAll<HTMLButtonElement>('[data-joystick],[data-hold]')) {
      button.disabled = !this.fresh() || this.pending !== null || movingWithoutGesture
        || this.action === 'open' || this.action === 'close'
        || (!!button.dataset.hold?.startsWith('spin') && (this.positionCandidate||!this.state?.target_control?.tool_spin_supported))
        || (!!button.dataset.hold?.startsWith('pitch') && (!this.positionCandidate||!this.state?.finite_position?.pitch_enabled));
      button.classList.toggle('is-held', button === this.capture);
      if(button.dataset.hold?.startsWith('pitch'))button.title=this.state?.finite_position?.pitch_enabled
        ?'绕工具自身横轴俯仰；松手完成当前小步并保持姿态':'本次运行范围未包含俯仰控制';
    }
    for (const button of this.root.querySelectorAll<HTMLButtonElement>('[data-claw]')) {
      button.disabled = !this.clawReady() || this.motionHeld() || finiteBusy
        || (this.pending !== null && button !== this.capture);
      button.classList.toggle('is-held', button === this.capture);
    }
    this.status.textContent = !this.enabled ? '先解锁，再点击进入准备姿态'
      : this.feedbackPaused?'反馈暂缓，已释放当前手势；更新后重新按住即可'
      : boundary?`${boundary} · 松开后重新按住，反向或更换方向调整`
      : this.positionCandidate&&this.state?.finite_position?.phase==='draining'?'松开后正在完成当前小步'
      : this.motionHeld() ? (this.positionCandidate?'长按每步 2 mm；松开后完成当前小步':'摇杆松开回中停止；坐标固定于机械臂基座')
      : this.action === 'open' || this.action === 'close' ? `目标开口 ${((1 - this.clawTarget) * 100).toFixed(0)}% · 松开保持`
      : this.pending ? '等待上次夹爪请求收尾；不自动重发' : this.positionCandidate?
        this.state?.finite_position?.pitch_enabled?'平移每步 2 mm · 俯仰每步 0.5°；松开保持':'每步 2 mm；长按摇杆或升降，松开完成当前步'
        :'摇杆控制平面；升降、自转和夹爪长按操作';
  }
  private moveStick(e: PointerEvent): void {
    const box = this.stick.getBoundingClientRect(), radius = Math.max(1, Math.min(box.width, box.height) / 2);
    let x = (e.clientX - box.left - box.width / 2) / radius;
    let y = -(e.clientY - box.top - box.height / 2) / radius;
    const distance = Math.hypot(x, y);
    if (distance <= FINITE_POSITION_DEADZONE) x = y = 0;
    else {const scale = (this.positionCandidate?Math.min(1,distance):Math.min(1, (distance - .15) / .85)) / distance; x *= scale; y *= scale;}
    this.xy = {x, y};
    this.knob.style.transform = `translate(calc(-50% + ${x * 44}px), calc(-50% - ${y * 44}px))`;
  }
  private onDown = (e: PointerEvent): void => {
    if (!(e.target instanceof Element)) return;
    const button = e.target.closest<HTMLButtonElement>('[data-joystick],[data-hold],[data-claw]');
    if (!button || button.disabled || !this.fresh() || e.button !== 0 || this.pointer !== null || !this.state) return;
    if(button.dataset.claw&&!this.fresh(100))return;
    e.preventDefault(); button.focus({preventScroll: true});
    this.feedbackPaused=false;
    this.pointer = e.pointerId; this.capture = button;
    this.action = button.hasAttribute('data-joystick') ? 'joystick' : (button.dataset.hold ?? button.dataset.claw) as Action;
    this.previousTime = null;
    this.spinAxis.set(0, 0, 1).applyQuaternion(new Quaternion(...this.state.actual_tcp.q)).normalize();
    if (this.action === 'joystick') this.moveStick(e);
    if (this.action === 'open' || this.action === 'close') {
      this.clawTarget = this.clawLastSent = this.state.gripper;
      this.clawLastSendMs = Number.NEGATIVE_INFINITY;
    }
    try {button.setPointerCapture(e.pointerId);} catch {this.cancel();}
    this.refresh();
  };
  private onMove = (e: PointerEvent): void => {
    if (e.pointerId !== this.pointer || this.action !== 'joystick') return;
    const held = this.motionHeld(); this.moveStick(e);
    if (held && !this.motionHeld()) {
      if(this.positionCandidate)this.positionIntent([0,0,0],'release');else this.onRelease();
    }
  };
  private onUp = (e: PointerEvent): void => {if (e.pointerId === this.pointer) this.finishGesture(e.type==='pointerup'?'release':'abort');};
  private onVisibility = (): void => {if (document.visibilityState !== 'visible') this.cancel();};
  private onFocus = (e: FocusEvent): void => {if (e.target instanceof Node && !this.root.contains(e.target)) this.cancel();};
}
