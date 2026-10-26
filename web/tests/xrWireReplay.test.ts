// @vitest-environment jsdom
// In-memory transport and synthetic XR only; no browser, headset or robot connection.
import {afterEach, expect, it, vi} from 'vitest';
import scenario from '../../schemas/fixtures/xr-grip-release-wire.json';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import {forwardTeleopFrame} from '../src/appFrameForwarding';
import type {RobotStateMessage} from '../src/protocol/messages';
import {TeleopSocket} from '../src/transport/teleopSocket';
import {ArmPanel} from '../src/ui/armPanel';
import {XRSessionController, type XRRenderHost} from '../src/xr/session';
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame} from '../src/xr/finiteInput';

class MemorySocket {
  readyState = 0;
  sent: string[] = [];
  onopen: (() => void) | null = null;
  onmessage: ((event: {data: string}) => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: (() => void) | null = null;
  send(value: string): void { this.sent.push(value); }
  close(): void { this.readyState = 3; }
  open(): void { this.readyState = 1; this.onopen?.(); }
  receive(message: unknown): void { this.onmessage?.({data: JSON.stringify(message)}); }
}

function inputSource(input: {grip?: boolean; trigger?: number; a?: boolean; b?: boolean}): XRInputSource {
  const buttons = Array.from({length: 7}, () => ({value: 0, pressed: false}));
  buttons[0].value = input.trigger ?? 0;
  for (const [index, value] of [[1, input.grip], [4, input.a], [5, input.b]] as const) {
    buttons[index] = {value: value ? 1 : 0, pressed: value ?? false};
  }
  return {handedness: 'right', profiles: ['meta-quest-touch-plus'], gripSpace: {},
    gamepad: {buttons}} as unknown as XRInputSource;
}

function setup(finiteMode=false) {
  document.body.innerHTML = '<div id="panel"></div>';
  const wire = new MemorySocket();
  let panel: ArmPanel;
  let finite:FiniteXRInput;let position:FinitePositionInput;let generation=-1;
  const socket = new TeleopSocket('ws://synthetic.invalid', (state) => {
    panel.setRuntimeIdentity(state.backend ?? null);
    panel.setFault(state.fault ?? null);
    panel.setConstraint(state.constraint ?? null);
    panel.setRecoveryPhase(state.recovery_phase ?? null);
    panel.setPreparationState(state.preparation_required ?? false, state.preparation_unlocked ?? false);
    panel.setMode(state.mode);
    if(finiteMode){
      if(state.finite_position&&state.finite_position.generation!==generation){
        generation=state.finite_position.generation;position.reset('xr-finite-replay',generation);
      }
      finite.update(state,now);
    }
  }, () => wire as unknown as WebSocket,
  (status) => panel.setConnectionStatus(status), (ack) => panel.handleArmFeedback(ack));
  panel = new ArmPanel(document.querySelector('#panel')!, (message) => socket.sendControl(message));
  const session = Object.assign(new EventTarget(), {
    visibilityState: 'visible', inputSources: [] as XRInputSource[],
    requestReferenceSpace: async () => ({} as XRReferenceSpace),
    end: async () => { session.dispatchEvent(new Event('end')); },
  });
  let loop: XRFrameRequestCallback | null = null;
  const host: XRRenderHost = {
    startXR: async (_session, callback) => { loop = callback; },
    stopXR: async () => { loop = null; }, updateXRPresentation: () => {}, renderXR: () => {},
  };
  let now = 0;
  position=new FinitePositionInput(m=>socket.sendPositionInput(m),()=>socket.sendControl({v:1,type:'disarm',request_id:'xr-abort'}));
  finite=new FiniteXRInput(position,m=>socket.sendGripper(m),id=>socket.cancelGripper(id),()=>now);
  let disarm = 0;
  const controller = new XRSessionController({
    xr: {isSessionSupported: async () => true, requestSession: async () => session} as unknown as XRSystem,
    host, createSessionId: () => scenario.session_id, now: () => now,
    onFrame: (frame) => {if(finiteMode){finite.apply(frame);frame=finiteXRTransportFrame(frame);}forwardTeleopFrame(frame, 'xr', false, (message) => socket.sendFrame(message));},
    onController: (state) => { if (state.tracking) panel.observeGrip(state.grip); },
    onArmRequest: () => { panel.requestXRPrimaryAction(); },
    onStopOrResetRequest: () => { panel.requestStopOrReset('xr'); },
    onDisarm: () => socket.sendControl({v: 1, type: 'disarm',
      request_id: `vr-disarm-${++disarm}`, client_mono_ms: now}),
    onLockReset: () => panel.resetToLocked(), onStatus: (status) => panel.setVRStatus(status),
  });
  function emit(at: number, input: Parameters<typeof inputSource>[0], tracking = true, p=[.4,.5,.6]) {
    now = at;
    session.inputSources = [inputSource(input)];
    loop?.(at, {getPose: () => tracking ? {transform: {
      position: {x:p[0],y:p[1],z:p[2]}, orientation: {x: 0, y: 0, z: 0, w: 1},
    }} : null, getViewerPose: () => null} as unknown as XRFrame);
  }
  function feedback(mode: RobotStateMessage['mode'], fault: string | null = null,
    extra: Partial<RobotStateMessage> = {}) {
    wire.receive({...robotFixture, backend: 'LEBAI_FAKE', mode, fault, ...extra});
  }
  return {wire, socket, panel, controller, emit, feedback, setNow: (at: number) => { now = at; },
    savedLoop: () => loop};
}

afterEach(() => vi.restoreAllMocks());

it('routes real XR Grip motion through finite intents, preserving normal release and B stop',async()=>{
 const r=setup(true);const sent=()=>r.wire.sent.map(v=>JSON.parse(v));
 try{
  r.socket.connect();r.wire.open();await r.controller.enterVR();
  r.feedback('ARMED',null,{manual_input_ready:true,translation_scale:1,finite_position:{policy:'finite_position_offline',
   phase:'ready',generation:1,entered_steps:0,motion_id:null,reason:null,hardware_verified:false,vr_enabled:true}});
  r.emit(20,{});r.emit(40,{grip:true});r.emit(60,{grip:true},true,[.41,.5,.6]);
  expect(sent().filter(m=>m.type==='vr_frame').every(m=>!m.right.grip&&m.right.trigger===0)).toBe(true);
  expect(sent().find(m=>m.type==='finite_position_input')).toMatchObject({session_id:'xr-finite-replay',kind:'held',vector:[1,0,0]});
  const stops=sent().filter(m=>m.type==='disarm').length;
  r.emit(80,{});expect(sent().filter(m=>m.type==='finite_position_input').at(-1).kind).toBe('release');
  expect(sent().filter(m=>m.type==='disarm')).toHaveLength(stops);
  r.emit(90,{b:true});expect(sent().filter(m=>m.type==='disarm')).toHaveLength(stops+1);
  expect(sent().some(m=>m.type==='home_request')).toBe(false);
 }finally{await r.controller.dispose();r.socket.close();}
});

it('requires a second released A gesture for preparation and B revokes pending completion', async()=>{
  const r=setup();
  const controls=()=>r.wire.sent.map(value=>JSON.parse(value)).filter(m=>m.type!=='vr_frame');
  try {
    r.socket.connect();r.wire.open();await r.controller.enterVR();
    r.feedback('READY',null,{preparation_required:true,preparation_unlocked:false});
    r.emit(100,{});r.emit(120,{a:true});
    const arm=controls().at(-1);expect(arm.type).toBe('arm_request');
    r.wire.receive({v:1,type:'arm_ack',request_id:arm.request_id});
    r.feedback('READY',null,{preparation_required:true,preparation_unlocked:true});
    r.emit(140,{a:true});r.emit(160,{a:true});
    expect(controls().map(m=>m.type)).toEqual(['hello','disarm','arm_request']);
    expect(r.panel.isArmed).toBe(false);
    r.emit(180,{});r.emit(200,{a:true});
    const prepare=controls().at(-1);expect(prepare.type).toBe('prepare_request');
    r.feedback('READY',null,{preparation_required:true,preparation_unlocked:false,recovery_phase:'homing'});
    r.emit(220,{});r.emit(240,{b:true});
    expect(controls().map(m=>m.type)).toEqual(['hello','disarm','arm_request','prepare_request','disarm']);
    r.wire.receive({v:1,type:'prepare_ack',request_id:prepare.request_id});
    r.feedback('ARMED',null,{preparation_required:true});
    expect(r.panel.isArmed).toBe(false); // Late completion cannot undo B.
    expect(r.panel.safetyState.phase).toBe('stopped');
  } finally {await r.controller.dispose();r.socket.close();}
});

it('accepts Grip and analog Trigger frames only after authoritative preparation state',async()=>{
  const r=setup();
  const lastControl=()=>r.wire.sent.map(value=>JSON.parse(value)).filter(m=>m.type!=='vr_frame').at(-1);
  try {
    r.socket.connect();r.wire.open();await r.controller.enterVR();
    r.feedback('READY',null,{preparation_required:true});r.emit(100,{});r.emit(120,{a:true});
    r.wire.receive({v:1,type:'arm_ack',request_id:lastControl().request_id});
    r.emit(140,{});r.emit(160,{a:true});
    expect(lastControl().type).toBe('prepare_request');
    r.wire.receive({v:1,type:'prepare_ack',request_id:lastControl().request_id});
    expect(r.panel.isArmed).toBe(false);
    r.feedback('ARMED',null,{preparation_required:true});expect(r.panel.isArmed).toBe(true);
    r.emit(180,{grip:true,trigger:.4});
    expect(JSON.parse(r.wire.sent.at(-1)!)).toMatchObject({type:'vr_frame',right:{grip:true,trigger:.4}});
    r.emit(200,{});expect(JSON.parse(r.wire.sent.at(-1)!)).toMatchObject({right:{grip:false,trigger:0}});
    r.emit(220,{},false);
    expect(JSON.parse(r.wire.sent.at(-1)!)).toMatchObject({tracking_valid:false,right:{grip:false,trigger:0}});
  } finally {await r.controller.dispose();r.socket.close();}
});

it('generates the shared wire fixture through XR, ArmPanel and TeleopSocket', async () => {
  const rig = setup();
  try {
    for (const step of scenario.steps) {
      const before = rig.wire.sent.length;
      rig.setNow(step.at_ms);
      if (step.name === 'connect') { rig.socket.connect(); rig.wire.open(); }
      else if (step.name === 'enter') await rig.controller.enterVR();
      else if (step.name === 'exit') {
        const stale = rig.savedLoop();
        await rig.controller.exitVR();
        stale?.(300, {} as XRFrame); // Revoked callback cannot even read this invalid frame.
      } else rig.emit(step.at_ms, step.input ?? {}, step.tracking !== false);
      expect(rig.wire.sent.slice(before).map((value) => JSON.parse(value)), step.name).toEqual(step.messages);
      if (step.name === 'arm') rig.wire.receive({v: 1, type: 'arm_ack', request_id: 'xr-arm_request-1'});
      if (step.mode) rig.feedback(step.mode as RobotStateMessage['mode']);
    }
    expect(rig.wire.sent.map((value) => JSON.parse(value).type).filter((type) => type === 'arm_request')).toHaveLength(1);
    expect(rig.wire.sent.some((value) => ['home_request', 'fault_reset'].includes(JSON.parse(value).type))).toBe(false);
  } finally { await rig.controller.dispose(); rig.socket.close(); }
});

it.each(['stop_unverified', 'stop_incomplete'])('keeps %s locked after feedback and transport reconnect', async (fault) => {
  const rig = setup();
  try {
    rig.socket.connect(); rig.wire.open();
    await rig.controller.enterVR();
    rig.feedback('FAULT', fault);
    rig.emit(100, {});
    const before = rig.wire.sent.length;
    rig.emit(120, {a: true});
    rig.emit(140, {});
    rig.emit(160, {b: true});
    expect(rig.wire.sent.slice(before).every((value) => JSON.parse(value).type === 'vr_frame')).toBe(true);
    rig.socket.close(); rig.socket.connect(); rig.wire.open();
    rig.emit(180, {});
    const reconnectBefore = rig.wire.sent.length;
    rig.emit(200, {a: true}); rig.emit(220, {}); rig.emit(240, {b: true});
    expect(rig.wire.sent.slice(reconnectBefore).every((value) => JSON.parse(value).type === 'vr_frame')).toBe(true);
    expect(rig.panel.armButton.disabled).toBe(true);
    expect(rig.panel.stopButton.disabled).toBe(true);
  } finally { await rig.controller.dispose(); rig.socket.close(); }
});
