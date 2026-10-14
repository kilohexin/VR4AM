// @vitest-environment jsdom
import {beforeEach, expect, it} from 'vitest';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import {isRobotStateMessage, type ClientControlMessage} from '../src/protocol/messages';
import {Hud} from '../src/ui/hud';
import {ArmPanel} from '../src/ui/armPanel';
import {describeVrSafety} from '../src/scenes/vrSafetyPanel';

const status = {
  policy:'segmented_experimental', phase:'awaiting_feedback',
  samples_observed:1, samples_required:3, hardware_verified:false,
} as const;

beforeEach(() => { document.body.innerHTML = '<div id="app"></div>'; });

it('accepts exact optional segment metadata and old messages', () => {
  expect(isRobotStateMessage(robotFixture)).toBe(true);
  expect(isRobotStateMessage({...robotFixture, target_control:status})).toBe(true);
  for (const change of [{hardware_verified:true},{samples_observed:4},{samples_observed:true},
    {phase:'stopped'},{samples_required:2},{typo:true}]) {
    expect(isRobotStateMessage({...robotFixture,target_control:{...status,...change}})).toBe(false);
  }
});

it('shows unverified candidate and waiting count, and clears stale status on disconnect', () => {
  const root = document.querySelector('#app')!;
  const hud = new Hud(root);
  hud.setRuntimeIdentity('LEBAI', 'control');
  hud.setRobotState({mode:'ACTIVE',backendState:'IDLE',sampleAgeMs:1,fault:null,
    constraint:null,recoveryPhase:null,targetControl:status});
  expect(root.textContent).toContain('分段候选 · 未真机验收');
  expect(root.textContent).toContain('等待本段反馈 1/3');
  expect(root.textContent).not.toContain('已停止');
  hud.setConnectionStatus({state:'disconnected'});
  expect(root.querySelector('[data-field="target-control"]')?.textContent).toBe('未确认');
});

it('B remains stop and cannot send unsupported Home or reset in segment mode', () => {
  const messages: ClientControlMessage[] = [];
  const panel = new ArmPanel(document.querySelector('#app')!, m => messages.push(m));
  panel.setConnected(true);
  panel.setMode('READY');
  panel.observeGrip(false);
  panel.setTargetControl(status);
  panel.requestStopOrReset('xr');
  expect(messages.map(m => m.type)).toEqual(['disarm']);
  expect(panel.stopButton.textContent).not.toContain('Home');
  panel.setFault('ik_unreachable');
  panel.requestStopOrReset('xr');
  expect(messages.map(m => m.type)).toEqual(['disarm']);
});

it('VR waiting copy keeps stopping controls and fault precedence', () => {
  const panel = new ArmPanel(document.querySelector('#app')!, () => {});
  panel.setConnected(true);
  panel.setMode('ARMED');
  panel.setMode('ACTIVE');
  const summary = {backend:'LEBAI',realRobotMode:'control',actualTcp:null,gripper:0,
    latencyMs:1,hardwareVerified:false,targetControl:status} as const;
  const view = describeVrSafety(panel.safetyState,true,summary);
  expect(view.instruction).toContain('等待本段反馈 1/3');
  expect(view.statusLine).toContain('分段候选');
  expect(view.footer).toContain('B停止');
  expect(view.footer).not.toContain('Home');
  panel.setFault('stop_unverified');
  expect(describeVrSafety(panel.safetyState,true,summary).title).toContain('未确认');
});

it('local segment fault cannot keep an ACTIVE screen looking healthy', () => {
  const panel = new ArmPanel(document.querySelector('#app')!, () => {});
  panel.setConnected(true);panel.setMode('ARMED');panel.setMode('ACTIVE');
  const target={...status,phase:'fault'} as const;
  const summary={backend:'LEBAI',realRobotMode:'control',actualTcp:null,gripper:0,
    latencyMs:1,hardwareVerified:false,targetControl:target} as const;
  const view=describeVrSafety(panel.safetyState,true,summary);
  expect(view.tone).toBe('red');
  expect(view.title).toBe('分段控制已中止');
  const hud=new Hud(document.querySelector('#app')!);
  hud.setRobotState({mode:'ACTIVE',backendState:'IDLE',sampleAgeMs:1,fault:null,
    constraint:null,recoveryPhase:null,targetControl:target});
  expect(document.querySelector('[data-field="mode"]')?.textContent).toContain('FAULT');
  expect(document.querySelector('[data-field="fault"]')?.textContent).toContain('运动状态未确认');
});

it.each(['workspace_boundary','tracking_lag'] as const)('segment fault replaces %s retreat guidance', constraint => {
  const panel=new ArmPanel(document.querySelector('#app')!,()=>{});
  panel.setConnected(true);panel.setMode('ARMED');panel.setMode('ACTIVE');panel.setConstraint(constraint);
  const summary={backend:'LEBAI',realRobotMode:'control',actualTcp:null,gripper:0,
    latencyMs:1,hardwareVerified:false,targetControl:{...status,phase:'fault'}} as const;
  const view=describeVrSafety(panel.safetyState,true,summary);
  expect(view.tone).toBe('red');
  expect(view.title).toBe('分段控制已中止');
  expect(view.instruction).toContain('保持 Grip 松开');
});
