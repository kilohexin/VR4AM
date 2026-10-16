// @vitest-environment jsdom
import {beforeEach,it,expect,vi} from 'vitest';
import {ArmPanel} from '../src/ui/armPanel';
import {Hud} from '../src/ui/hud';
import {isArmFeedbackMessage,isClientControlMessage,isRobotStateMessage} from '../src/protocol/messages';
beforeEach(()=>{document.body.innerHTML='<div id="panel"></div>';});
function rig(){const send=vi.fn();const p=new ArmPanel(document.querySelector('#panel')!,send);
 p.setConnectionStatus({state:'connected'});p.setMode('READY');p.observeGrip(false);
 p.setPreparationState(true,false);return {p,send};}
it('shows a separate preparation action and unlock alone never sends it',()=>{
 const {p,send}=rig();expect(p.prepareButton.disabled).toBe(true);p.armButton.click();
 expect(send.mock.calls.map(c=>c[0].type)).toEqual(['arm_request']);
 p.handleArmFeedback({v:1,type:'arm_ack',request_id:send.mock.calls[0][0].request_id});
 expect(p.isArmed).toBe(false);expect(p.isArmPending).toBe(false);
 expect(p.prepareButton.disabled).toBe(false);p.prepareButton.click();
 expect(send.mock.calls.map(c=>c[0].type)).toEqual(['arm_request','prepare_request']);
 expect(p.prepareButton.disabled).toBe(true);p.setRecoveryPhase('homing');
 expect(p.prepareButton.textContent).toContain('准备');expect(p.stopButton.disabled).toBe(false);
});
it('requires authoritative armed mode after preparation acknowledgement',()=>{
 const {p,send}=rig();p.armButton.click();p.handleArmFeedback({v:1,type:'arm_ack',request_id:send.mock.calls[0][0].request_id});
 p.prepareButton.click();p.handleArmFeedback({v:1,type:'prepare_ack',request_id:send.mock.calls[1][0].request_id});
 expect(p.isArmed).toBe(false);p.setMode('ARMED');expect(p.isArmed).toBe(true);
});
it('stop and reconnect revoke preparation eligibility without Home',()=>{
 const {p,send}=rig();p.armButton.click();p.handleArmFeedback({v:1,type:'arm_ack',request_id:send.mock.calls[0][0].request_id});
 p.stopButton.click();expect(send.mock.calls.at(-1)![0].type).toBe('disarm');
 expect(p.prepareButton.disabled).toBe(true);p.onSocketReconnect();p.observeGrip(false);
 expect(p.prepareButton.disabled).toBe(true);
});
it('accepts only the precise additional preparation messages',()=>{
 expect(isClientControlMessage({v:1,type:'prepare_request',request_id:'p'})).toBe(true);
 expect(isArmFeedbackMessage({v:1,type:'prepare_ack',request_id:'p'})).toBe(true);
 expect(isArmFeedbackMessage({v:1,type:'prepare_rejected',request_id:'p',message:'not unlocked'})).toBe(true);
 expect(isArmFeedbackMessage({v:1,type:'prepare_ack',request_id:'p',accepted:true})).toBe(false);
});

it('shows a rejected preparation even when the locked state arrives first',()=>{
 const {p,send}=rig();p.armButton.click();p.handleArmFeedback({v:1,type:'arm_ack',request_id:send.mock.calls[0][0].request_id});
 p.prepareButton.click();const id=send.mock.calls[1][0].request_id;p.setMode('DISARMED');
 p.handleArmFeedback({v:1,type:'prepare_rejected',request_id:id,message:'failed'});
 expect(p.feedbackText).toContain('准备未完成');expect(p.isArmPending).toBe(false);
 expect(p.isArmed).toBe(false);expect(p.prepareButton.disabled).toBe(true);
});

it('shows preparation progress in the primary status instead of asking to unlock again',()=>{
 const hud=new Hud(document.querySelector('#panel')!);
 hud.setRobotState({mode:'READY',backendState:'MOVING',sampleAgeMs:1,fault:null,constraint:null,recoveryPhase:'homing'});
 expect(document.querySelector('[data-field="mode"]')?.textContent).toContain('准备位');
});

it('keeps local stop dominant over an in-flight old ARMED state',()=>{
 const {p}=rig();p.setMode('ARMED');expect(p.isArmed).toBe(true);
 p.requestDisarm();p.setMode('ARMED');expect(p.isArmed).toBe(false);
 expect(p.safetyState.phase).toBe('stopped');
});
