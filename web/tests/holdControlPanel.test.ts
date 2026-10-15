// @vitest-environment jsdom
import {afterEach,describe,it,expect,vi} from 'vitest';
import {HoldControlPanel} from '../src/ui/holdControlPanel';
import type {RobotStateMessage,VRFrame} from '../src/protocol/messages';
const state:RobotStateMessage={v:1,type:'robot_state',server_mono_ns:1,mode:'ARMED',robot_state:'IDLE',
 manual_input_ready:true,actual_tcp:{p:[.3,0,.4],q:[0,0,0,1]},actual_q:[0,0,0,0,0,0],
 gripper:.5,backend:'LEBAI_FAKE',target_control:{policy:'segmented_experimental',phase:'ready',
 samples_observed:0,samples_required:3,hardware_verified:false,tool_spin_supported:true}};
function frame(t:number):VRFrame{return {v:1,type:'vr_frame',session_id:'manual',seq:t,client_mono_ms:t,
 tracking_valid:true,visibility:'visible',right:{p:[.56,.42,.18],q:[0,0,0,1],grip:false,trigger:0}};}
function pointer(type:string,x=100,y=100,id=7):Event{const e=new Event(type,{bubbles:true});Object.assign(e,{pointerId:id,button:0,clientX:x,clientY:y});return e;}
const cleanup:Array<()=>void>=[];
afterEach(()=>{cleanup.splice(0).forEach(f=>f());Object.defineProperty(document,'visibilityState',{configurable:true,value:'visible'});});
function rig(){const root=document.createElement('div');document.body.append(root);let now=0;
 const send=vi.fn(),release=vi.fn(),cancel=vi.fn();
 const panel=new HoldControlPanel(root,send,()=>now,release,cancel);
 const find=(selector:string)=>root.querySelector<HTMLButtonElement>(selector)!;
 for(const b of root.querySelectorAll<HTMLButtonElement>('button')){
  b.setPointerCapture=vi.fn();b.hasPointerCapture=vi.fn(()=>false);b.releasePointerCapture=vi.fn();
  b.getBoundingClientRect=()=>({x:0,y:0,left:0,top:0,right:200,bottom:200,width:200,height:200,toJSON:()=>({})});
 }
 const step=(t:number,mode:RobotStateMessage['mode']='ACTIVE')=>{now=t;panel.update({...state,mode},true,t);return panel.applyFrame(frame(t));};
 panel.update(state,true);cleanup.push(()=>{panel.dispose();root.remove();});
 return {root,panel,send,release,cancel,find,step,clock:(t:number)=>{now=t;}};
}
describe('joystick and progressive claw',()=>{
 it('uses proportional XY, waits for ACTIVE and releases immediately',()=>{
  const r=rig(),stick=r.find('[data-joystick]');expect(stick).not.toBeNull();
  stick.dispatchEvent(pointer('pointerdown',180,100));expect(r.panel.applyFrame(frame(0)).right.grip).toBe(true);
  expect(r.step(20,'ARMED').right.p).toEqual([.56,.42,.18]);
  const moved=r.step(40);expect(moved.right.p[0]).toBeGreaterThan(.56);expect(moved.right.p[1]).toBe(.42);
  stick.dispatchEvent(pointer('pointerup'));expect(r.release).toHaveBeenCalledTimes(1);
  expect(r.step(60,'HOLD').right.grip).toBe(false);
 });
 it('has a dead zone, maps up to +Y and caps diagonal magnitude',()=>{
  const r=rig(),stick=r.find('[data-joystick]');stick.dispatchEvent(pointer('pointerdown'));
  expect(r.panel.applyFrame(frame(0)).right.grip).toBe(false);
  stick.dispatchEvent(pointer('pointermove',200,0));const a=r.step(20);
  expect(a.right.p[0]).toBeGreaterThan(.56);expect(a.right.p[1]).toBeGreaterThan(.42);
  expect(Math.hypot(a.right.p[0]-.56,a.right.p[1]-.42)).toBeLessThanOrEqual(.000400001);
  stick.dispatchEvent(pointer('pointermove'));expect(r.step(40).right.grip).toBe(false);
 });
 it.each(['blur','hidden','disabled','stale','gap','pointercancel','lostpointercapture'])('cancels %s without resuming',reason=>{
  const r=rig(),stick=r.find('[data-joystick]');stick.dispatchEvent(pointer('pointerdown',180,100));r.panel.applyFrame(frame(0));
  if(reason==='blur')window.dispatchEvent(new Event('blur'));
  if(reason==='hidden'){Object.defineProperty(document,'visibilityState',{configurable:true,value:'hidden'});document.dispatchEvent(new Event('visibilitychange'));}
  if(reason==='disabled')r.panel.update(state,false);
  if(reason==='pointercancel'||reason==='lostpointercapture')stick.dispatchEvent(pointer(reason));
  if(reason==='stale')r.clock(101);
  expect(r.panel.applyFrame(frame(reason==='gap'?150:20)).right.grip).toBe(false);
  expect(r.step(160,'HOLD').right.grip).toBe(false);
 });
 it.each(['open','close'])('ramps claw %s through intermediate values and holds on release',action=>{
  const r=rig(),b=r.find(`[data-claw="${action}"]`);b.dispatchEvent(pointer('pointerdown'));
  r.panel.applyFrame(frame(0));r.step(40,'HOLD');expect(r.send).toHaveBeenCalledTimes(1);
  const first=r.send.mock.calls[0][0];expect(first.value).toBeGreaterThan(0);expect(first.value).toBeLessThan(1);
  expect(action==='open'?first.value<.5:first.value>.5).toBe(true);
  r.panel.onGripperResult({v:1,type:'gripper_result',request_id:first.request_id,accepted:true,message:'returned'});
  r.step(140,'HOLD');expect(r.send).toHaveBeenCalledTimes(2);
  b.dispatchEvent(pointer('pointerup'));expect(r.cancel).toHaveBeenCalledTimes(1);
  const count=r.send.mock.calls.length;r.step(160,'HOLD');r.step(180,'HOLD');expect(r.send).toHaveBeenCalledTimes(count);
  expect(r.panel.applyFrame(frame(200)).right.grip).toBe(false);
 });
 it('never queues increments while waiting for one claw reply',()=>{
  const r=rig(),b=r.find('[data-claw="close"]');b.dispatchEvent(pointer('pointerdown'));r.panel.applyFrame(frame(0));
  for(let t=40;t<=1000;t+=40)r.step(t,'HOLD');expect(r.send).toHaveBeenCalledTimes(1);
  window.dispatchEvent(new Event('blur'));expect(r.cancel).toHaveBeenCalledTimes(1);
  const m=r.send.mock.calls[0][0];r.panel.onGripperResult({v:1,type:'gripper_result',request_id:m.request_id,accepted:true,message:'late'});
  r.step(1040,'HOLD');expect(r.send).toHaveBeenCalledTimes(1);
 });
 it('refuses claw during motion and spin without backend capability',()=>{
  const r=rig();r.panel.update({...state,mode:'ACTIVE',robot_state:'MOVING',target_control:{...state.target_control!,tool_spin_supported:false}},true);
  expect(r.find('[data-claw="close"]').disabled).toBe(true);expect(r.find('[data-hold="spin+"]').disabled).toBe(true);
 });
 it('rotates about the tool axis rather than world Z',()=>{
  const r=rig(),button=r.find('[data-hold="spin+"]');
  r.panel.update({...state,actual_tcp:{...state.actual_tcp,q:[0,Math.SQRT1_2,0,Math.SQRT1_2]}},true);
  button.dispatchEvent(pointer('pointerdown'));r.panel.applyFrame(frame(0));
  const f=r.step(20);expect(Math.abs(f.right.q[0])).toBeGreaterThan(0);expect(Math.abs(f.right.q[2])).toBeLessThan(1e-8);
 });
 it('does not let an unrelated pointer release the held stick',()=>{
  const r=rig(),b=r.find('[data-joystick]');b.dispatchEvent(pointer('pointerdown',180,100));
  b.dispatchEvent(pointer('pointerup',100,100,8));expect(r.panel.applyFrame(frame(0)).right.grip).toBe(true);
  b.dispatchEvent(pointer('pointerup'));expect(r.panel.applyFrame(frame(20)).right.grip).toBe(false);
 });
 it('cannot refresh stale telemetry by redrawing the UI',()=>{
  const r=rig(),b=r.find('[data-joystick]');b.dispatchEvent(pointer('pointerdown',180,100));r.panel.applyFrame(frame(0));
  r.clock(101);r.panel.update(state,true,0);expect(r.panel.applyFrame(frame(20)).right.grip).toBe(false);
  r.step(120,'HOLD');expect(r.panel.applyFrame(frame(140)).right.grip).toBe(false);
 });
});
