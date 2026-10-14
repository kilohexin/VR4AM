// @vitest-environment jsdom
import {describe,it,expect,vi} from 'vitest';
import {HoldControlPanel} from '../src/ui/holdControlPanel';
import type {RobotStateMessage,VRFrame} from '../src/protocol/messages';
const state:RobotStateMessage={v:1,type:'robot_state',server_mono_ns:1,mode:'ARMED',robot_state:'IDLE',
 manual_input_ready:true,
 actual_tcp:{p:[.3,0,.4],q:[0,0,0,1]},actual_q:[0,0,0,0,0,0],gripper:0,backend:'LEBAI_FAKE',
 target_control:{policy:'segmented_experimental',phase:'ready',samples_observed:0,samples_required:3,
 hardware_verified:false,tool_spin_supported:true}};
function frame(t:number):VRFrame{return {v:1,type:'vr_frame',session_id:'manual',seq:t,client_mono_ms:t,
 tracking_valid:true,visibility:'visible',right:{p:[.56,.42,.18],q:[0,0,0,1],grip:false,trigger:0}};}
function pointer(type:string,id=7):Event {const e=new Event(type,{bubbles:true});Object.assign(e,{pointerId:id,button:0});return e;}
function rig(){const root=document.createElement('div');document.body.append(root);let now=0;const send=vi.fn();
 const panel=new HoldControlPanel(root,send,()=>now);
 const button=root.querySelector<HTMLButtonElement>('[data-hold="x+"]')!;
 button.setPointerCapture=vi.fn();button.hasPointerCapture=vi.fn(()=>true);button.releasePointerCapture=vi.fn();
 return {root,panel,send,button,clock:(t:number)=>{now=t;},time:(t:number)=>{now=t;panel.update(state,true);}};}
describe('hold controls use the existing VR frame motion contract',()=>{
 it('starts neutral, waits for ACTIVE and stops on pointer release',()=>{
  const r=rig();try{r.time(0);r.button.dispatchEvent(pointer('pointerdown'));
   expect(r.panel.applyFrame(frame(0)).right.grip).toBe(true);
   expect(r.panel.applyFrame(frame(20)).right.p).toEqual([.56,.42,.18]);
   r.panel.update({...state,mode:'ACTIVE'},true);
   const moved=r.panel.applyFrame(frame(40));expect(moved.right.p[0]).toBeGreaterThan(.56);
   r.button.dispatchEvent(pointer('pointerup'));
   const released=r.panel.applyFrame(frame(60));expect(released.right.grip).toBe(false);
   expect(released.right.p).toEqual(moved.right.p);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it.each(['blur','hidden','disabled','stale','gap'])('cancels %s without automatically resuming',reason=>{
  const r=rig();try{r.time(0);r.button.dispatchEvent(pointer('pointerdown'));r.panel.applyFrame(frame(0));
   if(reason==='blur')window.dispatchEvent(new Event('blur'));
   if(reason==='hidden'){Object.defineProperty(document,'visibilityState',{configurable:true,value:'hidden'});document.dispatchEvent(new Event('visibilitychange'));}
   if(reason==='disabled')r.panel.update(state,false);
   const t=(reason==='stale'||reason==='gap')?150:20;
   expect(r.panel.applyFrame(frame(t)).right.grip).toBe(false);
   Object.defineProperty(document,'visibilityState',{configurable:true,value:'visible'});
   r.time(t+1);expect(r.panel.applyFrame(frame(t+1)).right.grip).toBe(false);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it('disables spin until advertised and rotates around a tilted tool axis',()=>{
  const r=rig();try{r.panel.update({...state,target_control:{...state.target_control!,tool_spin_supported:false}},true);
   const b=r.root.querySelector<HTMLButtonElement>('[data-hold="spin+"]')!;expect(b.disabled).toBe(true);
   b.setPointerCapture=vi.fn();b.hasPointerCapture=vi.fn(()=>false);
   r.panel.update({...state,actual_tcp:{...state.actual_tcp,q:[0,Math.SQRT1_2,0,Math.SQRT1_2]}},true);
   b.dispatchEvent(pointer('pointerdown'));r.panel.applyFrame(frame(0));
   r.panel.update({...state,mode:'ACTIVE'},true);const f=r.panel.applyFrame(frame(20));
   expect(Math.abs(f.right.q[0])).toBeGreaterThan(0);expect(Math.abs(f.right.q[2])).toBeLessThan(1e-8);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it('sends one explicit claw request and blocks pending, locked and moving clicks',()=>{
  const r=rig();try{r.time(0);const b=r.root.querySelector<HTMLButtonElement>('[data-claw="1"]')!;
   b.click();b.click();expect(r.send).toHaveBeenCalledTimes(1);
   const request=r.send.mock.calls[0][0];expect(request).toMatchObject({type:'gripper_request',value:1});
   r.panel.onGripperResult({v:1,type:'gripper_result',request_id:request.request_id,accepted:true,message:'returned'});
   r.panel.update({...state,mode:'READY'},true);b.click();expect(r.send).toHaveBeenCalledTimes(1);
   r.panel.update({...state,mode:'ACTIVE',robot_state:'MOVING'},true);b.click();expect(r.send).toHaveBeenCalledTimes(1);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it('discards an old pending identity on disconnect without resending',()=>{
  const r=rig();try{r.time(0);const b=r.root.querySelector<HTMLButtonElement>('[data-claw="1"]')!;
   b.click();const old=r.send.mock.calls[0][0];r.panel.update(null,false);r.time(5);
   expect(b.disabled).toBe(false);expect(r.send).toHaveBeenCalledTimes(1);
   b.click();expect(r.send).toHaveBeenCalledTimes(2);
   r.panel.onGripperResult({v:1,type:'gripper_result',request_id:old.request_id,accepted:true,message:'late'});
   expect(b.disabled).toBe(true);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it('uses feedback arrival time and cannot refresh its age from UI updates',()=>{
  const r=rig();try{r.time(0);r.button.dispatchEvent(pointer('pointerdown'));r.panel.applyFrame(frame(0));
   r.clock(101);r.panel.update(state,true,0);
   expect(r.panel.applyFrame(frame(20)).right.grip).toBe(false);
   r.time(102);expect(r.panel.applyFrame(frame(40)).right.grip).toBe(false);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it.each(['pointercancel','lostpointercapture'])('stops on %s only for the owning pointer',kind=>{
  const r=rig();try{r.time(0);r.button.dispatchEvent(pointer('pointerdown'));
   r.button.dispatchEvent(pointer(kind,8));expect(r.panel.applyFrame(frame(0)).right.grip).toBe(true);
   r.button.dispatchEvent(pointer(kind,7));expect(r.panel.applyFrame(frame(20)).right.grip).toBe(false);
  }finally{r.panel.dispose();r.root.remove();}
 });
 it('publishes a release callback before the next animation frame, once',()=>{
  const root=document.createElement('div');document.body.append(root);const release=vi.fn();
  const panel=new HoldControlPanel(root,vi.fn(),()=>0,release);
  try{panel.update(state,true);const b=root.querySelector<HTMLButtonElement>('[data-hold="x+"]')!;
   b.setPointerCapture=vi.fn();b.hasPointerCapture=vi.fn(()=>false);
   b.dispatchEvent(pointer('pointerdown'));b.dispatchEvent(pointer('pointerup'));
   expect(release).toHaveBeenCalledTimes(1);panel.cancel();expect(release).toHaveBeenCalledTimes(1);
   expect(panel.applyFrame(frame(0)).right.grip).toBe(false);
  }finally{panel.dispose();root.remove();}
 });
});
