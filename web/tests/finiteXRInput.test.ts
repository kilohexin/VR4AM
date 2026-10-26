import {it,expect} from 'vitest';
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame} from '../src/xr/finiteInput';
import type {RobotStateMessage,VRFrame} from '../src/protocol/messages';

const state:RobotStateMessage={v:1,type:'robot_state',server_mono_ns:1,mode:'ARMED',robot_state:'IDLE',
 actual_tcp:{p:[.3,0,.4],q:[0,0,0,1]},actual_q:[0,0,0,0,0,0],gripper:.5,backend:'LEBAI_FAKE',manual_input_ready:true,
 translation_scale:1,finite_position:{policy:'finite_position_offline',phase:'ready',generation:1,entered_steps:0,
 motion_id:null,reason:null,hardware_verified:false,vr_enabled:true}};
it('keeps XR Grip and Trigger out of legacy transport without mutating controller input',()=>{
 const f:VRFrame={v:1,type:'vr_frame',session_id:'quest-test',seq:1,client_mono_ms:1,tracking_valid:true,
  visibility:'visible',right:{p:[0,0,0],q:[0,0,0,1],grip:true,trigger:.8}};
 expect(finiteXRTransportFrame(f).right).toMatchObject({grip:false,trigger:0});
 expect(f.right).toMatchObject({grip:true,trigger:.8});
});
function rig(){
 let now=0;const inputs:any[]=[],claws:any[]=[],cancelled:string[]=[],stops:string[]=[];
 const input=new FinitePositionInput(m=>inputs.push(m),r=>stops.push(r??'stop'));input.reset('xr-finite-test',1);
 const xr=new FiniteXRInput(input,m=>claws.push(m),id=>cancelled.push(id),()=>now);
 function frame(t:number,grip=false,p:[number,number,number]=[0,0,0],trigger=0,tracked=true){now=t;xr.update(state,t);
  xr.apply({v:1,type:'vr_frame',session_id:'quest-test',seq:t,client_mono_ms:t,tracking_valid:tracked,visibility:'visible',
   right:{p,q:[0,0,0,1],grip,trigger}} as VRFrame);}
 return {xr,input,inputs,claws,cancelled,stops,frame,setNow:(t:number)=>{now=t;}};
}
it('maps a released-then-gripped hand into finite directions and releases without disarming',()=>{
 const r=rig();r.frame(0);r.frame(20,true);r.frame(40,true,[.01,0,0]);
 expect(r.inputs.at(-1)).toMatchObject({kind:'held',vector:[1,0,0]});
 r.frame(60,false,[.01,0,0]);expect(r.inputs.at(-1).kind).toBe('release');expect(r.stops).toEqual([]);
});
it('uses reported TCP to stop following a reached target instead of accumulating goals',()=>{
 const r=rig();r.frame(0);r.frame(20,true);r.frame(40,true,[.002,0,0]);
 r.xr.update({...state,actual_tcp:{p:[.302,0,.4],q:[0,0,0,1]}},40);
 r.xr.apply({v:1,type:'vr_frame',session_id:'quest-test',seq:41,client_mono_ms:40,tracking_valid:true,
  visibility:'visible',right:{p:[.002,0,0],q:[0,0,0,1],grip:true,trigger:0}});
 expect(r.inputs.at(-1).kind).toBe('release');
});
it('requires release after tracking loss and never resumes an old held gesture',()=>{
 const r=rig();r.frame(0);r.frame(20,true);r.frame(40,true,[.01,0,0]);r.frame(60,true,[.01,0,0],0,false);
 const count=r.inputs.length;r.frame(80,true,[.1,0,0]);expect(r.inputs).toHaveLength(count);
 expect(r.stops).toHaveLength(1);r.frame(100);r.frame(120,true);r.frame(140,true,[0,.01,0]);
 expect(r.inputs.at(-1).vector).toEqual([0,1,0]);
});
it('keeps one claw target in flight and does not cancel it when Trigger is released',()=>{
 const r=rig();r.frame(0);r.frame(20,false,[0,0,0],.7);expect(r.claws).toHaveLength(1);
 r.frame(40);expect(r.claws).toHaveLength(1);expect(r.cancelled).toEqual([]);
 r.xr.onGripperResult({v:1,type:'gripper_result',request_id:r.claws[0].request_id,accepted:true,message:'returned'});
 r.frame(140);expect(r.claws).toHaveLength(2);expect(r.claws[1].value).toBe(0);
});
it('rejects unprepared, stale and VR-disabled states without commands',()=>{
 for(const patch of [{manual_input_ready:false},{finite_position:{...state.finite_position!,vr_enabled:false}}]){
  const r=rig();r.xr.update({...state,...patch},0);
  r.xr.apply({v:1,type:'vr_frame',session_id:'quest-test',seq:1,client_mono_ms:1,tracking_valid:true,
   visibility:'visible',right:{p:[1,0,0],q:[0,0,0,1],grip:true,trigger:1}});
  expect(r.inputs).toEqual([]);expect(r.claws).toEqual([]);
 }
 const r=rig();r.frame(0);r.setNow(101);r.xr.apply({v:1,type:'vr_frame',session_id:'quest-test',seq:1,
  client_mono_ms:101,tracking_valid:true,visibility:'visible',right:{p:[1,0,0],q:[0,0,0,1],grip:true,trigger:1}});
 expect(r.inputs).toEqual([]);expect(r.claws).toEqual([]);
});
it('revokes the old hand anchor on authority generation changes',()=>{
 const r=rig();r.frame(0);r.frame(20,true);r.frame(40,true,[.01,0,0]);
 r.xr.update({...state,finite_position:{...state.finite_position!,generation:2}},40);
 const count=r.inputs.length;
 r.xr.apply({v:1,type:'vr_frame',session_id:'quest-test',seq:60,client_mono_ms:60,tracking_valid:true,
  visibility:'visible',right:{p:[.1,0,0],q:[0,0,0,1],grip:true,trigger:0}});
 expect(r.inputs).toHaveLength(count);expect(r.stops).toContain('xr_generation_changed');
});
it('waits for a selected claw result across feedback latency and blocks a competing Grip',()=>{
 const r=rig();r.frame(0);r.frame(20,false,[0,0,0],.7);
 r.setNow(121);r.xr.apply({v:1,type:'vr_frame',session_id:'quest-test',seq:121,client_mono_ms:100,
  tracking_valid:true,visibility:'visible',right:{p:[.01,0,0],q:[0,0,0,1],grip:true,trigger:0}});
 expect(r.cancelled).toEqual([]);expect(r.inputs).toEqual([]);expect(r.claws).toHaveLength(1);
});
