// Product WebSocket and actual FiniteXRInput, synthetic hand, local Docker only.
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame} from '../src/xr/finiteInput';
import {Quaternion} from 'three';
const port=process.argv[2]??'8774';
if(!/^\d{4,5}$/.test(port))throw Error('invalid_port');
const origin=`http://127.0.0.1:${port}`;
const before:any=await(await fetch(origin+'/simulation/status')).json();
if(before.runtime!=='LEBAI_DOCKER'||before.sdk_target!=='127.0.0.1:3030'||!before.pose_follow_vr)throw Error('wrong_runtime');
const socket=new WebSocket(`ws://127.0.0.1:${port}/ws/v1/teleop`);
let state:any=null,generation=-1,seq=0,received=-Infinity,grip=false,hand=[0,0,0],timer:any;
let handQ:[number,number,number,number]=[0,0,0,1],clawDirection:-1|0|1=0;
const replies=new Map();
const send=(m:any)=>{if(socket.readyState===1)socket.send(JSON.stringify(m));};
const input=new FinitePositionInput(send,()=>send({v:1,type:'disarm',request_id:'input-abort'}));
const xr=new FiniteXRInput(input,m=>send(m),id=>send({v:1,type:'gripper_cancel',request_id:id}));
socket.onmessage=e=>{
 const m=JSON.parse(String(e.data));
 if(m.type==='robot_state'){
  state=m;received=performance.now();
  if(generation!==m.finite_position.generation){generation=m.finite_position.generation;input.reset('xr-finite-pose-check',generation);}
  input.observe(m.finite_position.phase);xr.update(m,received);
 }else if(m.request_id){replies.set(m.request_id,m);if(m.type==='gripper_result')xr.onGripperResult(m);}
};
const sleep=(ms:number)=>new Promise(r=>setTimeout(r,ms));
async function wait(p:()=>boolean,label:string,ms=12000){const end=performance.now()+ms;
 while(!p()){if(state?.fault)throw Error(label+':'+state.fault);if(performance.now()>end)throw Error(label+':timeout:'+JSON.stringify(state?.finite_position));await sleep(20);}}
async function command(type:string,id:string){send({v:1,type,request_id:id});await wait(()=>replies.has(id),id);
 if(String(replies.get(id).type).endsWith('rejected')||replies.get(id).accepted===false)throw Error(id+':rejected:'+JSON.stringify(replies.get(id)));}
await new Promise<void>((resolve,reject)=>{socket.onopen=()=>resolve();socket.onerror=()=>reject(Error('websocket_open'));});
try{
 send({v:1,type:'hello',request_id:'hello'});
 timer=setInterval(()=>{
  const t=performance.now();
  const f:any={v:1,type:'vr_frame',session_id:'quest-pose-check',seq:++seq,client_mono_ms:t,
   tracking_valid:true,visibility:'visible',right:{p:[...hand],q:[...handQ],grip,trigger:0}};
  xr.holdClaw(clawDirection,t);xr.apply(f);send(finiteXRTransportFrame(f));
 },20);
 await wait(()=>state?.mode==='READY'&&state.ack_seq>0,'ready');
 await command('arm_request','arm');await command('prepare_request','prepare');
 await wait(()=>state.manual_input_ready&&state.finite_position.phase==='ready','prepared',30000);
 await sleep(100);grip=true;
 await wait(()=>state.finite_position.phase==='executing','pose_started');
 const base=[...state.actual_tcp.p],steps=state.finite_position.entered_steps;
 const start=performance.now();
 while(performance.now()-start<2000){hand=[.0015*Math.min(1,(performance.now()-start)/1000),0,0];await sleep(20);}
 await wait(()=>state.finite_position.entered_steps>=steps+25&&state.actual_tcp.p[0]-base[0]>.001,'full_pose_progress');
 const fixed=state.actual_tcp.p[0];await sleep(500);
 if(state.actual_tcp.p[0]-base[0]>.005)throw Error('stationary_hand_kept_integrating');
 grip=false;input.release();await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','release');
 console.log('PASS actual XR module -> full hand pose -> controller IK/PVAT, fixed hand, release',state.finite_position.entered_steps);
 // Re-grip at a different hand origin must capture a new zero, not replay.
 hand=[.1,.1,.1];await sleep(100);const second=[...state.actual_tcp.p];grip=true;
 await wait(()=>state.finite_position.phase==='executing','regrip');await sleep(500);
 if(Math.hypot(...state.actual_tcp.p.map((v:number,i:number)=>v-second[i]))>.001)throw Error('regrip_jump');
 grip=false;input.release();await wait(()=>state.finite_position.phase==='ready','second_release');
 console.log('PASS re-grip captures a fresh zero');
 if(process.argv.includes('--extended')){
  for(const [handAxis,sign,robotAxis] of [[1,1,2],[2,-1,1]]){
   const initial=[...state.actual_tcp.p];grip=true;
   await wait(()=>state.finite_position.phase==='executing','axis_start');
   hand[handAxis]+=.001*sign;
   await wait(()=>state.actual_tcp.p[robotAxis]-initial[robotAxis]>.0006,'axis_progress');
   grip=false;input.release();await wait(()=>state.finite_position.phase==='ready','axis_release');
   console.log('PASS XR axis mapping',handAxis,robotAxis);
  }
  for(const axis of [0,1]){
   const initial=new Quaternion(...state.actual_tcp.q);grip=true;
   await wait(()=>state.finite_position.phase==='executing','rotation_start');
   handQ=[0,0,0,Math.cos(.005)];handQ[axis]=Math.sin(.005);
   await wait(()=>new Quaternion(...state.actual_tcp.q).angleTo(initial)>.003,'rotation_progress');
   grip=false;input.release();await wait(()=>state.finite_position.phase==='ready','rotation_release');
   console.log('PASS complete quaternion rotation',axis);
  }
  grip=true;await wait(()=>state.finite_position.phase==='executing','claw_parallel_start');
  const initial=state.gripper,steps=state.finite_position.entered_steps;
  clawDirection=initial>.5?-1:1;await sleep(200);clawDirection=0;
  await wait(()=>Math.abs(state.gripper-initial)>.05&&state.finite_position.entered_steps>steps+3,'claw_parallel');
  grip=false;input.release();await wait(()=>state.finite_position.phase==='ready','claw_parallel_release');
  console.log('PASS held X/Y analogue claw while Grip tracking continues');
 }
 await command('disarm','disarm');
}finally{
 grip=false;input.release();if(timer)clearInterval(timer);socket.close();
}
await sleep(3000);
const after:any=await(await fetch(origin+'/simulation/status')).json();
if(after.control_fault||after.session_owned||after.unknown_reads.length||after.unknown_writes.length)throw Error('unclean_end:'+JSON.stringify(after));
console.log('PASS clean product pose disconnect',JSON.stringify(after));
