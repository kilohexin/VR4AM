// Product WebSocket and actual FiniteXRInput, synthetic hand, local Docker only.
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame} from '../src/xr/finiteInput';
import {Quaternion} from 'three';
const port=process.argv[2]??'8774';
if(!/^\d{4,5}$/.test(port))throw Error('invalid_port');
const origin=`http://127.0.0.1:${port}`;
const before:any=await(await fetch(origin+'/simulation/status')).json();
if(before.runtime!=='LEBAI_DOCKER'||!before.pose_follow_vr||!before.simulator||before.hardware_verified
 ||!(before.sdk_target==='127.0.0.1:3030'||before.isolated_test===true
  &&before.sdk_target==='127.0.0.1:13030'&&before.admission?.port===13030))throw Error('wrong_runtime');
const socket=new WebSocket(`ws://127.0.0.1:${port}/ws/v1/teleop`);
let state:any=null,generation=-1,seq=0,received=-Infinity,grip=false,hand=[0,0,0],timer:any;
let handQ:[number,number,number,number]=[0,0,0,1],clawDirection:-1|0|1=0;
let lastPoseSeq=0;const holds=new Set<string>();
const replies=new Map();
const send=(m:any)=>{if(socket.readyState===1){socket.send(JSON.stringify(m));
 if(m.type==='finite_position_input'&&m.hand_pose)lastPoseSeq=m.seq;}};
let maxFrameGap=0,lastFrameTime:number|null=null;
const input=new FinitePositionInput(send,reason=>{
 console.error('XR_INPUT_ABORT',JSON.stringify({reason,maxFrameGap,phase:state?.finite_position?.phase}));
 send({v:1,type:'disarm',request_id:'input-abort'});
});
const xr=new FiniteXRInput(input,m=>send(m),id=>send({v:1,type:'gripper_cancel',request_id:id}));
socket.onmessage=e=>{
 const m=JSON.parse(String(e.data));
 if(m.type==='robot_state'){
  state=m;received=performance.now();
  if(m.finite_position?.phase==='hold'&&m.finite_position.reason)holds.add(m.finite_position.reason);
  if(generation!==m.finite_position.generation){generation=m.finite_position.generation;input.reset('xr-finite-pose-check',generation);}
  input.observe(m.finite_position.phase);xr.update(m,received);
 }else if(m.request_id){replies.set(m.request_id,m);if(m.type==='gripper_result')xr.onGripperResult(m);}
};
const sleep=(ms:number)=>new Promise(r=>setTimeout(r,ms));
async function wait(p:()=>boolean,label:string,ms=12000){const end=performance.now()+ms;
 while(!p()){if(state?.fault)throw Error(label+':'+state.fault);if(performance.now()>end)throw Error(label+':timeout:'+JSON.stringify(state?.finite_position));await sleep(20);}}
async function command(type:string,id:string){send({v:1,type,request_id:id});await wait(()=>replies.has(id),id);
 if(String(replies.get(id).type).endsWith('rejected')||replies.get(id).accepted===false)throw Error(id+':rejected:'+JSON.stringify(replies.get(id)));}
async function startFollow(label:string){
 grip=false;await sleep(60);grip=true;await wait(()=>state.finite_position.phase==='executing',label);
 grip=false;await sleep(60);
}
async function stopFollow(label:string){
 grip=false;await sleep(60);grip=true;await sleep(60);grip=false;
 await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE',label);
}
await new Promise<void>((resolve,reject)=>{socket.onopen=()=>resolve();socket.onerror=()=>reject(Error('websocket_open'));});
try{
 send({v:1,type:'hello',request_id:'hello'});
 timer=setInterval(()=>{
  const t=performance.now();
  if(lastFrameTime!==null)maxFrameGap=Math.max(maxFrameGap,t-lastFrameTime);lastFrameTime=t;
  const f:any={v:1,type:'vr_frame',session_id:'quest-pose-check',seq:++seq,client_mono_ms:t,
   tracking_valid:true,visibility:'visible',right:{p:[...hand],q:[...handQ],grip,trigger:0}};
  xr.holdClaw(clawDirection,t);xr.apply(f);send(finiteXRTransportFrame(f));
 },20);
 await wait(()=>state?.mode==='READY'&&state.ack_seq>0,'ready');
 await command('arm_request','arm');await command('prepare_request','prepare');
 await wait(()=>state.manual_input_ready&&state.finite_position.phase==='ready','prepared',30000);
 await sleep(100);await startFollow('pose_started');
 const base=[...state.actual_tcp.p],steps=state.finite_position.entered_steps;
 const start=performance.now();
 const humanSpeed=process.argv.includes('--human-speed');
 while(performance.now()-start<2000){hand=[(humanSpeed?.015:.0015)*Math.min(1,(performance.now()-start)/(humanSpeed?400:1000)),0,0];await sleep(20);}
 await wait(()=>state.finite_position.entered_steps>=steps+25&&state.actual_tcp.p[0]-base[0]>.001,'full_pose_progress');
 const fixed=state.actual_tcp.p[0];await sleep(500);
 if(state.actual_tcp.p[0]-base[0]>(humanSpeed?.035:.005))throw Error('stationary_hand_kept_integrating');
 await stopFollow('toggle_off');
 console.log('PASS actual XR module -> full hand pose -> controller IK/PVAT, fixed hand, toggle off',state.finite_position.entered_steps);
 // Re-grip at a different hand origin must capture a new zero, not replay.
 hand=[.1,.1,.1];await sleep(100);const second=[...state.actual_tcp.p];
 await startFollow('regrip');await sleep(500);
 if(Math.hypot(...state.actual_tcp.p.map((v:number,i:number)=>v-second[i]))>.001)throw Error('regrip_jump');
 await stopFollow('second_toggle_off');
 console.log('PASS re-grip captures a fresh zero');
 if(process.argv.includes('--rotation-response')){
  if(before.pose_reference_speed_radps!==.2)throw Error('wrong_response_profile');
  const initial=new Quaternion(...state.actual_tcp.q),zero=new Quaternion(...handQ);
  await startFollow('response_start');
  const target=new Quaternion(0,Math.sin(.15),0,Math.cos(.15));
  handQ=zero.clone().multiply(target).toArray() as typeof handQ;
  const since=performance.now();
  await wait(()=>new Quaternion(...state.actual_tcp.q).angleTo(initial)>.27,'response_90_percent',5000);
  const responseMs=performance.now()-since;
  if(responseMs>=3500)throw Error('response_still_too_slow:'+responseMs);
  handQ=zero.toArray() as typeof handQ;
  await wait(()=>new Quaternion(...state.actual_tcp.q).angleTo(initial)<.03,'response_return',6000);
  await stopFollow('response_toggle_off');
  console.log('PASS measured 0.3rad orientation step to 90%',JSON.stringify({responseMs}));
  await startFollow('rotation_limit_start');
  handQ=zero.clone().multiply(new Quaternion(0,Math.sin(.975),0,Math.cos(.975))).toArray() as typeof handQ;
  await wait(()=>state.finite_position.reason==='pvat_rotation_saturated','orientation_saturation');
  const initialSteps=state.finite_position.entered_steps;
  await sleep(2500);
  if(state.finite_position.phase!=='executing'||state.finite_position.entered_steps<initialSteps+20)throw Error('rotation_limit_exited_follow');
  const edge=new Quaternion(...state.actual_tcp.q).angleTo(initial);
  handQ=zero.toArray() as typeof handQ;
  await wait(()=>state.finite_position.reason!=='pvat_rotation_saturated'
   &&new Quaternion(...state.actual_tcp.q).angleTo(initial)<edge-.03,'rotation_turn_back',8000);
  await stopFollow('rotation_limit_toggle_off');
  console.log('PASS oversized orientation clips within original range, stays following and turns back');
 }
 if(process.argv.includes('--continuation')){
  if(before.pose_follow_window_s!==300||before.pose_follow_grip_span_mm!==100)throw Error('wrong_pose_profile');
  const initial=[...state.actual_tcp.p],handOrigin=[...hand],initialSteps=state.finite_position.entered_steps;
  await startFollow('continuation_start');
  const since=performance.now();let peak=0,maxJointSpan=0;const joints=[...state.actual_q];
  const holdMs=process.argv.includes('--hold-90')?90000:40000;
  while(performance.now()-since<holdMs){
   const t=(performance.now()-since)/1000;
   const u=t<10?t/10:t<22?1:t<32?(32-t)/10:0;
   hand=[handOrigin[0]+.015*u,handOrigin[1],handOrigin[2]];
   peak=Math.max(peak,state.actual_tcp.p[0]-initial[0]);
   maxJointSpan=Math.max(maxJointSpan,...state.actual_q.map((v:number,i:number)=>Math.abs(v-joints[i])));
   if(state.fault||state.finite_position.phase==='hold'||state.finite_position.phase==='fault'
      ||state.finite_position.phase==='ready')throw Error('continuation_interrupted:'+JSON.stringify(state.finite_position));
   await sleep(25);
  }
  if(peak<.022||state.finite_position.entered_steps-initialSteps<Math.floor(holdMs/80*.8)||lastPoseSeq<1024)throw Error('continuation_insufficient');
  if(grip)throw Error('button_should_remain_released_during_follow');
  await stopFollow('continuation_toggle_off');
  console.log(`PASS single Grip >${holdMs/1000}s, beyond 20mm, sequence wrap`,JSON.stringify({peak,maxJointSpan,lastPoseSeq}));
  const stationary=[...state.actual_tcp.p];await startFollow('boundary_start');
  hand=[hand[0]+.06,hand[1],hand[2]]; // +120mm target outside fixed 100mm Grip box.
  await wait(()=>state.finite_position.reason==='pvat_target_saturated','fixed_grip_saturation');
  const limitedSteps=state.finite_position.entered_steps;await sleep(400);
  if(state.finite_position.phase!=='executing'||state.finite_position.entered_steps<=limitedSteps
    ||Math.hypot(...state.actual_tcp.p.map((v:number,i:number)=>v-stationary[i]))>.102)throw Error('saturation_did_not_continue_inside_fixed_box');
  hand[0]-=.06;
  await wait(()=>state.finite_position.reason!=='pvat_target_saturated','return_inside_fixed_box');
  await stopFollow('after_saturation_toggle_off');
  await startFollow('after_boundary_regrip');
  await sleep(400);await stopFollow('after_boundary_toggle_off');
  console.log('PASS fixed Grip box clips targets, continues and returns inward; explicit re-grip works');
 }
 if(process.argv.includes('--extended')){
  for(const [handAxis,sign,robotAxis] of [[1,1,2],[2,-1,1]]){
   const initial=[...state.actual_tcp.p];await startFollow('axis_start');
   hand[handAxis]+=.001*sign;
   await wait(()=>state.actual_tcp.p[robotAxis]-initial[robotAxis]>.0006,'axis_progress');
   await stopFollow('axis_toggle_off');
   console.log('PASS XR axis mapping',handAxis,robotAxis);
  }
  for(const axis of [0,1]){
   const initial=new Quaternion(...state.actual_tcp.q);await startFollow('rotation_start');
   handQ=[0,0,0,Math.cos(.005)];handQ[axis]=Math.sin(.005);
   await wait(()=>new Quaternion(...state.actual_tcp.q).angleTo(initial)>.003,'rotation_progress');
   await stopFollow('rotation_toggle_off');
   console.log('PASS complete quaternion rotation',axis);
  }
  await startFollow('claw_parallel_start');
  const initial=state.gripper,steps=state.finite_position.entered_steps;
  clawDirection=initial>.5?-1:1;await sleep(200);clawDirection=0;
  await wait(()=>Math.abs(state.gripper-initial)>.05&&state.finite_position.entered_steps>steps+3,'claw_parallel');
  await stopFollow('claw_parallel_toggle_off');
  console.log('PASS held X/Y analogue claw while Grip tracking continues');
 }
 if(process.argv.includes('--disconnect-active')){
  await startFollow('disconnect_grip');
  const initial=[...state.actual_tcp.p],steps=state.finite_position.entered_steps;
  hand[0]+=.002;
  await wait(()=>state.finite_position.entered_steps>=steps+6&&state.actual_tcp.p[0]-initial[0]>.0005,'active_disconnect_progress');
  console.log('PASS reached actual pose motion before active disconnect');
 }else await command('disarm','disarm');
}finally{
 // Active-disconnect scenario deliberately sends NO release packet: transport
 // cleanup and the controller input lease must retire the running stream.
 grip=false;if(!process.argv.includes('--disconnect-active'))input.release();
 if(timer)clearInterval(timer);socket.close();
 console.log('XR_FRAME_METRICS',JSON.stringify({maxFrameGap}));
}
await sleep(3000);
const after:any=await(await fetch(origin+'/simulation/status')).json();
if(after.control_fault||after.session_owned||after.unknown_reads.length||after.unknown_writes.length)throw Error('unclean_end:'+JSON.stringify(after));
console.log('PASS clean product pose disconnect',JSON.stringify(after));
