// Explicit Docker-only acceptance client. Close other control pages before running.
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame,relativeToolPitch,relativeToolSpin} from '../src/xr/finiteInput';

const port=process.argv[2]??'8773';
if(!/^\d{4,5}$/.test(port)||Number(port)<1024||Number(port)>65535)throw Error('invalid_local_port');
const origin=`http://127.0.0.1:${port}`;
const admitted=await(await fetch(origin+'/simulation/status')).json() as any;
if(admitted.runtime!=='LEBAI_DOCKER'||admitted.sdk_target!=='127.0.0.1:3030'||admitted.joint_speed_cap_radps!==.6)throw Error('wrong_simulator');
const ws=new WebSocket(`ws://127.0.0.1:${port}/ws/v1/teleop`);
let state:any=null,generation=-1,seq=0,timer:any=null,closed=false;
let continuous=false;
let directInput=false;
const replies=new Map<string,any>();
const hand={p:[0,0,0],q:[0,0,0,1],grip:false,trigger:0,tracked:true};
const send=(m:any)=>{if(ws.readyState===1)ws.send(JSON.stringify(m));};
const input=new FinitePositionInput(send,reason=>send({v:1,type:'disarm',request_id:`xr-abort-${reason}-${Date.now()}`}));
const xr=new FiniteXRInput(input,send,id=>send({v:1,type:'gripper_cancel',request_id:id}));
ws.onmessage=e=>{
 const m=JSON.parse(String(e.data));
 if(m.type==='robot_state'){
  if(m.backend!=='LEBAI_DOCKER')throw Error('not_Docker');
  if(m.finite_position.generation!==generation){generation=m.finite_position.generation;input.reset('xr-finite-synthetic',generation);}
  state=m;input.observe(m.finite_position.phase);xr.update(m,performance.now());
 }else{
  if(m.request_id)replies.set(m.request_id,m);
  if(m.type==='gripper_result')xr.onGripperResult(m);
  if(m.type==='connection_rejected')throw Error('connection_rejected:'+m.reason);
 }
};
const sleep=(ms:number)=>new Promise(r=>setTimeout(r,ms));
async function wait(p:()=>boolean,label:string,ms=8000){
 const end=performance.now()+ms;
 while(!p()){
  if(state?.fault)throw Error(label+':'+state.fault);
  if(performance.now()>end)throw Error(label+':timeout:'+JSON.stringify(state?.finite_position));
  await sleep(15);
 }
}
async function command(type:string,id:string){
 send({v:1,type,request_id:id});await wait(()=>replies.has(id),id,type==='prepare_request'?65000:5000);
 if(replies.get(id).type.endsWith('rejected'))throw Error(JSON.stringify(replies.get(id)));
}
async function prepare(id:string){
 await wait(()=>state?.mode==='READY'&&state.ack_seq>0,'ready_with_released_frame');
 await command('arm_request','arm-'+id);await command('prepare_request','prepare-'+id);
 await wait(()=>state?.manual_input_ready===true,'prepared');
 await sleep(100); // Real released XR frames establish the new generation's release barrier.
 console.log('PASS',id,'prepare');
}
async function status(){return await(await fetch(origin+'/simulation/status')).json() as any;}
try{
 await new Promise<void>((resolve,reject)=>{ws.onopen=()=>resolve();ws.onerror=()=>reject(Error('socket_error'));});
 send({v:1,type:'hello',request_id:'hello'});
 timer=setInterval(()=>{
  const now=performance.now();const frame:any={v:1,type:'vr_frame',session_id:'quest-synthetic',seq:++seq,
   client_mono_ms:now,tracking_valid:hand.tracked,visibility:'visible',
   right:{p:[...hand.p],q:[...hand.q],grip:hand.grip,trigger:hand.trigger}};
  if(continuous)input.offer([0,0,0],0,Math.floor(state.finite_position.entered_steps/2)%2===0?1:-1);
  else if(!directInput)xr.apply(frame);
  send(finiteXRTransportFrame(frame));
 },20);
 await prepare('initial');
 const preparedGeneration=generation;
 if(process.argv.includes('--continuous')){
  const started=performance.now();directInput=true;continuous=true;
  await wait(()=>state.finite_position.entered_steps>=12&&performance.now()-started>11000,'continuous_gesture',45000);
  continuous=false;input.release();
  await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','continuous_release');
  console.log('PASS continuous gesture beyond 10s/10 steps',state.finite_position.entered_steps,performance.now()-started);
  xr.reset();directInput=false;await sleep(100);
 }
 for(const axis of [0,1])for(const sign of [1,-1]){
  hand.p=[0,0,0];hand.q=[0,0,0,1];hand.grip=false;await sleep(100);
  const x=state.actual_tcp.p[axis];hand.grip=true;await sleep(80);hand.p[axis]=sign*.018;
  await wait(()=>sign*(state.actual_tcp.p[axis]-x)>.012,'XR_planar_continuation',12000);
  hand.grip=false;
  await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_planar_release');
  if(generation!==preparedGeneration)throw Error('unexpected_reprepare');
  console.log('PASS XR planar continuation/reversal',axis,sign,state.actual_tcp.p[axis]-x);
 }
 hand.p=[0,0,0];await sleep(100);
 const z=state.actual_tcp.p[2];hand.grip=true;await sleep(80);hand.p[2]=.01;
 await wait(()=>state.actual_tcp.p[2]>z+.004,'XR_translation');hand.grip=false;
 await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_translation_arrival');
 console.log('PASS XR translation/release',state.actual_tcp.p[2]-z);
 const q=[...state.actual_tcp.q];hand.p=[0,0,0];hand.q=[0,0,0,1];await sleep(80);
 hand.grip=true;await sleep(80);hand.q=[Math.sin(.032/2),0,0,Math.cos(.032/2)];
 await wait(()=>Math.abs(relativeToolPitch(q as any,state.actual_tcp.q))>.013,'XR_pitch');hand.grip=false;
 await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_pitch_arrival');
 console.log('PASS XR pitch/release',relativeToolPitch(q as any,state.actual_tcp.q));
 const spinQ=[...state.actual_tcp.q];hand.p=[0,0,0];hand.q=[0,0,0,1];await sleep(100);
 hand.grip=true;await sleep(80);hand.q=[0,0,Math.sin(.032/2),Math.cos(.032/2)];
 await wait(()=>Math.abs(relativeToolSpin(spinQ as any,state.actual_tcp.q))>.013,'XR_spin');hand.grip=false;
 await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_spin_arrival');
 console.log('PASS XR spin/release',relativeToolSpin(spinQ as any,state.actual_tcp.q));
 // Reposition a released hand far away, then regrip without moving it.
 // Reacquisition must anchor to the current TCP, not replay the old hand target.
 hand.p=[.3,.2,.1];hand.q=[0,Math.sin(.2),0,Math.cos(.2)];await sleep(120);
 const beforeRegrip=await status();const regripTcp=[...state.actual_tcp.p];
 hand.grip=true;await sleep(350);hand.grip=false;
 const afterRegrip=await status();
 if(afterRegrip.moves!==beforeRegrip.moves||Math.hypot(...state.actual_tcp.p.map((v:number,i:number)=>v-regripTcp[i]))>.0001)
  throw Error('stationary_regrip_moved');
 console.log('PASS stationary regrip no jump');
 hand.trigger=.35;await wait(()=>state.gripper<.5,'XR_claw_open');
 hand.trigger=.6;await wait(()=>state.gripper>.55,'XR_claw_close');
 console.log('PASS XR analogue claw',state.gripper);
 hand.p=[0,0,0];await sleep(80);hand.grip=true;await sleep(80);hand.p[2]=.01;
 await wait(()=>state.finite_position.phase==='executing','XR_loss_entry');hand.tracked=false;
 await wait(()=>['DISARMED','READY'].includes(state.mode)&&state.robot_state==='IDLE','XR_tracking_stop',6000);
 hand.grip=false;hand.tracked=true;hand.p=[0,0,0];hand.q=[0,0,0,1];await sleep(100);
 await prepare('after_tracking_loss');console.log('PASS tracking loss/reprepare');
 clearInterval(timer);timer=null;ws.close();closed=true;
 let s=await status();const end=performance.now()+6000;
 while(s.session_owned&&performance.now()<end){await sleep(50);s=await status();}
 if(s.session_owned||s.control_fault||s.unknown_reads.length||s.unknown_writes.length)throw Error('disconnect:'+JSON.stringify(s));
 console.log('PASS disconnect',JSON.stringify(s));
}finally{if(timer)clearInterval(timer);if(!closed)ws.close();}
