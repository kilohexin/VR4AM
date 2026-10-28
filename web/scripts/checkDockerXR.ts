// Explicit Docker-only acceptance client. Close other control pages before running.
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {FiniteXRInput,finiteXRTransportFrame,relativeToolPitch} from '../src/xr/finiteInput';

const port=process.argv[2]??'8773';
if(!/^\d{4,5}$/.test(port)||Number(port)<1024||Number(port)>65535)throw Error('invalid_local_port');
const origin=`http://127.0.0.1:${port}`;
const admitted=await(await fetch(origin+'/simulation/status')).json() as any;
if(admitted.runtime!=='LEBAI_DOCKER'||admitted.sdk_target!=='127.0.0.1:3030'||admitted.joint_speed_cap_radps!==.3)throw Error('wrong_simulator');
const ws=new WebSocket(`ws://127.0.0.1:${port}/ws/v1/teleop`);
let state:any=null,generation=-1,seq=0,timer:any=null,closed=false;
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
 await wait(()=>state?.mode==='READY','ready');
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
  xr.apply(frame);send(finiteXRTransportFrame(frame));
 },20);
 await prepare('initial');
 const preparedGeneration=generation;
 for(const sign of [1,-1]){
  hand.p=[0,0,0];hand.q=[0,0,0,1];hand.grip=false;await sleep(100);
  const x=state.actual_tcp.p[0];hand.grip=true;await sleep(80);hand.p[0]=sign*.0045;
  await wait(()=>sign*(state.actual_tcp.p[0]-x)>.003,'XR_planar_continuation',12000);
  hand.grip=false;
  await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_planar_release');
  if(generation!==preparedGeneration)throw Error('unexpected_reprepare');
  console.log('PASS XR planar continuation/reversal',sign,state.actual_tcp.p[0]-x);
 }
 hand.p=[0,0,0];await sleep(100);
 const z=state.actual_tcp.p[2];hand.grip=true;await sleep(80);hand.p[2]=.0025;
 await wait(()=>state.actual_tcp.p[2]>z+.001,'XR_translation');hand.grip=false;
 await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_translation_arrival');
 console.log('PASS XR translation/release',state.actual_tcp.p[2]-z);
 const q=[...state.actual_tcp.q];hand.p=[0,0,0];hand.q=[0,0,0,1];await sleep(80);
 hand.grip=true;await sleep(80);hand.q=[Math.sin(.012/2),0,0,Math.cos(.012/2)];
 await wait(()=>Math.abs(relativeToolPitch(q as any,state.actual_tcp.q))>.004,'XR_pitch');hand.grip=false;
 await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','XR_pitch_arrival');
 console.log('PASS XR pitch/release',relativeToolPitch(q as any,state.actual_tcp.q));
 hand.trigger=.35;await wait(()=>state.gripper<.5,'XR_claw_open');
 hand.trigger=.6;await wait(()=>state.gripper>.55,'XR_claw_close');
 console.log('PASS XR analogue claw',state.gripper);
 hand.p=[0,0,0];await sleep(80);hand.grip=true;await sleep(80);hand.p[2]=.0025;
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
