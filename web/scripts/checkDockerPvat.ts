// Synthetic XR through the product WebSocket, verified local Docker only.
import {FinitePositionInput} from '../src/ui/finitePositionInput';
import {finiteXRTransportFrame} from '../src/xr/finiteInput';
const port=process.argv[2]??'8774',origin=`http://127.0.0.1:${port}`;
if(!/^\d{4,5}$/.test(port))throw Error('invalid_port');
const before:any=await(await fetch(origin+'/simulation/status')).json();
if(before.runtime!=='LEBAI_DOCKER'||!before.pvat_vr||before.sdk_target!=='127.0.0.1:3030')throw Error('wrong_runtime');
const ws=new WebSocket(`ws://127.0.0.1:${port}/ws/v1/teleop`);
let state:any=null,generation=-1,seq=0,vector=[0,0,0],timer:any;
let transportSession=process.argv.includes('--source-switch')?'desktop-pvat-handoff':'quest-pvat-test';
const replies=new Map();const send=(m:any)=>{if(ws.readyState===1)ws.send(JSON.stringify(m));};
const input=new FinitePositionInput(send,()=>{});
ws.onmessage=e=>{const m=JSON.parse(String(e.data));
 if(m.type==='robot_state'){
  state=m;if(generation!==m.finite_position.generation){generation=m.finite_position.generation;input.reset('xr-finite-pvat-acceptance',generation);}
  input.observe(m.finite_position.phase);
 }else if(m.request_id)replies.set(m.request_id,m);
};
const sleep=(ms:number)=>new Promise(r=>setTimeout(r,ms));
async function wait(p:()=>boolean,label:string,ms=10000){const end=performance.now()+ms;
 while(!p()){if(state?.fault)throw Error(label+':'+state.fault);if(performance.now()>end)throw Error(label+':timeout:'+JSON.stringify(state));await sleep(20);}}
async function command(type:string,id:string){send({v:1,type,request_id:id});await wait(()=>replies.has(id),id,65000);if(String(replies.get(id).type).endsWith('rejected'))throw Error(JSON.stringify(replies.get(id)));}
try{
 await new Promise<void>((r,j)=>{ws.onopen=()=>r();ws.onerror=()=>j(Error('socket'));});send({v:1,type:'hello',request_id:'hello'});
 timer=setInterval(()=>{
  const frame:any={v:1,type:'vr_frame',session_id:transportSession,seq:++seq,client_mono_ms:performance.now(),tracking_valid:true,visibility:'visible',right:{p:[0,0,0],q:[0,0,0,1],grip:vector.some(Boolean),trigger:0}};
  send(finiteXRTransportFrame(frame));
  if(vector.some(Boolean))input.offer(vector as [number,number,number]);
 },20);
 await wait(()=>state?.mode==='READY'&&state.ack_seq>0,'ready');
 if(process.argv.includes('--source-switch')){
  const old=generation;transportSession='quest-pvat-test';seq=0;
  await wait(()=>generation>old&&state.mode==='READY'&&state.ack_seq>0,'desktop_to_xr_ready',6000);
  console.log('PASS desktop to XR source switch returns READY with fresh released frames');
 }
 await command('arm_request','unlock');await command('prepare_request','prepare');await wait(()=>state?.manual_input_ready,'prepared');
 for(const direction of [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[0,-1,1],[-1,0,-1]]){
  const start=[...state.actual_tcp.p];
  const steps=state.finite_position.entered_steps;vector=direction;
  await wait(()=>state.finite_position.entered_steps>=steps+8,'pvat_points');
  vector=[0,0,0];input.release();
  await wait(()=>state.finite_position.phase==='ready'&&state.robot_state==='IDLE','release_drain');
  for(let axis=0;axis<3;axis++)if(direction[axis]&&(state.actual_tcp.p[axis]-start[axis])*direction[axis]<.0003)throw Error('insufficient_actual_progress_axis_'+axis);
  console.log('PASS XR PVAT direction / release',direction,state.finite_position.entered_steps);
 }
 if(process.argv.includes('--long-hold')){
  const started=performance.now();vector=[0,1,0];
  const alternate=setInterval(()=>{vector=[0,Math.floor((performance.now()-started)/800)%2?1:-1,0];},80);
  try{await wait(()=>state.finite_position.phase==='hold','bounded_hold',14000);}
  finally{clearInterval(alternate);}
  const stopped=state.finite_position.entered_steps;
  await sleep(250);
  if(state.finite_position.entered_steps!==stopped)throw Error('held_restarted_after_limit');
  vector=[0,0,0];input.release();await wait(()=>state.finite_position.phase==='ready','release_after_limit');
  vector=[0,-1,0];await wait(()=>state.finite_position.entered_steps>=stopped+6,'regrip_after_limit');
  vector=[0,0,0];input.release();await wait(()=>state.finite_position.phase==='ready','regrip_drain');
  console.log('PASS bounded continuous hold / no automatic restart / explicit regrip');
 }
 send({v:1,type:'gripper_request',request_id:'claw-middle',value:.4});
 await wait(()=>replies.has('claw-middle'),'claw_reply');
 if(!replies.get('claw-middle').accepted)throw Error('claw_rejected:'+JSON.stringify(replies.get('claw-middle')));
 await wait(()=>Math.abs(state.gripper-.4)<.05,'claw_middle');
 console.log('PASS proportional claw after PVAT drain');
 if(process.argv.includes('--disconnect-active')){
  const steps=state.finite_position.entered_steps;vector=[0,0,-1];
  await wait(()=>state.finite_position.entered_steps>=steps+4,'active_disconnect');
  console.log('PASS reached active PVAT before disconnect');
 }else{
 await command('disarm','disarm');
 }
}finally{vector=[0,0,0];input.release();if(timer)clearInterval(timer);ws.close();}
await sleep(3500);
const after:any=await(await fetch(origin+'/simulation/status')).json();
if(after.control_fault||after.session_owned||after.unknown_reads.length||after.unknown_writes.length)throw Error('unclean_end:'+JSON.stringify(after));
console.log('PASS product XR PVAT clean disconnect',JSON.stringify(after));
