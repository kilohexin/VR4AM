import type {FinitePositionInputMessage,Vec3,Pose} from '../protocol/messages';

export const FINITE_POSITION_DEADZONE=.15;

/** No timer, pending target or reconnect replay. Real UI frames supply freshness. */
export class FinitePositionInput {
 private sessionId='';private generation=0;private seq=0;private held=false;private outstanding=false;private sawBusy=false;
 constructor(private send:(message:FinitePositionInputMessage)=>void,private onAbort:(reason?:string)=>void){}
 reset(sessionId:string,generation:number):void {
  this.sessionId=sessionId;this.generation=generation;this.seq=0;this.held=false;this.outstanding=false;this.sawBusy=false;
 }
 offer(vector:Vec3,pitch=0,spin=0):void {
  if(!this.sessionId||!vector.every(Number.isFinite)||!Number.isFinite(pitch)||Math.abs(pitch)>1||!Number.isFinite(spin)||Math.abs(spin)>1)return;
  pitch=Math.abs(pitch)>FINITE_POSITION_DEADZONE?Math.sign(pitch):0;
  spin=Math.abs(spin)>FINITE_POSITION_DEADZONE?Math.sign(spin):0;
  const length=Math.hypot(...vector);if(length<=FINITE_POSITION_DEADZONE&&!pitch&&!spin){this.release();return;}
  this.held=true;this.outstanding=true;this.send({v:1,type:'finite_position_input',session_id:this.sessionId,
   generation:this.generation,seq:++this.seq,kind:'held',vector:length>FINITE_POSITION_DEADZONE?vector.map(x=>x/length) as Vec3:[0,0,0],
   ...(pitch?{pitch}:{}),...(spin?{spin}:{})});
 }
 release():void {
  if(!this.held)return;this.held=false;
  this.send({v:1,type:'finite_position_input',session_id:this.sessionId,generation:this.generation,
   seq:++this.seq,kind:'release',vector:[0,0,0]});
 }
 offerPose(pose:Pose):void {
  if(!this.sessionId.startsWith('xr-finite-')||!pose.p.every(Number.isFinite)
    ||!pose.q.every(Number.isFinite)||Math.abs(Math.hypot(...pose.q)-1)>1e-3)return;
  this.held=true;this.outstanding=true;
  this.send({v:1,type:'finite_position_input',session_id:this.sessionId,generation:this.generation,
   seq:++this.seq,kind:'held',vector:[0,0,0],hand_pose:{p:[...pose.p],q:[...pose.q]}});
 }
 observe(phase:string):void {
  if(['planning','executing','draining'].includes(phase))this.sawBusy=true;
  else if(this.sawBusy&&!this.held){this.outstanding=false;this.sawBusy=false;}
 }
 abort(reason?:string):void {if(this.outstanding){this.held=false;this.outstanding=false;this.sawBusy=false;this.onAbort(reason);}}
}
