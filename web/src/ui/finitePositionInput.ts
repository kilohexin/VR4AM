import type {FinitePositionInputMessage,Vec3} from '../protocol/messages';

export const FINITE_POSITION_DEADZONE=.15;

/** No timer, pending target or reconnect replay. Real UI frames supply freshness. */
export class FinitePositionInput {
 private sessionId='';private generation=0;private seq=0;private held=false;private outstanding=false;private sawBusy=false;
 constructor(private send:(message:FinitePositionInputMessage)=>void,private onAbort:(reason?:string)=>void){}
 reset(sessionId:string,generation:number):void {
  this.sessionId=sessionId;this.generation=generation;this.seq=0;this.held=false;this.outstanding=false;this.sawBusy=false;
 }
 offer(vector:Vec3):void {
  if(!this.sessionId||!vector.every(Number.isFinite))return;
  const length=Math.hypot(...vector);if(length<=FINITE_POSITION_DEADZONE){this.release();return;}
  this.held=true;this.outstanding=true;this.send({v:1,type:'finite_position_input',session_id:this.sessionId,
   generation:this.generation,seq:++this.seq,kind:'held',vector:vector.map(x=>x/length) as Vec3});
 }
 release():void {
  if(!this.held)return;this.held=false;
  this.send({v:1,type:'finite_position_input',session_id:this.sessionId,generation:this.generation,
   seq:++this.seq,kind:'release',vector:[0,0,0]});
 }
 observe(phase:string):void {
  if(['planning','executing','draining'].includes(phase))this.sawBusy=true;
  else if(this.sawBusy&&!this.held){this.outstanding=false;this.sawBusy=false;}
 }
 abort(reason?:string):void {if(this.outstanding){this.held=false;this.outstanding=false;this.sawBusy=false;this.onAbort(reason);}}
}
