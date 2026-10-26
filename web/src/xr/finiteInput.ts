import {Quaternion} from 'three';
import type {GripperRequestMessage,GripperResultMessage,RobotStateMessage,Vec3,Quat,VRFrame} from '../protocol/messages';
import {FinitePositionInput} from '../ui/finitePositionInput';

export function finiteXRTransportFrame(frame:VRFrame):VRFrame {
 return {...frame,right:{...frame.right,grip:false,trigger:0}};
}

export function relativeToolPitch(from:Quat,to:Quat):number {
 const q=new Quaternion(...from).invert().multiply(new Quaternion(...to));
 // q and -q describe the same pose; use the shortest signed local-X twist.
 const sign=q.w<0?-1:1;
 return Math.hypot(q.x,q.w)<1e-10?0:2*Math.atan2(q.x*sign,q.w*sign);
}

/** XR samples feed the same finite goals as the desktop, without a goal queue. */
export class FiniteXRInput {
 private state:RobotStateMessage|null=null;private receivedAt=-Infinity;
 private anchor:{hand:Vec3;tcp:Vec3;handQ:Quat;tcpQ:Quat}|null=null;private mustRelease=true;
 private lastTime:number|null=null;private trigger:number|null=null;
 private pending:string|null=null;private lastClaw=-Infinity;private sequence=0;
 constructor(private position:FinitePositionInput,private sendClaw:(m:GripperRequestMessage)=>void,
  private cancelClaw:(id:string)=>void,private now:()=>number=()=>performance.now()){}
 update(state:RobotStateMessage|null,receivedAt:number):void {
  if(this.state?.finite_position?.generation!==state?.finite_position?.generation)this.reset('xr_generation_changed');
  this.state=state;this.receivedAt=receivedAt;
 }
 reset(reason='xr_reset'):void {
  this.position.abort(reason);
  if(this.pending)this.cancelClaw(this.pending);
  this.pending=null;this.anchor=null;this.mustRelease=true;this.lastTime=null;this.trigger=null;
 }
 apply(frame:VRFrame):void {
  const s=this.state,t=this.now();
  const ready=s?.finite_position?.vr_enabled===true&&s.manual_input_ready===true&&!s.fault&&!s.recovery_phase
   &&['ARMED','ACTIVE','HOLD'].includes(s.mode)&&['IDLE','MOVING'].includes(s.robot_state)
   &&frame.tracking_valid&&frame.visibility==='visible'
   &&frame.right.p.every(Number.isFinite)&&Number.isFinite(frame.right.trigger)
   &&frame.right.q.every(Number.isFinite)&&Math.abs(Math.hypot(...frame.right.q)-1)<1e-3;
  if(!ready){this.reset('xr_input_unavailable');return;}
  if(this.lastTime!==null&&(frame.client_mono_ms<this.lastTime||frame.client_mono_ms-this.lastTime>100)){
   this.reset('xr_frame_gap');return;
  }
  this.lastTime=frame.client_mono_ms;
  const feedbackFresh=t>=this.receivedAt&&t-this.receivedAt<100;
  if(this.pending&&(!feedbackFresh||frame.right.grip)){
   this.position.release();this.anchor=null;this.mustRelease=true;return;
  }
  if(!feedbackFresh){this.reset('xr_feedback_expired');return;}
  if(!frame.right.grip){this.position.release();this.anchor=null;this.mustRelease=false;}
  else if(!this.mustRelease){
   if(!this.anchor){
    if(['planning','executing','draining'].includes(s.finite_position!.phase)){this.mustRelease=true;return;}
    this.anchor={hand:[...frame.right.p],tcp:[...s.actual_tcp.p],handQ:[...frame.right.q],tcpQ:[...s.actual_tcp.q]};
   }
   const scale=s.translation_scale??1;
   const error=this.anchor.tcp.map((v,i)=>v+(frame.right.p[i]-this.anchor!.hand[i])*scale-s.actual_tcp.p[i]) as Vec3;
   const distance=Math.hypot(...error);
   const handPitch=Math.max(-Math.PI/9,Math.min(Math.PI/9,relativeToolPitch(this.anchor.handQ,frame.right.q)));
   const pitchError=s.finite_position!.pitch_enabled?handPitch-relativeToolPitch(this.anchor.tcpQ,s.actual_tcp.q):0;
   const pitch=Math.abs(pitchError)>Math.PI/600?Math.sign(pitchError):0;
   if(distance<=.001&&!pitch)this.position.release();
   else this.position.offer(distance>.001?error.map(v=>v/distance) as Vec3:[0,0,0],pitch);
  }
  // No default opening on preparation. Once Trigger changes, its analogue
  // value controls closure; a pending target finishes even on normal release.
  if(this.trigger===null){this.trigger=frame.right.trigger;return;}
  const value=Math.max(0,Math.min(1,frame.right.trigger));
  if(!frame.right.grip&&s.robot_state==='IDLE'&&!['planning','executing','draining'].includes(s.finite_position!.phase)
    &&this.pending===null&&Math.abs(value-this.trigger)>=.005&&t-this.lastClaw>=100){
   this.trigger=value;this.lastClaw=t;this.pending=`xr-claw-${++this.sequence}-${Date.now()}`;
   this.sendClaw({v:1,type:'gripper_request',request_id:this.pending,value});
  }
 }
 onGripperResult(result:GripperResultMessage):void {
  if(result.request_id!==this.pending)return;
  this.pending=null;
  if(!result.accepted)this.reset('xr_claw_rejected');
 }
}
