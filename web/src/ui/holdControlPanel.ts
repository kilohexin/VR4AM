import {Quaternion,Vector3} from 'three';
import type {GripperRequestMessage,GripperResultMessage,RobotStateMessage,VRFrame,Vec3,Quat} from '../protocol/messages';

type Action='x+'|'x-'|'y+'|'y-'|'z+'|'z-'|'spin+'|'spin-';
const ACTIONS:ReadonlyArray<readonly [Action,string]>=
 [['y+','前 +Y'],['y-','后 −Y'],['x-','左 −X'],['x+','右 +X'],
 ['z+','上升 +Z'],['z-','下降 −Z'],['spin-','逆自转'],['spin+','顺自转']];

export class HoldControlPanel {
 private state:RobotStateMessage|null=null;
 private enabled=false;
 private receivedAt=Number.NEGATIVE_INFINITY;
 private action:Action|null=null;
 private pointer:number|null=null;
 private capture:HTMLButtonElement|null=null;
 private position:Vector3|null=null;
 private rotation:Quaternion|null=null;
 private spinAxis=new Vector3(0,0,1);
 private previousTime:number|null=null;
 private pending:string|null=null;
 private disposed=false;
 private readonly status:HTMLElement;
 private readonly feedback:HTMLElement;
 constructor(private readonly root:HTMLElement,private readonly send:(m:GripperRequestMessage)=>void,
  private readonly now:()=>number=()=>performance.now(),private readonly onRelease:()=>void=()=>{}){
  root.classList.add('hold-control-panel');
  root.innerHTML=`<h2>网页控制</h2><p>长按运动，松开停止 · 固定 X/Y 平面、Z 高度</p>
   <div class="hold-buttons">${ACTIONS.map(([a,label])=>`<button type="button" data-hold="${a}" disabled>${label}</button>`).join('')}</div>
   <div class="claw-buttons"><button type="button" data-claw="0" disabled>打开夹爪</button><button type="button" data-claw="1" disabled>闭合夹爪</button></div>
   <p data-field="hold-status" role="status">先解锁，再操作</p><p data-field="hold-feedback">等待实际反馈</p>`;
  this.status=root.querySelector('[data-field="hold-status"]')!;
  this.feedback=root.querySelector('[data-field="hold-feedback"]')!;
  root.addEventListener('pointerdown',this.onDown);
  root.addEventListener('pointerup',this.onUp);
  root.addEventListener('pointercancel',this.onUp);
  root.addEventListener('lostpointercapture',this.onUp);
  root.addEventListener('click',this.onClick);
  window.addEventListener('blur',this.cancel);
  document.addEventListener('visibilitychange',this.onVisibility);
  document.addEventListener('focusin',this.onFocus);
 }
 update(state:RobotStateMessage|null,available:boolean,receivedAt=this.now()):void{
  this.state=state;
  this.receivedAt=receivedAt;
  this.enabled=available&&state!==null&&state.manual_input_ready===true&&!state.fault&&!state.recovery_phase
   &&['ARMED','ACTIVE','HOLD'].includes(state.mode)&&['IDLE','MOVING'].includes(state.robot_state);
  if(!this.enabled){this.pending=null;this.cancel();this.position=this.rotation=null;this.previousTime=null;}
  if(state)this.feedback.textContent=`实际 XYZ：${state.actual_tcp.p.map(n=>(n*1000).toFixed(2)).join(' / ')} mm · 夹爪 ${(state.gripper*100).toFixed(0)}%`;
  this.refresh();
 }
 applyFrame(frame:VRFrame):VRFrame{
  const time=frame.client_mono_ms;
  if(!this.position){this.position=new Vector3(...frame.right.p);this.rotation=new Quaternion(...frame.right.q);}
  const delta=this.previousTime===null?0:time-this.previousTime;
  this.previousTime=time;
  const fresh=this.fresh()&&frame.tracking_valid&&frame.visibility==='visible';
  if(!fresh||delta<0||delta>100)this.cancel();
  if(this.action&&this.state?.mode==='ACTIVE'){
   const dt=Math.min(.02,Math.max(0,delta));
   const action=this.action;
   const sign=action.endsWith('+')?1:-1;
   if(action.startsWith('spin'))this.rotation!.premultiply(new Quaternion().setFromAxisAngle(this.spinAxis,sign*dt*Math.PI/60));
   else {const axis=action[0] as 'x'|'y'|'z';this.position[axis]+=sign*dt*.02;}
  }
  return {...frame,right:{...frame.right,p:this.position.toArray() as Vec3,q:this.rotation!.toArray() as Quat,
   grip:this.action!==null&&fresh,trigger:0}};
 }
 onGripperResult(result:GripperResultMessage):void{
  if(result.request_id!==this.pending)return;
  this.pending=null;
  this.status.textContent=result.accepted?'夹爪目标已提交，请观察实际反馈':'夹爪请求未执行：请检查解锁、静止和故障状态';
  this.refresh(false);
 }
 cancel=():void=>{
  const button=this.capture,id=this.pointer;
  const held=this.action!==null;
  this.action=null;this.pointer=null;this.capture=null;
  if(button&&id!==null){try{if(button.hasPointerCapture(id))button.releasePointerCapture(id);}catch{}}
  this.refresh();
  if(held)this.onRelease();
 };
 dispose():void{
  if(this.disposed)return;this.cancel();this.disposed=true;
  this.root.removeEventListener('pointerdown',this.onDown);this.root.removeEventListener('pointerup',this.onUp);
  this.root.removeEventListener('pointercancel',this.onUp);this.root.removeEventListener('lostpointercapture',this.onUp);
  this.root.removeEventListener('click',this.onClick);window.removeEventListener('blur',this.cancel);
  document.removeEventListener('visibilitychange',this.onVisibility);document.removeEventListener('focusin',this.onFocus);
 }
 private fresh(time=this.now()):boolean{return !this.disposed&&this.enabled&&time>=this.receivedAt&&time-this.receivedAt<=100;}
 private refresh(copy=true):void{
  for(const button of this.root.querySelectorAll<HTMLButtonElement>('[data-hold]')){
   button.disabled=!this.fresh()||this.pending!==null||(this.state?.mode==='ACTIVE'&&this.action===null)
    ||(button.dataset.hold!.startsWith('spin')&&!this.state?.target_control?.tool_spin_supported);
   button.classList.toggle('is-held',button.dataset.hold===this.action);
  }
  for(const button of this.root.querySelectorAll<HTMLButtonElement>('[data-claw]'))button.disabled=
   !this.fresh()||this.pending!==null||this.action!==null||this.state?.robot_state!=='IDLE'||this.state?.mode==='ACTIVE';
  if(copy)this.status.textContent=this.pending?'等待夹爪回执；不自动重试':!this.enabled?'先解锁，再操作':
   this.action?'持续操纵中；松开按钮停止':'长按方向或自转；夹爪只在静止时操作';
 }
 private onDown=(e:PointerEvent):void=>{
  const button=(e.target as Element).closest<HTMLButtonElement>('[data-hold]');
  if(!button||button.disabled||!this.fresh()||e.button!==0||this.pointer!==null||!this.state)return;
  e.preventDefault();button.focus({preventScroll:true});
  this.pointer=e.pointerId;this.capture=button;this.action=button.dataset.hold as Action;
  this.previousTime=null;
  this.spinAxis.set(0,0,1).applyQuaternion(new Quaternion(...this.state.actual_tcp.q)).normalize();
  try{button.setPointerCapture(e.pointerId);}catch{this.cancel();}
  this.refresh();
 };
 private onUp=(e:PointerEvent):void=>{if(e.pointerId===this.pointer)this.cancel();};
 private onVisibility=():void=>{if(document.visibilityState!=='visible')this.cancel();};
 private onFocus=(e:FocusEvent):void=>{if(e.target instanceof Node&&!this.root.contains(e.target))this.cancel();};
 private onClick=(e:MouseEvent):void=>{
  const button=(e.target as Element).closest<HTMLButtonElement>('[data-claw]');
  if(!button||button.disabled||!this.fresh()||this.action||this.pending)return;
  this.pending=`web-claw-${Date.now()}-${Math.random().toString(36).slice(2,10)}`;
  this.refresh();this.send({v:1,type:'gripper_request',request_id:this.pending,value:Number(button.dataset.claw)});
 };
}
