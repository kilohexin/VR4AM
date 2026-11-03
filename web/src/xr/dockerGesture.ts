import type {Quat,Vec3} from '../protocol/messages';
import {Quaternion} from 'three';
/** Docker delivery mode: a clutch-centred spatial velocity joystick.
 * Grip captures the centre. Displacement selects direction; holding that
 * displacement keeps producing fresh intent without chasing a pose endpoint.
 */
export class DockerGesture {
 private anchor:Vec3|null=null;private lastTime:number|null=null;private active=false;
 private orientation:Quat|null=null;private pitchActive=false;private spinActive=false;
 reset():void {this.anchor=null;this.orientation=null;this.lastTime=null;this.active=false;this.pitchActive=false;this.spinActive=false;}
 sample(t:number,p:Vec3,q:Quat,scale:number,wrist=false):{vector:Vec3;pitch:number;spin:number} {
  const zero={vector:[0,0,0] as Vec3,pitch:0,spin:0};
  if(this.lastTime!==null&&(t<=this.lastTime||t-this.lastTime>100))this.reset();
  this.lastTime=t;
  if(this.anchor===null){this.anchor=[...p];this.orientation=[...q];return zero;}
  const d=p.map((v,i)=>(v-this.anchor![i])*scale);
  // Scene/WebXR: +X right, +Y up, -Z forward. Controller +Z is up,
  // +Y points towards scene -Z; use the controller/visual base convention.
  const delta:Vec3=[d[0],-d[2],d[1]],distance=Math.hypot(...delta);
  // 8 mm entry / 4 mm exit prevents centre jitter while still letting the
  // operator stop without releasing and re-pressing Grip.
  this.active=distance>(this.active?.004:.008);
  let pitch=0,spin=0;
  if(wrist&&this.orientation){
   const relative=new Quaternion(...this.orientation).invert().multiply(new Quaternion(...q));
   const sign=relative.w<0?-1:1;
   const x=Math.hypot(relative.x,relative.w)<1e-10?0:2*Math.atan2(relative.x*sign,relative.w*sign);
   const z=Math.hypot(relative.z,relative.w)<1e-10?0:2*Math.atan2(relative.z*sign,relative.w*sign);
   this.pitchActive=Math.abs(x)>(this.pitchActive?Math.PI/120:Math.PI/60);
   this.spinActive=Math.abs(z)>(this.spinActive?Math.PI/120:Math.PI/60);
   pitch=this.pitchActive?-Math.sign(x):0;spin=this.spinActive?Math.sign(z):0;
  }
  return {vector:this.active?delta.map(v=>v===0?0:v/distance) as Vec3:zero.vector,pitch,spin};
 }
}
