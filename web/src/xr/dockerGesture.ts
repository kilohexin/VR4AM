import type {Quat,Vec3} from '../protocol/messages';
/** Docker delivery mode: a clutch-centred spatial velocity joystick.
 * Grip captures the centre. Displacement selects direction; holding that
 * displacement keeps producing fresh intent without chasing a pose endpoint.
 */
export class DockerGesture {
 private anchor:Vec3|null=null;private lastTime:number|null=null;private active=false;
 reset():void {this.anchor=null;this.lastTime=null;this.active=false;}
 sample(t:number,p:Vec3,_q:Quat,scale:number):{vector:Vec3;pitch:number;spin:number} {
  const zero={vector:[0,0,0] as Vec3,pitch:0,spin:0};
  if(this.lastTime!==null&&(t<=this.lastTime||t-this.lastTime>100))this.reset();
  this.lastTime=t;
  if(this.anchor===null){this.anchor=[...p];return zero;}
  const d=p.map((v,i)=>(v-this.anchor![i])*scale);
  // Scene/WebXR: +X right, +Y up, -Z forward. Controller +Z is up,
  // +Y points towards scene -Z; use the controller/visual base convention.
  const delta:Vec3=[d[0],-d[2],d[1]],distance=Math.hypot(...delta);
  // 8 mm entry / 4 mm exit prevents centre jitter while still letting the
  // operator stop without releasing and re-pressing Grip.
  this.active=distance>(this.active?.004:.008);
  return {...zero,vector:this.active?delta.map(v=>v===0?0:v/distance) as Vec3:zero.vector};
 }
}
