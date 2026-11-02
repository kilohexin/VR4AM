import type {Quat,Vec3} from '../protocol/messages';
type Sample={t:number;p:Vec3};
/** Docker delivery mode: live translation only. Never chase an old endpoint. */
export class DockerGesture {
 private samples:Sample[]=[];
 reset():void {this.samples=[];}
 sample(t:number,p:Vec3,_q:Quat,scale:number):{vector:Vec3;pitch:number;spin:number} {
  const zero={vector:[0,0,0] as Vec3,pitch:0,spin:0};
  const last=this.samples.at(-1);
  if(last&&(t<=last.t||t-last.t>100))this.reset();
  this.samples.push({t,p:[...p]});
  while(this.samples.length>1&&this.samples[1].t<=t-80)this.samples.shift();
  const first=this.samples[0],dt=t-first.t;
  if(dt<20)return zero;
  const d=p.map((v,i)=>(v-first.p[i])*scale);
  // Scene/WebXR: +X right, +Y up, -Z forward. Controller +Z is up,
  // +Y points towards scene -Z; use the controller/visual base convention.
  const delta:Vec3=[d[0],-d[2],d[1]],distance=Math.hypot(...delta);
  return {...zero,vector:distance>Math.max(.001,.012*dt/1000)?delta.map(v=>v===0?0:v/distance) as Vec3:zero.vector};
 }
}
