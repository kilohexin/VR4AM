import {Quaternion} from 'three';
import type {Quat,Vec3} from '../protocol/messages';

type Sample={t:number;p:Vec3;q:Quat};
/** Live motion window, not an accumulated endpoint. Holding a displaced hand
 * still stops requesting motion; clutch/pause resets discard old movement. */
export class DockerGesture {
 private samples:Sample[]=[];
 reset():void {this.samples=[];}
 sample(t:number,p:Vec3,q:Quat,scale:number):{vector:Vec3;pitch:number;spin:number} {
  const zero={vector:[0,0,0] as Vec3,pitch:0,spin:0};
  const last=this.samples.at(-1);
  if(last&&(t<=last.t||t-last.t>100))this.reset();
  this.samples.push({t,p:[...p],q:[...q]});
  while(this.samples.length>1&&this.samples[1].t<=t-80)this.samples.shift();
  const first=this.samples[0],dt=t-first.t;
  if(dt<20)return zero;
  const delta=p.map((v,i)=>(v-first.p[i])*scale) as Vec3;
  const distance=Math.hypot(...delta);
  const rotation=new Quaternion(...first.q).invert().multiply(new Quaternion(...q));
  if(rotation.w<0)rotation.set(-rotation.x,-rotation.y,-rotation.z,-rotation.w);
  const length=Math.hypot(rotation.x,rotation.y,rotation.z);
  const factor=length<1e-10?2:2*Math.atan2(length,rotation.w)/length;
  // WebXR grip +Y points along the forearm towards the body. Pronation around
  // the forward forearm (-Y) maps to tool +Z spin, not grip-space +Z.
  // https://www.w3.org/TR/webxr/#dom-xrinputsource-gripspace
  const pitch=rotation.x*factor,spin=-rotation.y*factor;
  const angularDeadband=Math.max(.0035,.045*dt/1000);
  const strongest=Math.max(Math.abs(pitch),Math.abs(spin));
  const pitchActive=Math.abs(pitch)>Math.max(angularDeadband,strongest*.35);
  const spinActive=Math.abs(spin)>Math.max(angularDeadband,strongest*.35);
  // A wrist pivot also moves the tracked grip origin. Do not turn that
  // millimetre arc into a full-strength translation command.
  const translationThreshold=(pitchActive||spinActive)?Math.max(.002,.025*dt/1000):Math.max(.0006,.008*dt/1000);
  return {
   vector:distance>translationThreshold?delta.map(v=>v/distance) as Vec3:zero.vector,
   pitch:pitchActive?Math.sign(pitch):0,
   spin:spinActive?Math.sign(spin):0,
  };
 }
}
