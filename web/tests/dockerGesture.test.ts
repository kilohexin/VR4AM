import {expect,it} from 'vitest';
import {Quaternion,Vector3} from 'three';
import {DockerGesture} from '../src/xr/dockerGesture';
import type {Quat} from '../src/protocol/messages';
it('stops requesting movement when the hand rests, independent of robot endpoint feedback',()=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 expect(g.sample(40,[.004,0,0],[0,0,0,1],1).vector).toEqual([1,0,0]);
 for(let t=60;t<=180;t+=20)g.sample(t,[.004,0,0],[0,0,0,1],1);
 expect(g.sample(200,[.004,0,0],[0,0,0,1],1)).toEqual({vector:[0,0,0],pitch:0,spin:0});
});
it.each([0,1.1,-1.5])('recognizes wrist flexion and forearm twist at starting tilt %s',(tilt)=>{
 const initial=new Quaternion().setFromAxisAngle(new Vector3(0,0,1),tilt);
 for(const [axis,expected] of [[new Vector3(1,0,0),{pitch:1,spin:0}],[new Vector3(0,-1,0),{pitch:0,spin:1}]] as const){
  const g=new DockerGesture();g.sample(0,[0,0,0],initial.toArray() as Quat,1);
  const q=initial.clone().multiply(new Quaternion().setFromAxisAngle(axis,.025)).toArray() as Quat;
  expect(g.sample(40,[.0001,0,0],q,1)).toEqual({vector:[0,0,0],...expected});
 }
});
it('ignores grip-Z sideways swing instead of calling it forearm roll',()=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 expect(g.sample(40,[0,0,0],[0,0,Math.sin(.05),Math.cos(.05)],1).spin).toBe(0);
});
it('allows simultaneous translation and flexion, then discards clutch history and tracking gaps',()=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 const q:Quat=[Math.sin(.02),0,0,Math.cos(.02)];
 expect(g.sample(40,[0,.004,0],q,1)).toEqual({vector:[0,1,0],pitch:1,spin:0});
 g.reset();expect(g.sample(60,[.2,.2,.2],q,1)).toEqual({vector:[0,0,0],pitch:0,spin:0});
 expect(g.sample(200,[.4,.4,.4],[0,0,0,1],1)).toEqual({vector:[0,0,0],pitch:0,spin:0});
});
it('rejects small tracking jitter',()=>{
 const g=new DockerGesture();
 for(let t=0;t<=400;t+=20){const sign=t%40?1:-1;
  expect(g.sample(t,[sign*.0001,0,0],[Math.sin(sign*.0005),0,0,Math.cos(sign*.0005)],1)).toEqual({vector:[0,0,0],pitch:0,spin:0});
 }
});

it('filters wrist pivot translation and weaker cross-axis wobble',()=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 const q=new Quaternion().setFromAxisAngle(new Vector3(1,.1,0).normalize(),.03).toArray() as Quat;
 expect(g.sample(40,[.001,.0003,0],q,1)).toEqual({vector:[0,0,0],pitch:1,spin:0});
});
