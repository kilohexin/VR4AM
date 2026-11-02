import {expect,it} from 'vitest';
import {DockerGesture} from '../src/xr/dockerGesture';
import type {Vec3} from '../src/protocol/messages';
it.each([
 [[.005,0,0],[1,0,0]],[[-.005,0,0],[-1,0,0]],
 [[0,.005,0],[0,0,1]],[[0,-.005,0],[0,0,-1]],
 [[0,0,-.005],[0,1,0]],[[0,0,.005],[0,-1,0]],
])('maps scene hand %s to controller direction %s',(hand,expected)=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 expect(g.sample(40,hand as Vec3,[.5,.5,.5,.5],1)).toEqual({vector:expected,pitch:0,spin:0});
});
it('ignores all wrist rotations and small tracking noise',()=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 expect(g.sample(40,[.0001,0,0],[.5,.5,.5,.5],1)).toEqual({vector:[0,0,0],pitch:0,spin:0});
});
it('stops on stationary hand and discards clutch history',()=>{
 const g=new DockerGesture();g.sample(0,[0,0,0],[0,0,0,1],1);
 expect(g.sample(40,[.004,0,0],[0,0,0,1],1).vector[0]).toBe(1);
 for(let t=60;t<=180;t+=20)g.sample(t,[.004,0,0],[0,0,0,1],1);
 expect(g.sample(200,[.004,0,0],[0,0,0,1],1).vector).toEqual([0,0,0]);
 g.reset();expect(g.sample(220,[.5,.5,.5],[0,0,0,1],1).vector).toEqual([0,0,0]);
});
