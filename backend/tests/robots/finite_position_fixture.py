import math
from dataclasses import replace
import numpy as np
from tests.robots.fake_lebai import FakeLebaiClient,IDLE_Q
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


class LinearModel:
    model_sha256='c'*64
    factory_calibrated=False
    def forward(self,q):return ((.3+q[0],q[1]+1,.4+q[2]-1),(0.,0.,0.,1.))
    def jacobian(self,q):return np.diag((1.,1.,1.,0.,0.,0.))
    def jdot(self,q,qd):return np.zeros((6,6))


class PositionClient(FakeLebaiClient):
    def __init__(self,clock):
        super().__init__();self.clock=clock;self.model=LinearModel();self.start_ns=None
        self.start=tuple(IDLE_Q);self.goal=self.start;self.no_progress=False;self.foreign=False
    def advance(self):
        if self.start_ns is None:return
        u=min(1.,max(0.,(self.clock.now_ns()-self.start_ns)/200_000_000))
        q=self.start if self.no_progress else tuple(a+(b-a)*u for a,b in zip(self.start,self.goal))
        p,_=self.model.forward(q)
        for side in ('actual','target'):
            self.kin_data[side+'_joint_pose']=list(q if side=='actual' else self.goal)
            self.kin_data[side+'_tcp_pose']=dict(x=p[0],y=p[1],z=p[2],rz=0.,ry=0.,rx=0.)
        self.robot_state='MOVING' if u<1 else 'IDLE'
        self.motion_state='RUNNING' if u<1 else 'FINISHED'
        self.running_motion=(999 if self.foreign else 7) if u<1 else 0
    async def get_robot_state(self):self.advance();return await super().get_robot_state()
    async def kinematics_inverse(self,p,joints):return [p['x']-.3,p['y']-1,p['z']+.6,*joints[3:]]
    async def kinematics_forward(self,q):
        p,_=self.model.forward(q);return dict(x=p[0],y=p[1],z=p[2],rz=0.,ry=0.,rx=0.)
    async def movej(self,p,a,v,t,r):
        await super().movej(p,a,v,t,r)
        self.start_ns=self.clock.now_ns();self.start=tuple(self.kin_data['actual_joint_pose']);self.goal=tuple(p)
        return 7
    async def stop_move(self):
        await super().stop_move();self.advance();self.start_ns=None
        self.robot_state='IDLE';self.motion_state='FINISHED';self.running_motion=0


def fixture():
    clock=FakeClock();client=PositionClient(clock)
    settings=control_settings(ip='digital-twin')
    return settings,client,clock
