import pytest
from scripts.docker_velocity_probe import probe

class Client:
    def __init__(self):
        self.pose=dict(x=.3,y=0.,z=.3,rx=0.,ry=0.,rz=0.)
        self.pending=[];self.calls=0;self.stops=0
    async def get_robot_state(self):return 'IDLE'
    async def get_kin_data(self):return dict(actual_joint_speed=[0.]*6,actual_tcp_pose=dict(self.pose))
    async def movel(self,p,a,v,t,r):
        assert len(self.pending)<2
        assert a==.1 and v==.02 and t==0 and r==.001
        self.calls+=1;self.pending.append(self.calls);self.pose=dict(p);return self.calls
    async def get_motion_state(self,ident):
        assert ident==self.pending.pop(0)
        return 'FINISHED'
    async def stop_move(self):self.stops+=1

async def test_bounded_blend_never_submits_more_than_two_unfinished_targets():
    c=Client();r=await probe(c,bounded=True)
    assert r['passed'] and len(r['commands'])==6 and c.stops==0
    assert c.pose['z']==pytest.approx(.324)

async def test_failed_motion_is_not_retried_and_stops_once():
    c=Client()
    async def fail(*args):c.calls+=1;raise RuntimeError('test_failure')
    c.movel=fail
    r=await probe(c,bounded=True)
    assert not r['passed'] and c.calls==1 and c.stops==1
    assert r['unknown']==['movel']
