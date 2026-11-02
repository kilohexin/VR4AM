"""Bounded native method surface for a separately authorized two-segment trial."""
from collections import Counter


class TrialWindow:
    READS=frozenset({'is_connected','get_robot_state','get_estop_reason','get_kin_data',
        'get_tcp','get_claw','get_running_motion','get_motion_state','get_velocity_factor',
        'kinematics_inverse','kinematics_forward'})

    def __init__(self,authorize,clock):
        self.authorize,self.clock=authorize,clock;self.start=clock()
        self.entries=Counter();self.unknown=False

    def entry(self,name):
        elapsed=self.clock()-self.start
        if not 0<=elapsed<25_000_000_000:raise ValueError('blend_trial_cleanup_expired')
        if name=='stop_move':cap=1
        elif name in self.READS:
            cap=10 if name=='kinematics_forward' else 8 if name=='kinematics_inverse' else 4096
            if self.entries['reads']>=4096:raise ValueError('blend_trial_read_budget')
        elif name in ('connect','movel'):
            self.authorize()
            if self.unknown or elapsed>=20_000_000_000:raise ValueError('blend_trial_write_revoked')
            cap=1 if name=='connect' else 2
        else:raise ValueError('blend_trial_method_disabled')
        if self.entries[name]>=cap:raise ValueError('blend_trial_method_consumed')
        self.entries[name]+=1
        if name in self.READS:self.entries['reads']+=1


class TrialClient:
    def __init__(self,native,window):self.native,self.window=native,window
    def __getattr__(self,name):
        if name not in TrialWindow.READS|{'movel','stop_move'}:raise AttributeError(name)
        async def call(*args,**kwargs):
            self.window.entry(name)
            try:return await getattr(self.native,name)(*args,**kwargs)
            except BaseException:
                if name in ('movel','stop_move'):self.window.unknown=True
                raise
        return call
