-- Bounded joint-space PVAT reference from the latest full-pose IK solution.
-- No I/O or motion calls; all calculations finish before state is committed.
local M={}
local function finite(x) return type(x)=='number' and x==x and math.abs(x)<1000000 end
local function clamp(x,lo,hi) return math.max(lo,math.min(hi,x)) end
local function velocity_jerk_ceiling(v,a,dt)
    -- Keep enough velocity headroom to cancel positive acceleration with the
    -- available jerk. Checking only next tick's velocity can enter a state
    -- from which no bounded jerk can avoid exceeding the following tick.
    local reserve=.06-v
    if a>1e-10 and .5*a*dt>reserve then
        assert(reserve>0,'pose_tracker_infeasible_rate')
        -- Acceleration crosses zero inside this interval; bound its interior
        -- velocity peak as well as the endpoint.
        return -a*a/(2*reserve)
    end
    local A=dt*dt -- dt^2 / (2*maximum_jerk), maximum_jerk=.5
    local B=.5*dt*dt+2*a*dt
    local C=v+a*dt+a*a-.06
    return (-B+math.sqrt(math.max(0,B*B-4*A*C)))/(2*A)
end

function M.new(origin,bounds)
    assert(type(origin)=='table' and #origin==6, 'pose_tracker_origin')
    if bounds then
        assert(type(bounds)=='table' and type(bounds.joint_min)=='table' and #bounds.joint_min==6
            and type(bounds.joint_max)=='table' and #bounds.joint_max==6,'pose_tracker_bounds')
    end
    local self={q={},v={},a={},origin={},lower={},upper={},steps=0}
    for i=1,6 do
        assert(finite(origin[i]), 'pose_tracker_origin')
        local lo=bounds and bounds.joint_min[i] or origin[i]-.08
        local hi=bounds and bounds.joint_max[i] or origin[i]+.08
        assert(finite(lo) and finite(hi) and lo<hi and origin[i]>=lo and origin[i]<=hi,'pose_tracker_bounds')
        self.lower[i]=lo;self.upper[i]=hi
        self.q[i]=origin[i];self.origin[i]=origin[i];self.v[i]=0;self.a[i]=0
    end
    function self.advance(target)
        assert(type(target)=='table' and #target==6, 'pose_tracker_target')
        local dt=.08
        local np,nv,na={},{},{}
        for i=1,6 do
            assert(finite(target[i]) and target[i]>=self.lower[i] and target[i]<=self.upper[i], 'pose_tracker_target_region')
            -- Critically damped triple pole at -4/s before bounded saturation.
            local j=64*(target[i]-self.q[i])-48*self.v[i]-12*self.a[i]
            local lo=math.max(-.5,(-.15-self.a[i])/dt,2*(-.06-self.v[i]-self.a[i]*dt)/(dt*dt))
            local hi=math.min(.5,(.15-self.a[i])/dt,2*(.06-self.v[i]-self.a[i]*dt)/(dt*dt))
            lo=math.max(lo,-velocity_jerk_ceiling(-self.v[i],-self.a[i],dt))
            hi=math.min(hi,velocity_jerk_ceiling(self.v[i],self.a[i],dt))
            -- Envelope roots can differ from the hard jerk endpoint by a few
            -- floating-point ulps (e.g. -.5000000000000006 versus -.5).
            assert(lo<=hi+1e-12, 'pose_tracker_infeasible_rate')
            if lo>hi then lo=(lo+hi)/2;hi=lo end
            j=clamp(j,lo,hi)
            j=clamp(j,-.5,.5)
            np[i]=self.q[i]+self.v[i]*dt+self.a[i]*dt*dt/2+j*dt*dt*dt/6
            nv[i]=self.v[i]+self.a[i]*dt+j*dt*dt/2
            na[i]=self.a[i]+j*dt
            assert(finite(np[i]) and np[i]>=self.lower[i] and np[i]<=self.upper[i], 'pose_tracker_reference_region')
            assert(math.abs(nv[i])<=.060000000001 and math.abs(na[i])<=.150000000001, 'pose_tracker_rate')
        end
        -- Keep internal snapshots private from callers modifying return arrays.
        for i=1,6 do self.q[i]=np[i];self.v[i]=nv[i];self.a[i]=na[i] end
        self.steps=self.steps+1
        return np,nv,na,dt
    end
    return self
end

return M
