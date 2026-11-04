-- Bounded joint-space PVAT reference from the latest full-pose IK solution.
-- No I/O or motion calls; all calculations finish before state is committed.
local M={}
local function finite(x) return type(x)=='number' and x==x and math.abs(x)<1000000 end
local function clamp(x,lo,hi) return math.max(lo,math.min(hi,x)) end
function M.fresh_solution(goal,current,now,similar)
    -- A moving hand need not coincide with the last completed IK. Its genuine
    -- sample age still governs use; never relabel it with the latest timestamp.
    return now>=goal.sample.captured_ms and now-goal.sample.captured_ms+1<100
        and now>=current.captured_ms and now-current.captured_ms+1<100
        and similar(goal.solved_pose,current.pose,.010,.05)
end
local function velocity_jerk_ceiling(v,a,dt)
    -- Keep enough velocity headroom to cancel positive acceleration with the
    -- available jerk. Checking only next tick's velocity can enter a state
    -- from which no bounded jerk can avoid exceeding the following tick.
    local function safe(j)
        local nv=v+a*dt+j*dt*dt/2
        local na=a+j*dt
        local peak=math.max(v,nv)
        if a>0 and j<0 and na<0 then peak=math.max(peak,v-a*a/(2*j)) end
        -- Only POSITIVE final acceleration needs upper braking reserve.
        -- Squaring negative acceleration incorrectly constrains reversals.
        return peak<=.06+1e-13 and nv+math.max(na,0)^2<=.06+1e-13
    end
    if safe(.5) then return .5 end
    assert(safe(-.5),'pose_tracker_infeasible_rate')
    local lo,hi=-.5,.5
    for i=1,44 do
        local middle=(lo+hi)/2
        if safe(middle) then lo=middle else hi=middle end
    end
    return lo
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
