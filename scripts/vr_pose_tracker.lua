-- Bounded joint-space PVAT reference from the latest full-pose IK solution.
-- No I/O or motion calls; all calculations finish before state is committed.
local M={}
local function finite(x) return type(x)=='number' and x==x and math.abs(x)<1000000 end
local function clamp(x,lo,hi) return math.max(lo,math.min(hi,x)) end
function M.fresh_solution(goal,current,now,similar,input_ttl)
    -- A moving hand need not coincide with the last completed IK. Its genuine
    -- sample age still governs use; never relabel it with the latest timestamp.
    local ttl=input_ttl or 100
    return now>=goal.sample.captured_ms and now-goal.sample.captured_ms+1<ttl
        and now>=current.captured_ms and now-current.captured_ms+1<ttl
        -- A bounded IK path section may lag the final hand endpoint. Only its
        -- genuine input age authorizes it; the physical reference stays rate
        -- limited, and old sections are never queued or restamped.
end
local function velocity_jerk_ceiling(v,a,dt,speed,jerk)
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
        return peak<=speed+1e-13 and nv+math.max(na,0)^2/(2*jerk)<=speed+1e-13
    end
    if safe(jerk) then return jerk end
    assert(safe(-jerk),'pose_tracker_infeasible_rate')
    local lo,hi=-jerk,jerk
    for i=1,44 do
        local middle=(lo+hi)/2
        if safe(middle) then lo=middle else hi=middle end
    end
    return lo
end

function M.new(origin,bounds,rates)
    assert(type(origin)=='table' and #origin==6, 'pose_tracker_origin')
    local speed=rates and rates.speed or .06
    local acceleration=rates and rates.acceleration or .15
    local jerk=rates and rates.jerk or .5
    local frequency=rates and rates.frequency or 4
    assert(finite(speed) and speed>0 and speed<=.2 and finite(acceleration) and acceleration>0 and acceleration<=.2
        and finite(jerk) and jerk>0 and jerk<=1 and finite(frequency) and frequency>=1 and frequency<=8,'pose_tracker_profile')
    if bounds then
        assert(type(bounds)=='table' and type(bounds.joint_min)=='table' and #bounds.joint_min==6
            and type(bounds.joint_max)=='table' and #bounds.joint_max==6,'pose_tracker_bounds')
    end
    local self={q={},v={},a={},origin={},lower={},upper={},steps=0,history={}}
    for i=1,6 do
        assert(finite(origin[i]), 'pose_tracker_origin')
        local lo=bounds and bounds.joint_min[i] or origin[i]-.08
        local hi=bounds and bounds.joint_max[i] or origin[i]+.08
        assert(finite(lo) and finite(hi) and lo<hi and origin[i]>=lo and origin[i]<=hi,'pose_tracker_bounds')
        self.lower[i]=lo;self.upper[i]=hi
        self.q[i]=origin[i];self.origin[i]=origin[i];self.v[i]=0;self.a[i]=0
    end
    function self.matches_report(report)
        -- Actual and target come from the same returned kinematics report.
        -- A queued future endpoint is NOT the instantaneous servo reference.
        local actual,target=report.actual_joint_pose,report.target_joint_pose
        if type(actual)~='table' or #actual~=6 or type(target)~='table' or #target~=6 then return false,'pose_tracking_report' end
        for i=1,6 do
            if not finite(actual[i]) or not finite(target[i]) then return false,'pose_tracking_report',i end
            if math.abs(actual[i]-target[i])>=.03 then return false,'pose_tracking_error',i end
            -- Also reject a reported target outside the recent generated
            -- trajectory envelope. History is fixed at eight 80ms segments;
            -- it cannot keep authorizing an indefinitely stalled controller.
            local lo,hi=self.q[i],self.q[i]
            if self.steps==0 then lo=self.origin[i];hi=lo end
            for _,segment in ipairs(self.history) do
                lo=math.min(lo,segment.q[i]);hi=math.max(hi,segment.q[i])
            end
            if target[i]<lo-.03 or target[i]>hi+.03 then return false,'pose_tracking_reference',i end
        end
        return true
    end
    function self.advance(target)
        assert(type(target)=='table' and #target==6, 'pose_tracker_target')
        local dt=.08
        local np,nv,na={},{},{}
        for i=1,6 do
            assert(finite(target[i]) and target[i]>=self.lower[i] and target[i]<=self.upper[i], 'pose_tracker_target_region')
            -- Triple pole at -frequency/s before explicit bounded saturation.
            local j=frequency^3*(target[i]-self.q[i])-3*frequency^2*self.v[i]-3*frequency*self.a[i]
            local lo=math.max(-jerk,(-acceleration-self.a[i])/dt,2*(-speed-self.v[i]-self.a[i]*dt)/(dt*dt))
            local hi=math.min(jerk,(acceleration-self.a[i])/dt,2*(speed-self.v[i]-self.a[i]*dt)/(dt*dt))
            lo=math.max(lo,-velocity_jerk_ceiling(-self.v[i],-self.a[i],dt,speed,jerk))
            hi=math.min(hi,velocity_jerk_ceiling(self.v[i],self.a[i],dt,speed,jerk))
            -- Envelope roots can differ from the hard jerk endpoint by a few
            -- floating-point ulps (e.g. -.5000000000000006 versus -.5).
            assert(lo<=hi+1e-12, 'pose_tracker_infeasible_rate')
            if lo>hi then lo=(lo+hi)/2;hi=lo end
            j=clamp(j,lo,hi)
            j=clamp(j,-jerk,jerk)
            np[i]=self.q[i]+self.v[i]*dt+self.a[i]*dt*dt/2+j*dt*dt*dt/6
            nv[i]=self.v[i]+self.a[i]*dt+j*dt*dt/2
            na[i]=self.a[i]+j*dt
            assert(finite(np[i]) and np[i]>=self.lower[i] and np[i]<=self.upper[i], 'pose_tracker_reference_region')
            assert(math.abs(nv[i])<=speed+1e-12 and math.abs(na[i])<=acceleration+1e-12, 'pose_tracker_rate')
        end
        -- Only the last eight generated reference segments are retained for
        -- time-aligned feedback checks. This is not a queue of hand targets.
        local segment={q={}}
        for i=1,6 do
            segment.q[i]=self.q[i]
        end
        self.history[#self.history+1]=segment
        if #self.history>8 then table.remove(self.history,1) end
        -- Keep internal snapshots private from callers modifying return arrays.
        for i=1,6 do self.q[i]=np[i];self.v[i]=nv[i];self.a[i]=na[i] end
        self.steps=self.steps+1
        return np,nv,na,dt
    end
    return self
end

return M
