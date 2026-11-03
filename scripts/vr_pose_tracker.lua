-- Bounded joint-space PVAT reference from the latest full-pose IK solution.
-- No I/O or motion calls; all calculations finish before state is committed.
local M={}
local function finite(x) return type(x)=='number' and x==x and math.abs(x)<1000000 end
local function clamp(x,lo,hi) return math.max(lo,math.min(hi,x)) end

function M.new(origin)
    assert(type(origin)=='table' and #origin==6, 'pose_tracker_origin')
    local self={q={},v={},a={},origin={},steps=0}
    for i=1,6 do
        assert(finite(origin[i]), 'pose_tracker_origin')
        self.q[i]=origin[i];self.origin[i]=origin[i];self.v[i]=0;self.a[i]=0
    end
    function self.advance(target)
        assert(type(target)=='table' and #target==6, 'pose_tracker_target')
        local dt=.08
        local np,nv,na={},{},{}
        for i=1,6 do
            assert(finite(target[i]) and math.abs(target[i]-self.origin[i])<=.08, 'pose_tracker_target_region')
            -- Critically damped triple pole at -4/s before bounded saturation.
            local j=64*(target[i]-self.q[i])-48*self.v[i]-12*self.a[i]
            local lo=math.max(-.5,(-.15-self.a[i])/dt,2*(-.06-self.v[i]-self.a[i]*dt)/(dt*dt))
            local hi=math.min(.5,(.15-self.a[i])/dt,2*(.06-self.v[i]-self.a[i]*dt)/(dt*dt))
            assert(lo<=hi, 'pose_tracker_infeasible_rate')
            j=clamp(j,lo,hi)
            np[i]=self.q[i]+self.v[i]*dt+self.a[i]*dt*dt/2+j*dt*dt*dt/6
            nv[i]=self.v[i]+self.a[i]*dt+j*dt*dt/2
            na[i]=self.a[i]+j*dt
            assert(finite(np[i]) and math.abs(np[i]-self.origin[i])<=.08, 'pose_tracker_reference_region')
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
