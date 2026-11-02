program_begin(0)
disable_auto_sync()
local interrupt = __INTERRUPT__
local phase_clock = __PHASE_CLOCK__
local direct_api = __DIRECT_API__
local horizon = phase_clock and (direct_api and 0.02 or 0.04) or 0.08
local entered = false
local start = timestamp()
local origin
local function sample()
    local state = lebai:get_robot_state()
    local k = lebai:get_kin_data()
    assert(state == 'IDLE' or state == 'MOVING', 'state')
    for i=1,6 do
        local p,v = k.actual_joint_pose[i],k.actual_joint_speed[i]
        assert(p == p and v == v and math.abs(v)<0.3, 'feedback')
        if origin then assert(math.abs(p-origin[i])<0.04, 'displacement') end
    end
    return state,k
end
local function quiet_tail(seconds)
    local deadline = timestamp()+seconds*1000
    local quiet = nil
    local previous = timestamp()
    while timestamp()<deadline do
        local state,k = sample()
        local now = timestamp()
        if now-previous>=100 or now<previous then quiet=nil end
        previous=now
        local stationary = state=='IDLE'
        for i=1,6 do stationary=stationary and math.abs(k.actual_joint_speed[i])<=0.002 end
        print('PVAT_TAIL',now-start,state,k.actual_joint_pose[1],k.actual_joint_speed[1])
        if stationary then
            quiet=quiet or now
            if now-quiet>=200 then return true end
        else quiet=nil end
        rt_sleep(20)
    end
    return false
end
local ok,err = pcall(function()
    local state,k=sample()
    assert(state=='IDLE','requires_idle')
    for i=1,6 do assert(math.abs(k.actual_joint_speed[i])<0.002,'requires_quiet') end
    origin=k.actual_joint_pose
    assert(math.abs(origin[1])<2,'initial_joint_range')
    print('PVAT_BEGIN',origin[1],horizon)
    start=timestamp()
    local cutoff=interrupt and 1000 or 2000
    local due=start
    local last=nil
    local count=0
    while timestamp()-start<cutoff and count<(phase_clock and math.floor(2/horizon) or 100) do
        local now=timestamp()
        if now<due then rt_sleep(due-now) end
        now=timestamp()
        local elapsed=now-start
        assert(elapsed>=0,'clock_reversed')
        if elapsed>=cutoff then break end
        if last then assert(now-last<80,'send_gap') end
        -- In phase-clock mode each packet owns a distinct trajectory
        -- interval. Keep at most three intervals nominally ahead, rather
        -- than sending overlapping 80 ms windows at 20 ms wall intervals.
        local reference_time=phase_clock and (count+1)*horizon or (elapsed/1000+horizon)
        local u=math.min(1,reference_time/2)
        local p,v,a={},{},{}
        for i=1,6 do p[i]=origin[i];v[i]=0;a[i]=0 end
        p[1]=p[1]+0.01*(10*u^3-15*u^4+6*u^5)
        v[1]=0.005*(30*u^2-60*u^3+30*u^4)
        a[1]=0.0025*(60*u-180*u^2+120*u^3)
        entered=true
        if direct_api then
            local jp,jv,ja={},{},{}
            for i=1,6 do jp['j'..i]=p[i];jv['j'..i]=v[i];ja['j'..i]=a[i] end
            local accepted=move_pvat(jp,jv,ja,horizon)
            print('PVAT_NATIVE_RESULT',type(accepted),tostring(accepted))
            assert(accepted==true, 'native_pvat_rejected')
        else lebai:move_pvat(p,v,a,horizon) end
        local returned=timestamp()
        last=now;count=count+1
        local _,feedback=sample()
        print('PVAT_POINT',elapsed,p[1],feedback.actual_joint_pose[1],feedback.actual_joint_speed[1],returned-now)
        due=phase_clock and (start+math.max(0,(count+1)*horizon*1000-3*horizon*1000))
            or (start+(math.floor((timestamp()-start)/20)+1)*20)
    end
    print('PVAT_RELEASE',timestamp()-start,count)
    assert(quiet_tail(5),'natural_finish_timeout')
end)
if not ok and entered then
    local stopped=pcall(function() lebai:stop_move() end)
    local observed,quiet=pcall(function() return quiet_tail(2) end)
    print('PVAT_ABORT',stopped,observed and quiet)
end
print('PVAT_DONE',ok,tostring(err))
program_end(0)
