program_begin(0)
disable_auto_sync()
local mailbox=(function()
__POSE_MAILBOX_MODULE__
end)()
local tracker_module=(function()
__POSE_TRACKER_MODULE__
end)()
local entered=false
local origin,base
local started=timestamp()
local count=0
local function read_feedback(for_stop)
    local state=lebai:get_robot_state()
    local k=lebai:get_kin_data()
    assert(state=='IDLE' or state=='MOVING','pose_follow_state')
    for i=1,6 do
        local p,v=k.actual_joint_pose[i],k.actual_joint_speed[i]
        -- Preparation uses the existing Docker .6 rad/s feedback cap. The
        -- small-region PVAT experiment retains its stricter .3 rad/s guard.
        local cap=origin and .3 or .6
        if not(p==p and v==v and math.abs(p)<1000000 and math.abs(v)<1000000
            and (for_stop or math.abs(v)<=cap)) then
            print('POSE_FOLLOW_BAD_FEEDBACK',i,p,v,cap)
            error('pose_follow_feedback')
        end
        if origin and not for_stop then assert(math.abs(p-origin[i])<=.1,'pose_follow_joint_region') end
    end
    return state,k
end
local function quiet_tail(deadline)
    local since,previous=nil,timestamp()
    while timestamp()<deadline do
        local state,k=read_feedback(true);local now=timestamp()
        local quiet=state=='IDLE'
        for i=1,6 do quiet=quiet and math.abs(k.actual_joint_speed[i])<=.002 end
        if now<previous or now-previous>=100 then since=nil end
        previous=now
        if quiet then
            since=since or now
            if now-since>=200 then return k end
        else since=nil end
        rt_sleep(20)
    end
    error('pose_follow_quiet_unverified')
end
local function values(p)
    local sx,cx=math.sin(p.rx/2),math.cos(p.rx/2)
    local sy,cy=math.sin(p.ry/2),math.cos(p.ry/2)
    local sz,cz=math.sin(p.rz/2),math.cos(p.rz/2)
    local raw={p.x*1e6,p.y*1e6,p.z*1e6,(sx*cy*cz-cx*sy*sz)*1e7,
        (cx*sy*cz+sx*cy*sz)*1e7,(cx*cy*sz-sx*sy*cz)*1e7,(cx*cy*cz+sx*sy*sz)*1e7}
    for i=1,7 do raw[i]=math.floor(raw[i]+.5) end
    return raw
end
local ok,err=pcall(function()
    -- Exercise the reference generator without motion before any startup.
    local offline=tracker_module.new({0,0,0,0,0,0})
    local oldq,oldv,olda={0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}
    for tick=1,100 do
        -- The controller watchdog requires cooperative yields even for math-only tests.
        if tick%5==0 then rt_sleep(1) end
        local p,v,a,dt=offline.advance({.02,0,0,0,0,0})
        for i=1,6 do
            local jerk=(a[i]-olda[i])/dt
            assert(math.abs(jerk)<=.50000000001,'pose_follow_test_jerk')
            assert(math.abs(p[i]-(oldq[i]+oldv[i]*dt+olda[i]*dt^2/2+jerk*dt^3/6))<1e-12,'pose_follow_test_continuity')
            assert(math.abs(v[i]-(oldv[i]+olda[i]*dt+jerk*dt^2/2))<1e-12,'pose_follow_test_velocity')
        end
        oldq,oldv,olda=p,v,a
    end
    assert(math.abs(oldq[1]-.02)<1e-6 and math.abs(oldv[1])<1e-6,'pose_follow_test_convergence')
    local before=offline.steps
    local rejected=pcall(function() offline.advance({0,0,0,0,0,.081}) end)
    assert(not rejected and before==offline.steps,'pose_follow_test_atomic_rejection')
    print('POSE_FOLLOW_CHECK','reference_continuity_limits_convergence')

    assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'pose_follow_mailbox_occupied')
    local state=lebai:get_robot_state()
    if state=='ROBOT_ON' or state=='STOP' then
        lebai:start_sys()
        local deadline=timestamp()+8000
        while timestamp()<deadline do
            state=lebai:get_robot_state()
            if state=='IDLE' then break end
            assert(state=='ROBOT_ON' or state=='STOP' or state=='STARTING' or state=='BOOTING','pose_follow_startup')
            rt_sleep(100)
        end
    end
    assert(state=='IDLE','pose_follow_requires_idle')
    local ready={0,-math.pi/4,math.pi/2,-math.pi/4,math.pi/2,0}
    entered=true
    print('POSE_FOLLOW_MOTION_ENTER','movej')
    lebai:movej(ready,.2,.3,0,0)
    local prepared=quiet_tail(timestamp()+20000)
    for i=1,6 do assert(math.abs(prepared.actual_joint_pose[i]-ready[i])<.002,'pose_follow_prepare_error') end
    origin=prepared.actual_joint_pose;base=prepared.actual_tcp_pose
    local tracker=tracker_module.new(origin)
    started=timestamp()
    local reader=mailbox.new(function(ch) return lebai:get_signal(ch) end,timestamp,started,function() end,
        function(ch,n) return lebai:get_signals(ch,n) end)
    local seed=origin;local last_send=nil;local phase_start=nil
    local finish=false;local failure=nil;local latest=nil;local planned=nil
    print('POSE_FOLLOW_BEGIN',base.x,base.y,base.z)
    local function worker(fn)
        return async_task(function()
            local passed,reason=pcall(fn)
            if not passed then failure=failure or reason;finish=true end
            return passed
        end)
    end
    local source=worker(function()
        local seq=0;local previous=nil
        while not finish do
            local now=timestamp()
            assert(not previous or now-previous<100,'pose_follow_source_gap')
            local elapsed=(now-started)/1000
            -- Synthetic hand stream at about 20ms, independent of submission.
            local u=math.min(1,elapsed/2)
            local wave=elapsed<2 and math.sin(math.pi*u)^2 or 0
            local target={};for key,value in pairs(base) do target[key]=value end
            target.x=target.x+.003*wave;target.y=target.y+.001*wave;target.z=target.z+.001*wave
            target.rx=target.rx+.01*wave;target.rz=target.rz+.01*wave
            local payload=values(target);local offset=240+(seq%2)*8
            seq=seq+1
            lebai:set_signal(offset,0)
            lebai:set_signals(offset+1,payload)
            assert(timestamp()-now+1<100,'pose_follow_publish_expired')
            lebai:set_signal(offset,(now%2097152)*1024+seq%1024)
            previous=now
            rt_sleep(20)
        end
    end)
    local receiver=worker(function()
        while not finish do
            local sample=reader.read_latest()
            if sample then latest=sample end
            rt_sleep(5)
        end
    end)
    local planner=worker(function()
        local previous=nil
        while not finish do
            local sample=latest
            if sample and sample.captured_ms~=previous then
            local _,feedback=read_feedback()
            for i=1,6 do assert(math.abs(feedback.actual_joint_pose[i]-tracker.q[i])<.03,'pose_follow_tracking_error') end
            for _,axis in ipairs({'x','y','z'}) do
                assert(math.abs(feedback.actual_tcp_pose[axis]-base[axis])<=.025,'pose_follow_tcp_region')
            end
            -- Feedback RPCs yield: choose the newest input AFTER that wait,
            -- rather than spending its freshness budget on unrelated reads.
            sample=latest
            assert(sample and timestamp()-sample.captured_ms+1<100,'pose_follow_sample_expired')
            local solved=lebai:kinematics_inverse(sample.pose,seed)
            assert(solved and #solved==6,'pose_follow_ik')
            for i=1,6 do assert(solved[i]==solved[i] and math.abs(solved[i]-seed[i])<.03,'pose_follow_branch') end
            assert(timestamp()-sample.captured_ms+1<100,'pose_follow_plan_expired')
            -- One newest completed solution, not a historical endpoint queue.
            planned={sample=sample,joints=solved,feedback=feedback}
            previous=sample.captured_ms;seed=solved
            end
            rt_sleep(5)
        end
    end)
    local motion=worker(function()
        while not planned do
            assert(not finish and timestamp()-started<1500,'pose_follow_first_frame')
            rt_sleep(5)
        end
        for tick=1,70 do
            assert(not finish,'pose_follow_worker_failed')
            -- Unique phase intervals, at most two nominal intervals ahead.
            local due=phase_start and phase_start+math.max(0,(tick-2)*80) or timestamp()
            if timestamp()<due then rt_sleep(due-timestamp()) end
            local plan=planned
            local sample,feedback=plan.sample,plan.feedback
            local p,v,a,dt=tracker.advance(plan.joints)
            assert(not finish and timestamp()-sample.captured_ms+1<100,'pose_follow_write_expired')
            assert(not last_send or timestamp()-last_send<160,'pose_follow_cadence')
            last_send=timestamp()
            phase_start=phase_start or last_send
            local entry_age=last_send-sample.captured_ms
            lebai:move_pvat(p,v,a,dt)
            count=count+1
            print('POSE_FOLLOW_POINT',tick,last_send-started,sample.pose.x,feedback.actual_tcp_pose.x,
                feedback.actual_tcp_pose.y,feedback.actual_tcp_pose.z,v[1],entry_age,timestamp()-last_send)
            assert(timestamp()-(phase_start+math.max(0,(tick-1)*80))<80,'pose_follow_clock_lost')
        end
        finish=true
    end)
    -- Join all work, including any signal/SDK calls already entered. A worker
    -- failure revokes siblings before another motion, without cancelling I/O.
    wait_all(source,receiver,planner,motion)
    assert(not failure,tostring(failure))
    print('POSE_FOLLOW_RELEASE',timestamp()-started,count)
    local final=quiet_tail(timestamp()+3000)
    local error=0
    for _,axis in ipairs({'x','y','z'}) do error=math.max(error,math.abs(final.actual_tcp_pose[axis]-base[axis])) end
    print('POSE_FOLLOW_QUIET',timestamp()-started,error)
    assert(error<.0007,'pose_follow_final_error')
    lebai:set_signal(240,0);lebai:set_signal(248,0)
    assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'pose_follow_cleanup_unverified')
end)
if not ok and entered then
    local returned=pcall(function() lebai:stop_move() end)
    local quiet=pcall(function() quiet_tail(timestamp()+2500) end)
    print('POSE_FOLLOW_ABORT',returned,quiet)
end
print('POSE_FOLLOW_DONE',ok,tostring(err),count)
program_end(0)
