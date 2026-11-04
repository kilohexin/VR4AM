program_begin(0)
disable_auto_sync()
local mailbox=(function()
__POSE_MAILBOX_MODULE__
end)()
local tracker_module=(function()
__POSE_TRACKER_MODULE__
end)()
local path_module=(function()
__POSE_PATH_MODULE__
end)()
local function matrix(p)
    local sx,cx=math.sin(p.rx),math.cos(p.rx)
    local sy,cy=math.sin(p.ry),math.cos(p.ry)
    local sz,cz=math.sin(p.rz),math.cos(p.rz)
    return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
        sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
end
local function angle(a,b)
    local x,y=matrix(a),matrix(b);local sum=0
    for i=1,9 do sum=sum+x[i]*y[i] end
    return math.acos(math.max(-1,math.min(1,(sum-1)/2)))
end
local function equivalent_pose(a,b,position_tolerance,angle_tolerance)
    local d2=(a.x-b.x)^2+(a.y-b.y)^2+(a.z-b.z)^2
    return d2<=(position_tolerance or .00025)^2 and angle(a,b)<=(angle_tolerance or .002)
end
-- PRODUCT_POSE_BOUNDS
-- PRODUCT_POSE_PROFILE
local entered=false;local moves=0;local start=timestamp();local origin,base
local function feedback(for_stop)
    local state=lebai:get_robot_state();local k=lebai:get_kin_data()
    assert(state=='IDLE' or state=='MOVING','pose_robot_state')
    for i=1,6 do
        local p,v=k.actual_joint_pose[i],k.actual_joint_speed[i]
        assert(p==p and v==v and math.abs(p)<1000000 and math.abs(v)<1000000,'pose_nonfinite')
        if not for_stop then
            assert(math.abs(v)<=.3,'pose_speed')
        end
    end
    if not for_stop then
        assert(not product_bounds(k.actual_tcp_pose,k.actual_joint_pose),'pose_product_bounds')
        if base then
            for _,axis in ipairs({'x','y','z'}) do
                assert(math.abs(k.actual_tcp_pose[axis]-base[axis])<=pose_profile.feedback_tcp_span,'pose_tcp_region')
            end
        end
    end
    return state,k
end
local function tail()
    local deadline=timestamp()+2500;local since=nil;local previous=timestamp()
    while timestamp()<deadline do
        local state,k=feedback(true);local now=timestamp();local quiet=state=='IDLE'
        for i=1,6 do quiet=quiet and math.abs(k.actual_joint_speed[i])<=.002 end
        if now<previous or now-previous>=100 then since=nil end
        previous=now
        if quiet then
            since=since or now
            if now-since>=200 then print('LIVE_QUIET',now-start);return true end
        else since=nil end
        rt_sleep(20)
    end
    return false
end
local ok,err=pcall(function()
    assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'pose_mailbox_occupied')
    local state,k=feedback(false);assert(state=='IDLE','pose_requires_idle')
    for i=1,6 do assert(math.abs(k.actual_joint_speed[i])<=.002,'pose_requires_quiet') end
    origin=k.actual_joint_pose;base=k.actual_tcp_pose
    -- Full IK supplies each goal. Use the fixed absolute product joint limits,
    -- not the old differential-IK probe's .08rad box around its first point.
    local tracker=tracker_module.new(origin,{joint_min=pose_joint_min,joint_max=pose_joint_max})
    -- Warm the output channel before any hand target is admitted. Clock
    -- samples are diagnostics; only the final READY binds the frozen epoch.
    for i=1,3 do print('POSE_CLOCK',timestamp());rt_sleep(40) end
    start=timestamp()
    local reader=mailbox.new(function(ch) return lebai:get_signal(ch) end,timestamp,start,function() end,
        function(ch,n) return lebai:get_signals(ch,n) end,pose_profile.mailbox_ttl_ms)
    local latest,planned=nil,nil;local finish=false;local failure=nil;local boundary=nil
    local seed=origin;local phase_start,last_send=nil,nil
    local function worker(fn)
        return async_task(function()
            local good,reason=pcall(fn)
            if not good then failure=failure or reason;finish=true end
            return good
        end)
    end
    print('LIVE_READY',start)
    local receive=worker(function()
        while not finish do
            local good,sample=pcall(reader.read_latest)
            if not good then
                if string.find(tostring(sample),'pose_mailbox_input_expired',1,true) then
                    finish=true;print('LIVE_EXPIRED',timestamp()-start,moves);return
                end
                error(sample)
            end
            if sample then latest=sample end
            if timestamp()-start>=pose_profile.duration_ms then finish=true;return end
            rt_sleep(5)
        end
    end)
    local plan=worker(function()
        local previous=nil
        local cached_pose,cached_joints=nil,nil;local was_clipped=false
        while not finish do
            if latest and latest.captured_ms~=previous then
                local _,f=feedback(false)
                if finish then return end
                local sample=latest
                if timestamp()-sample.captured_ms+1>=pose_profile.mailbox_ttl_ms then
                    -- A read yielded while input advanced/expired. Discard this
                    -- computation; only receiver expiry retires the input stream.
                    previous=sample.captured_ms
                else
                boundary=product_bounds(sample.pose,nil)
                if boundary then finish=true;return end
                local target,clipped=path_module.clip(sample.pose,base,pose_profile.target_tcp_span)
                if clipped~=was_clipped then print('LIVE_SATURATED',clipped);was_clipped=clipped end
                -- Advance the IK branch through bounded Cartesian sections;
                -- a quick hand change is not itself an IK branch jump.
                local section=path_module.step(cached_pose or base,target,.004,.02)
                local joints
                -- Refresh the math cache BEFORE the outer reuse allowance is
                -- exhausted; otherwise every moving target waits at its edge.
                if cached_pose and equivalent_pose(cached_pose,section,.00010,.0008) then joints=cached_joints
                else
                    joints=lebai:kinematics_inverse(section,seed)
                    cached_pose=section;cached_joints=joints
                end
                if finish then return end
                assert(joints and #joints==6,'pose_ik_unreachable')
                for i=1,6 do
                    assert(joints[i]==joints[i] and math.abs(joints[i])<1000000,'pose_ik_nonfinite')
                    if math.abs(joints[i]-seed[i])>=.15 then
                        boundary='ik_branch';finish=true;return
                    end
                    assert(math.abs(f.actual_joint_pose[i]-tracker.q[i])<.03,'pose_tracking_error')
                end
                boundary=boundary or product_bounds(section,joints)
                if boundary then finish=true;return end
                -- Reuse this section only when a fresh full hand target agrees
                -- with its ORIGINAL source. Do not roll the source centre.
                local source_pose=sample.pose
                if latest.captured_ms>=sample.captured_ms and equivalent_pose(source_pose,latest.pose) then sample=latest end
                if timestamp()-sample.captured_ms+1<pose_profile.mailbox_ttl_ms then
                    planned={sample=sample,source_pose=source_pose,solved_pose=cached_pose,joints=joints,feedback=f}
                else
                    print('LIVE_PLAN_DROPPED',timestamp()-start,sample.sequence_mod)
                end
                previous=sample.captured_ms;seed=joints
                end
            end
            rt_sleep(5)
        end
    end)
    local motion=worker(function()
        local waiting=0
        while not planned do
            if finish then return end
            if timestamp()-start>=1500 then finish=true;return end
            rt_sleep(5)
        end
        for tick=1,pose_profile.maximum_points do
            if finish then return end
            local due=phase_start and phase_start+math.max(0,(tick-2)*80) or timestamp()
            if timestamp()<due then rt_sleep(due-timestamp()) end
            if finish then return end
            local goal=planned
            local current=latest
            boundary=product_bounds(current.pose,nil)
            if boundary then finish=true;return end
            if timestamp()-current.captured_ms+1>=pose_profile.mailbox_ttl_ms then
                print('POSE_MOTION_EXPIRED',timestamp(),current.captured_ms,current.sequence_mod,reader.last_capture,reader.max_read_ms,reader.max_age_ms)
                finish=true;print('LIVE_EXPIRED',timestamp()-start,moves);return
            end
            if equivalent_pose(goal.source_pose,current.pose) then
                goal={sample=current,joints=goal.joints,feedback=goal.feedback};waiting=0
            elseif tracker_module.fresh_solution(goal,current,timestamp(),equivalent_pose,pose_profile.mailbox_ttl_ms) then
                -- Keep the section's original source deadline, without
                -- relabeling it as a solution of the newer hand target.
                waiting=0
            else
                waiting=waiting+1
                if waiting>5 then boundary='planning_lag';finish=true;return end
                -- Fresh Grip, but its new IK solution is not ready. Hold the
                -- CURRENT reference with bounded P/V/A, never chase an old hand
                -- endpoint or stamp an old solution as the new target. Only
                -- known computation lag can use this <=400ms hold allowance.
                goal={sample=current,joints=tracker.q,feedback=goal.feedback}
                print('LIVE_PLAN_WAIT',timestamp()-start,waiting)
            end
            local p,v,a,dt=tracker.advance(goal.joints)
            assert(not product_bounds(nil,p),'pose_reference_bounds')
            if timestamp()-goal.sample.captured_ms+1>=pose_profile.mailbox_ttl_ms then
                finish=true;print('LIVE_EXPIRED',timestamp()-start,moves);return
            end
            assert(not last_send or timestamp()-last_send<160,'pose_cadence_lost')
            entered=true;last_send=timestamp();phase_start=phase_start or last_send
            lebai:move_pvat(p,v,a,dt);moves=moves+1
            print('LIVE_POINT',last_send-start,goal.sample.sequence_mod,goal.feedback.actual_tcp_pose.x,
                goal.feedback.actual_tcp_pose.y,goal.feedback.actual_tcp_pose.z,goal.feedback.actual_joint_speed[1])
            if finish then return end
            assert(timestamp()-(phase_start+math.max(0,(tick-1)*80))<80,'pose_clock_lost')
        end
        finish=true
    end)
    wait_all(receive,plan,motion)
    print('POSE_INPUT_METRICS',reader.max_read_ms,reader.max_age_ms)
    assert(not failure,tostring(failure))
    if boundary then print('LIVE_BOUNDARY',timestamp()-start,boundary) end
    print('LIVE_RELEASE',timestamp()-start,moves)
    assert(tail(),'pose_natural_tail_unverified')
end)
if not ok and entered then
    local returned=pcall(function() lebai:stop_move() end)
    local observed,quiet=pcall(tail)
    print('LIVE_ABORT',returned,observed and quiet)
end
print('LIVE_DONE',ok,tostring(err),moves)
program_end(0)
