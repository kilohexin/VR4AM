program_begin(0)
disable_auto_sync()
-- Isolated Cartesian-direction experiment, not a product or physical selector.
-- One int32 mailbox holds timestamp and direction atomically (no pose queue).
local channel=240
local modulus=33554432
local step=0.08
local start=timestamp()
local entered=false
local expired=false
local moves=0
local origin,q,v,a,gain,base
local function read_feedback()
    local state=lebai:get_robot_state()
    local k=lebai:get_kin_data()
    assert(state=='IDLE' or state=='MOVING','robot_state')
    for i=1,6 do
        assert(k.actual_joint_pose[i]==k.actual_joint_pose[i],'nonfinite_pose')
        assert(math.abs(k.actual_joint_speed[i])<=0.3,'speed_limit')
        if origin then assert(math.abs(k.actual_joint_pose[i]-origin[i])<=0.1,'joint_region') end
    end
    if base then
        for _,axis in ipairs({'x','y','z'}) do
            assert(math.abs(k.actual_tcp_pose[axis]-base[axis])<=0.025,'tcp_region')
        end
    end
    return state,k
end
local function input()
    local word=lebai:get_signal(channel)
    assert(word>=0 and word<2147483648 and word==math.floor(word),'mailbox_invalid')
    local stamp=math.floor(word/64)
    local action=word%64
    assert(action<=6,'mailbox_action')
    local age=(timestamp()%modulus-stamp+modulus)%modulus
    if age>=100 then return nil end
    -- Once input expires this program may only drain, never restart on a late packet.
    if expired then return nil end
    return action,age
end
local function tail()
    local deadline=timestamp()+2500;local since=nil;local previous=timestamp()
    while timestamp()<deadline do
        local state,k=read_feedback();local now=timestamp()
        local quiet=state=='IDLE'
        for i=1,6 do quiet=quiet and math.abs(k.actual_joint_speed[i])<=0.002 end
        if now<previous or now-previous>=100 then since=nil end
        previous=now
        if quiet then
            since=since or now
            if now-since>=200 then
                print('LIVE_QUIET',now-start,k.actual_tcp_pose.x,k.actual_tcp_pose.y,k.actual_tcp_pose.z)
                return true
            end
        else since=nil end
        rt_sleep(20)
    end
    return false
end
local ok,err=pcall(function()
    assert(lebai:get_signal(channel)==0,'mailbox_occupied')
    local state,k=read_feedback()
    assert(state=='IDLE','requires_idle')
    origin=k.actual_joint_pose;base=k.actual_tcp_pose
    q={};v={};a={};gain={}
    for i=1,6 do
        assert(math.abs(k.actual_joint_speed[i])<=0.002,'requires_quiet')
        q[i]=origin[i];v[i]=0;a[i]=0
    end
    -- Freeze a local differential IK map for this short, 25 mm box only.
    -- This is a small-region candidate, not a global Cartesian path guarantee.
    for axis,name in ipairs({'x','y','z'}) do
        local p={};for key,value in pairs(base) do p[key]=value end
        p[name]=p[name]+0.001
        local solution=lebai:kinematics_inverse(p,origin)
        assert(solution and #solution==6,'ik_unreachable')
        gain[axis]={}
        for i=1,6 do
            local d=(solution[i]-origin[i])/0.001
            assert(d==d and math.abs(d)<30,'ik_gain')
            gain[axis][i]=d
        end
    end
    start=timestamp();print('LIVE_READY',start)
    local due=start;local started=nil;local last_send=nil
    while timestamp()-start<10000 and moves<125 do
        local action=input()
        if not action then
            if started then expired=true;print('LIVE_EXPIRED',timestamp()-start,moves);break end
            assert(timestamp()-start<1500,'input_start_timeout')
            rt_sleep(10)
        else
            started=started or timestamp()
            -- Read feedback ahead of the nominal send deadline instead of
            -- adding the read RPC duration after every scheduled send time.
            if timestamp()<due-30 then rt_sleep(due-30-timestamp()) end
            local state,feedback=read_feedback()
            if timestamp()<due-10 then rt_sleep(due-10-timestamp()) end
            local input_age
            action,input_age=input()
            local captured=timestamp()
            if not action then expired=true;print('LIVE_EXPIRED',timestamp()-start,moves);break end
            if last_send then assert(timestamp()-last_send<160,'cadence_lost') end
            local goal={0,0,0,0,0,0}
            if action>0 then
                local axis=math.floor((action-1)/2)+1
                local sign=action%2==1 and 1 or -1
                for i=1,6 do goal[i]=gain[axis][i]*0.005*sign end
            end
            local maximum=0
            for i=1,6 do maximum=math.max(maximum,math.abs(goal[i])) end
            local scale=maximum>0.03 and 0.03/maximum or 1
            local np,nv,na={},{},{}
            for i=1,6 do
                assert(math.abs(feedback.actual_joint_pose[i]-q[i])<0.03,'tracking_error')
                local jerk=64*(goal[i]*scale-v[i])-16*a[i]
                jerk=math.max(-0.5,math.min(0.5,jerk))
                jerk=math.max((-0.15-a[i])/step,math.min((0.15-a[i])/step,jerk))
                np[i]=q[i]+v[i]*step+a[i]*step^2/2+jerk*step^3/6
                nv[i]=v[i]+a[i]*step+jerk*step^2/2
                na[i]=a[i]+jerk*step
                assert(math.abs(nv[i])<=0.06 and math.abs(np[i]-origin[i])<=0.08,'reference_limit')
            end
            -- Recheck freshness at write entry, including time spent computing.
            -- Calculation above does not yield. Do not fault on a legitimate
            -- direction change arriving meanwhile; consume it on the next tick.
            if timestamp()-captured+input_age>=100 then
                expired=true;print('LIVE_EXPIRED',timestamp()-start,moves);break
            end
            entered=true;last_send=timestamp()
            lebai:move_pvat(np,nv,na,step)
            q=np;v=nv;a=na;moves=moves+1
            print('LIVE_POINT',last_send-start,action,feedback.actual_tcp_pose.x,feedback.actual_tcp_pose.y,
                feedback.actual_tcp_pose.z,feedback.actual_joint_speed[1])
            -- Two distinct intervals maximum nominal lead; never resend an
            -- overlapping future window. Absorb read-RPC jitter without an
            -- ever-growing command queue.
            due=started+math.max(0,moves*80-80)
            assert(timestamp()-due<80,'trajectory_clock_lost')
        end
    end
    print('LIVE_RELEASE',timestamp()-start,moves)
    assert(tail(),'natural_tail_failed')
end)
if not ok and entered then
    local stopped=pcall(function() lebai:stop_move() end)
    local observed,quiet=pcall(tail)
    print('LIVE_ABORT',stopped,observed and quiet)
end
print('LIVE_DONE',ok,tostring(err),moves)
program_end(0)
