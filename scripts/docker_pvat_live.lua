program_begin(0)
disable_auto_sync()
-- Isolated Cartesian-direction experiment, not a product or physical selector.
-- One int32 mailbox holds timestamp and direction atomically (no pose queue).
local channel=240
local vector_mailbox = false
local pose_mailbox = false
local tcp_speed = 0.005
local function matrix(p)
    local sx,cx=math.sin(p.rx),math.cos(p.rx)
    local sy,cy=math.sin(p.ry),math.cos(p.ry)
    local sz,cz=math.sin(p.rz),math.cos(p.rz)
    return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
        sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
end
local function tool_offset(p,axis,angle)
    local r=matrix(p);local c,s=math.cos(angle),math.sin(angle)
    for row=0,2 do
        local first=axis==4 and row*3+2 or row*3+1
        local second=axis==4 and row*3+3 or row*3+2
        local x,y=r[first],r[second]
        r[first]=x*c+y*s;r[second]=-x*s+y*c
    end
    local out={};for key,value in pairs(p) do out[key]=value end
    out.ry=math.asin(math.max(-1,math.min(1,-r[7])))
    if math.abs(math.cos(out.ry))<1e-8 then out.rx=0;out.rz=math.atan2(-r[2],r[5])
    else out.rx=math.atan2(r[8],r[9]);out.rz=math.atan2(r[4],r[1]) end
    return out
end
local function rotation_distance(p,reference)
    local a,b=matrix(p),matrix(reference);local trace=0
    for i=1,9 do trace=trace+a[i]*b[i] end
    return math.acos(math.max(-1,math.min(1,(trace-1)/2)))
end
local function product_boundary(k,goal,direction) return nil end
local modulus=pose_mailbox and 2048 or vector_mailbox and 65536 or 33554432
local step=0.08
local start=timestamp()
local entered=false
local expired=false
local moves=0
local braking=false
local brake_reason='reference_boundary'
local origin,q,v,a,gain,base
local region_failure=nil
local function read_feedback(for_stop)
    local state=lebai:get_robot_state()
    local k=lebai:get_kin_data()
    assert(state=='IDLE' or state=='MOVING','robot_state')
    for i=1,6 do
        assert(k.actual_joint_pose[i]==k.actual_joint_pose[i] and math.abs(k.actual_joint_pose[i])<1000000,'nonfinite_pose')
        assert(k.actual_joint_speed[i]==k.actual_joint_speed[i] and math.abs(k.actual_joint_speed[i])<1000000,'nonfinite_speed')
        if not for_stop then assert(math.abs(k.actual_joint_speed[i])<=0.3,'speed_limit') end
        if origin and math.abs(k.actual_joint_pose[i]-origin[i])>0.1 then
            region_failure='joint_region'
            if not for_stop then error('joint_region') end
        end
    end
    -- PRODUCT_HARD_BOUNDS
    if base then
        for _,axis in ipairs({'x','y','z'}) do
            if math.abs(k.actual_tcp_pose[axis]-base[axis])>0.025 then
                region_failure='tcp_region'
                if not for_stop then error('tcp_region') end
            end
        end
    end
    return state,k
end
local function input()
    local word=lebai:get_signal(channel)
    assert(word>=0 and word<2147483648 and word==math.floor(word),'mailbox_invalid')
    if (vector_mailbox or pose_mailbox) and word==0 then return nil end
    local width=pose_mailbox and 1048576 or vector_mailbox and 32768 or 64
    local stamp=math.floor(word/width)
    local action=word%width
    if pose_mailbox then
        for i=0,4 do assert(math.floor(action/16^i)%16<15,'mailbox_pose') end
        action=action+1
    elseif vector_mailbox then
        assert(math.floor(action/1024)<31 and math.floor(action/32)%32<31 and action%32<31,'mailbox_vector')
        action=action+1
    else assert(action<=6,'mailbox_action') end
    local age=(timestamp()%modulus-stamp+modulus)%modulus
    if age>=100 then return nil end
    -- Once input expires this program may only drain, never restart on a late packet.
    if expired then return nil end
    return action,age
end
local function tail()
    local deadline=timestamp()+2500;local since=nil;local previous=timestamp()
    while timestamp()<deadline do
        local state,k=read_feedback(true);local now=timestamp()
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
    if pose_mailbox then
        for axis=4,5 do
            local solution=lebai:kinematics_inverse(tool_offset(base,axis,0.001),origin)
            assert(solution and #solution==6,'angular_ik_unreachable');gain[axis]={}
            for i=1,6 do
                local d=(solution[i]-origin[i])/0.001
                assert(d==d and math.abs(d)<30,'angular_ik_gain');gain[axis][i]=d
            end
        end
    end
    start=timestamp();print('LIVE_READY',start)
    local due=start;local started=nil;local last_send=nil
    while timestamp()-start<10000 and moves<125 do
        if not started and timestamp()-start>=1500 then break end
        local action=input()
        if not started and timestamp()-start>=1500 then break end
        if not action then
            if started then expired=true;print('LIVE_EXPIRED',timestamp()-start,moves);break end
            if timestamp()-start>=1500 then break end
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
            if not last_send then assert(timestamp()-started<100,'startup_feedback_expired') end
            if last_send then assert(timestamp()-last_send<160,'cadence_lost') end
            local goal={0,0,0,0,0,0}
            local direction={0,0,0}
            if pose_mailbox then
                local payload=action-1;direction={}
                for axis=1,5 do direction[axis]=(math.floor(payload/16^(5-axis))%16-7)/7 end
                local length=math.sqrt(direction[1]^2+direction[2]^2+direction[3]^2)
                if length>0 then for axis=1,3 do direction[axis]=direction[axis]/length end end
                for i=1,6 do
                    for axis=1,5 do
                        goal[i]=goal[i]+gain[axis][i]*(axis<=3 and tcp_speed or axis==5 and 0.10 or 0.04)*direction[axis]
                    end
                end
            elseif vector_mailbox then
                local payload=action-1
                direction={math.floor(payload/1024)-15,math.floor(payload/32)%32-15,payload%32-15}
                local length=math.sqrt(direction[1]^2+direction[2]^2+direction[3]^2)
                if length>0 then
                    for axis=1,3 do direction[axis]=direction[axis]/length end
                    for i=1,6 do
                        for axis=1,3 do goal[i]=goal[i]+gain[axis][i]*tcp_speed*direction[axis] end
                    end
                end
            elseif action>0 then
                local axis=math.floor((action-1)/2)+1
                local sign=action%2==1 and 1 or -1
                direction[axis]=sign
                for i=1,6 do goal[i]=gain[axis][i]*tcp_speed*sign end
            end
            if not braking then
                local reason=product_boundary(feedback,goal,direction)
                for axis,name in ipairs({'x','y','z'}) do
                    local delta=feedback.actual_tcp_pose[name]-base[name]
                    if math.abs(delta)>=0.014 and delta*direction[axis]>0 then reason=reason or 'reference_boundary' end
                end
                if reason then braking=true;brake_reason=reason;print('LIVE_BRAKE',timestamp()-start,reason) end
            end
            -- Reserve room to decelerate before the hard 0.08 rad reference
            -- guard. A held input cannot override this gesture-local boundary.
            for i=1,6 do
                local reference_brake=math.abs(v[i])>0.031 and 0.050 or 0.060
                if not braking and math.abs(q[i]-origin[i])>=reference_brake and (q[i]-origin[i])*v[i]>0 then
                    braking=true;print('LIVE_BRAKE',timestamp()-start,'reference_boundary')
                end
            end
            if braking then
                local settled=true
                for i=1,6 do
                    goal[i]=0
                    settled=settled and math.abs(v[i])<0.0002 and math.abs(a[i])<0.002
                end
                if settled then print('LIVE_BOUNDARY',timestamp()-start,brake_reason);break end
            end
            local maximum=0
            for i=1,6 do maximum=math.max(maximum,math.abs(goal[i])) end
            local cap=pose_mailbox and direction[5]~=0 and 0.045 or 0.03
            local scale=maximum>cap and cap/maximum or 1
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
    assert(not region_failure,region_failure)
end)
if not ok and entered then
    local stopped=pcall(function() lebai:stop_move() end)
    local observed,quiet=pcall(tail)
    print('LIVE_ABORT',stopped,observed and quiet)
end
print('LIVE_DONE',ok,tostring(err),moves)
program_end(0)
