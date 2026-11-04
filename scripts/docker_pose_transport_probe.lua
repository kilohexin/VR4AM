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

local function near(a,b,tol) assert(math.abs(a-b)<=tol, 'probe_numeric_mismatch') end
local function expect_error(fn,code)
    local ok,err=pcall(fn)
    assert(not ok and string.find(tostring(err),code,1,true), 'probe_missing_rejection_'..code)
end
local function test(name,fn) fn();print('POSE_PROBE_CHECK',name) end
local function fake()
    local f={now=1000,mem={},allowed=true,hook=nil}
    for i=240,255 do f.mem[i]=0 end
    function f.read(ch)
        local value=f.mem[ch]
        if f.hook then f.hook(ch) end
        return value
    end
    function f.reader(epoch,bulk,ttl)
        return mailbox.new(f.read,function() return f.now end,epoch or f.now,
            function() assert(f.allowed,'revoked') end,bulk and function(ch,n)
                local result={};for i=1,n do result[i]=f.read(ch+i-1) end;return result
            end or nil,ttl)
    end
    function f.publish(bank,stamp,seq,x)
        local offset=240+bank*8
        f.mem[offset]=0
        local values={x or 1000,200000,300000,0,0,0,10000000}
        for i=1,7 do f.mem[offset+i]=values[i] end
        f.mem[offset]=(stamp%2097152)*1024+seq%1024
    end
    return f
end
local function matrix(p)
    local sx,cx=math.sin(p.rx),math.cos(p.rx)
    local sy,cy=math.sin(p.ry),math.cos(p.ry)
    local sz,cz=math.sin(p.rz),math.cos(p.rz)
    return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
        sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
end
local function quaternion(p)
    local sx,cx=math.sin(p.rx/2),math.cos(p.rx/2)
    local sy,cy=math.sin(p.ry/2),math.cos(p.ry/2)
    local sz,cz=math.sin(p.rz/2),math.cos(p.rz/2)
    return {sx*cy*cz-cx*sy*sz,cx*sy*cz+sx*cy*sz,cx*cy*sz-sx*sy*cz,cx*cy*cz+sx*sy*sz}
end
local function encode(p)
    local values={math.floor(p.x*1e6+.5),math.floor(p.y*1e6+.5),math.floor(p.z*1e6+.5)}
    for _,v in ipairs(quaternion(p)) do values[#values+1]=math.floor(v*1e7+.5) end
    return values
end

local ok,err=pcall(function()
    test('bounded_pose_path',function()
        local base={x=0,y=0,z=0,rx=0,ry=0,rz=math.pi-.01}
        local target={x=.3,y=-.3,z=0,rx=0,ry=0,rz=-math.pi+.01}
        local clipped,saturated=path_module.clip(target,base,.1)
        assert(saturated and clipped.x==.1 and clipped.y==-.1 and target.x==.3,'fixed_box_clamp')
        local p=path_module.step(base,clipped,.004,.005)
        near(math.sqrt(p.x*p.x+p.y*p.y),.004,1e-10)
        assert(p.rz>3.12 or p.rz< -3.12,'rotation_took_long_path')
        local a,b=matrix(base),matrix(p);local dot=0;for i=1,9 do dot=dot+a[i]*b[i] end
        near(math.acos(math.max(-1,math.min(1,(dot-1)/2))),.005,1e-9)
        for tick=1,100 do if tick%10==0 then rt_sleep(1) end;p=path_module.step(p,clipped,.004,.005) end
        near(p.x,.1,1e-9);near(p.y,-.1,1e-9)
        local returned,again=path_module.clip(base,base,.1);assert(not again and returned.x==0,'cannot_return_from_edge')
    end)
    test('empty',function() local f=fake();assert(f.reader().read_latest()==nil) end)
    test('explicit_virtual_mailbox_budget',function()
        local f=fake();f.publish(0,1000,1);local r=f.reader(1000,false,250)
        r.read_latest();f.now=1150;assert(r.read_latest().sequence_mod==1)
        f.now=1249;expect_error(r.read_latest,'input_expired')
        f.now=1250;f.publish(1,1250,2);expect_error(r.read_latest,'retired')
        local strict=fake();strict.publish(0,1000,1);local old=strict.reader();old.read_latest()
        strict.now=1100;expect_error(old.read_latest,'input_expired')
    end)
    test('clock_diagnostic_preserves_rejection',function()
        for _,invalid in ipairs({false,1000.5,999}) do
            local f=fake();local r=f.reader();f.now=invalid
            expect_error(r.read_latest,'pose_mailbox_clock')
            assert(r.closed and not r.busy,'clock_diagnostic_masked_rejection')
        end
    end)
    test('batched_header_sandwich',function()
        for bank=0,1 do
            for field=1,7 do
                local f=fake();f.publish(bank,1000,1,1000);local r=f.reader(1000,true)
                local reads=0
                f.hook=function(ch)
                    reads=reads+1
                    if reads>9 and ch==240+bank*8+field then
                        f.hook=nil;f.now=1001;f.publish(bank,1001,2,2000)
                    end
                end
                assert(r.read_latest()==nil,'torn_batch_accepted')
                assert(r.read_latest().sequence_mod==2,'new_batch_missing')
            end
        end
        local f=fake();f.publish(0,1000,1,1000)
        local original=f.read;local calls=0
        f.read=function(ch) calls=calls+1;return original(ch) end
        assert(f.reader(1000,true).read_latest().sequence_mod==1)
        assert(calls==17,'batch_layout') -- 9 header-batch fields, 7 payload, 1 recheck.
    end)
    test('fresh_ik_following',function()
        local g={sample={captured_ms=1000},solved_pose={x=0}}
        local c={captured_ms=1020,pose={x=.006}}
        local function similar(a,b,p,r)
            assert(p==.010 and r==.05,'following_envelope')
            return math.abs(a.x-b.x)<=p
        end
        assert(tracker_module.fresh_solution(g,c,1021,similar),'moving_hand_should_follow')
        assert(g.sample.captured_ms==1000,'old_solution_timestamp_changed')
        assert(not tracker_module.fresh_solution(g,c,1099,similar),'expired_ik_accepted')
        assert(not tracker_module.fresh_solution(g,c,999,similar),'rollback_accepted')
        c.pose.x=.011;assert(tracker_module.fresh_solution(g,c,1021,similar),'fresh_path_section_rejected')
    end)
    test('absolute_joint_reference',function()
        local lo={-1,-1,-1,-1,-1,-1};local hi={1,1,1,1,1,1}
        local t=tracker_module.new({0,0,0,0,0,0},{joint_min=lo,joint_max=hi})
        lo[1]=.2;hi[1]=.01 -- Cannot move a bound through the caller's arrays.
        local function advance(goal,count)
            for tick=1,count do
                if tick%5==0 then rt_sleep(1) end
                local oldv,olda=t.v[1],t.a[1]
                local p,v,a=t.advance({goal,0,0,0,0,0})
                assert(math.abs(v[1])<=.060000000001 and math.abs(a[1])<=.150000000001,'extended_reference_rate')
                local jerk=(a[1]-olda)/.08
                assert(math.abs(jerk)<=.500000000001,'extended_reference_jerk')
                for sub=0,10 do
                    local dt=sub*.008
                    assert(math.abs(oldv+olda*dt+jerk*dt*dt/2)<=.060000000001,'interior_velocity_peak')
                end
            end
        end
        advance(.3,30) -- Reverse while still travelling at the positive cap.
        advance(-.3,300);near(t.q[1],-.3,1e-5)
        advance(.3,300)
        near(t.q[1],.3,1e-5)
        -- All six axes reverse under moving IK goals, not only one large step.
        for tick=1,400 do
            if tick%3==0 then rt_sleep(1) end
            local goal={};for i=1,6 do goal[i]=.3*math.sin(tick*.13+i) end
            local ov,oa={},{};for i=1,6 do ov[i]=t.v[i];oa[i]=t.a[i] end
            local p,v,a=t.advance(goal)
            for i=1,6 do
                local j=(a[i]-oa[i])/.08
                assert(math.abs(j)<=.500000000001,'moving_jerk')
                for sub=0,10 do
                    local dt=sub*.008
                    assert(math.abs(ov[i]+oa[i]*dt+j*dt*dt/2)<=.060000000001,'moving_interior_velocity')
                end
            end
        end
        local steps=t.steps;local previous=t.q[1]
        expect_error(function() t.advance({0,0,0,0,0,1.01}) end,'target_region')
        assert(t.steps==steps and t.q[1]==previous,'reference_rejection_changed_state')
        local diagnostic=tracker_module.new({0,0,0,0,0,0})
        expect_error(function() diagnostic.advance({.081,0,0,0,0,0}) end,'target_region')
    end)
    test('latest',function()
        local f=fake();f.publish(0,1000,1,1000);f.publish(1,1001,2,2000);f.now=1001
        local s=f.reader(1000).read_latest();assert(s.sequence_mod==2);near(s.pose.x,.002,1e-9)
    end)
    test('torn_each_field',function()
        for field=1,7 do
            local f=fake();f.publish(0,1000,1);local r=f.reader()
            f.hook=function(ch)
                if ch==240+field then f.hook=nil;f.now=1001;f.publish(0,1001,2,2000) end
            end
            assert(r.read_latest()==nil);assert(r.read_latest().sequence_mod==2)
        end
    end)
    test('other_bank_does_not_starve',function()
        local f=fake();f.publish(0,1000,1);local r=f.reader()
        f.hook=function(ch)
            if ch==241 then f.hook=nil;f.now=1001;f.publish(1,1001,2,2000) end
        end
        assert(r.read_latest().sequence_mod==1);assert(r.read_latest().sequence_mod==2)
    end)
    test('expiry_permanent',function()
        local f=fake();f.publish(0,1000,1);local r=f.reader();r.read_latest();f.now=1099
        expect_error(r.read_latest,'input_expired');f.publish(1,1099,2)
        expect_error(r.read_latest,'retired')
    end)
    test('rollback',function()
        local f=fake();f.publish(0,1000,1);local r=f.reader();f.now=1001;f.publish(1,1001,2)
        r.read_latest();f.mem[248]=0;expect_error(r.read_latest,'target_rollback')
    end)
    test('immutable_duplicate',function()
        local f=fake();f.publish(0,1000,1);local r=f.reader();r.read_latest()
        f.mem[241]=2000;expect_error(r.read_latest,'target_mutated')
    end)
    test('revocation_permanent',function()
        local f=fake();local r=f.reader();f.allowed=false;expect_error(r.read_latest,'revoked')
        f.allowed=true;expect_error(r.read_latest,'retired')
    end)
    test('clock_wrap',function()
        local f=fake();f.now=2097151;f.publish(0,f.now,1023);local r=f.reader()
        assert(r.read_latest().sequence_mod==1023);f.now=2097153;f.publish(1,f.now,1024)
        assert(r.read_latest().sequence_mod==0)
        f.now=f.now+1048576;expect_error(r.read_latest,'clock')
    end)
    test('quaternion_rejected',function()
        expect_error(function() mailbox.decode({0,0,0,0,0,0,0}) end,'quaternion')
    end)
    test('rotation_conventions',function()
        local cases=__ROTATION_CASES__
        for _,case in ipairs(cases) do
            local decoded=mailbox.decode(case.values);local actual=matrix(decoded)
            for i=1,9 do near(actual[i],case.rotation[i],3e-7) end
        end
    end)

    -- Native API timing. This container has no network interface or host ports.
    -- Only signal writes and kinematics calculations: no startup or motion API.
    assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'mailbox_occupied')
    local before=lebai:get_kin_data()
    local origin={0,-math.pi/4,math.pi/2,-math.pi/4,math.pi/2,0}
    local base=lebai:kinematics_forward(origin)
    assert(base and base.x and base.rx,'fk_shape')
    local epoch=timestamp()
    local reader=mailbox.new(function(ch) return lebai:get_signal(ch) end,timestamp,epoch,function() end,
        function(ch,n) return lebai:get_signals(ch,n) end)
    local seed=origin
    for seq=1,20 do
        local p={};for key,value in pairs(base) do p[key]=value end
        p.x=p.x+0.0005*math.sin(seq*.4)
        p.y=p.y+0.0005*math.cos(seq*.4)
        p.z=p.z+0.0005*math.sin(seq*.3)
        p.rx=p.rx+0.005*math.sin(seq*.2);p.rz=p.rz+0.005*math.cos(seq*.2)
        local values=encode(p)
        local stamp=timestamp();local offset=240+((seq-1)%2)*8
        lebai:set_signal(offset,0)
        lebai:set_signals(offset+1,values)
        lebai:set_signal(offset,(stamp%2097152)*1024+seq)
        local written=timestamp()
        local sample=reader.read_latest();assert(sample and sample.sequence_mod==seq,'native_sample_missing')
        local read=timestamp()
        local joints=lebai:kinematics_inverse(sample.pose,seed)
        assert(joints and #joints==6,'ik_unreachable')
        for i=1,6 do assert(joints[i]==joints[i] and math.abs(joints[i]-seed[i])<.05,'ik_branch_jump') end
        local solved=timestamp()
        local returned=lebai:kinematics_forward(joints)
        for _,axis in ipairs({'x','y','z'}) do near(returned[axis],sample.pose[axis],1e-5) end
        local actual,expected=matrix(returned),matrix(sample.pose)
        for i=1,9 do near(actual[i],expected[i],1e-4) end
        print('POSE_PROBE_POINT',seq,written-stamp,read-written,solved-read,timestamp()-stamp)
        seed=joints
    end
    local after=lebai:get_kin_data()
    for i=1,6 do near(after.actual_joint_pose[i],before.actual_joint_pose[i],1e-9) end
    -- Cleanup only on known completion of all writes and no motion submitted.
    lebai:set_signal(240,0);lebai:set_signal(248,0)
    assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'cleanup_unverified')
end)
print('POSE_PROBE_DONE',ok,tostring(err))
program_end(0)
