program_begin(0)
disable_auto_sync()
local mailbox=(function()
__POSE_MAILBOX_MODULE__
end)()
local tracker_module=(function()
__POSE_TRACKER_MODULE__
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
    function f.reader(epoch)
        return mailbox.new(f.read,function() return f.now end,epoch or f.now,
            function() assert(f.allowed,'revoked') end)
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
    test('empty',function() local f=fake();assert(f.reader().read_latest()==nil) end)
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
