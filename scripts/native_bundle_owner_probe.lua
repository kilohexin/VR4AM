program_begin(0)
disable_auto_sync()
local mailbox=(function()
__MAILBOX__
end)()
local owner_module=(function()
__OWNER__
end)()
local sdk_result=(function()
__SDK_RESULT__
end)()
local words=__PACKETS__
local expected_tail=__TAIL__
local binding={session=string.rep('01',16),model=string.rep('a9',32),generation=1}
local function checked(name) print('OWNER_CHECK',name);rt_sleep(1) end
local function refuse(fn) assert(not pcall(fn),'expected_refusal') end
local function packet(i)
    local value=mailbox.decode(words[i]);rt_sleep(1) -- Stand in for the real mailbox I/O yield.
    return value
end
local function same(a,b,tolerance)
    for _,key in ipairs({'q','v','a'}) do for i=1,6 do
        assert(math.abs(a[key][i]-b[key][i])<=(tolerance or 0),'reference_difference')
    end end
end
-- TEST validator. No actuator is present. A future executor must replace
-- this injection with its actual model/path-certified packet admission.
local function validator(p,tail)
    return #tail==p.braking_ticks and p.braking_ticks>0
end
local function fixture(validate,prefix)
    local time=100
    local owner=owner_module.new(binding,packet(1).start,function() return time end,validate or validator,500,nil,prefix)
    owner.offer_authority(100)
    return owner,function(t) time=t end
end
local function drain(owner,settime,release)
    if release then owner.finish('release') end
    for _=1,70 do
        if owner.phase=='ready' or owner.phase=='holding' then break end
        local time=200+math.max(0,owner.head_tick-1)*80
        settime(time)
        if not release then owner.offer_authority(time) end
        owner.take_next()
    end
end
local ok,err=pcall(function()
    local before=lebai:get_kin_data()
    for i=1,6 do assert(math.abs(before.actual_joint_speed[i])<1e-9,'not_quiet') end
    sdk_result.call_void(function() end);sdk_result.call_void(function() return nil end)
    for _,value in ipairs({false,true,1,'unexpected'}) do
        refuse(function() sdk_result.call_void(function() return value end) end)
    end
    refuse(function() sdk_result.call_void(function() error('unknown_sdk_call') end) end)
    sdk_result.call_native_boolean(function() return true end)
    refuse(function() sdk_result.call_native_boolean(function() end) end)
    refuse(function() sdk_result.call_native_boolean(function() return false end) end)
    checked('sdk_void_contract')
    local native=sdk_result.native_joint(packet(1).start.q)
    local fk=kinematics_forward(native)
    local sdk_fk=lebai:kinematics_forward(packet(1).start.q)
    assert(type(fk)=='table' and math.abs(fk[1]-sdk_fk.x)<1e-9
        and math.abs(fk[2]-sdk_fk.y)<1e-9 and math.abs(fk[3]-sdk_fk.z)<1e-9,'native_joint_fk')
    refuse(function() sdk_result.native_joint({0,0,0}) end)
    refuse(function() sdk_result.native_joint({0,0,0,0,0,math.huge}) end)
    checked('native_joint_format')
    do
        local owner,settime=fixture();assert(owner.stage(packet(1)))
        settime(200);local lead,mode=owner.take_next()
        assert(mode=='following' and #owner.tail==#expected_tail)
        same(owner.tail[1].start,lead.next);checked('full_backup_before_output')
        for i=1,#expected_tail do same(owner.tail[i].next,expected_tail[i],1e-10) end
        checked('precise_tail_reconstruction')
    end
    do
        local owner,settime=fixture();assert(owner.stage(packet(1)));assert(owner.stage(packet(2)))
        local accepted,reason=owner.stage(packet(3))
        assert(not accepted and reason=='queue_full' and #owner.queue==2);checked('at_most_two')
        owner.finish('release');assert(owner.phase=='ready' and #owner.queue==0 and owner.head_tick==0)
        refuse(owner.take_next);checked('release_discards_future')
    end
    do
        local owner,settime=fixture();assert(owner.stage(packet(1)));assert(owner.stage(packet(2)))
        settime(219);owner.take_next() -- First API call begins 19ms after due.
        settime(260);assert(owner.stage(packet(3))) -- RX can stage while the call awaits.
        settime(278);assert(owner.take_next()) -- 59ms call returned; no extra yield.
        settime(338);assert(owner.take_next()) -- The original 80ms lateness guard stays in force.
        local old,oldtime=fixture();assert(old.stage(packet(1)));assert(old.stage(packet(2)))
        oldtime(219);old.take_next();oldtime(298);refuse(old.take_next)
        assert(old.phase=='fault' and old.requires_stop)
        checked('bounded_preload_schedule')
    end
    do
        local owner,settime=fixture();assert(owner.stage(packet(1)));settime(200)
        owner.take_next();owner.take_next() -- One reference, then its already-owned brake.
        local accepted,reason=owner.stage(packet(2))
        assert(not accepted and reason=='stale_parent' and owner.head_tick==2)
        checked('known_late_no_commit')
    end
    do
        local settime
        local owner
        owner,settime=fixture(function(p,tail) settime(201);return true end)
        local accepted,reason=owner.stage(packet(1))
        assert(not accepted and reason=='late_bundle' and #owner.tail==0 and #owner.queue==0 and owner.head_tick==0)
        checked('late_validation_no_commit')
    end
    do
        local owner,settime=fixture();local p=packet(1);assert(owner.stage(p));p.next.q[1]=99
        settime(200);local segment=owner.take_next();assert(segment.next.q[1]~=99)
        checked('caller_mutation_isolated')
        local bad=packet(2);bad.generation=2;refuse(function() owner.stage(bad) end)
        assert(owner.phase=='fault' and owner.requires_stop);refuse(owner.take_next)
        checked('model_binding_fault')
    end
    do
        local owner,settime=fixture();local p=packet(1);p.start.q[1]=p.start.q[1]+.001
        local accepted,reason=owner.stage(p)
        assert(not accepted and reason=='start_mismatch' and #owner.tail==0 and #owner.queue==0)
        checked('bad_start_no_commit')
    end
    do
        local owner,settime=fixture();refuse(function() owner.offer_authority(99) end)
        assert(owner.phase=='fault');checked('input_rollback_fault')
    end
    do
        local owner,settime=fixture(function() error('injected_unknown') end)
        refuse(function() owner.stage(packet(1)) end);assert(owner.phase=='fault' and #owner.tail==0)
        refuse(function() owner.stage(packet(2)) end);checked('unknown_validation_fault')
    end
    do
        local owner
        owner=fixture(function(p,tail) owner.stage(packet(2));return true end)
        refuse(function() owner.stage(packet(1)) end)
        assert(owner.phase=='fault' and owner.reason=='bundle_owner_stage_inflight' and #owner.queue==0)
        checked('concurrent_stage_fault')
    end
    do
        local owner,settime=fixture();assert(owner.stage(packet(1)));settime(200);owner.take_next()
        -- Keep the native cadence, but stop delivering fresh input authority.
        for _=1,8 do
            if owner.released or owner.phase=='holding' then break end
            settime(200+math.max(0,owner.head_tick-1)*80);owner.take_next()
        end
        assert(owner.released and owner.reason=='input_expired')
        refuse(function() owner.offer_authority(1000) end);drain(owner,settime,true)
        assert(owner.phase=='ready');checked('authority_expiry_permanent')
    end
    do
        local owner,settime=fixture();assert(owner.stage(packet(1)));settime(200);owner.take_next()
        drain(owner,settime,false);assert(owner.phase=='holding')
        local saved=owner.describe().head
        local time=200+math.max(0,owner.head_tick-1)*80;settime(time);owner.offer_authority(time)
        assert(owner.take_next()==nil)
        same(owner.describe().head,saved)
        local p=packet(2);p.parent_tick=owner.head_tick;p.due_ms=200+math.max(0,p.parent_tick-1)*80
        p.captured_ms=time;p.start=saved;p.next=saved;p.terminal=saved
        assert(owner.stage(p));settime(p.due_ms);owner.offer_authority(p.due_ms)
        local segment,mode=owner.take_next();assert(mode=='following');same(segment.start,saved)
        assert(owner.epoch==200) -- No new zero, fixed origin or epoch.
        checked('holding_without_recenter')
    end
    for _,release in ipairs({false,true}) do
        local owner,settime=fixture(nil,true);assert(owner.stage(packet(1)));settime(200);owner.take_next()
        drain(owner,settime,false);assert(owner.phase=='holding')
        local tick=owner.head_tick;local saved=owner.describe().head
        local due=200+math.max(0,tick-1)*80;settime(due-20);owner.offer_authority(due-20)
        local p=packet(2);p.parent_tick=tick+1;p.due_ms=due+80;p.captured_ms=due-20
        p.start=saved;p.next=saved;p.terminal=saved;p.braking_ticks=1
        assert(owner.stage(p))
        if release then
            owner.finish('release');assert(owner.phase=='ready' and #owner.queue==0)
            checked('quiet_prefix_release')
        else
            settime(due);assert(owner.take_next()==nil and owner.head_tick==tick+1)
            same(owner.head,saved);assert(#owner.queue==1 and owner.epoch==200)
            settime(due+80);owner.offer_authority(due+80)
            local s,mode=owner.take_next();assert(s and mode=='following');same(s.start,saved)
            checked('quiet_prefix_no_write')
        end
    end
    do
        local owner,settime=fixture(nil,true);assert(owner.stage(packet(1)));settime(200);owner.take_next()
        local p=packet(2);p.parent_tick=owner.head_tick+1;p.start=owner.describe().head
        local accepted,why=owner.stage(p);assert(not accepted and why=='stale_parent')
        checked('moving_prefix_refused')
    end
    do
        local time=100
        local owner=owner_module.new(binding,packet(1).start,function() return time end,
            function() time=201;return true end,500,100)
        owner.offer_authority(100)
        local p=packet(1);p.due_ms=500
        local accepted,why=owner.stage(p)
        assert(not accepted and why=='stale_input' and #owner.queue==0 and #owner.tail==0)
        checked('reference_age_rechecked')
    end
    local after=lebai:get_kin_data()
    for i=1,6 do assert(math.abs(after.actual_joint_pose[i]-before.actual_joint_pose[i])<1e-9
        and math.abs(after.actual_joint_speed[i])<1e-9,'joint_changed') end
    checked('joints_unchanged')
end)
print('OWNER_PROBE_DONE',ok,tostring(err))
program_end(0)
