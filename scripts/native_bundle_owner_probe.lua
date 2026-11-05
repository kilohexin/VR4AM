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
local curve_witness=(function()
__CURVE_WITNESS__
end)()
local local_model=(function()
__LUA_MDH__
end)()
local words=__PACKETS__
local prefix_words=__PREFIX_PACKET__
local expected_tail=__TAIL__
local expected_tails=__ALL_TAILS__
local binding={session=string.rep('01',16),model=string.rep('a9',32),generation=1}
local tool_cases=__TOOL_CASES__
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
local function fixture(validate,prefix,braking_prefix)
    local time=100
    local owner=owner_module.new(binding,packet(1).start,function() return time end,validate or validator,500,nil,prefix,braking_prefix)
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
    do
        local bound=local_model.with_tool(tool_cases.p,tool_cases.q)
        local function compare()
            for _,case in ipairs(tool_cases.poses) do
                local actual=bound.pose(case.joints)
                for i=1,3 do assert(math.abs(actual.p[i]-case.p[i])<1e-12,'bound_tool_position') end
                for i=1,9 do assert(math.abs(actual.r[i]-case.r[i])<1e-12,'bound_tool_rotation') end
            end
        end
        compare();checked('lua_bound_tool_matches_python')
        tool_cases.p[1]=99;tool_cases.q[1]=99
        local poisoned=bound.pose(tool_cases.poses[1].joints);poisoned.p[1]=99;poisoned.r[1]=99
        compare();checked('lua_bound_tool_copies_geometry')
        for _,bad in ipairs({{0,0,0,0},{0,0,0,2},{0,0,0,0/0}}) do
            refuse(function() local_model.with_tool({0,0,.1},bad) end)
        end
        refuse(function() local_model.with_tool({0,0,1},{0,0,0,1}) end)
        refuse(function() local_model.with_tool({0,0},{0,0,0,1}) end)
        checked('lua_bound_tool_invalid_rejected')
    end
    do
        local curve={identity=7,coefficients={}}
        for i=1,6 do curve[i]={-2,2};curve.coefficients[i]={0,i==1 and 1 or 0,0,0,0,0} end
        local function line(_,t,d)
            return {q={t/d,0,0,0,0,0},v={1/d,0,0,0,0,0},a={0,0,0,0,0,0}}
        end
        local found=curve_witness.closest({curve},{.371923,0,0,0,0,0},line)
        assert(found.identity==7 and math.abs(found.u-.371923)<1e-12 and found.cost<1e-20)
        checked('curve_witness_exact_point')
        found=curve_witness.closest({curve},{.371923,.001,0,0,0,0},line)
        assert(found.cost>=1e-6-1e-15 and math.abs(found.state.q[2])<1e-15)
        checked('curve_witness_keeps_deviation')
        local function constant() return {q={0,0,0,0,0,0},v={0,0,0,0,0,0},a={0,0,0,0,0,0}} end
        for i=1,6 do curve.coefficients[i]={0,0,0,0,0,0} end
        found=curve_witness.closest({curve},{0,0,0,0,0,0},constant)
        assert(found.cost==0);checked('curve_witness_constant')
    end
    do
        local p=packet(1);local pool={}
        for index=1,8 do
            local curve={identity=index,coefficients=owner_module.coefficients(p.start,p.next,.08)}
            for i=1,6 do curve[i]={-3,3} end;pool[index]=curve
        end
        local target=owner_module.evaluate(pool[1].coefficients,.371923*.08,.08).q
        local before=timestamp()
        local found=curve_witness.closest(pool,target,owner_module.evaluate)
        print('CURVE_WITNESS_WORK_MS',timestamp()-before)
        assert(found.cost<1e-12 and found.evaluated_curves<=3);checked('curve_witness_eight_curves')
    end
    do
        local function rotation(p)
            local sx,cx=math.sin(p.rx),math.cos(p.rx);local sy,cy=math.sin(p.ry),math.cos(p.ry)
            local sz,cz=math.sin(p.rz),math.cos(p.rz)
            return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
        end
        for _,q in ipairs({packet(1).start.q,{.2,-.8,1.4,-.6,1.2,.3},{-.3,-.4,.9,.2,-1,.4}}) do
            local computed=local_model.pose(q);local vendor=lebai:kinematics_forward(q);local r=rotation(vendor)
            for i=1,9 do assert(math.abs(computed.r[i]-r[i])<1e-9,'lua_mdh_rotation') end
            assert(math.abs(computed.p[1]-vendor.x)<1e-9 and math.abs(computed.p[2]-vendor.y)<1e-9
                and math.abs(computed.p[3]-vendor.z)<1e-9,'lua_mdh_position')
        end
        checked('lua_mdh_matches_sdk')
    end
    do
        for index=1,#words do
            local p=packet(index);local duration=p.braking_ticks*.08
            local coefficients=owner_module.coefficients(p.next,p.terminal,duration)
            for tick=1,p.braking_ticks-1 do
                local state=owner_module.evaluate(coefficients,tick*.08,duration)
                for _,field in ipairs({'q','v','a'}) do for i=1,6 do
                    assert(string.pack('<d',state[field][i])==string.pack('<d',expected_tails[index][tick][field][i]),
                        'binary_tail_difference')
                end end
                rt_sleep(1)
            end
        end
        checked('binary_tail_identity')
    end
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
        local p=mailbox.decode(prefix_words)
        local old,oldtime=fixture();assert(old.stage(packet(1)));oldtime(200);old.take_next()
        local accepted,why=old.stage(p);assert(not accepted and why=='stale_parent')
        checked('braking_prefix_opt_in')
        local owner,settime=fixture(nil,nil,true);assert(owner.stage(packet(1)));settime(200)
        owner.take_next();assert(owner.stage(p))
        local previous=owner.tail[1];local brake,mode=owner.take_next()
        assert(mode=='braking' and #owner.queue==1);same(brake.next,previous.next)
        settime(280);local lead,kind=owner.take_next()
        assert(kind=='following' and #owner.queue==0 and #owner.tail==p.braking_ticks)
        same(lead.start,brake.next);same(lead.next,p.next)
        checked('braking_prefix_owned_then_follow')
    end
    do
        local p=mailbox.decode(prefix_words)
        local owner,settime=fixture(nil,nil,true);assert(owner.stage(packet(1)));settime(200);owner.take_next()
        local terminal=owner.tail[#owner.tail].next;assert(owner.stage(p));owner.finish('release')
        assert(#owner.queue==0);drain(owner,settime,true);same(owner.head,terminal)
        assert(owner.phase=='ready');checked('braking_prefix_release_old_tail')
    end
    do
        local p=mailbox.decode(prefix_words);p.start.q[1]=p.start.q[1]+1e-12
        local owner,settime=fixture(nil,nil,true);assert(owner.stage(packet(1)));settime(200);owner.take_next()
        local accepted,why=owner.stage(p)
        assert(not accepted and why=='start_mismatch' and #owner.queue==0)
        checked('braking_prefix_bad_start')
    end
    do
        local owner,settime;local consume=false
        owner,settime=fixture(function()
            if consume then owner.take_next() end
            return true
        end,nil,true)
        assert(owner.stage(packet(1)));settime(200);owner.take_next();consume=true
        assert(owner.stage(mailbox.decode(prefix_words)) and owner.head_tick==2)
        settime(280);local _,mode=owner.take_next();assert(mode=='following')
        checked('braking_prefix_yield_recheck')
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
