program_begin(0)
disable_auto_sync()
__BOOT_CLOCK__
-- Deliberately ZERO actuator APIs. Tests live host planning and transport.
local mailbox=(function()
__MAILBOX__
end)()
local authority_module=(function()
__AUTHORITY__
end)()
local owner_module=(function()
__OWNER__
end)()
local origin=__ORIGIN__
local binding=__BINDING__
-- LIVE_EXTRA_HELPERS
local epoch=cartesian_now();local clock=function() return cartesian_now()-epoch end
local function report(tag,...)
    print(tag,...)
end
local function state_words(s)
    local values={}
    for _,key in ipairs({'q','v','a'}) do for i=1,6 do values[#values+1]=string.format('%.17g',s[key][i]) end end
    return table.concat(values,'\t')
end
local initial=lebai:get_kin_data()
-- LIVE_EXTRA_INITIAL
local ok,err=pcall(function()
    for i=1,6 do assert(initial.actual_joint_pose[i]==origin.q[i]
        and math.abs(initial.actual_joint_speed[i])<1e-9,'dry_origin') end
    -- LIVE_EXTRA_PRECHECK
    for _,i in ipairs({0,120,240,254}) do assert(get_signal(i)==0,'dry_occupied') end
    assert(set_signals(246,{0,0,0,0,0,0,0,0,0,0}),'dry_metadata_initialize')
    epoch=cartesian_now();report('CART_READY',epoch)
    local function certified(packet,tail)
        -- The private host publisher admits ONLY a fully checked pending
        -- transaction. This callback is a structural guard, not an independent
        -- Cartesian certificate. This executable cannot actuate in any case.
        for _,s in ipairs({packet.start,packet.next,packet.terminal}) do
            for i=1,6 do assert(math.abs(s.q[i])<=3 and math.abs(s.v[i])<=.2
                and math.abs(s.a[i])<=.2,'dry_reference_bounds') end
        end
        return #tail==packet.braking_ticks
    end
    local owner=owner_module.new(binding,origin,clock,certified,500,100,true,true)
    local lease=authority_module.new(get_signals,get_signal,clock,binding,500)
    local reader=mailbox.new(get_signals,get_signal,clock,binding,500,lease.fresh,true)
    local started=false;local finish=false;local failure=nil;local emitted=0;local profiles={}
    local ack_sequence=0;local ack_status=0;local status_sequence=0;local status_busy=false;local status_dirty=false
    local status_requested=false
    local function request_status() status_requested=true end
    local phase_code={waiting=0,armed=1,active=2,holding=3,finishing=4,ready=5,fault=6}
    local function publish_status()
        if status_busy then status_dirty=true;return end
        status_busy=true
        repeat
            status_dirty=false
            local payload={owner.head_tick,phase_code[owner.phase],ack_sequence,ack_status}
            local sequence=owner.emitted_sequence or 0
            status_sequence=status_sequence+2
            assert(set_signal(253,status_sequence-1),'dry_status_write')
            assert(set_signals(246,payload),'dry_status_write')
            assert(set_signal(255,sequence),'dry_status_write')
            assert(set_signal(253,status_sequence),'dry_status_write')
        until not status_dirty
        status_busy=false
    end
    local function worker(fn)
        return async_task(function()
            local good,reason=pcall(fn)
            if not good then
                failure=failure or reason;finish=true
                -- Independent fault signal; never wait for stdout logging to
                -- revoke further host publications. Preserve it on failure.
                pcall(set_signal,254,1)
            end
        end)
    end
    local receiver=worker(function()
        while not finish do
            if not started and get_signal(240)~=0 then started=true end
            if started and not owner.released then
                local stamp,status=lease.poll()
                if stamp then owner.offer_authority(stamp)
                elseif status=='released' or status=='expired' then
                    owner.finish(status);report('CART_RELEASE',status,clock())
                end
                if not owner.released then
                    local began=clock()
                    local packet=reader.read_latest()
                    if packet and not finish and not owner.released then
                        local decoded=clock()
                        local accepted,why=owner.stage(packet)
                        local staged=clock()
                        ack_sequence=packet.sequence;ack_status=accepted and 1 or 2;request_status()
                        assert(#profiles<128,'dry_profile_budget')
                        profiles[#profiles+1]={packet.sequence,decoded-began,staged-decoded,clock()-staged}
                        report('CART_ACK',packet.sequence,accepted,tostring(why),clock())
                    end
                end
            end
            rt_sleep(2)
        end
    end)
    local sender=worker(function()
        local previous_tick=-1
        while not finish and owner.phase~='ready' do
            local emitted_before=emitted
            assert(clock()<15000,'dry_runtime')
            if started then
                local segment,mode=owner.take_next()
                -- LIVE_EXTRA_EMIT
                if segment then emitted=emitted+1;report('CART_SEGMENT',owner.head_tick,mode) end
                if owner.head_tick~=previous_tick then
                    previous_tick=owner.head_tick
                    request_status()
                    report('CART_HEAD',clock(),owner.head_tick,owner.phase,state_words(owner.head))
                end
            end
            if emitted==emitted_before then rt_sleep(1) end
        end
        finish=true
    end)
    local status_worker=worker(function()
        while not finish or status_requested do
            if status_requested then status_requested=false;publish_status() end
            rt_sleep(1)
        end
    end)
    local clock_service=worker(function()
        local nonce=0
        while not finish do
            local requested=get_signal(250)
            if requested~=nonce then
                nonce=requested;assert(set_signals(251,{clock(),nonce}),'dry_clock_write')
            end
            rt_sleep(1)
        end
    end)
    -- LIVE_EXTRA_WORKERS
    wait_all(receiver,sender,clock_service,status_worker)
    for _,row in ipairs(profiles) do report('CART_PROFILE',table.unpack(row)) end
    assert(not failure,tostring(failure))
    -- LIVE_EXTRA_QUIET
    local after=lebai:get_kin_data()
    for i=1,6 do assert(after.actual_joint_pose[i]==initial.actual_joint_pose[i]
        and math.abs(after.actual_joint_speed[i])<1e-9,'dry_joint_changed') end
    lebai:set_signal(0,0);lebai:set_signal(120,0);lebai:set_signal(240,0)
    report('CART_QUIET',emitted,owner.phase,owner.reason)
end)
-- LIVE_EXTRA_ABORT
report('CART_DONE',ok,tostring(err))
-- LIVE_EXTRA_LOG_END
program_end(0)
