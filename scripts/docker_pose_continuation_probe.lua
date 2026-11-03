program_begin(0)
disable_auto_sync()
local function quiet(deadline)
    local since=nil;local previous=timestamp()
    while timestamp()<deadline do
        local state=lebai:get_robot_state();local k=lebai:get_kin_data();local now=timestamp()
        assert(state=='IDLE' or state=='MOVING','prepare_state')
        local stopped=state=='IDLE'
        for i=1,6 do
            assert(k.actual_joint_pose[i]==k.actual_joint_pose[i] and math.abs(k.actual_joint_speed[i])<=.6,'prepare_feedback')
            stopped=stopped and math.abs(k.actual_joint_speed[i])<=.002
        end
        if now<previous or now-previous>=100 then since=nil end
        previous=now
        if stopped then
            since=since or now
            if now-since>=200 then return k end
        else since=nil end
        rt_sleep(20)
    end
    error('prepare_quiet_unverified')
end
local entered=false
local prepared,reason=pcall(function()
    assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'probe_mailbox_occupied')
    local state=lebai:get_robot_state()
    if state=='ROBOT_ON' or state=='STOP' then
        lebai:start_sys();local deadline=timestamp()+8000
        repeat
            state=lebai:get_robot_state()
            if state=='IDLE' then break end
            assert(state=='ROBOT_ON' or state=='STARTING' or state=='BOOTING' or state=='STOP','prepare_start')
            rt_sleep(100)
        until timestamp()>=deadline
    end
    assert(state=='IDLE','prepare_requires_idle')
    local ready={0,-math.pi/4,math.pi/2,-math.pi/4,math.pi/2,0}
    entered=true;lebai:movej(ready,.2,.3,0,0)
    local k=quiet(timestamp()+20000)
    for i=1,6 do assert(math.abs(k.actual_joint_pose[i]-ready[i])<.002,'prepare_endpoint') end
end)
if not prepared then
    if entered then
        local returned=pcall(function() lebai:stop_move() end)
        local observed=pcall(function() quiet(timestamp()+2500) end)
        print('CONT_PREP_ABORT',returned,observed)
    end
    print('CONT_DONE',false,tostring(reason),0)
else
    local good,err,moves=(function()
__PRODUCT_BODY__
    end)()
    local cleared=false
    if good then
        cleared=pcall(function()
            lebai:set_signal(240,0);lebai:set_signal(248,0)
            assert(lebai:get_signal(240)==0 and lebai:get_signal(248)==0,'cleanup_unverified')
        end)
    end
    print('CONT_DONE',good and cleared,tostring(err),moves)
end
program_end(0)
