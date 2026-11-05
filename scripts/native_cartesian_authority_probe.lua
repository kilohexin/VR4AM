program_begin(0)
disable_auto_sync()
local module=(function()
__AUTHORITY__
end)()
local mailbox=(function()
__MAILBOX__
end)()
local binding={words={17,23,31,41},generation=3}
local function checked(name) print('AUTHORITY_CHECK',name);rt_sleep(1) end
local function refuse(fn) assert(not pcall(fn),'expected_refusal') end
local function fixture()
    local words={102400+1024+1,17,23,31,41,3};local time=100;local torn=false
    local function readmany() return {table.unpack(words)} end
    local function readone() return words[1]+(torn and 1 or 0) end
    local lease=module.new(readmany,readone,function() return time end,binding,500)
    return lease,words,function(v) time=v end,function() torn=true end
end
local ok,err=pcall(function()
    local before=lebai:get_kin_data()
    for i=1,6 do assert(math.abs(before.actual_joint_speed[i])<1e-9,'not_quiet') end
    do
        local lease,words,settime=fixture()
        local stamp,status=lease.poll();assert(stamp==100 and status=='fresh')
        settime(599);local v,why=lease.poll();assert(v==nil and why=='duplicate' and lease.last_capture==100)
        settime(600);v,why=lease.poll();assert(v==nil and why=='expired' and lease.closed)
        words[1]=601*1024+2;refuse(lease.poll);checked('duplicates_never_renew')
    end
    do
        local lease,words=fixture();lease.poll();words[1]=0
        local _,why=lease.poll();assert(why=='released' and lease.closed)
        words[1]=102401;refuse(lease.poll);checked('release_permanent')
    end
    do
        local lease,words=fixture();words[1]=0
        local _,why=lease.poll();assert(why=='waiting' and not lease.closed)
        words[1]=-1;_,why=lease.poll();assert(why=='released' and lease.closed)
        refuse(lease.poll);checked('early_bound_withdrawal')
    end
    for index=2,6 do
        local lease,words=fixture();words[1]=-1;words[index]=words[index]+1
        refuse(lease.poll);assert(lease.closed);checked('withdraw_binding_'..index)
    end
    do
        local lease,words=fixture();words[1]=-2
        refuse(lease.poll);assert(lease.closed);checked('invalid_negative_word')
    end
    do
        local lease,words,_,tear=fixture();tear()
        local value,why=lease.poll();assert(value==nil and why=='torn' and lease.last_capture==nil)
        checked('torn_not_committed')
    end
    for index=2,6 do
        local lease,words=fixture();lease.poll();words[index]=words[index]+1
        refuse(lease.poll);assert(lease.closed);checked('binding_'..index)
    end
    do
        local lease,words,settime=fixture();lease.poll();settime(110);words[1]=100*1024+2
        refuse(lease.poll);assert(lease.closed);checked('capture_rollback')
    end
    do
        local lease,words=fixture();words[1]=102*1024+1
        refuse(lease.poll);assert(lease.closed);checked('future_capture')
    end
    do
        local lease,words,settime=fixture();lease.poll();settime(99)
        refuse(lease.poll);assert(lease.closed);checked('clock_rollback')
    end
    do
        local lease,words,settime=fixture();lease.poll();settime(120)
        words[1]=121*1024+2;local stamp=lease.poll();assert(stamp==120)
        settime(619);assert(lease.fresh());settime(620);assert(not lease.fresh() and lease.closed)
        checked('independent_freshness')
    end
    do
        local time=100;local fresh=true
        local words=__PACKET__
        local reader=mailbox.new(function(i,n)
            local result={};for k=1,n do result[k]=(i+k<=120) and words[i+k] or 0 end;return result
        end,function(i) return words[i+1] end,function() return time end,
            {session=string.rep('01',16),model=string.rep('a9',32),generation=1},500,
            function() return fresh end)
        assert(reader.read_latest());time=1000
        assert(reader.read_latest()==nil and not reader.closed and reader.last_capture==100)
        fresh=false;refuse(reader.read_latest);assert(reader.closed)
        checked('math_silence_uses_independent_authority')
    end
    local after=lebai:get_kin_data()
    for i=1,6 do assert(before.actual_joint_pose[i]==after.actual_joint_pose[i]
        and math.abs(after.actual_joint_speed[i])<1e-9,'joint_changed') end
    checked('joints_unchanged')
end)
print('AUTHORITY_DONE',ok,tostring(err))
program_end(0)
