program_begin(0)
disable_auto_sync()
local mail=(function()
__MODULE__
end)()
local fixture=__FIXTURE__
local latest=__LATEST__
local mutated=__MUTATED__
local invalid=__INVALID__
local native_a=__NATIVE_A__
local native_b=__NATIVE_B__
local binding={session=string.rep('01',16),model=string.rep('a9',32),generation=1}
local function copy(x) local y={};for i=1,#x do y[i]=x[i] end;return y end
local function checked(name) print('BUNDLE_CHECK',name) end
local function refuse(fn) assert(not pcall(fn),'expected_rejection') end
local function memory(a,b)
    local mem={};for i=0,239 do mem[i]=0 end
    local function put(offset,words) for i=1,120 do mem[offset+i-1]=words[i] end end
    if a then put(0,a) end;if b then put(120,b) end
    local function many(index,n) local values={};for i=1,n do values[i]=mem[index+i-1] end;return values end
    local function one(index) return mem[index] end
    return mem,put,many,one
end
local ok,err=pcall(function()
    assert(_VERSION=='Lua 5.4','bundle_lua_version')
    local before=lebai:get_kin_data()
    for i=1,6 do assert(math.abs(before.actual_joint_speed[i])<1e-9,'not_quiet') end
    local p=mail.decode(fixture)
    assert(p.sequence==23 and p.session==binding.session and p.model==binding.model and p.generation==1)
    assert(p.start.q[2]==-.8 and p.next.v[1]==.001 and p.terminal.v[1]==0 and p.braking_ticks==10)
    assert(string.pack('<d',p.start.q[6])==string.pack('<d',-0.0),'signed_zero')
    checked('lossless')
    for i=1,120 do
        local bad=copy(fixture);bad[i]=bad[i]~1;refuse(function() mail.decode(bad) end)
        if i%4==0 then rt_sleep(1) end -- Synthetic tight loop must yield to the native interpreter guard.
    end
    checked('every_word_crc')
    for _,bad in ipairs(invalid) do refuse(function() mail.decode(bad) end) end
    checked('invalid_float_and_reserved')
    do
        local mem,put,many,one=memory(fixture,latest);local time=370
        local reader=mail.new(many,one,function() return time end,binding,100)
        assert(reader.read_latest().sequence==24);checked('latest')
    end
    for field=1,120 do
        local mem,put,many,one=memory(fixture,nil);local time=350
        local function torn(index,n)
            local values=many(index,n)
            if n==120 then values[field]=values[field]~1;mem[0]=0 end
            return values
        end
        local reader=mail.new(torn,one,function() return time end,binding,100)
        assert(reader.read_latest()==nil and not reader.closed and reader.last_sequence==0)
        if field%4==0 then rt_sleep(1) end
    end
    checked('torn_each_word')
    do
        local mem,put,many,one=memory(fixture,nil);local time=350
        local reader=mail.new(many,one,function() return time end,binding,100)
        assert(reader.read_latest());time=439;assert(reader.read_latest()==nil)
        time=440;refuse(reader.read_latest);assert(reader.closed);checked('duplicate_expiry')
        put(120,latest);refuse(reader.read_latest);checked('expiry_permanent')
    end
    do
        local mem,put,many,one=memory(fixture,nil)
        local reader=mail.new(many,one,function() return 350 end,binding,100)
        assert(reader.read_latest());put(0,mutated)
        refuse(reader.read_latest);assert(reader.closed);checked('immutable_duplicate')
    end
    do
        local mem,put,many,one=memory(fixture,nil)
        local other={session=binding.session,model=binding.model,generation=2}
        local reader=mail.new(many,one,function() return 350 end,other,100)
        refuse(reader.read_latest);assert(reader.closed);checked('binding')
    end
    do
        local mem,put,many,one=memory(fixture,nil);local time=350
        local reader=mail.new(many,one,function() return time end,binding,100)
        assert(reader.read_latest());time=349;refuse(reader.read_latest);checked('rollback')
    end
    -- The only native writes in this probe are bounded signal-bank payloads.
    assert(lebai:get_signal(0)==0 and lebai:get_signal(120)==0,'banks_occupied')
    local epoch=timestamp()
    local reader=mail.new(function(i,n) return lebai:get_signals(i,n) end,
        function(i) return lebai:get_signal(i) end,function() return timestamp()-epoch end,binding,500)
    for bank,words in ipairs({native_a,native_b}) do
        local offset=(bank-1)*120
        lebai:set_signal(offset,0)
        local payload={};for i=2,120 do payload[#payload+1]=words[i] end
        lebai:set_signals(offset+1,payload);lebai:set_signal(offset,words[1])
        local started=timestamp();local decoded=reader.read_latest();local elapsed=timestamp()-started
        assert(decoded and decoded.sequence==words[1],'native_decode')
        print('BUNDLE_NATIVE_READ',decoded.sequence,elapsed)
    end
    checked('native_banks')
    lebai:set_signal(0,0);lebai:set_signal(120,0)
    local after=lebai:get_kin_data()
    for i=1,6 do
        assert(math.abs(after.actual_joint_speed[i])<1e-9
            and math.abs(after.actual_joint_pose[i]-before.actual_joint_pose[i])<1e-9,'joint_changed')
    end
    checked('joint_unchanged')
end)
print('BUNDLE_MAILBOX_DONE',ok,tostring(err))
program_end(0)
