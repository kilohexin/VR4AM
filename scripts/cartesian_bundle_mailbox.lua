-- Lossless bundle decoder/reader only. No actuator or endpoint APIs.
-- Lua 5.4; two 120-word banks, CRC32 over header+118 payload words.
local M={}
local SIZE=120
local function integer(v,low,high)
    return type(v)=='number' and v==v and v==math.floor(v) and v>=low and v<=high
end
local crc_table={}
for i=0,255 do
    local c=i
    for _=1,8 do
        if c&1~=0 then c=(c>>1)~0xedb88320 else c=c>>1 end
    end
    crc_table[i+1]=c
end
local function crc32(words)
    local crc=0xffffffff
    for i=1,SIZE-1 do
        local word=words[i]&0xffffffff
        for shift=0,24,8 do
            local byte=(word>>shift)&255
            crc=(crc>>8)~crc_table[((crc~byte)&255)+1]
        end
    end
    crc=(crc~0xffffffff)&0xffffffff
    return crc>=0x80000000 and crc-0x100000000 or crc
end
local function hex(words,first,last)
    local result={}
    for i=first,last do
        local raw=string.pack('<i4',words[i])
        for j=1,4 do result[#result+1]=string.format('%02x',string.byte(raw,j)) end
    end
    return table.concat(result)
end
function M.decode(words)
    assert(type(words)=='table' and #words==SIZE,'bundle_wire_shape')
    for i=1,SIZE do assert(integer(words[i],-2147483648,2147483647),'bundle_wire_integer') end
    assert(words[1]>0 and words[2]==0x43504631 and crc32(words)==words[SIZE],'bundle_wire_integrity')
    assert(words[3]>0 and words[16]>=0 and words[17]>=0 and words[18]>=0
        and words[18]<=words[17] and words[19]>=1 and words[19]<=60,'bundle_wire_structure')
    for i=104,SIZE-1 do assert(words[i]==0,'bundle_wire_reserved') end
    local values={}
    for i=20,103,2 do
        local v=string.unpack('<d',string.pack('<i4i4',words[i],words[i+1]))
        assert(v==v and math.abs(v)<=1000000 and (v==0 or math.abs(v)>=2^-1022),'bundle_wire_float')
        values[#values+1]=v
    end
    local function state(first,terminal)
        local q,v,a={},{},{}
        for i=1,6 do
            q[i]=values[first+i-1]
            v[i]=terminal and 0 or values[first+i+5]
            a[i]=terminal and 0 or values[first+i+11]
        end
        return {q=q,v=v,a=a}
    end
    return {sequence=words[1],generation=words[3],session=hex(words,4,7),model=hex(words,8,15),
        parent_tick=words[16],due_ms=words[17],captured_ms=words[18],braking_ticks=words[19],
        start=state(1,false),next=state(19,false),terminal=state(37,true)}
end

function M.new(read_many,read_one,clock,binding,ttl,input_fresh,deliver_late)
    assert(type(ttl)=='number' and ttl==math.floor(ttl) and ttl>=1 and ttl<=500,'bundle_reader_ttl')
    assert(input_fresh==nil or type(input_fresh)=='function','bundle_reader_authority')
    assert(deliver_late==nil or type(deliver_late)=='boolean','bundle_reader_authority')
    assert(not deliver_late or input_fresh,'bundle_reader_authority')
    local self={closed=false,busy=false,last_now=clock(),last_sequence=0,last_capture=nil,last_words=nil}
    local function now()
        local n=clock()
        assert(integer(n,0,2147483647) and n>=self.last_now,'bundle_reader_clock')
        self.last_now=n
        return n
    end
    local function empty()
        if self.last_capture and now()-self.last_capture>=ttl then
            -- Only independently polled genuine input may keep a quiet math
            -- channel open. Duplicate payloads never renew their own age.
            assert(input_fresh and input_fresh()==true,'bundle_reader_expired')
        end
        return nil
    end
    local function latest()
        now()
        local headers=read_many(0,SIZE+1);now()
        assert(type(headers)=='table' and #headers==SIZE+1,'bundle_reader_headers')
        local a,b=headers[1],headers[SIZE+1]
        assert(integer(a,0,2147483647) and integer(b,0,2147483647),'bundle_reader_headers')
        assert(a==0 or b==0 or a~=b,'bundle_reader_ambiguous')
        if a==0 and b==0 then return empty() end
        local offset=(a>b) and 0 or SIZE;local seq=math.max(a,b)
        assert(seq>=self.last_sequence,'bundle_reader_rollback')
        local words=read_many(offset,SIZE);now()
        local after=read_one(offset);now()
        assert(type(words)=='table' and #words==SIZE and integer(after,0,2147483647),'bundle_reader_payload')
        if after~=seq or words[1]~=seq then return empty() end -- Torn inactive bank, no installation.
        local packet=M.decode(words)
        assert(packet.session==binding.session and packet.model==binding.model
            and packet.generation==binding.generation,'bundle_reader_binding')
        if seq==self.last_sequence then
            for i=1,SIZE do assert(words[i]==self.last_words[i],'bundle_reader_mutated') end
            return empty() -- Duplicate never refreshes input age.
        end
        local time=now()
        assert(not self.last_capture or packet.captured_ms>=self.last_capture,'bundle_reader_capture_rollback')
        assert(packet.captured_ms<=time and time-packet.captured_ms<ttl,'bundle_reader_expired')
        -- An explicitly live caller may pass a known-late, intact packet to
        -- the owner for a ZERO-install rejection. It cannot execute that lead.
        -- All other reader contracts remain fail closed, including binding,
        -- input age, CRC, mutation and clock rollback.
        assert(packet.due_ms>time or deliver_late and input_fresh()==true,'bundle_reader_late')
        local copy={};for i=1,SIZE do copy[i]=words[i] end
        self.last_sequence=seq;self.last_capture=packet.captured_ms;self.last_words=copy
        return packet
    end
    function self.read_latest()
        assert(not self.closed and not self.busy,'bundle_reader_retired')
        self.busy=true
        local ok,result=pcall(latest)
        self.busy=false
        if not ok then self.closed=true;error(result) end
        return result
    end
    return self
end
return M
