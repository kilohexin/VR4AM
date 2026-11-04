-- Full-pose mailbox reader candidate. Injected I/O, no motion API calls.
-- Wire format matches app.control.vr_pose_mailbox. Not a product selector.
local M = {}
local BASE, SIZE, MOD, SEQ, TTL = 240, 8, 2097152, 1024, 100

local function integer(x)
    return type(x)=='number' and x==x and x==math.floor(x) and x>=-2147483648 and x<2147483648
end

function M.decode(values)
    assert(type(values)=='table' and #values==7, 'pose_mailbox_payload')
    for i=1,7 do assert(integer(values[i]), 'pose_mailbox_payload') end
    local x,y,z,w = values[4]/1e7, values[5]/1e7, values[6]/1e7, values[7]/1e7
    local norm = math.sqrt(x*x+y*y+z*z+w*w)
    assert(math.abs(norm-1)<=1e-5, 'pose_mailbox_quaternion')
    x,y,z,w = x/norm,y/norm,z/norm,w/norm
    local ry = math.asin(math.max(-1, math.min(1, 2*(w*y-z*x))))
    local rx,rz
    if math.abs(math.cos(ry))<1e-8 then
        rx=0
        rz=math.atan2(2*(w*z-x*y), 1-2*(x*x+z*z))
    else
        rx=math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        rz=math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    end
    return {x=values[1]/1e6,y=values[2]/1e6,z=values[3]/1e6,rx=rx,ry=ry,rz=rz}, {x,y,z,w}
end

function M.new(io, clock, not_before, authorize, read_many)
    assert(type(not_before)=='number' and not_before>=0 and not_before==math.floor(not_before), 'pose_mailbox_epoch')
    local self={closed=false,busy=false,last_now=not_before,last_capture=nil,last_word=nil,last_values=nil,
        max_read_ms=0,max_age_ms=0,last_headers={0,0}}
    local function check()
        assert(not self.closed, 'pose_mailbox_reader_retired')
        authorize()
        local now=clock()
        local valid=type(now)=='number' and now==math.floor(now) and now>=self.last_now
            and now-not_before<MOD/2
        if not valid then
            local reason=type(now)~='number' and 'type' or now~=math.floor(now) and 'format'
                or now<self.last_now and 'rollback' or 'epoch_span'
            print('POSE_MAILBOX_CLOCK',reason,now,self.last_now,not_before)
        end
        assert(valid, 'pose_mailbox_clock')
        self.last_now=now
        return now
    end
    local function no_fresh_sample()
        local now=check()
        if self.last_capture and now-self.last_capture+1>=TTL then
            print('POSE_MAILBOX_EXPIRED',now,self.last_capture,self.last_headers[1],self.last_headers[2],self.max_read_ms,self.max_age_ms)
            error('pose_mailbox_input_expired')
        end
        return nil
    end
    local function read(channel)
        check()
        local value=io(channel)
        check()
        assert(integer(value), 'pose_mailbox_signal')
        return value
    end
    local function latest()
        self.read_started=check()
        local raw,heads
        if read_many then
            -- Only the two headers are used from this first batch. Payload
            -- must be read AFTER it returns, then its chosen header rechecked.
            -- Never assume that fields inside a batch are read in order.
            check();raw=read_many(BASE,SIZE+1);check()
            assert(type(raw)=='table' and #raw==SIZE+1,'pose_mailbox_payload')
            assert(integer(raw[1]) and integer(raw[SIZE+1]),'pose_mailbox_signal')
            heads={raw[1],raw[SIZE+1]}
        else heads={read(BASE),read(BASE+SIZE)} end
        self.last_headers=heads
        local now=check()
        local selected,capture=nil,nil
        for bank=1,2 do
            local word=heads[bank]
            assert(word>=0, 'pose_mailbox_header')
            local age=(now-math.floor(word/SEQ))%MOD
            local stamp=now-age
            if word~=0 and age+1<TTL and stamp>=not_before then
                assert(capture~=stamp, 'pose_mailbox_ambiguous_stamp')
                if not capture or stamp>capture then selected=bank;capture=stamp end
            end
        end
        if not selected then return no_fresh_sample() end
        assert(not self.last_capture or capture>=self.last_capture, 'pose_mailbox_target_rollback')
        local offset=BASE+(selected-1)*SIZE
        local word,values=heads[selected],{}
        if read_many then
            check();values=read_many(offset+1,7);check()
            assert(type(values)=='table' and #values==7,'pose_mailbox_payload')
            for i=1,7 do assert(integer(values[i]),'pose_mailbox_payload') end
        else
            for i=1,7 do values[i]=read(offset+i) end
        end
        if read(offset)~=word then return no_fresh_sample() end
        now=check()
        self.max_age_ms=math.max(self.max_age_ms,now-capture+1)
        assert(now-capture+1<TTL, 'pose_mailbox_input_expired')
        if capture==self.last_capture then
            assert(word==self.last_word, 'pose_mailbox_target_mutated')
            for i=1,7 do assert(values[i]==self.last_values[i], 'pose_mailbox_target_mutated') end
        end
        local pose,q=M.decode(values)
        self.last_capture,self.last_word,self.last_values=capture,word,values
        return {pose=pose,q=q,captured_ms=capture,sequence_mod=word%SEQ}
    end
    function self.read_latest()
        assert(not self.busy, 'pose_mailbox_read_inflight')
        self.busy=true
        self.read_started=nil
        local ok,result=pcall(latest)
        if self.read_started then
            self.max_read_ms=math.max(self.max_read_ms,self.last_now-self.read_started)
        end
        self.busy=false
        if not ok then self.closed=true;error(result) end
        return result
    end
    return self
end

return M
