-- Reference/backup ownership state machine only; no I/O or motion APIs.
-- The caller MUST certify Cartesian paths and provide validate(packet,tail).
-- Two queued leads at most. Backup is owned before a lead can be emitted.
local M={}
local function clone(x)
    if type(x)~='table' then return x end
    local y={};for k,v in pairs(x) do y[k]=clone(v) end;return y
end
local function equal(a,b)
    for _,field in ipairs({'q','v','a'}) do
        if type(a[field])~='table' or type(b[field])~='table' then return false end
        for i=1,6 do if a[field][i]~=b[field][i] then return false end end
    end
    return true
end
local function quiet(s)
    for i=1,6 do if s.v[i]~=0 or s.a[i]~=0 then return false end end
    return true
end
function M.coefficients(first,last,duration)
    local c={}
    for axis=1,6 do
        local p=first.q[axis];local v=first.v[axis]*duration;local a=first.a[axis]*duration*duration/2
        local b0=last.q[axis]-p-v-a
        local b1=last.v[axis]*duration-v-2*a;local b2=last.a[axis]*duration*duration-2*a
        c[axis]={p,v,a,10*b0-4*b1+b2/2,-15*b0+7*b1-b2,6*b0-3*b1+b2/2}
    end
    return c
end
function M.evaluate(c,time,duration)
    local u=time/duration;local s={q={},v={},a={}}
    for axis=1,6 do
        local row=c[axis];local p=row[6];local v=5*row[6];local a=20*row[6]
        for degree=4,0,-1 do p=p*u+row[degree+1] end
        for degree=4,1,-1 do v=v*u+degree*row[degree+1] end
        for degree=4,2,-1 do a=a*u+degree*(degree-1)*row[degree+1] end
        s.q[axis]=p;s.v[axis]=v/duration;s.a[axis]=a/duration/duration
    end
    return s
end
local function expand(packet)
    local duration=packet.braking_ticks*.08
    local c=M.coefficients(packet.next,packet.terminal,duration)
    local tail={};local previous=clone(packet.next)
    for i=1,packet.braking_ticks do
        local next=i==packet.braking_ticks and clone(packet.terminal) or M.evaluate(c,i*.08,duration)
        tail[i]={start=previous,next=next};previous=next
    end
    return tail
end

function M.new(binding,origin,clock,validate,ttl,reference_ttl,allow_quiet_prefix,allow_braking_prefix)
    assert(type(validate)=='function' and quiet(origin),'bundle_owner_configuration')
    assert(type(ttl)=='number' and ttl>=1 and ttl<=500 and ttl==math.floor(ttl),'bundle_owner_configuration')
    reference_ttl=reference_ttl or ttl
    assert(type(reference_ttl)=='number' and reference_ttl>=1 and reference_ttl<=ttl
        and reference_ttl==math.floor(reference_ttl),'bundle_owner_configuration')
    assert(allow_quiet_prefix==nil or type(allow_quiet_prefix)=='boolean','bundle_owner_configuration')
    assert(allow_braking_prefix==nil or type(allow_braking_prefix)=='boolean','bundle_owner_configuration')
    local self={phase='waiting',head=clone(origin),head_tick=0,queue={},tail={},epoch=nil,
        released=false,last_sequence=0,last_authority=nil,last_now=clock(),reason=nil,requires_stop=false}
    local function fail(reason)
        self.phase='fault';self.reason=reason;self.queue={};self.requires_stop=self.head_tick>0
        error(reason)
    end
    local function now()
        local time=clock()
        if type(time)~='number' or time~=math.floor(time) or time<self.last_now or time<0 then
            fail('bundle_owner_clock')
        end
        self.last_now=time;return time
    end
    local function live()
        if self.phase=='fault' or self.phase=='ready' then error('bundle_owner_retired') end
    end
    local function due(tick)
        return self.epoch+math.max(0,tick-1)*80
    end
    function self.finish(reason)
        live();if self.released then return end
        self.released=true;self.queue={};self.reason=reason or 'release'
        self.phase=#self.tail==0 and quiet(self.head) and 'ready' or 'finishing'
    end
    local function authority(time)
        if self.released then return false end
        if not self.last_authority then return false end
        if time-self.last_authority>=ttl then self.finish('input_expired');return false end
        return true
    end
    local function expected_parent(packet)
        if #self.queue>0 then return packet.parent_tick==self.queue[#self.queue].packet.parent_tick+1 end
        return packet.parent_tick==self.head_tick or allow_quiet_prefix and self.phase=='holding'
            and #self.tail==0 and quiet(self.head) and packet.parent_tick==self.head_tick+1
            or allow_braking_prefix and #self.tail>0 and packet.parent_tick==self.head_tick+1
    end
    local function expected_start(packet)
        if #self.queue>0 then return self.queue[#self.queue].packet.next end
        if allow_braking_prefix and packet.parent_tick==self.head_tick+1 and #self.tail>0 then
            return self.tail[1].next
        end
        return self.head
    end
    function self.offer_authority(captured_ms)
        live();local time=now()
        if self.released then error('bundle_owner_retired') end
        if self.last_authority and time-self.last_authority>=ttl then
            self.finish('input_expired');error('bundle_owner_expired')
        end
        if type(captured_ms)~='number' or captured_ms~=math.floor(captured_ms) or captured_ms<0
            or captured_ms>time or time-captured_ms>=ttl
            or self.last_authority and captured_ms<self.last_authority then fail('bundle_owner_input') end
        if captured_ms==self.last_authority then return false end
        self.last_authority=captured_ms;return true
    end
    local function stage(input)
        live();local time=now()
        if not authority(time) then return false,'authority_missing' end
        if type(input)~='table' then fail('bundle_owner_payload') end
        local packet=clone(input)
        for _,field in ipairs({'sequence','parent_tick','due_ms','captured_ms','braking_ticks'}) do
            local value=packet[field]
            if type(value)~='number' or value~=math.floor(value) or value<0 then fail('bundle_owner_payload') end
        end
        if packet.sequence<1 or packet.braking_ticks<1 or packet.braking_ticks>60 then fail('bundle_owner_payload') end
        if packet.session~=binding.session or packet.model~=binding.model
            or packet.generation~=binding.generation then fail('bundle_owner_binding') end
        if packet.sequence<=self.last_sequence then fail('bundle_owner_sequence') end
        self.last_sequence=packet.sequence
        if #self.queue>=2 then return false,'queue_full' end
        if not expected_parent(packet) then return false,'stale_parent' end
        local expected=expected_start(packet)
        if not equal(packet.start,expected) then return false,'start_mismatch' end
        local epoch=self.epoch or packet.due_ms
        local deadline=epoch+math.max(0,packet.parent_tick-1)*80
        if packet.due_ms~=deadline or time>=deadline then return false,'late_bundle' end
        if packet.captured_ms>time or time-packet.captured_ms>=reference_ttl then return false,'stale_input' end
        local good,tail=pcall(expand,packet)
        if not good then fail('bundle_owner_expansion') end
        local passed,accepted=pcall(validate,packet,tail)
        if not passed or accepted~=true then
            if self.phase=='fault' then error(self.reason) end
            fail('bundle_owner_uncertified')
        end
        -- Validation may yield. Recheck genuine time and input authority;
        -- the old head/tail remain untouched on any late result.
        live();time=now()
        if time-packet.captured_ms>=reference_ttl then return false,'stale_input' end
        if not authority(time) then return false,'authority_expired' end
        if time>=deadline then return false,'late_bundle' end
        if self.released then return false,'released' end
        local current=expected_start(packet)
        if not expected_parent(packet) or not equal(packet.start,current) then
            return false,'stale_parent'
        end
        self.epoch=epoch
        self.queue[#self.queue+1]={packet=packet,tail=tail}
        if self.phase=='waiting' then self.phase='armed' end
        return true
    end
    function self.stage(input)
        live()
        if self.stage_busy then fail('bundle_owner_stage_inflight') end
        self.stage_busy=true
        local passed,a,b=pcall(stage,input)
        self.stage_busy=false
        if not passed then
            if self.phase~='fault' then fail('bundle_owner_stage_error') end
            error(a)
        end
        return a,b
    end
    function self.take_next()
        live();local time=now();authority(time)
        if self.phase=='ready' then return nil end
        if not self.epoch or time<due(self.head_tick) then return nil end
        if time-due(self.head_tick)>=80 then fail('bundle_owner_cadence_lost') end
        local segment,mode
        if #self.queue>0 and self.queue[1].packet.parent_tick==self.head_tick+1 then
            if allow_braking_prefix and #self.tail>0 then
                -- Consume the previously installed backup FIRST. The future
                -- lead cannot replace it or start from a guessed dynamic PVA.
                segment=table.remove(self.tail,1);mode='braking'
            else
                if not allow_quiet_prefix or #self.tail~=0 or not quiet(self.head) then fail('bundle_owner_chain') end
                -- One exact zero-rate hold tick, no actuator write, epoch unchanged.
                self.head_tick=self.head_tick+1;self.phase='holding';return nil
            end
        end
        if not segment and #self.queue>0 and not self.released then
            local staged=table.remove(self.queue,1)
            if staged.packet.parent_tick~=self.head_tick or not equal(staged.packet.start,self.head) then
                fail('bundle_owner_chain')
            end
            -- Install the FULL backup BEFORE returning an actuator candidate.
            self.tail=staged.tail;segment={start=staged.packet.start,next=staged.packet.next};mode='following'
            self.emitted_sequence=staged.packet.sequence
        elseif not segment and #self.tail>0 then segment=table.remove(self.tail,1);mode='braking'
        elseif not segment and quiet(self.head) then
            if self.released then self.phase='ready';return nil end
            self.phase='holding';self.head_tick=self.head_tick+1;return nil
        elseif not segment then fail('bundle_owner_missing_tail') end
        if not equal(segment.start,self.head) then fail('bundle_owner_chain') end
        self.head=clone(segment.next);self.head_tick=self.head_tick+1
        if self.released then self.phase=#self.tail==0 and 'ready' or 'finishing'
        else self.phase=#self.tail==0 and quiet(self.head) and 'holding' or 'active' end
        return clone(segment),mode
    end
    function self.describe()
        return {phase=self.phase,head_tick=self.head_tick,head=clone(self.head),queued=#self.queue,
            backup_points=#self.tail,last_sequence=self.last_sequence,last_authority=self.last_authority,
            reason=self.reason,requires_stop=self.requires_stop}
    end
    return self
end
return M
