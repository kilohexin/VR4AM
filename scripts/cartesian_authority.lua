-- Independent genuine-input lease. Channels 240..245, no motion APIs.
-- Writer fills session/generation first, then atomically commits word 240.
local M={}
local function integer(v,lo,hi)
    return type(v)=='number' and v==math.floor(v) and v>=lo and v<=hi
end
function M.new(read_many,read_one,clock,binding,ttl)
    assert(integer(ttl,1,500),'cartesian_authority_configuration')
    local self={closed=false,last_word=nil,last_capture=nil,last_now=clock()}
    local function now()
        local n=clock()
        assert(integer(n,0,2147483647) and n>=self.last_now,'cartesian_authority_clock')
        self.last_now=n;return n
    end
    local function live()
        if self.last_capture and now()-self.last_capture>=ttl then
            self.closed=true;return false
        end
        return not self.closed
    end
    local function poll()
        if not live() then return nil,'expired' end
        local words=read_many(240,6);now()
        assert(type(words)=='table' and #words==6,'cartesian_authority_shape')
        local commit=read_one(240);now()
        assert(integer(commit,-1,2147483647),'cartesian_authority_word')
        if not live() then return nil,'expired' end
        if commit~=words[1] then return nil,'torn' end
        if commit==0 then
            if self.last_capture==nil then return nil,'waiting' end
            self.closed=true;return nil,'released'
        end
        for i=1,4 do assert(words[i+1]==binding.words[i],'cartesian_authority_binding') end
        assert(words[6]==binding.generation,'cartesian_authority_binding')
        -- -1 is a bound withdrawal even before a first live input. Zero is
        -- startup neutral until activated; an old active zero-release stays
        -- compatible. Wrong-session withdrawals cannot retire this owner.
        if commit==-1 then self.closed=true;return nil,'released' end
        local capture=math.floor(commit/1024)-1
        assert(integer(capture,0,2147483647) and capture<=now(),'cartesian_authority_capture')
        assert(not self.last_capture or capture>=self.last_capture,'cartesian_authority_rollback')
        if now()-capture>=ttl then self.closed=true;return nil,'expired' end
        if commit==self.last_word then return nil,'duplicate' end
        self.last_word=commit;self.last_capture=capture
        return capture,'fresh'
    end
    function self.poll()
        assert(not self.closed,'cartesian_authority_retired')
        local ok,a,b=pcall(poll)
        if not ok then self.closed=true;error(a) end
        return a,b
    end
    function self.fresh()
        if not live() then return false end
        return self.last_capture~=nil
    end
    return self
end
return M
