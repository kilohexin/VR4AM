-- Explicit Linux/Docker diagnostic clock, NOT a hardware API assumption.
-- /proc/uptime has 10ms resolution. Consumers must include that uncertainty.
local function cartesian_now()
    local file=assert(io.open('/proc/uptime','r'),'cartesian_boot_clock_unavailable')
    local line=file:read('*l');file:close()
    local seconds,fraction=string.match(line or '','^(%d+)%.(%d%d)%s')
    assert(seconds and fraction,'cartesian_boot_clock_format')
    return tonumber(seconds)*1000+tonumber(fraction)*10
end
