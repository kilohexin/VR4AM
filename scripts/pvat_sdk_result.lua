-- Lua SDK Robot:move_pvat returns no value (SDK, not the global Lua API).
-- No actuator/endpoints here; exceptions from send propagate unchanged.
local M={}
function M.native_joint(values)
    assert(type(values)=='table' and #values==6,'owned_native_joint_shape')
    local result={}
    for i=1,6 do
        local v=values[i]
        assert(type(v)=='number' and v==v and math.abs(v)<1000000,'owned_native_joint_value')
        result['j'..i]=v
    end
    return result
end
function M.call_void(send)
    local values=table.pack(send())
    assert(values.n==0 or (values.n==1 and values[1]==nil),'owned_write_unexpected_result')
end
function M.call_native_boolean(send)
    local values=table.pack(send())
    assert(values.n==1 and values[1]==true,'owned_native_write_unverified')
end
return M
