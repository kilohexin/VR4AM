-- Published theoretical LM3 MDH, literal alpha=1.5708, EXPLICIT ZERO TOOL.
-- Pure fresh numerical FK. No SDK, clocks, actuator or state caching.
-- Caller MUST bind/check the actual zero tool and compare manufacturer FK
-- before using this for a Docker diagnostic. Not factory calibration.
local M={}
local d={.21583,0,0,.12063,.09833,.08343}
local links={0,0,-.28,-.26,0,0}
local alpha={0,1.5708,0,0,1.5708,-1.5708}
local ca,sa={},{}
for i=1,6 do ca[i]=math.cos(alpha[i]);sa[i]=math.sin(alpha[i]) end
function M.pose(q)
    assert(type(q)=='table' and #q==6,'cartesian_lua_model_shape')
    local r={1,0,0,0,1,0,0,0,1};local p={0,0,0}
    for joint=1,6 do
        assert(type(q[joint])=='number' and q[joint]==q[joint] and math.abs(q[joint])<1000000,
            'cartesian_lua_model_nonfinite')
        local ct,st=math.cos(q[joint]),math.sin(q[joint])
        local local_r={ct,-st,0,st*ca[joint],ct*ca[joint],-sa[joint],st*sa[joint],ct*sa[joint],ca[joint]}
        local local_p={links[joint],-d[joint]*sa[joint],d[joint]*ca[joint]}
        local following={}
        for row=1,3 do
            local offset=(row-1)*3
            local value=0
            for axis=1,3 do value=value+r[offset+axis]*local_p[axis] end
            p[row]=p[row]+value
            for column=1,3 do
                value=0
                for axis=1,3 do value=value+r[offset+axis]*local_r[(axis-1)*3+column] end
                following[offset+column]=value
            end
        end
        r=following
    end
    return {p=p,r=r}
end
return M
