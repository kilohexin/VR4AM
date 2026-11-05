-- Published theoretical LM3 MDH, literal alpha=1.5708.
-- Pure fresh numerical FK. No SDK, clocks, actuator or state caching.
-- pose() is the explicit zero-tool model; with_tool() binds a copied tool.
-- Caller MUST check the selected tool and compare manufacturer FK
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
function M.with_tool(position,quaternion)
    assert(type(position)=='table' and #position==3 and type(quaternion)=='table' and #quaternion==4,
        'cartesian_lua_tool_shape')
    local p={};local q={};local radius,norm=0,0
    for i=1,3 do
        local v=position[i]
        assert(type(v)=='number' and v==v and math.abs(v)<1,'cartesian_lua_tool_finite')
        p[i]=v;radius=radius+v*v
    end
    assert(radius<1,'cartesian_lua_tool_radius')
    for i=1,4 do
        local v=quaternion[i]
        assert(type(v)=='number' and v==v and math.abs(v)<=1,'cartesian_lua_tool_finite')
        q[i]=v;norm=norm+v*v
    end
    assert(math.abs(norm-1)<1e-9,'cartesian_lua_tool_rotation')
    local x,y,z,w=q[1],q[2],q[3],q[4]
    local r={1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}
    return {pose=function(joints)
        local base=M.pose(joints);local out={p={},r={}}
        for row=1,3 do
            local offset=(row-1)*3;local value=base.p[row]
            for axis=1,3 do value=value+base.r[offset+axis]*p[axis] end
            out.p[row]=value
            for column=1,3 do
                value=0
                for axis=1,3 do value=value+base.r[offset+axis]*r[(axis-1)*3+column] end
                out.r[offset+column]=value
            end
        end
        return out
    end}
end
return M
