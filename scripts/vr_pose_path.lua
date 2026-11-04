-- Pure Cartesian planning helpers; no actuator or clock access.
local M={}
local function quaternion(p)
    local sx,cx=math.sin(p.rx/2),math.cos(p.rx/2)
    local sy,cy=math.sin(p.ry/2),math.cos(p.ry/2)
    local sz,cz=math.sin(p.rz/2),math.cos(p.rz/2)
    return {sx*cy*cz-cx*sy*sz,cx*sy*cz+sx*cy*sz,cx*cy*sz-sx*sy*cz,cx*cy*cz+sx*sy*sz}
end
function M.clip(p,base,span)
    local out={};local clipped=false
    for k,v in pairs(p) do out[k]=v end
    for _,axis in ipairs({'x','y','z'}) do
        out[axis]=math.max(base[axis]-span,math.min(base[axis]+span,p[axis]))
        clipped=clipped or out[axis]~=p[axis]
    end
    return out,clipped
end
function M.step(from,to,position_step,angle_step)
    local d=math.sqrt((to.x-from.x)^2+(to.y-from.y)^2+(to.z-from.z)^2)
    local t=d>position_step and position_step/d or 1
    local out={};for _,axis in ipairs({'x','y','z'}) do out[axis]=from[axis]+t*(to[axis]-from[axis]) end
    local a,b=quaternion(from),quaternion(to);local dot=0
    for i=1,4 do dot=dot+a[i]*b[i] end
    if dot<0 then for i=1,4 do b[i]=-b[i] end;dot=-dot end
    local half=math.acos(math.max(-1,math.min(1,dot)))
    local u=half>angle_step/2 and angle_step/(2*half) or 1
    local q={};local norm=0
    for i=1,4 do
        q[i]=half<1e-8 and ((1-u)*a[i]+u*b[i])
            or ((math.sin((1-u)*half)*a[i]+math.sin(u*half)*b[i])/math.sin(half))
        norm=norm+q[i]*q[i]
    end
    norm=math.sqrt(norm);for i=1,4 do q[i]=q[i]/norm end
    local x,y,z,w=q[1],q[2],q[3],q[4]
    out.ry=math.asin(math.max(-1,math.min(1,2*(w*y-z*x))))
    if math.abs(math.cos(out.ry))<1e-8 then
        out.rx=0;out.rz=math.atan2(2*(w*z-x*y),1-2*(x*x+z*z))
    else
        out.rx=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        out.rz=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    end
    return out
end
function M.limit_rotation(p,reference,limit)
    local a,b=quaternion(reference),quaternion(p);local dot=0
    for i=1,4 do dot=dot+a[i]*b[i] end
    local distance=2*math.acos(math.max(-1,math.min(1,math.abs(dot))))
    if distance<=limit then return p,false end
    local origin={x=p.x,y=p.y,z=p.z,rx=reference.rx,ry=reference.ry,rz=reference.rz}
    return M.step(origin,p,1,limit),true
end
return M
