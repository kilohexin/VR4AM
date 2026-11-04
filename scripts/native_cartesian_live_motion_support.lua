local actual_entries=0;local actual_returns=0;local actual_samples=0
local actual_max_drift=0;local actual_max_error=0;local actual_previous=nil
local actual_recent={}
local function actual_rotation(p)
    local sx,cx=math.sin(p.rx),math.cos(p.rx);local sy,cy=math.sin(p.ry),math.cos(p.ry)
    local sz,cz=math.sin(p.rz),math.cos(p.rz)
    return {cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}
end
local actual_base_rotation=nil
local function actual_envelope(segment,identity)
    local c=owner_module.coefficients(segment.start,segment.next,.08);local ranges={}
    local choose5={1,5,10,10,5,1}
    for axis=1,6 do
        local low,high=math.huge,-math.huge
        for degree=0,5 do
            local value=0;local choose=1
            for j=0,degree do
                value=value+choose*c[axis][j+1]/choose5[j+1]
                choose=choose*(degree-j)/(j+1)
            end
            low=math.min(low,value);high=math.max(high,value)
        end
        ranges[axis]={low-2e-6,high+2e-6}
    end
    ranges.identity=identity or actual_entries+1
    actual_recent[#actual_recent+1]=ranges
    if #actual_recent>8 then table.remove(actual_recent,1) end
end
local function actual_observe(k)
    local now=clock();local p=k.actual_tcp_pose;local base=initial.actual_tcp_pose
    assert(not actual_previous or now>=actual_previous and now-actual_previous<100,'live_actual_feedback_gap')
    actual_previous=now
    local drift=math.sqrt((p.x-base.x)^2+(p.y-base.y)^2+(p.z-base.z)^2)
    assert(drift==drift and drift<=.03,'live_actual_fixed_region')
    local rotation=actual_rotation(p);local dot=0
    for i=1,9 do dot=dot+rotation[i]*actual_base_rotation[i] end
    assert(math.acos(math.max(-1,math.min(1,(dot-1)/2)))<=.30,'live_actual_rotation_region')
    local error=0;local matched=false
    for i=1,6 do
        local q=k.target_joint_pose[i]
        assert(type(q)=='number' and q==q and math.abs(q)<=3,'live_actual_target_joint')
        error=math.max(error,math.abs(q-k.actual_joint_pose[i]))
    end
    assert(error<=.03,'live_actual_tracking')
    for _,range in ipairs(actual_recent) do
        local inside=true
        for i=1,6 do inside=inside and k.target_joint_pose[i]>=range[i][1]
            and k.target_joint_pose[i]<=range[i][2] end
        matched=matched or inside
    end
    local values={}
    for _,field in ipairs({'actual_joint_pose','target_joint_pose','actual_joint_speed','target_joint_speed','target_joint_acc'}) do
        for i=1,6 do
            local value=type(k[field])=='table' and k[field][i] or nil
            values[#values+1]=(type(value)=='number' and value==value and math.abs(value)<1000000)
                and string.format('%.17g',value) or 'NA'
        end
    end
    report('CART_ACTUAL_KIN',now,table.concat(values,'\t'))
    if not matched then
        for _,range in ipairs(actual_recent) do
            local bounds={}
            for i=1,6 do bounds[#bounds+1]=string.format('%.17g',range[i][1])
                bounds[#bounds+1]=string.format('%.17g',range[i][2]) end
            report('CART_ACTUAL_REJECT_RANGE',now,range.identity,table.concat(bounds,'\t'))
        end
    end
    assert(matched,'live_actual_target_not_owned')
    actual_samples=actual_samples+1;actual_max_drift=math.max(actual_max_drift,drift)
    actual_max_error=math.max(actual_max_error,error)
    report('CART_ACTUAL_SAMPLE',now,drift,error)
end
