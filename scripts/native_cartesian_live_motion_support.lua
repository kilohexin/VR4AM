local actual_entries=0;local actual_returns=0;local actual_samples=0
local actual_max_drift=0;local actual_max_error=0;local actual_previous=nil
local actual_recent={}
local curve_witness=(function()
__CURVE_WITNESS__
end)()
local actual_model=(function()
__LUA_MDH__
end)().with_tool(__BOUND_TOOL_POSITION__,__BOUND_TOOL_QUATERNION__)
-- Lower-bound search pruning only. Acceptance uses the FK Cartesian witness,
-- never this joint box, and makes no firmware quantization assumption.
local actual_target_readout_epsilon=__TARGET_READOUT_EPSILON__
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
        ranges[axis]={low-actual_target_readout_epsilon,high+actual_target_readout_epsilon}
    end
    ranges.identity=identity or actual_entries+1
    ranges.coefficients=c
    actual_recent[#actual_recent+1]=ranges
    if #actual_recent>8 then table.remove(actual_recent,1) end
end
local function actual_finite_pose(p)
    assert(type(p)=='table','live_actual_pose_shape')
    for _,key in ipairs({'x','y','z','rx','ry','rz'}) do
        local value=p[key]
        assert(type(value)=='number' and value==value and math.abs(value)<1000000,'live_actual_pose_finite')
    end
end
local function actual_pose_error(a,b)
    actual_finite_pose(b)
    local position=math.sqrt((a.p[1]-b.x)^2+(a.p[2]-b.y)^2+(a.p[3]-b.z)^2)
    local ra,rb=a.r,actual_rotation(b);local dot=0
    for i=1,9 do dot=dot+ra[i]*rb[i] end
    return position,math.acos(math.max(-1,math.min(1,(dot-1)/2)))
end
local function actual_admit_model()
    for _,q in ipairs({origin.q,{.2,-.8,1.4,-.6,1.2,.3},{-.3,-.4,.9,.2,-1,.4}}) do
        local computed=actual_model.pose(q);local vendor=lebai:kinematics_forward(q)
        local position,angle=actual_pose_error(computed,vendor)
        assert(position<1e-9 and angle<1e-6,'live_actual_model_mismatch')
    end
end
local function actual_observe(k)
    local now=clock();local p=k.actual_tcp_pose;local base=initial.actual_tcp_pose
    actual_finite_pose(p)
    assert(not actual_previous or now>=actual_previous and now-actual_previous<100,'live_actual_feedback_gap')
    actual_previous=now
    local drift=math.sqrt((p.x-base.x)^2+(p.y-base.y)^2+(p.z-base.z)^2)
    assert(drift==drift and drift<=.03,'live_actual_fixed_region')
    local rotation=actual_rotation(p);local dot=0
    for i=1,9 do dot=dot+rotation[i]*actual_base_rotation[i] end
    assert(math.acos(math.max(-1,math.min(1,(dot-1)/2)))<=.30,'live_actual_rotation_region')
    local error=0
    for i=1,6 do
        local q=k.target_joint_pose[i]
        assert(type(q)=='number' and q==q and math.abs(q)<=3,'live_actual_target_joint')
        error=math.max(error,math.abs(q-k.actual_joint_pose[i]))
    end
    assert(error<=.03,'live_actual_tracking')
    -- Freeze references before yielding; only curves already entered when
    -- this feedback was read may supply its witness.
    local curves={};for i=1,#actual_recent do curves[i]=actual_recent[i] end
    local found=curve_witness.closest(curves,k.target_joint_pose,owner_module.evaluate)
    local reference=actual_model.pose(found.state.q)
    local target_distance,target_angle=actual_pose_error(reference,k.target_tcp_pose)
    local actual_distance,actual_angle=actual_pose_error(reference,k.actual_tcp_pose)
    local matched=target_distance<=.001 and target_angle<=.01 and actual_distance<=.002 and actual_angle<=.01
    report('CART_ACTUAL_WITNESS',now,found.identity,found.u,target_distance,actual_distance,target_angle,actual_angle)
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
    assert(matched,'live_actual_cartesian_witness')
    actual_samples=actual_samples+1;actual_max_drift=math.max(actual_max_drift,drift)
    actual_max_error=math.max(actual_max_error,error)
    report('CART_ACTUAL_SAMPLE',now,drift,error)
end
