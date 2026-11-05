-- Fixed preloaded finite scope, no transport/endpoint selection.
-- Builder injects validated immutable contract, tool and geometry before this.
local events={};local pending=false;local entered=0;local returned=0
local failure=nil;local finished=false;local last_clock=nil;local reads=0
local function clock()
 local n=timestamp()
 assert(type(n)=='number' and n==n and n>=0 and (not last_clock or n>=last_clock),'clock_rollback')
 last_clock=n;return n
end
local function event(kind,...)
 assert(#events<1200,'event_budget');events[#events+1]={kind,...}
end
local function finite(v)
 assert(type(v)=='number' and v==v and math.abs(v)<1000000,'feedback_nonfinite');return v
end
local function vector(v)
 assert(type(v)=='table' and #v==6,'feedback_shape');for i=1,6 do finite(v[i]) end
end
local function pose(p)
 for _,key in ipairs({'x','y','z','rx','ry','rz'}) do finite(p[key]) end
 local sx,cx=math.sin(p.rx),math.cos(p.rx);local sy,cy=math.sin(p.ry),math.cos(p.ry)
 local sz,cz=math.sin(p.rz),math.cos(p.rz)
 return {p={p.x,p.y,p.z},r={cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
  sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,-sy,cy*sx,cy*cx}}
end
local function distance(a,b)
 local d=0;for i=1,3 do d=d+(a[i]-b[i])^2 end;return math.sqrt(d)
end
local function angle(a,b)
 local trace=0;for i=1,9 do trace=trace+a[i]*b[i] end
 return math.acos(math.min(1,math.max(-1,(trace-1)/2)))
end
local base=model.pose(origin);local goal=model.pose(points[20].p)
local previous_sample=nil;local stable=nil;local arrival=nil;local acceleration_count=0
local function read()
 reads=reads+1;assert(reads<=512,'read_budget')
 local capture=clock();local state=lebai:get_robot_state();local k=lebai:get_kin_data();local now=clock()
 assert(now-capture<100,'feedback_gap')
 assert(not previous_sample or capture>previous_sample and now-previous_sample<100,'feedback_gap')
 previous_sample=capture
 assert(state=='IDLE' or state=='MOVING','robot_state')
 local fields={'actual_joint_pose','actual_joint_speed','actual_joint_acc','target_joint_pose','target_joint_speed','target_joint_acc'}
 local row={'RP_SAMPLE',capture,now,state}
 for _,field in ipairs(fields) do vector(k[field]);for i=1,6 do row[#row+1]=k[field][i] end end
 local tcp=k.actual_tcp_pose;local observed=pose(tcp)
 for _,key in ipairs({'x','y','z','rx','ry','rz'}) do row[#row+1]=tcp[key] end
 events[#events+1]=row
 local peak=0
 for i=1,6 do
  for _,prefix in ipairs({'actual','target'}) do
   local q=k[prefix..'_joint_pose'][i]
   assert(q>=lower[i] and q<=upper[i],'joint_bounds')
   assert(math.abs(k[prefix..'_joint_speed'][i])<=.05+1e-10,'joint_speed')
  end
  assert(math.abs(k.target_joint_acc[i])<=.2+1e-10,'target_acceleration')
  peak=math.max(peak,math.abs(k.actual_joint_acc[i]))
 end
 acceleration_count=peak>.2 and acceleration_count+1 or 0
 assert(peak<=.4 and acceleration_count<3,'actual_acceleration')
 for i=1,3 do assert(observed.p[i]>=box_lower[i] and observed.p[i]<=box_upper[i],'workspace') end
 local actual=model.pose(k.actual_joint_pose);local target=model.pose(k.target_joint_pose)
 for _,p in ipairs({observed,actual,target}) do
  local nearest={base.p[1],base.p[2],math.min(goal.p[3],math.max(base.p[3],p.p[3]))}
  assert(distance(p.p,nearest)<=.001 and angle(p.r,base.r)<=math.rad(.2),'tcp_corridor')
 end
 assert(distance(observed.p,actual.p)<=.001 and angle(observed.r,actual.r)<=math.rad(.2),'model_disagreement')
 return k,observed,state,capture,now,target
end
local function stable_delta(k,p,anchor,tcp_drift,joint_drift)
 if distance(p.p,anchor.pose.p)>tcp_drift then return false end
 for i=1,6 do if math.abs(k.actual_joint_pose[i]-anchor.q[i])>joint_drift then return false end end
 return true
end
local function anchor(k,p,t)
 local q={};for i=1,6 do q[i]=k.actual_joint_pose[i] end
 return {q=q,pose=p,time=t}
end
local function quiet(k,cap)
 for i=1,6 do if math.abs(k.actual_joint_speed[i])>cap or math.abs(k.target_joint_speed[i])>cap then return false end end
 return true
end
local ok,err=pcall(function()
 local t=pose(lebai:get_tcp())
 assert(distance(t.p,tool_pose.p)<1e-9 and angle(t.r,tool_pose.r)<1e-6,'tool_changed')
 assert(lebai:get_velocity_factor()==100,'velocity_factor_changed')
 local k,p,state,capture,now=read();assert(state=='IDLE','requires_idle')
 for i=1,6 do
  assert(math.abs(k.actual_joint_pose[i]-origin[i])<=1e-6 and math.abs(k.target_joint_pose[i]-origin[i])<=1e-6,'origin_changed')
  assert(k.actual_joint_speed[i]==0 and k.actual_joint_acc[i]==0 and k.target_joint_speed[i]==0 and k.target_joint_acc[i]==0,'origin_not_quiet')
 end
 local fk=pose(lebai:kinematics_forward(origin))
 assert(distance(fk.p,base.p)<=.001 and angle(fk.r,base.r)<=math.rad(.2),'factory_fk_disagreement')
 assert(clock()-capture<100,'origin_expired')
 event('RP_ADMIT',clock(),capture)
 local epoch=nil;local previous=nil
 local function worker(fn)
  return async_task(function() local passed,reason=pcall(fn)
   if not passed then failure=failure or tostring(reason);finished=true end
  end)
 end
 local monitor=worker(function()
  while not finished do
   rt_sleep(20)
   if finished then return end
   local sample,tcp,s,c,n,target=read()
   if epoch then
    assert(arrival or n-epoch<5000,'arrival_deadline')
    local endpoint=distance(tcp.p,goal.p)<=.001 and angle(tcp.r,goal.r)<=math.rad(.2)
     and tcp.p[3]-base.p[3]>=.001 and distance(target.p,goal.p)<=.001 and angle(target.r,goal.r)<=math.rad(.2)
    for i=1,6 do endpoint=endpoint and math.abs(sample.actual_joint_pose[i]-points[20].p[i])<=math.rad(.1)
     and math.abs(sample.target_joint_pose[i]-points[20].p[i])<=math.rad(.1) end
    if arrival then
     assert(endpoint and s=='IDLE' and quiet(sample,.02) and stable_delta(sample,tcp,arrival,.001,math.rad(.1)),'tail_drift')
     if c-arrival.time>=2000 then event('RP_TAIL',c);finished=true;return end
    elseif returned==20 and endpoint and s=='IDLE' and quiet(sample,.002) then
     if stable and not stable_delta(sample,tcp,stable,.0001,math.rad(.02)) then stable=nil end
     stable=stable or anchor(sample,tcp,c)
     if c-stable.time>=200 then arrival=anchor(sample,tcp,c);event('RP_ARRIVAL',c) end
    else stable=nil end
   end
  end
 end)
 local sender=worker(function()
  epoch=clock()
  assert(epoch-capture<100,'origin_expired')
  for index,item in ipairs(points) do
   local due=epoch+(index-1)*79
   if clock()<due then rt_sleep(due-clock()) end
   if finished then return end
   local n=clock();assert(n-previous_sample<100,'feedback_gap')
   assert(not previous or n-previous<80,'pvat_cadence')
   assert(index*80-(n-epoch)<=160,'queue_lead')
   pending=true;entered=index;previous=n;event('RP_ENTER',index,n)
   local sent,reason=pcall(function() send_pvat(item) end)
   pending=false -- Call activity ended; this does NOT resolve a failed write.
   if not sent then event('RP_WRITE_ERROR',index,clock());error('pvat_write_unknown') end
   local r=clock();returned=returned+1;event('RP_RETURN',index,r)
   assert(r-n<(index==1 and first_reply_ms or 60),'pvat_reply_deadline')
  end
 end)
 wait_all(monitor,sender);assert(not failure,tostring(failure));assert(arrival and returned==20,'incomplete')
end)
if not ok and entered>0 and not pending then
 -- One exception request only. Its return is NEVER stop verification.
 -- A failed clock must not suppress the sole stop attempt or trace flush.
 local timed,stamp=pcall(clock);event('RP_STOP_ENTER',timed and stamp or 'unknown_clock')
 local stopped=pcall(function() lebai:stop_move() end)
 if stopped then
  local known,receipt=pcall(clock);event('RP_STOP_RETURN',known and receipt or 'unknown_clock')
 end
end
for _,row in ipairs(events) do
 local values={};for i,v in ipairs(row) do values[i]=type(v)=='number' and string.format('%.17g',v) or tostring(v) end
 print(table.concat(values,'\t'))
end
print('RP_DONE',ok,tostring(err),returned)
