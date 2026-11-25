'use strict';
const $ = id => document.getElementById(id);
const data = window.LM3_DEMO;
const runKind = 'site-detour-eight-full';
const short = runKind==='short', entry = runKind==='entry', quarter = ['site-detour-quarter','site-detour-quarter-capture'].includes(runKind);
const fullEight = runKind==='site-detour-eight-full';
const eight = fullEight || runKind==='site-detour-eight-capture';
const siteDetour = runKind==='site-detour' || quarter || eight, detour = !short&&!entry;
const green = '#6ef0c0', blue = '#69b9ff', red = '#ff797f', yellow = '#edbb68', purple = '#d4a4ff';
let time = 0, phase = 2, animation = 0, playing = false;
let modelViews;
let physical, comparisons = [];
const duration = () => Math.max(data.playbackEnd,physical?.frames.at(-1).t ?? 0);
const feedbackFrames = () => data[$('feedback-source').value];

function context(id) {
  const canvas = $(id), {width:w,height:h} = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(w*dpr); canvas.height = Math.round(h*dpr);
  const ctx = canvas.getContext('2d'); ctx.scale(dpr,dpr);
  return {ctx,w,h};
}

function line(ctx, points, color, width, dash = []) {
  if (!points.length) return;
  ctx.beginPath(); ctx.moveTo(...points[0]);
  for (const point of points.slice(1)) ctx.lineTo(...point);
  ctx.strokeStyle=color; ctx.lineWidth=width; ctx.setLineDash(dash); ctx.lineJoin='round'; ctx.lineCap='round';
  ctx.stroke(); ctx.setLineDash([]);
}

function interpolate(frames, t) {
  let lo=0, hi=frames.length-1;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(frames[mid].t<t)lo=mid;else hi=mid}
  const a=frames[lo], b=frames[hi];
  const amount=Math.min(1,Math.max(0,(t-a.t)/(b.t-a.t)));
  const mix=(p,q)=>p.map((value,i)=>value+(q[i]-value)*amount);
  return {p:mix(a.p,b.p),q:a.q&&mix(a.q,b.q),links:a.links?.map((v,i)=>mix(v,b.links[i]))};
}

function plannerPose() {
  if (phase===0) return interpolate(data.baseline,time);
  if (phase===1) return interpolate(data.baseline,data.collision[Math.floor(data.collision.length/2)].t);
  return interpolate(data.nominal,time);
}

function scene(id, isPlanner) {
  if (modelViews) {
    const view = isPlanner ? modelViews.planned : modelViews.feedback;
    const pose = isPlanner ? plannerPose() : phase===2 ? interpolate(feedbackFrames(),time) : null;
    view.render(pose,phase);
    return;
  }
  const {ctx,w,h}=context(id);
  if (!data) return;
  const scale=Math.min(w/1.03,h/.59);
  const view=([x,y,z])=>[w*.54+(.57*x-.82*y-.045)*scale,h*.70+(-.37*x-.28*y-.88*z-.060)*scale];
  const start=data.nominal[0].p, goal=data.nominal.at(-1).p;
  const trace=(frames,color,width,dash)=>line(ctx,frames.map(x=>view(x.p)),color,width,dash);

  for(let x=-.9;x<=.21;x+=.1)line(ctx,[view([x,-.63,0]),view([x,.34,0])],'#253546',1);
  for(let y=-.63;y<=.34;y+=.1)line(ctx,[view([-.9,y,0]),view([.21,y,0])],'#253546',1);

  function box() {
    const obstacle=data.scene.find(item=>item.id==='box');
    if(!obstacle)return;
    const {center:[x,y,z],size:[a,b,c]}=obstacle;
    const vertices=[[-1,-1],[-1,1],[1,1],[1,-1]];
    const layers=[-1,1].map(k=>vertices.map(([i,j])=>view([x+i*a/2,y+j*b/2,z+k*c/2])));
    function face(points,color){
      ctx.beginPath();ctx.moveTo(...points[0]);points.slice(1).forEach(p=>ctx.lineTo(...p));
      ctx.closePath();ctx.fillStyle=color;ctx.fill();ctx.strokeStyle='#edbb689c';ctx.lineWidth=1;ctx.stroke();
    }
    face([layers[0][0],layers[0][1],layers[1][1],layers[1][0]],'#95683266');
    face([layers[0][1],layers[0][2],layers[1][2],layers[1][1]],'#bb804e8c');
    face(layers[1],'#edbb6880');
  }
  function dot(position,color,label) {
    const [x,y]=view(position);
    ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();
    if(label){ctx.font='12px sans-serif';ctx.fillText(label,x+8,y-8)}
  }
  function robot(frame,color) {
    const parts=frame.links.map(view);
    for(let i=1;i<parts.length;i++){
      const width=i===parts.length-1?8:i<3?11:9;
      line(ctx,[parts[i-1],parts[i]],'#0a121d',width+3);
      line(ctx,[parts[i-1],parts[i]],i===parts.length-1?color:'#738a9e',width);
    }
    for(const [index,[x,y]] of parts.entries()){
      if(index===parts.length-1)break;
      ctx.beginPath();ctx.arc(x,y,6,0,Math.PI*2);
      ctx.fillStyle='#202d3d';ctx.fill();ctx.strokeStyle=color;ctx.lineWidth=1;ctx.stroke();
    }
    dot(frame.p,color);
  }

  if(phase>0)box();
  if(isPlanner){
    trace(data.baseline,phase===0?'#a5b5c4':red,2,[5,5]);
    if(phase===2)trace(data.nominal,green,3);
    robot(plannerPose(),phase===1?red:green);
  } else if(phase===2){
    trace(data.nominal,'#8496a9',1.5,[4,5]);
    trace(data.docker,blue,3.5);
    trace(data.mujoco,green,1.65);
    if(physical)trace(physical.frames,purple,2,[4,3]);
    const source=$('feedback-source').value;
    const colors={mujoco:green,docker:blue,physical:purple};
    const pose=interpolate(feedbackFrames(),time);
    if(pose.links)robot(pose,colors[source]);
    for(const [name,frames] of [['mujoco',data.mujoco],['docker',data.docker],['physical',physical?.frames]]){
      if(frames && (name!==source || !pose.links))dot(interpolate(frames,time).p,colors[name]);
    }
  }
  if(!short){
    dot(start,'#dce8f2','起点');
    dot(goal,green,'目标');
  }
}

function errorPlot(){
  const {ctx,w,h}=context('error');
  if(!data) return;
  const chartW=w-70,chartH=h-42,top=13,left=49,end=duration();
  const primaryOnly=detour&&physical&&!$('show-docker-errors').checked;
  const plotted=comparisons.filter(row=>row.errors&&(!primaryOnly||row.primary));
  const peak=Math.max(0,...plotted.map(item=>item.peak));
  const limit=Math.max(.05,Math.ceil(peak*10)/10);
  const xy=([t,value])=>[left+t/end*chartW,top+(1-value/limit)*chartH];
  ctx.font='11px sans-serif';ctx.fillStyle='#a2b2c4';
  for(let i=0;i<4;i++){
    const value=limit*i/3,y=xy([0,value])[1];
    line(ctx,[[left,y],[left+chartW,y]],'#2a3a4c',1);
    ctx.fillText(value.toFixed(2),8,y+4);
  }
  for(let i=0;i<5;i++){
    const t=end*i/4,[x]=xy([t,0]);ctx.fillText(t.toFixed(1),x-8,h-9);
  }
  for(const row of plotted){
    line(ctx,row.errors.map(xy),row.color,row.primary?2.7:2.0,row.dash);
  }
  if(phase===2){
    const cursor=xy([time,0])[0];line(ctx,[[cursor,top],[cursor,top+chartH]],blue,1,[3,5]);
  }
}

function tcpPlot(){
  const {ctx,w,h}=context('tcp-progress');
  const startZ=data.nominal[0].p[2], end=duration();
  const series=[[data.nominal,'#bac6d3',[4,4]],[data.docker,blue,[]],[data.mujoco,green,[]]];
  if(physical)series.push([physical.frames,purple,[6,3]]);
  let low=-.5, high=10.5;
  for(const [frames] of series){
    for(const frame of frames){
      const z=(frame.p[2]-startZ)*1000;
      low=Math.min(low,z);high=Math.max(high,z);
    }
  }
  const xy=(t,z)=>[49+t/end*(w-70),13+(1-(z-low)/(high-low))*(h-42)];
  ctx.font='11px sans-serif';ctx.fillStyle='#a2b2c4';
  for(const z of [0,2.5,5,7.5,10]){
    const [x,y]=xy(0,z);
    line(ctx,[[x,y],[w-21,y]],'#2a3a4c',1);
    ctx.fillText(z.toFixed(1),10,y+4);
  }
  for(let i=0;i<5;i++){
    const t=end*i/4;ctx.fillText(t.toFixed(1),xy(t,0)[0]-8,h-9);
  }
  for(const [frames,color,dash] of series){
    const first=Math.max(0,frames[0].t), last=Math.min(end,frames.at(-1).t);
    if(last<=first)continue;
    const points=[{t:first,p:interpolate(frames,first).p},...frames.filter(f=>f.t>first&&f.t<last),
      {t:last,p:interpolate(frames,last).p}];
    if(frames===data.nominal && end>last)points.push({...points.at(-1),t:end});
    line(ctx,points.map(({t,p})=>xy(t,(p[2]-startZ)*1000)),color,2,dash);
  }
  const [x]=xy(time,0);line(ctx,[[x,13],[x,h-29]],blue,1,[3,5]);
}

function render() {
  for(const tab of document.querySelectorAll('[data-step]'))
    tab.classList.toggle('active',Number(tab.dataset.step)===phase);
  if(data){
    for(const id of ['view-time','time-label'])$(id).textContent=
      id==='view-time'?time.toFixed(2)+' 秒':time.toFixed(2)+' / '+duration().toFixed(2)+' 秒';
    $('clock').value=Math.min(time,duration());
    $('plan-note').textContent=entry ? '从截图姿态到原绕箱起点 · '+data.planDuration.toFixed(2)+' 秒' : short ? '基座 +Z '+data.motion.requestedMm[2].toFixed(0)+' mm · 工具朝向保持' : [
      '空桌基准轨迹可通过',
      '原路径与箱子相交 · '+data.collision.length+' 个采样点',
      '绕行轨迹 · '+data.metrics.collisionChecks+' 个姿态通过复查'
    ][phase];
    $('status').textContent=short||entry ? 'MoveIt '+data.metrics.collisionChecks+' 个复查姿态 · MuJoCo 与 Docker 回放未检出碰撞' : [
      '空桌场景，回放原始规划。',
      '同一起点和目标，加入箱子后原路径被拒绝。',
      '采样范围内最小绕箱间距 '+data.metrics.clearance.toFixed(1)+' mm · MuJoCo 无障碍物接触'
    ][phase];
  }
  scene('planner',true);scene('execution',false);errorPlot();
  if(short && data)tcpPlot();
}

function start(){
  if(playing){cancelAnimationFrame(animation);playing=false;$('play').textContent='▶ 重播完整演示';return}
  playing=true;$('play').textContent='Ⅱ 暂停';
  const epoch=performance.now();
  const advance=now=>{
    const ellapsed=(now-epoch)/1000;
    phase=!detour?2:ellapsed<2?0:ellapsed<3.2?1:2;
    time=phase===0?ellapsed:phase===1?0:Math.min(duration(),ellapsed-(detour?3.2:0));
    render();
    if(phase<2 || time<duration())animation=requestAnimationFrame(advance);
    else{playing=false;$('play').textContent='▶ 重播完整演示'}
  };
  animation=requestAnimationFrame(advance);
}

const distance = value => value === null ? '未取得' : value < .01 ? (value*1000).toPrecision(3)+' μm' : value.toFixed(3)+' mm';

function updateComparisons() {
  const m = data.metrics;
  const label = 'LM3 真机';
  const simulation = {label:'MuJoCo / 乐白 Docker',color:green,swatch:'green',errors:data.errors,
    peak:m.peak,rawPeak:m.rawPeak,settled:m.settled};
  comparisons = [simulation];
  if(physical){
    const curve = ({peak,rawPeak,settled,errors}) => ({peak,rawPeak,settled,errors});
    const mujoco = {label:label+' / MuJoCo',color:purple,swatch:'purple',primary:detour,...curve(physical.comparisons.mujoco)};
    const docker = {label:label+' / 乐白 Docker',color:yellow,swatch:'yellow',dash:[2,3],...curve(physical.comparisons.docker)};
    comparisons = detour ? [mujoco,simulation,docker] : [simulation,mujoco,docker];
  }else if(detour){
    comparisons.unshift({label:'LM3 真机 / MuJoCo',swatch:'purple',primary:true});
  }
  $('comparisons').replaceChildren(...comparisons.map(row=>{
    const tr=document.createElement('tr'), label=document.createElement('th');
    if(row.primary)tr.className='primary-comparison';
    label.scope='row';label.className=row.swatch;label.textContent=row.label;tr.append(label);
    for(const value of [row.peak,row.rawPeak,row.settled]){
      const cell=document.createElement('td');cell.textContent=value===undefined?'待采集':distance(value);tr.append(cell);
    }
    return tr;
  }));
  $('clock').max=duration();
  $('mu-goal').textContent=distance(m.mujocoGoal);
  $('docker-goal').textContent=distance(m.dockerGoal);
  $('physical-goal-label').textContent=label+'末点误差';
  $('physical-goal').textContent=physical ? distance(physical.goalError) : '待采集';
  const complete = Boolean(physical);
  $('capture-summary').hidden=!complete;
  if(complete){
    const {peak,settled,errors}=physical.comparisons.mujoco;
    const [peakTime]=errors.reduce((largest,point)=>point[1]>largest[1]?point:largest);
    $('capture-peak').textContent=peak.toFixed(2)+' mm';
    $('capture-peak-time').textContent='出现在对齐后 '+peakTime.toFixed(3)+' 秒';
    $('capture-rmse').textContent=physical.motionRmse.toFixed(2)+' mm';
    $('capture-settled').textContent=distance(settled);
  }
  $('sampling').textContent=physical ? label+'最大采样间隔 '+physical.maxGapMs.toFixed(0)+' ms' : 'Docker 最大采样间隔 '+m.maxGapMs.toFixed(0)+' ms';
  const offsets='参考起动偏移 · Docker '+m.nativeOnsetOffsetMs.toFixed(0)+' ms';
  if(!detour){
    $('sample-note').textContent='曲线按起动时刻对齐，纵轴 mm · '+offsets+(physical ? ' · '+label+' '+physical.onsetOffsetMs.toFixed(0)+' ms' : '');
  }else{
    const point=siteDetour?'真机配置的工具 TCP':'模型夹尖';
    $('sample-note').textContent=physical ? label+'与 MuJoCo 统一到'+point+' · 起动对齐 · 纵轴 mm'
      : '主对比待采集 · 当前曲线为 MuJoCo / 乐白 Docker · 纵轴 mm';
  }
  $('docker-errors').hidden=!detour||!physical;
}

function setPhysical(report) {
  const valid = report?.schema==='lm3-public-replay-v1' && report.pathHash===data.pathHash &&
    report.planDuration===data.planDuration && report.frames?.length>1 &&
    [report.motionRmse,report.goalError,report.captureSeconds,report.sampleCount,report.maxGapMs].every(Number.isFinite) &&
    ['mujoco','docker'].every(name=>report.comparisons?.[name]?.errors?.length>1);
  if(!valid)throw new Error('Published replay and simulation do not match');
  physical=report;
  $('physical-legend').hidden=false;
  $('capture-status').textContent='8 秒轨迹完整采集 · 下发后记录 '+report.captureSeconds.toFixed(3)+
    ' 秒 · 原始采样 '+report.sampleCount+' 点 · 现场确认停稳';
  modelViews?.feedback.setPhysicalTrace(report.frames);
  updateComparisons();
  render();
}

const d = data?.dynamics ?? window.LM3_MODEL;
$(detour&&!siteDetour?'detour-link':runKind+'-link').setAttribute('aria-current','page');
if(siteDetour){
  document.title=$('demo-title').textContent=fullEight?'LM3 8 秒绕箱实验':quarter?'LM3 32 秒绕箱':eight?'LM3 8 秒绕箱中止记录':'LM3 实测起点绕箱';
  $('demo-subtitle').textContent='从实测真机姿态起步 · 现场空场，箱子仅存在于仿真场景。';
  $('error-title').textContent='统一 TCP 位置误差';
}
if(entry){
  document.title=$('demo-title').textContent='LM3 绕箱入位预演';
  $('demo-subtitle').textContent='从截图估算的姿态到原绕箱起点 · 仿真场景中的虚拟箱';
  $('mode').textContent='入位预演 · 仿真记录';
  $('capture-bar').hidden=$('phases').hidden=$('physical-summary').hidden=true;
  $('summary').classList.add('entry-summary');
  $('plan-legend').innerHTML='<span class="green">入位参考</span><span class="yellow">虚拟箱</span>';
}
if(short){
  document.title=$('demo-title').textContent='LM3 实验室短平移';
  $('demo-subtitle').textContent='用真机静止关节姿态起步。MuJoCo 和乐白 Docker 执行同一条 MoveIt 轨迹。';
  $('phases').hidden=true;
  $('planner-kind').textContent='笛卡尔路径';
  $('plan-legend').innerHTML='<span class="green">MoveIt 短平移参考</span>';
  $('error-title').textContent='统一 TCP 位置误差';
  if(data){
    $('short-detail').hidden=false;
    $('motion-note').textContent='真机配置 TCP · 参考时长 '+data.planDuration.toFixed(2)+' s · 峰值速度 '+data.motion.maxTcpSpeedMmS.toFixed(2)+' mm/s';
  }
}
if(d){
  $('engine').textContent='MuJoCo '+d.version;
  $('model-meta').innerHTML=[
    ['重力',d.gravity[2]+' m/s²'],['物理步长',(1000*d.step).toFixed(0)+' ms'],
    ['重力补偿',d.gravityCompensation===1?'开启':String(d.gravityCompensation)],
    ['夹爪估算质量',d.gripperMass.toFixed(3)+' kg'],['位置 / 速度增益',d.positionGain+' / '+d.velocityGain],
    ['关节转子惯量',d.links[0].armature+' kg·m²'],['六段连杆总质量',d.links.reduce((sum,x)=>sum+x.mass,0).toFixed(3)+' kg']
  ].map(([key,value])=>'<span>'+key+' <b>'+value+'</b></span>').join('');
  $('parameters').innerHTML=d.links.map(l=>'<tr>'+[
    l.name,l.mass.toFixed(3),l.center.map(x=>x.toFixed(4)).join(' / '),
    l.inertia.map(x=>x.toFixed(6)).join(' / '),l.damping.toFixed(2),l.effort[1].toFixed(1)
  ].map(value=>'<td>'+value+'</td>').join('')+'</tr>').join('');
}
if(data){
  updateComparisons();
  $('show-docker-errors').addEventListener('change',errorPlot);
  $('clock').disabled=false;
  $('clock').addEventListener('input',event=>{
    cancelAnimationFrame(animation);playing=false;$('play').textContent='▶ 播放完整演示';
    phase=2;time=Number(event.target.value);render();
  });
  $('play').disabled=false;$('play').addEventListener('click',start);
  $('feedback-source').disabled=false;
  $('feedback-source').addEventListener('change',render);
  fetch('./experiment-results.json')
    .then(response=>{
      if(!response.ok)throw new Error('Published replay unavailable');
      return response.json();
    })
    .then(setPhysical)
    .catch(()=>{
      $('capture-status').textContent='';
      $('capture-error').textContent='实验结果加载失败，请刷新网页。';
      $('capture-error').hidden=false;
    });
  if (window.LM3WebModel) {
    LM3WebModel.createModelViews($('planner'),$('execution'),data).then(views=>{
      modelViews=views;
      if(physical)modelViews.feedback.setPhysicalTrace(physical.frames);
      $('model-status').textContent='VR4Arm 三维模型';
      render();
    }).catch(error=>{
      $('model-status').textContent='三维显示不可用 · 关节示意';
      console.warn('LM3 model view could not be initialized',error);
    });
  }
}else {
  $('status').textContent=entry?'等待入位预演的仿真记录。':short?'等待实验室短平移的仿真记录。':siteDetour?'等待实测起点的绕箱仿真记录。':'运行数据尚未生成。请先运行 run-demo.ps1。';
}
window.addEventListener('resize',render);
render();
