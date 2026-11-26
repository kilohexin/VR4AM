import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { STLLoader } from '/vendor/STLLoader.js';

const byId = id => document.getElementById(id);
const labels = {cup:'杯子', tray:'托盘', block:'障碍物'};
const stageNames = {
  idle:'等待开始', approach:'接近杯子', lower:'下降到抓取位', grip:'闭合夹爪',
  lift:'抬起杯子', carry:'搬运杯子', place:'下降到托盘', release:'释放杯子',
  retreat:'夹爪撤离', settle:'等待杯子停稳',
};
const eventNames = {
  task_started:'开始抓放', plan_accepted:'已采用 MoveIt 路径', bilateral_grasp:'两指稳定接触，开始抬升',
  stopped_for_scene_change:'路径被阻挡或目标已移动，刹停', next_grasp_candidate:'尝试下一个抓取姿态',
  cup_released:'夹爪已释放杯子', placed_and_settled:'杯子已在托盘内停稳',
  no_valid_path:'当前场景没有可执行路径', grasp_failed:'抓取失败',
  placement_failed:'放置失败', tracking_failed:'轨迹跟踪失败', grasp_lost:'夹持丢失，停止搬运',
};

let state, selected, drag, pendingDrag, sending = false, online = false;
let lastEventKey = '';
const objects = {};
const buttons = [...document.querySelectorAll('[data-object]')];

function showError(message = '') {
  byId('error').textContent = message;
  byId('error').hidden = !message;
}

async function request(path, body) {
  const res = await fetch(path, {
    method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body),
  });
  if (!res.ok) {
    const result = await res.json();
    throw new Error(typeof result.detail === 'string' ? result.detail : '无法执行当前操作');
  }
}

function updateControls() {
  const ready = state && ['idle','done','blocked','failed'].includes(state.status) && !state.held;
  byId('start').disabled = !online || !ready || !byId('item').value || !byId('destination').value;
  byId('reset').disabled = !online;
  for (const button of buttons) button.disabled = !online || (button.dataset.object==='cup' && state?.held);
}

function select(name) {
  selected = name;
  for (const button of buttons) button.setAttribute('aria-pressed',String(button.dataset.object === name));
  if (name === 'cup') byId('item').value = 'cup';
  if (name === 'tray') byId('destination').value = 'tray';
  updateControls();
}

function updatePositionLabel() {
  const pose = state?.poses[selected]?.[0];
  byId('selection').textContent = pose
    ? `已选中${labels[selected]} · X ${pose[0].toFixed(3)} m · Y ${pose[1].toFixed(3)} m`
    : '选中物体后也可用方向键移动。';
}

async function flushDrag() {
  if (sending) return;
  sending = true;
  try {
    while (pendingDrag) {
      const {name, x, y, active} = pendingDrag;
      pendingDrag = null;
      try {
        await request('/api/drag/'+name, {x, y, active});
        showError();
      } catch (error) {
        showError(error.message);
      }
    }
  } finally {
    sending = false;
  }
}

function move(name, x, y, active = false) {
  pendingDrag = {name, x:THREE.MathUtils.clamp(x,-.76,-.29), y:THREE.MathUtils.clamp(y,-.40,.40), active};
  void flushDrag();
}

for (const button of buttons) {
  button.setAttribute('aria-pressed','false');
  button.addEventListener('click', () => {select(button.dataset.object); updatePositionLabel();});
}
for (const id of ['item','destination']) byId(id).addEventListener('change',updateControls);
byId('start').addEventListener('click',async () => {
  byId('start').disabled = true;
  showError();
  try {
    await request('/api/pick', {item:byId('item').value, destination:byId('destination').value});
  } catch (error) {
    showError(error.message);
    updateControls();
  }
});
byId('reset').addEventListener('click',async () => {
  showError();
  try { await request('/api/reset',{}); } catch (error) { showError(error.message); }
});
document.addEventListener('keydown',event => {
  if (!online || !selected || !state || ['SELECT','INPUT'].includes(event.target.tagName) || (selected==='cup' && state.held)) return;
  const offset = {ArrowLeft:[0,-1],ArrowRight:[0,1],ArrowUp:[-1,0],ArrowDown:[1,0]}[event.key];
  if (!offset) return;
  event.preventDefault();
  const [x,y] = state.poses[selected][0];
  move(selected,x+.025*offset[0],y+.025*offset[1]);
});

function material(color, roughness = .72) {
  return new THREE.MeshStandardMaterial({color,roughness,metalness:.12});
}

function box(center,size,paint) {
  const item = new THREE.Mesh(new THREE.BoxGeometry(...size),paint);
  item.position.set(...center);
  return item;
}

function createCup(radius,height) {
  const item = new THREE.Group();
  const blue = material(0x69bff4,.25);
  const profile = [[.001,0],[radius,0],[radius,height],[radius-.004,height],[radius-.004,.006],[.001,.006]];
  const geometry = new THREE.LatheGeometry(profile.map(([r,z])=>new THREE.Vector2(r,z)),48);
  geometry.rotateX(Math.PI/2);
  const shell = new THREE.Mesh(geometry,blue);
  shell.material.side = THREE.DoubleSide;
  item.add(shell);
  const rim = new THREE.Mesh(new THREE.TorusGeometry(radius-.002,.0025,10,48),material(0xb5e7ff,.24));
  rim.position.z=height;
  item.add(rim);
  return item;
}

async function init() {
  const config = await fetch('/api/scene').then(response=>response.json());
  const host = byId('viewport');
  const sim = new THREE.Scene();
  sim.background = new THREE.Color(0x14232d);
  const camera = new THREE.PerspectiveCamera(43,1,.02,6);
  camera.up.set(0,0,1);
  const renderer = new THREE.WebGLRenderer({antialias:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
  renderer.domElement.setAttribute('aria-label','LM3 仿真可拖动桌面');
  host.append(renderer.domElement);
  const controls = new OrbitControls(camera,renderer.domElement);
  controls.minDistance=.48;
  controls.maxDistance=2.5;
  controls.maxPolarAngle=Math.PI*.49;
  controls.enableDamping=true;
  controls.target.set(-.31,0,.065);
  let top = false;
  function setCamera() {
    camera.position.set(...(top ? [-.31,0,1.32] : [-1.02,-.85,.82]));
    if (top) camera.position.y -= .001;
    controls.update();
    byId('camera').textContent=top?'透视视角':'俯视桌面';
  }
  setCamera();
  byId('camera').addEventListener('click',()=>{top=!top;setCamera();});
  sim.add(new THREE.HemisphereLight(0xe4f4ff,0x42616c,2.5));
  const light = new THREE.DirectionalLight(0xffeed8,2.4);
  light.position.set(-.9,-.65,1.7);
  sim.add(light,box(...config.table,material(0x334d5c)));
  const grid=new THREE.GridHelper(1.14,19,0x547685,0x3e5b69);
  grid.rotation.x=Math.PI/2;
  grid.position.set(-.25,0,config.tableTop+.0008);
  sim.add(grid);
  const metal = material(0xc6d6dc,.41);
  const loader=new STLLoader();
  for (const [index,name] of config.arm.entries()) {
    const group = objects[name] = new THREE.Group();
    sim.add(group);
    loader.load('/mesh/visual_'+(index?'link'+index:'base_link')+'.stl',
      geometry => group.add(new THREE.Mesh(geometry,metal)),
      undefined, () => showError('部分机械臂网格加载失败'));
  }
  const gripperReadout = byId('gripper');
  const gripperPaint = material(0x637883,.42);
  const gripperMeshes = new Map();
  for (const [name, part] of Object.entries(config.gripper)) {
    const group = objects[name] = new THREE.Group();
    sim.add(group);
    if (!gripperMeshes.has(part.mesh)) {
      gripperMeshes.set(part.mesh, loader.loadAsync('/mesh/'+part.mesh));
    }
    gripperMeshes.get(part.mesh).then(geometry => {
      const link = new THREE.Mesh(geometry,gripperPaint);
      link.position.set(...part.position);
      link.quaternion.set(...part.rotation);
      group.add(link);
    }).catch(() => showError('部分 LMG-90 夹爪网格加载失败'));
  }
  objects.cup = createCup(...config.cup);
  objects.tray = new THREE.Group();
  const trayPaint=material(0x67bda1);
  for (const part of config.tray) objects.tray.add(box(...part,trayPaint));
  objects.block = new THREE.Group();
  objects.block.add(box([0,0,config.block[2]/2],config.block,material(0xd78248)));
  for (const name of Object.keys(labels)) sim.add(objects[name]);
  for (const name of Object.keys(labels)) objects[name].traverse(node=>node.userData.objectName=name);
  const route = new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x77ffd0,depthTest:false}));
  route.renderOrder = 3;
  sim.add(route);
  let lastPath = '';
  const resize = new ResizeObserver(() => {
    const {width,height} = host.getBoundingClientRect();
    if (!width || !height) return;
    camera.aspect=width/height;
    camera.updateProjectionMatrix();
    renderer.setSize(width,height,false);
  });
  resize.observe(host);
  renderer.setAnimationLoop(() => {controls.update();renderer.render(sim,camera);});

  const ray = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const onTable = point => new THREE.Plane(new THREE.Vector3(0,0,1),-point);
  function cast(event) {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.set((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1);
    ray.setFromCamera(pointer,camera);
  }
  const surface = renderer.domElement;
  surface.addEventListener('pointerdown',event => {
    if (!online || event.button!==0) return;
    cast(event);
    const hit=ray.intersectObjects(Object.keys(labels).map(name=>objects[name]),true)[0];
    const name=hit?.object.userData.objectName;
    if (!name || (name==='cup' && state?.held)) return;
    const point=ray.ray.intersectPlane(onTable(config.tableTop),new THREE.Vector3());
    if (!point) return;
    select(name);
    updatePositionLabel();
    drag = {name,id:event.pointerId, offset:objects[name].position.clone().sub(point),last:null};
    controls.enabled=false;
    surface.setPointerCapture(event.pointerId);
    surface.classList.add('dragging');
    event.stopImmediatePropagation();
  },true);
  surface.addEventListener('pointermove',event => {
    if (!drag || event.pointerId!==drag.id) return;
    cast(event);
    const point=ray.ray.intersectPlane(onTable(config.tableTop),new THREE.Vector3());
    if (!point) return;
    drag.last=point.add(drag.offset);
    move(drag.name,drag.last.x,drag.last.y,true);
  });
  function release(event) {
    if (!drag || event.pointerId!==drag.id) return;
    if (drag.last) move(drag.name,drag.last.x,drag.last.y);
    drag=null;
    controls.enabled=true;
    surface.classList.remove('dragging');
  }
  for (const type of ['pointerup','pointercancel','lostpointercapture']) surface.addEventListener(type,release);

  function connect() {
    const socket = new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/api/stream`);
    socket.addEventListener('close',()=>{
      online=false;
      byId('connection').textContent='连接中断，正在重连';
      updateControls();
      setTimeout(connect,1200);
    });
    socket.addEventListener('message',event=>{
      state=JSON.parse(event.data);
      if (!online) {
        online=true;
        byId('connection').textContent='MuJoCo · MoveIt 2 已连接';
      }
      for (const [name,[position,quaternion]] of Object.entries(state.poses)) {
        objects[name].position.set(...position);
        objects[name].quaternion.set(...quaternion);
      }
      const forces = state.fingerForces.map(force => force.toFixed(1)).join(' / ');
      gripperReadout.textContent=`LMG-90 · 开口 ${(state.gripperGap*1000).toFixed(0)} mm · 左 / 右接触 ${forces} N`;
      gripperReadout.hidden=false;
      const pathKey=JSON.stringify(state.path);
      if (lastPath!==pathKey) {
        lastPath=pathKey;
        route.geometry.dispose();
        route.geometry=new THREE.BufferGeometry().setFromPoints(state.path.map(point=>new THREE.Vector3(...point)));
      }
      const heading = {braking:'刹停待重规划', blocked:'路径受阻', failed:'任务失败',done:'放置完成'}[state.status];
      byId('stage').textContent = heading || stageNames[state.stage];
      byId('detail').textContent = state.detail;
      byId('replans').textContent = `重规划 ${state.replans} 次`;
      const key=JSON.stringify(state.events);
      if (key!==lastEventKey) {
        lastEventKey=key;
        byId('events').replaceChildren(...state.events.slice(-5).reverse().map(({t,event})=>{
          const row=document.createElement('li');
          row.textContent=`${t.toFixed(2)} s · ${eventNames[event] || event}`;
          return row;
        }));
      }
      updateControls();
      updatePositionLabel();
    });
  }
  connect();
}

init().catch(error => {
  byId('connection').textContent = '工作台加载失败';
  showError(error.message);
});
