"""Local web server for the simulation-only LM3 workbench."""

import asyncio
from contextlib import asynccontextmanager
from time import monotonic

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from physics import Physics
from scene import (ARM, BLOCK_SIZE, CUP_HEIGHT, CUP_RADIUS, GRIPPER, HERE,
    TABLE_CENTER, TABLE_SIZE, TABLE_TOP, model_directory, tray_boxes, visual_gripper)
from task import MoveIt, Task, TICK

ARCHIVE = model_directory()
GRIPPER_VISUALS = visual_gripper(ARCHIVE)
VISUAL_FILES = {'visual_base_link.stl', *(f'visual_link{i}.stl' for i in range(1,7)),
                *(part['mesh'] for part in GRIPPER_VISUALS.values())}
THREE = HERE.parents[2]/'web/node_modules/three'


class Drag(BaseModel):
    x: float = Field(ge=-.76, le=-.29, allow_inf_nan=False)
    y: float = Field(ge=-.40, le=.40, allow_inf_nan=False)
    active: bool = False


class Pick(BaseModel):
    item: str = 'cup'
    destination: str = 'tray'


def get_state(task):
    physics = task.physics
    return dict(time=float(physics.data.time),stage=task.stage,status=task.status,detail=task.detail,
                replans=task.replans,held=physics.relative is not None,path=task.preview,events=task.events,
                gripperGap=physics.jaw_gap(), fingerForces=physics.finger_forces(),
                poses={key:physics.pose(key) for key in (*ARM,*GRIPPER,'cup','tray','block')})


async def simulate(app):
    deadline = monotonic()
    task = app.state.task
    while True:
        if task.editing and monotonic()-app.state.drag_activity > 1:
            task.editing = False
        task.tick()
        task.physics.step(round(TICK/task.physics.model.opt.timestep))
        deadline += TICK
        now = monotonic()
        if deadline < now-TICK:
            deadline = now
        await asyncio.sleep(max(0,deadline-now))


@asynccontextmanager
async def lifespan(app):
    planner = await asyncio.to_thread(MoveIt,ARCHIVE,HERE,HERE/'moveit-workbench.log')
    app.state.task = Task(Physics(ARCHIVE),planner)
    app.state.drag_activity = 0.
    update = asyncio.create_task(simulate(app))
    try:
        yield
    finally:
        update.cancel()
        await asyncio.gather(update,return_exceptions=True)
        if app.state.task.future is not None:
            await asyncio.gather(app.state.task.future,return_exceptions=True)
        await asyncio.to_thread(planner.close)


app = FastAPI(lifespan=lifespan)
app.mount('/ui', StaticFiles(directory=HERE/'web'), name='ui')


@app.get('/')
def home():
    return FileResponse(HERE/'web/index.html')


@app.get('/vendor/{name:path}')
def vendor(name: str):
    files = {
        'three.module.js': THREE/'build/three.module.js',
        'three.core.js': THREE/'build/three.core.js',
        'STLLoader.js': THREE/'examples/jsm/loaders/STLLoader.js',
        'OrbitControls.js': THREE/'examples/jsm/controls/OrbitControls.js',
    }
    if name not in files or not files[name].is_file():
        raise HTTPException(404,'Run npm ci --prefix web from the VR4AM repository root.')
    return FileResponse(files[name],media_type='text/javascript')


@app.get('/mesh/{name}')
def mesh(name: str):
    if name not in VISUAL_FILES:
        raise HTTPException(404)
    return FileResponse(ARCHIVE/'assets'/name)


@app.get('/api/scene')
def scene():
    return dict(arm=ARM,table=[TABLE_CENTER,TABLE_SIZE],tableTop=TABLE_TOP,
                cup=[CUP_RADIUS,CUP_HEIGHT],tray=tray_boxes([0,0,0]),block=BLOCK_SIZE,
                gripper=GRIPPER_VISUALS)


@app.get('/api/state')
async def state():
    return get_state(app.state.task)


@app.post('/api/pick')
async def pick(request: Pick):
    if request.item != 'cup' or request.destination != 'tray':
        raise HTTPException(422,'请选择杯子和托盘')
    if not app.state.task.start():
        raise HTTPException(409,'当前任务仍在运行，或夹爪仍持有杯子')
    return {'ok':True}


@app.post('/api/reset')
async def reset():
    app.state.task.reset()
    return {'ok':True}


@app.post('/api/drag/{name}')
async def drag(name: str, point: Drag):
    if name not in ('cup','tray','block'):
        raise HTTPException(404)
    task = app.state.task
    app.state.drag_activity = monotonic()
    if not task.drag(name, point.x, point.y, point.active):
        raise HTTPException(409,'当前位置与其他物体重叠，或杯子正在被夹持')
    return {'ok':True}


@app.websocket('/api/stream')
async def stream(socket: WebSocket):
    origin = socket.headers.get('origin','')
    if origin not in ('',f"http://{socket.headers.get('host','')}"):
        await socket.close(1008)
        return
    await socket.accept()
    try:
        while True:
            await asyncio.wait_for(socket.send_json(get_state(app.state.task)),timeout=2)
            try:
                message = await asyncio.wait_for(socket.receive(), timeout=.05)
                if message['type']=='websocket.disconnect':
                    return
            except TimeoutError:
                pass
    except (WebSocketDisconnect,TimeoutError,OSError,RuntimeError):
        return
