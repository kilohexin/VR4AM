// @vitest-environment jsdom
// Actual app composition and controls; only WebGL and network are replaced.
import {afterEach,expect,it,vi} from 'vitest';
import fixture from '../../schemas/fixtures/robot-state-valid.json';
import type {VRFrame} from '../src/protocol/messages';

const driver=vi.hoisted(()=>({frame:null as null|((frame:VRFrame,source:'manual')=>void),startFails:false,stopFails:false}));
vi.mock('../src/scenes/simulationScene',async importOriginal=>{
 const actual=await importOriginal<typeof import('../src/scenes/simulationScene')>();
 return {...actual,SimulationScene:class {
  constructor(_root:HTMLElement,options:any){driver.frame=options.onFrame;}
  start(){} dispose(){} resetConnection(){} setArmSafetyState(){} setRuntimeSummary(){}
  setQuestControllerSupport(){} applyRobotState(){} resetCameraView(){}
  async startXR(){if(driver.startFails)throw Error('renderer unavailable');}
  async stopXR(){if(driver.stopFails)throw Error('renderer teardown failed');}
  updateXRPresentation(){} renderXR(){}
 }};
});

class MemorySocket {
 static current:MemorySocket;
 readyState=0;sent:any[]=[];onopen:(()=>void)|null=null;onclose:(()=>void)|null=null;
 onmessage:((event:{data:string})=>void)|null=null;
 constructor(){MemorySocket.current=this;}
 send(value:string){this.sent.push(JSON.parse(value));}
 close(){this.readyState=3;}
 receive(value:unknown){this.onmessage?.({data:JSON.stringify(value)});}
 open(){this.readyState=1;this.onopen?.();}
}
const originalXR=Object.getOwnPropertyDescriptor(navigator,'xr');
afterEach(()=>{window.dispatchEvent(new Event('pagehide'));vi.unstubAllGlobals();vi.restoreAllMocks();vi.resetModules();
 driver.startFails=driver.stopFails=false;driver.frame=null;document.body.innerHTML='';
 if(originalXR)Object.defineProperty(navigator,'xr',originalXR);else Reflect.deleteProperty(navigator,'xr');});

it.each(['unsupported','support-query','denied','reference-space','renderer','teardown'])(
 'restores desktop after VR %s failure only after stop acknowledgement and fresh preparation',async failure=>{
 let now=1;vi.spyOn(performance,'now').mockImplementation(()=>now);
 vi.stubGlobal('WebSocket',MemorySocket);
 driver.startFails=failure==='renderer';driver.stopFails=failure==='teardown';
 const session=Object.assign(new EventTarget(),{visibilityState:'visible',inputSources:[],
  requestReferenceSpace:async()=>{if(failure==='reference-space')throw Error('reference unavailable');return {};},
  end:async()=>{session.dispatchEvent(new Event('end'));}});
 Object.defineProperty(navigator,'xr',{configurable:true,value:{
  isSessionSupported:async()=>{if(failure==='support-query')throw Error('query failed');return failure!=='unsupported';},
  requestSession:async()=>{if(failure==='denied')throw Error('permission denied');return session;}}});
 document.body.innerHTML='<div id="app"></div>';
 await import('../src/main');const wire=MemorySocket.current;wire.open();
 let seq=0;
 function feedback(mode='READY',extra:Record<string,unknown>={}){wire.receive({...fixture,backend:'LEBAI_FAKE',mode,
  preparation_required:true,preparation_unlocked:false,manual_input_ready:false,
  finite_position:{policy:'finite_position_offline',phase:'ready',generation:1,entered_steps:0,
   motion_id:null,reason:null,hardware_verified:false,vr_enabled:true,pitch_enabled:true},...extra});}
 function frame(){now+=20;driver.frame!({v:1,type:'vr_frame',session_id:'manual',seq:++seq,client_mono_ms:performance.now(),
  tracking_valid:true,visibility:'visible',right:{p:[.3,0,.3],q:[0,0,0,1],grip:false,trigger:0}},'manual');}
 const button=(selector:string)=>document.querySelector<HTMLButtonElement>(selector)!;
 feedback();frame();
 button('.command-button--vr').click();
 if(failure==='teardown'){
  await vi.waitFor(()=>expect(button('.command-button--vr').dataset.state).toBe('active'));
  expect(button('[data-hold="z+"]').disabled).toBe(true);
  button('.command-button--vr').click();
 }
 await vi.waitFor(()=>expect(button('.command-button--vr').dataset.state).toBe('error'));
 const stopCount=wire.sent.filter(m=>m.type==='disarm').length;
 const stop=wire.sent.filter(m=>m.type==='disarm').at(-1);expect(stop).toBeDefined();
 feedback();frame();expect(button('.command-button--arm').disabled).toBe(true);
 expect(button('[data-hold="z+"]').disabled).toBe(true);
 wire.receive({v:1,type:'disarm_ack',request_id:stop.request_id});
 expect(button('.command-button--arm').disabled).toBe(true);
 feedback('ARMED',{manual_input_ready:true});frame();
 expect(button('[data-hold="z+"]').disabled).toBe(true);
 expect(button('.command-button--arm').disabled).toBe(true);
 feedback();frame();
 expect(button('.command-button--arm').disabled).toBe(false);
 button('.command-button--arm').click();const arm=wire.sent.at(-1);
 wire.receive({v:1,type:'arm_ack',request_id:arm.request_id});
 button('.command-button--prepare').click();const prepare=wire.sent.at(-1);
 expect(prepare.type).toBe('prepare_request');
 wire.receive({v:1,type:'prepare_ack',request_id:prepare.request_id});
 feedback('ARMED',{manual_input_ready:true});frame();
 const up=button('[data-hold="z+"]');expect(up.disabled).toBe(false);
 up.setPointerCapture=()=>{};up.hasPointerCapture=()=>false;
 const pointer=(type:string)=>{const event=new Event(type,{bubbles:true});Object.assign(event,{pointerId:7,button:0});up.dispatchEvent(event);};
 pointer('pointerdown');frame();
 expect(wire.sent.filter(m=>m.type==='finite_position_input').at(-1)).toMatchObject({kind:'held',vector:[0,0,1]});
 pointer('pointerup');expect(wire.sent.at(-1)).toMatchObject({type:'finite_position_input',kind:'release'});
 expect(wire.sent.filter(m=>m.type==='disarm')).toHaveLength(stopCount);
 feedback('FAULT',{fault:'stop_unverified'});frame();
 expect(up.disabled).toBe(true);expect(button('.command-button--arm').disabled).toBe(true);
},15000);
