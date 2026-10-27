// @vitest-environment jsdom
import {it,expect} from 'vitest';
import {Hud} from '../src/ui/hud';
import {HoldControlPanel} from '../src/ui/holdControlPanel';
import {isRobotStateMessage,type RobotStateMessage,type VRFrame} from '../src/protocol/messages';

const state={v:1,type:'robot_state',server_mono_ns:1,mode:'ARMED',robot_state:'IDLE',
 actual_tcp:{p:[-.4,-.1,.2],q:[0,0,0,1]},actual_q:[0,0,0,0,0,0],gripper:.5,backend:'LEBAI_DOCKER',
 manual_input_ready:true,finite_position:{policy:'finite_position_docker',phase:'ready',generation:1,
 entered_steps:0,motion_id:null,reason:null,hardware_verified:false,vr_enabled:true,pitch_enabled:true}};

it('recognizes the official Docker runtime without treating it as the physical robot',()=>{
 expect(isRobotStateMessage(state)).toBe(true);
 const root=document.createElement('div'),hud=new Hud(root);
 hud.setRuntimeIdentity('LEBAI_DOCKER','control');
 expect(root.querySelector('.simulator-label')?.textContent).toContain('Docker');
 expect(root.textContent).toContain('本地虚拟夹爪');
 expect(hud.rehearsalBanner.textContent).toContain('仿真');
 expect(hud.rehearsalBanner.textContent).not.toContain('真机控制模式');
});

it('sends Docker pitch through finite inputs and releases without a legacy grip target',()=>{
 const root=document.createElement('div');document.body.append(root);const intents:unknown[][]=[];
 const panel=new HoldControlPanel(root,()=>{},()=>0,()=>{},()=>{},(...args)=>intents.push(args));
 try{
  panel.update(state as RobotStateMessage,true,0);
  const button=root.querySelector<HTMLButtonElement>('[data-hold="pitch+"]')!;
  expect(button.disabled).toBe(false);button.setPointerCapture=()=>{};button.hasPointerCapture=()=>false;
  const event=(type:string)=>{const e=new Event(type,{bubbles:true});Object.assign(e,{pointerId:7,button:0});button.dispatchEvent(e);};
  event('pointerdown');const frame:VRFrame={v:1,type:'vr_frame',session_id:'desktop',seq:1,
   client_mono_ms:0,tracking_valid:true,visibility:'visible',right:{p:[0,0,0],q:[0,0,0,1],grip:false,trigger:0}};
  expect(panel.applyFrame(frame).right.grip).toBe(false);
  expect(intents.at(-1)).toEqual([[0,0,0],'held',undefined,1]);
  event('pointerup');expect(intents.at(-1)?.[1]).toBe('release');
 }finally{panel.dispose();root.remove();}
});

it('keeps live Docker direction input through a short display delay, then releases at its separate display deadline',()=>{
 let now=0;const root=document.createElement('div');document.body.append(root);const intents:any[][]=[];
 const panel=new HoldControlPanel(root,()=>{},()=>now,()=>{},()=>{},(...args)=>intents.push(args));
 try{
  panel.update(state as RobotStateMessage,true,0);const b=root.querySelector<HTMLButtonElement>('[data-hold="z+"]')!;
  b.setPointerCapture=()=>{};b.hasPointerCapture=()=>false;
  const e=new Event('pointerdown',{bubbles:true});Object.assign(e,{pointerId:7,button:0});b.dispatchEvent(e);
  for(now=0;now<=520;now+=20){
   panel.applyFrame({v:1,type:'vr_frame',session_id:'desktop',seq:now,client_mono_ms:now,
    tracking_valid:true,visibility:'visible',right:{p:[0,0,0],q:[0,0,0,1],grip:false,trigger:0}});
   if(now===200)expect(intents.at(-1)?.[1]).toBe('held');
  }
  expect(intents.at(-1)?.[1]).toBe('release');
  expect(intents.some(x=>x[1]==='abort')).toBe(false);
 }finally{panel.dispose();root.remove();}
});
