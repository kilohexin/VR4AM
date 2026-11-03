import {describe,it,expect} from 'vitest';
import config from '../vite.docker.config';
import poseConfig from '../vite.docker-pose-test.config';

describe('dedicated Docker headset transport',()=>{
 it('proxies the entire built application and WebSocket to the fixed simulator port',()=>{
  expect(config.server?.host).toBe('0.0.0.0');
  expect(config.server?.strictPort).toBe(true);
  expect(config.server?.https).toEqual({});
  const proxy=config.server?.proxy?.['/'];
  expect(proxy).toMatchObject({target:'http://127.0.0.1:8773',ws:true});
 });
 it('keeps the pose test HTTPS and WebSocket on separate fixed ports',()=>{
  expect(poseConfig.server).toMatchObject({host:'0.0.0.0',port:8444,strictPort:true,https:{}});
  expect(poseConfig.server?.proxy?.['/']).toMatchObject({target:'http://127.0.0.1:8774',ws:true});
  expect(config.server?.proxy?.['/']).toMatchObject({target:'http://127.0.0.1:8773'});
 });
});
