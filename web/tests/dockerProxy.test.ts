import {describe,it,expect} from 'vitest';
import config from '../vite.docker.config';

describe('dedicated Docker headset transport',()=>{
 it('proxies the entire built application and WebSocket to the fixed simulator port',()=>{
  expect(config.server?.host).toBe('0.0.0.0');
  expect(config.server?.strictPort).toBe(true);
  expect(config.server?.https).toEqual({});
  const proxy=config.server?.proxy?.['/'];
  expect(proxy).toMatchObject({target:'http://127.0.0.1:8773',ws:true});
 });
});
