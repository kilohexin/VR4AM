import type {UserConfig} from 'vite';
import {expect, it} from 'vitest';
import config from '../vite.config';

it('proxies same-origin WebSockets to the plaintext local backend', async () => {
  const resolved=typeof config==='function'?await config({mode:'development',command:'serve'}):config;
  const proxy = (resolved as UserConfig).server?.proxy?.['/ws'];

  expect(proxy).toEqual({
    target: 'ws://127.0.0.1:8000',
    ws: true,
  });
});
