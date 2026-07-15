import type {UserConfig} from 'vite';
import {expect, it} from 'vitest';
import config from '../vite.config';

it('proxies same-origin WebSockets to the plaintext local backend', () => {
  const proxy = (config as UserConfig).server?.proxy?.['/ws'];

  expect(proxy).toEqual({
    target: 'ws://127.0.0.1:8000',
    ws: true,
  });
});
