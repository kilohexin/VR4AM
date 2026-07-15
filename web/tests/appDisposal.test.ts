import {expect, it, vi} from 'vitest';
import {disposeAppForUnload} from '../src/appDisposal';

it('starts XR safety disposal before closing the socket on unload', () => {
  const events: string[] = [];
  const xrController = {
    dispose: vi.fn(() => {
      events.push('disarm');
      return new Promise<void>(() => {});
    }),
  };
  const scene = {dispose: vi.fn(() => events.push('scene-dispose'))};
  const socket = {close: vi.fn(() => events.push('socket-close'))};

  disposeAppForUnload(xrController, scene, socket);

  expect(events).toEqual(['disarm', 'scene-dispose', 'socket-close']);
});
