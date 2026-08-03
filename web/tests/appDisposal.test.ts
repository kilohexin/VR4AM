import {expect, it, vi} from 'vitest';
import {disposeAppForUnload} from '../src/appDisposal';

it('stops and disposes rehearsal before XR, scene, and socket disposal on unload', () => {
  const events: string[] = [];
  const rehearsal = {
    requestStop: vi.fn(() => events.push('rehearsal-stop')),
    dispose: vi.fn(() => events.push('rehearsal-dispose')),
  };
  const xrController = {
    dispose: vi.fn(() => {
      events.push('disarm');
      return new Promise<void>(() => {});
    }),
  };
  const scene = {dispose: vi.fn(() => events.push('scene-dispose'))};
  const socket = {close: vi.fn(() => events.push('socket-close'))};

  disposeAppForUnload(rehearsal, xrController, scene, socket);

  expect(events).toEqual([
    'rehearsal-stop',
    'rehearsal-dispose',
    'disarm',
    'scene-dispose',
    'socket-close',
  ]);
  expect(rehearsal.requestStop).toHaveBeenCalledWith('page_unload');
});
