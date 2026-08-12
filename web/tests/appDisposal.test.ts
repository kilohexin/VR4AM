// @vitest-environment jsdom

import {expect, it, vi} from 'vitest';
import {disposeAppForUnload, installRehearsalVisibilityStop} from '../src/appDisposal';

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

it('requests immediate fail-closed rehearsal cleanup only when the document becomes hidden', () => {
  Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'visible'});
  const requestStop = vi.fn();
  const dispose = installRehearsalVisibilityStop(document, {requestStop});

  document.dispatchEvent(new Event('visibilitychange'));
  expect(requestStop).not.toHaveBeenCalled();

  Object.defineProperty(document, 'visibilityState', {configurable: true, value: 'hidden'});
  document.dispatchEvent(new Event('visibilitychange'));
  expect(requestStop).toHaveBeenCalledOnce();
  expect(requestStop).toHaveBeenCalledWith('page_hidden');

  dispose();
  document.dispatchEvent(new Event('visibilitychange'));
  expect(requestStop).toHaveBeenCalledOnce();
});
