import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import vrFixture from '../../schemas/fixtures/vr-frame-valid.json';
import type {ClientControlMessage, RobotStateMessage, VRFrame} from '../src/protocol/messages';
import {TeleopSocket} from '../src/transport/teleopSocket';

class FakeSocket {
  readyState: number = WebSocket.CONNECTING;
  readonly sent: string[] = [];
  onopen: (() => void) | null = null;
  onclose: (() => void) | null = null;
  onmessage: ((event: {data: unknown}) => void) | null = null;
  onerror: (() => void) | null = null;

  send(value: string): void {
    this.sent.push(value);
  }

  close(): void {
    this.readyState = WebSocket.CLOSED;
  }

  open(): void {
    this.readyState = WebSocket.OPEN;
    this.onopen?.();
  }

  closeFromServer(): void {
    this.readyState = WebSocket.CLOSED;
    this.onclose?.();
  }

  message(data: unknown): void {
    this.onmessage?.({data});
  }
}

function setup(onRobotState: (state: RobotStateMessage) => void = () => {}) {
  const sockets: FakeSocket[] = [];
  const client = new TeleopSocket('wss://test', onRobotState, () => {
    const socket = new FakeSocket();
    sockets.push(socket);
    return socket as unknown as WebSocket;
  });
  return {client, sockets};
}

const validFrame = () => vrFixture as VRFrame;
const control = (type: ClientControlMessage['type']): ClientControlMessage => ({
  v: 1,
  type,
  request_id: `request-${type}`,
});

beforeEach(() => vi.useFakeTimers());
afterEach(() => {
  vi.clearAllTimers();
  vi.useRealTimers();
});

describe('TeleopSocket', () => {
  it('reports every open and close transition to lifecycle observers', () => {
    const states: boolean[] = [];
    const sockets: FakeSocket[] = [];
    const client = new TeleopSocket(
      'wss://test',
      () => {},
      () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket as unknown as WebSocket;
      },
      (connected) => states.push(connected),
    );

    client.connect();
    sockets[0].open();
    sockets[0].closeFromServer();

    expect(states).toEqual([true, false]);
  });

  it('drops frames before open and sends only hello when opened', () => {
    const {client, sockets} = setup();
    client.connect();
    client.sendFrame(validFrame());
    expect(sockets[0].sent).toEqual([]);

    sockets[0].open();
    const openedMessages = sockets[0].sent.map((value) => JSON.parse(value) as {type: string});
    expect(openedMessages).toHaveLength(1);
    expect(openedMessages[0].type).toBe('hello');
    expect(openedMessages.some(({type}) => type === 'arm_request')).toBe(false);
  });

  it('sends current frames and explicit controls only while open', () => {
    const {client, sockets} = setup();
    client.connect();
    sockets[0].open();

    client.sendFrame(validFrame());
    client.sendControl(control('arm_request'));
    expect(sockets[0].sent.map((value) => JSON.parse(value).type)).toEqual([
      'hello',
      'vr_frame',
      'arm_request',
    ]);

    sockets[0].readyState = WebSocket.CLOSING;
    client.sendFrame(validFrame());
    client.sendControl(control('disarm'));
    expect(sockets[0].sent).toHaveLength(3);
  });

  it('delivers only guard-approved robot state messages', () => {
    const states: RobotStateMessage[] = [];
    const {client, sockets} = setup((state) => states.push(state));
    client.connect();
    sockets[0].open();

    sockets[0].message('{not json');
    sockets[0].message(JSON.stringify({...robotFixture, extra: true}));
    sockets[0].message(JSON.stringify({...robotFixture, mode: 'CONNECTED'}));
    sockets[0].message(JSON.stringify(robotFixture));

    expect(states).toEqual([robotFixture]);
  });

  it('reconnects exponentially with a two-second cap and never auto-arms', () => {
    const {client, sockets} = setup();
    client.connect();

    const expectedDelays = [250, 500, 1000, 2000, 2000];
    expectedDelays.forEach((delay, index) => {
      sockets.at(-1)?.closeFromServer();
      vi.advanceTimersByTime(delay - 1);
      expect(sockets).toHaveLength(index + 1);
      vi.advanceTimersByTime(1);
    });

    expect(sockets).toHaveLength(6);
    expect(
      sockets.flatMap((socket) => socket.sent).some((value) => JSON.parse(value).type === 'arm_request'),
    ).toBe(false);
  });

  it('does not create duplicate reconnect timers', () => {
    const {client, sockets} = setup();
    client.connect();
    sockets[0].closeFromServer();
    sockets[0].closeFromServer();

    expect(vi.getTimerCount()).toBe(1);
    vi.advanceTimersByTime(250);
    expect(sockets).toHaveLength(2);
    expect(vi.getTimerCount()).toBe(0);
  });

  it('ignores stale events from an older socket generation', () => {
    const states: RobotStateMessage[] = [];
    const {client, sockets} = setup((state) => states.push(state));
    client.connect();
    const stale = sockets[0];
    stale.closeFromServer();
    vi.advanceTimersByTime(250);
    const current = sockets[1];

    stale.open();
    stale.message(JSON.stringify(robotFixture));
    stale.closeFromServer();
    expect(stale.sent).toEqual([]);
    expect(states).toEqual([]);
    expect(vi.getTimerCount()).toBe(0);

    current.open();
    current.message(JSON.stringify(robotFixture));
    expect(current.sent.map((value) => JSON.parse(value).type)).toEqual(['hello']);
    expect(states).toEqual([robotFixture]);
  });

  it('close cancels reconnect and permanently closes the client', () => {
    const {client, sockets} = setup();
    client.connect();
    sockets[0].closeFromServer();
    expect(vi.getTimerCount()).toBe(1);

    client.close();
    expect(vi.getTimerCount()).toBe(0);
    vi.advanceTimersByTime(10_000);
    client.connect();
    expect(sockets).toHaveLength(1);
  });
});
