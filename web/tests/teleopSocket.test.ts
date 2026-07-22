import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import robotFixture from '../../schemas/fixtures/robot-state-valid.json';
import vrFixture from '../../schemas/fixtures/vr-frame-valid.json';
import type {
  ClientControlMessage,
  FaultResetResultMessage,
  HomeResultMessage,
  RobotStateMessage,
  VRFrame,
} from '../src/protocol/messages';
import {TeleopSocket, type TeleopConnectionStatus} from '../src/transport/teleopSocket';

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

function setup(
  onRobotState: (state: RobotStateMessage) => void = () => {},
  onConnectionChange: (status: TeleopConnectionStatus) => void = () => {},
) {
  const sockets: FakeSocket[] = [];
  const client = new TeleopSocket('wss://test', onRobotState, () => {
    const socket = new FakeSocket();
    sockets.push(socket);
    return socket as unknown as WebSocket;
  }, onConnectionChange);
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
  it('reports connected, disconnected, then reconnecting for an ordinary close', () => {
    const states: TeleopConnectionStatus[] = [];
    const sockets: FakeSocket[] = [];
    const client = new TeleopSocket(
      'wss://test',
      () => {},
      () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket as unknown as WebSocket;
      },
      (status) => states.push(status),
    );

    client.connect();
    sockets[0].open();
    sockets[0].closeFromServer();

    expect(states).toEqual([
      {state: 'connected'},
      {state: 'disconnected'},
      {state: 'reconnecting'},
    ]);
  });

  it('reports unreachable then reconnecting when socket construction fails', () => {
    const states: TeleopConnectionStatus[] = [];
    const client = new TeleopSocket(
      'wss://test',
      () => {},
      () => {
        throw new Error('offline');
      },
      (status) => states.push(status),
    );

    client.connect();

    expect(states).toEqual([{state: 'unreachable'}, {state: 'reconnecting'}]);
    expect(vi.getTimerCount()).toBe(1);
  });

  it('preserves occupied state across close and retries at 2 then 4 then 5 seconds', () => {
    const states: TeleopConnectionStatus[] = [];
    const {client, sockets} = setup(() => {}, (status) => states.push(status));
    const rejection = JSON.stringify({
      v: 1,
      type: 'connection_rejected',
      reason: 'controller_occupied',
      message: '已有控制页面占用，请关闭电脑端网页后重试。',
    });
    const rejectAndClose = (socket: FakeSocket) => {
      socket.message(rejection);
      socket.closeFromServer();
    };

    client.connect();
    sockets[0].open();
    rejectAndClose(sockets[0]);

    expect(states.at(-1)).toEqual({
      state: 'occupied',
      message: '已有控制页面占用，请关闭电脑端网页后重试。',
    });
    vi.advanceTimersByTime(1_999);
    expect(sockets).toHaveLength(1);
    vi.advanceTimersByTime(1);
    expect(sockets).toHaveLength(2);

    sockets[1].open();
    rejectAndClose(sockets[1]);
    vi.advanceTimersByTime(3_999);
    expect(sockets).toHaveLength(2);
    vi.advanceTimersByTime(1);
    expect(sockets).toHaveLength(3);

    sockets[2].open();
    rejectAndClose(sockets[2]);
    vi.advanceTimersByTime(4_999);
    expect(sockets).toHaveLength(3);
    vi.advanceTimersByTime(1);
    expect(sockets).toHaveLength(4);

    sockets[3].open();
    expect(states.at(-1)).toEqual({state: 'connected'});
    expect(sockets[3].sent.map((value) => JSON.parse(value).type)).toEqual(['hello']);
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

  it('delivers protocol-valid arm acknowledgements and rejections', () => {
    const feedback: unknown[] = [];
    const sockets: FakeSocket[] = [];
    const client = new TeleopSocket(
      'wss://test',
      () => {},
      () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket as unknown as WebSocket;
      },
      () => {},
      (message) => feedback.push(message),
    );
    client.connect();
    sockets[0].open();

    sockets[0].message(JSON.stringify({v: 1, type: 'arm_ack', request_id: 'arm-1'}));
    sockets[0].message(JSON.stringify({
      v: 1,
      type: 'arm_rejected',
      request_id: 'arm-2',
      message: '请先松开手柄抓握键，再请求使能。',
    }));
    sockets[0].message(JSON.stringify({v: 1, type: 'arm_ack', request_id: ''}));

    expect(feedback).toEqual([
      {v: 1, type: 'arm_ack', request_id: 'arm-1'},
      {
        v: 1,
        type: 'arm_rejected',
        request_id: 'arm-2',
        message: '请先松开手柄抓握键，再请求使能。',
      },
    ]);
  });

  it('delivers only valid fault reset results in arrival order without disturbing other channels', () => {
    const results: FaultResetResultMessage[] = [];
    const states: RobotStateMessage[] = [];
    const connectionStates: TeleopConnectionStatus[] = [];
    const armFeedback: unknown[] = [];
    const sockets: FakeSocket[] = [];
    const client = new TeleopSocket(
      'wss://test',
      (state) => states.push(state),
      () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket as unknown as WebSocket;
      },
      (status) => connectionStates.push(status),
      (message) => armFeedback.push(message),
      (message) => results.push(message),
    );
    client.connect();
    sockets[0].open();

    const accepted = {
      v: 1,
      type: 'fault_reset_result',
      request_id: 'r1',
      accepted: true,
      mode: 'DISARMED',
    };
    const rejected = {
      v: 1,
      type: 'fault_reset_result',
      request_id: 'r2',
      accepted: false,
      reason: 'unrecoverable_fault',
      message: '该故障无法在线复位，请重启后端并重新检查。',
    };

    sockets[0].message(JSON.stringify(accepted));
    sockets[0].message(JSON.stringify({...accepted, extra: true}));
    sockets[0].message(JSON.stringify({...accepted, mode: 'READY'}));
    sockets[0].message(JSON.stringify({...accepted, accepted: false}));
    sockets[0].message(JSON.stringify({...rejected, reason: 'unknown'}));
    sockets[0].message(JSON.stringify({...rejected, accepted: true}));
    sockets[0].message(JSON.stringify(rejected));
    sockets[0].message(JSON.stringify(robotFixture));
    sockets[0].message(JSON.stringify({v: 1, type: 'arm_ack', request_id: 'arm-1'}));

    expect(results).toEqual([accepted, rejected]);
    expect(states).toEqual([robotFixture]);
    expect(armFeedback).toEqual([{v: 1, type: 'arm_ack', request_id: 'arm-1'}]);
    expect(connectionStates).toEqual([{state: 'connected'}]);
  });

  it('delivers only valid Home results through the Home callback', () => {
    const homeResults: HomeResultMessage[] = [];
    const faultResults: FaultResetResultMessage[] = [];
    const sockets: FakeSocket[] = [];
    const client = new TeleopSocket(
      'wss://test',
      () => {},
      () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket as unknown as WebSocket;
      },
      () => {},
      () => {},
      (message) => faultResults.push(message),
      (message) => homeResults.push(message),
    );
    client.connect();
    sockets[0].open();

    const accepted = {
      v: 1,
      type: 'home_result',
      request_id: 'home-1',
      accepted: true,
      mode: 'DISARMED',
    };
    const rejected = {
      v: 1,
      type: 'home_result',
      request_id: 'home-2',
      accepted: false,
      reason: 'grip_pressed',
      message: '请先松开手柄抓握键，再请求 Home。',
    };

    sockets[0].message(JSON.stringify(accepted));
    sockets[0].message(JSON.stringify({...accepted, extra: true}));
    sockets[0].message(JSON.stringify(rejected));

    expect(homeResults).toEqual([accepted, rejected]);
    expect(faultResults).toEqual([]);
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
