import {describe, expect, it} from 'vitest';
import {resolveTeleopSocketUrl} from '../src/transport/socketUrl';

describe('resolveTeleopSocketUrl', () => {
  it('returns a configured override unchanged', () => {
    const configured = 'wss://relay.example.test/custom';

    expect(resolveTeleopSocketUrl({protocol: 'https:', host: '192.168.1.10:5173'}, configured))
      .toBe(configured);
  });

  it('uses same-origin WSS for an HTTPS page', () => {
    expect(resolveTeleopSocketUrl({protocol: 'https:', host: '192.168.1.10:5173'}))
      .toBe('wss://192.168.1.10:5173/ws/v1/teleop');
  });

  it('uses same-origin WS for an HTTP page', () => {
    expect(resolveTeleopSocketUrl({protocol: 'http:', host: 'localhost:5173'}))
      .toBe('ws://localhost:5173/ws/v1/teleop');
  });
});
