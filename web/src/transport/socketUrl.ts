type SocketLocation = Pick<Location, 'protocol' | 'host'>;

export function resolveTeleopSocketUrl(location: SocketLocation, configured?: string): string {
  if (configured) return configured;
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${protocol}//${location.host}/ws/v1/teleop`;
}
