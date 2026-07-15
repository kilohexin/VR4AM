import {
  isArmFeedbackMessage,
  isRobotStateMessage,
  PROTOCOL_VERSION,
  type ClientControlMessage,
  type ArmFeedbackMessage,
  type RobotStateMessage,
  type VRFrame,
} from '../protocol/messages';

export type SocketFactory = (url: string) => WebSocket;

const INITIAL_RECONNECT_DELAY_MS = 250;
const MAX_RECONNECT_DELAY_MS = 2_000;
const OPEN_READY_STATE = 1;
const CONNECTING_READY_STATE = 0;

export class TeleopSocket {
  private socket: WebSocket | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectDelayMs = INITIAL_RECONNECT_DELAY_MS;
  private explicitClose = false;
  private generation = 0;

  constructor(
    private readonly url: string,
    private readonly onRobotState: (state: RobotStateMessage) => void,
    private readonly socketFactory: SocketFactory = (socketUrl) => new WebSocket(socketUrl),
    private readonly onConnectionChange: (connected: boolean) => void = () => {},
    private readonly onArmFeedback: (message: ArmFeedbackMessage) => void = () => {},
  ) {}

  connect(): void {
    if (this.explicitClose || this.reconnectTimer !== null) return;
    if (
      this.socket !== null &&
      (this.socket.readyState === CONNECTING_READY_STATE || this.socket.readyState === OPEN_READY_STATE)
    ) {
      return;
    }
    this.openSocket();
  }

  sendFrame(frame: VRFrame): void {
    this.send(frame);
  }

  sendControl(message: ClientControlMessage): void {
    this.send(message);
  }

  close(): void {
    if (this.explicitClose) return;
    this.explicitClose = true;
    this.generation += 1;
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    const socket = this.socket;
    this.socket = null;
    if (socket !== null) this.onConnectionChange(false);
    socket?.close();
  }

  private openSocket(): void {
    if (this.explicitClose) return;
    const generation = ++this.generation;
    let socket: WebSocket;
    try {
      socket = this.socketFactory(this.url);
    } catch {
      this.socket = null;
      this.scheduleReconnect(generation);
      return;
    }
    this.socket = socket;

    socket.onopen = () => {
      if (!this.isCurrent(socket, generation)) return;
      this.reconnectDelayMs = INITIAL_RECONNECT_DELAY_MS;
      this.onConnectionChange(true);
      this.sendControl({
        v: PROTOCOL_VERSION,
        type: 'hello',
        request_id: `hello-${generation}`,
      });
    };

    socket.onmessage = (event) => {
      if (!this.isCurrent(socket, generation) || typeof event.data !== 'string') return;
      try {
        const message: unknown = JSON.parse(event.data);
        if (isRobotStateMessage(message)) this.onRobotState(message);
        else if (isArmFeedbackMessage(message)) this.onArmFeedback(message);
      } catch {
        // Malformed or unsupported messages are ignored at the transport boundary.
      }
    };

    socket.onclose = () => {
      if (!this.isCurrent(socket, generation)) return;
      this.socket = null;
      this.onConnectionChange(false);
      this.scheduleReconnect(generation);
    };
  }

  private isCurrent(socket: WebSocket, generation: number): boolean {
    return !this.explicitClose && this.socket === socket && this.generation === generation;
  }

  private scheduleReconnect(generation: number): void {
    if (this.explicitClose || this.generation !== generation || this.reconnectTimer !== null) return;
    const delay = this.reconnectDelayMs;
    this.reconnectDelayMs = Math.min(this.reconnectDelayMs * 2, MAX_RECONNECT_DELAY_MS);
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      if (this.explicitClose || this.generation !== generation) return;
      this.openSocket();
    }, delay);
  }

  private send(message: VRFrame | ClientControlMessage): void {
    if (this.socket?.readyState !== OPEN_READY_STATE) return;
    this.socket.send(JSON.stringify(message));
  }
}
