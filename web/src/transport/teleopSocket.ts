import {
  isArmFeedbackMessage,
  isConnectionRejectedMessage,
  isDiagnosticsMessage,
  isFaultResetResultMessage,
  isHomeResultMessage,
  isRobotStateMessage,
  PROTOCOL_VERSION,
  type ClientControlMessage,
  type ArmFeedbackMessage,
  type DiagnosticsMessage,
  type FaultResetResultMessage,
  type HomeResultMessage,
  type RobotStateMessage,
  type VRFrame,
} from '../protocol/messages';

export type SocketFactory = (url: string) => WebSocket;

export type TeleopConnectionStatus =
  | {state: 'connected'}
  | {state: 'occupied'; message: string}
  | {state: 'unreachable'}
  | {state: 'disconnected'}
  | {state: 'reconnecting'};

const INITIAL_RECONNECT_DELAY_MS = 250;
const MAX_RECONNECT_DELAY_MS = 2_000;
const OCCUPIED_INITIAL_RECONNECT_DELAY_MS = 2_000;
const OCCUPIED_MAX_RECONNECT_DELAY_MS = 5_000;
const OPEN_READY_STATE = 1;
const CONNECTING_READY_STATE = 0;

export class TeleopSocket {
  private socket: WebSocket | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private occupied = false;
  private reconnectDelayMs = INITIAL_RECONNECT_DELAY_MS;
  private occupiedReconnectDelayMs = OCCUPIED_INITIAL_RECONNECT_DELAY_MS;
  private explicitClose = false;
  private generation = 0;

  constructor(
    private readonly url: string,
    private readonly onRobotState: (state: RobotStateMessage) => void,
    private readonly socketFactory: SocketFactory = (socketUrl) => new WebSocket(socketUrl),
    private readonly onConnectionChange: (status: TeleopConnectionStatus) => void = () => {},
    private readonly onArmFeedback: (message: ArmFeedbackMessage) => void = () => {},
    private readonly onFaultResetResult: (message: FaultResetResultMessage) => void = () => {},
    private readonly onHomeResult: (message: HomeResultMessage) => void = () => {},
    private readonly onDiagnostics: (message: DiagnosticsMessage) => void = () => {},
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
    if (socket !== null) this.onConnectionChange({state: 'disconnected'});
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
      this.onConnectionChange({state: 'unreachable'});
      this.onConnectionChange({state: 'reconnecting'});
      this.scheduleReconnect(generation);
      return;
    }
    this.socket = socket;

    socket.onopen = () => {
      if (!this.isCurrent(socket, generation)) return;
      this.occupied = false;
      this.reconnectDelayMs = INITIAL_RECONNECT_DELAY_MS;
      this.onConnectionChange({state: 'connected'});
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
        if (isConnectionRejectedMessage(message)) {
          this.occupied = true;
          this.reconnectDelayMs = this.occupiedReconnectDelayMs;
          this.onConnectionChange({state: 'occupied', message: message.message});
        } else if (isRobotStateMessage(message)) {
          this.occupiedReconnectDelayMs = OCCUPIED_INITIAL_RECONNECT_DELAY_MS;
          this.onRobotState(message);
        } else if (isArmFeedbackMessage(message)) this.onArmFeedback(message);
        else if (isFaultResetResultMessage(message)) this.onFaultResetResult(message);
        else if (isHomeResultMessage(message)) this.onHomeResult(message);
        else if (isDiagnosticsMessage(message)) this.onDiagnostics(message);
      } catch {
        // Malformed or unsupported messages are ignored at the transport boundary.
      }
    };

    socket.onclose = () => {
      if (!this.isCurrent(socket, generation)) return;
      this.socket = null;
      if (!this.occupied) {
        this.onConnectionChange({state: 'disconnected'});
        this.onConnectionChange({state: 'reconnecting'});
      }
      this.scheduleReconnect(generation);
    };
  }

  private isCurrent(socket: WebSocket, generation: number): boolean {
    return !this.explicitClose && this.socket === socket && this.generation === generation;
  }

  private scheduleReconnect(generation: number): void {
    if (this.explicitClose || this.generation !== generation || this.reconnectTimer !== null) return;
    const delay = this.reconnectDelayMs;
    const maxDelay = this.occupied ? OCCUPIED_MAX_RECONNECT_DELAY_MS : MAX_RECONNECT_DELAY_MS;
    this.reconnectDelayMs = Math.min(this.reconnectDelayMs * 2, maxDelay);
    if (this.occupied) this.occupiedReconnectDelayMs = this.reconnectDelayMs;
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
