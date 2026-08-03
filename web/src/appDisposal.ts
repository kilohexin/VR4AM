export interface AsyncDisposable {
  dispose(): Promise<void>;
}

export interface Disposable {
  dispose(): void;
}

export interface Closeable {
  close(): void;
}

export interface RehearsalDisposable extends Disposable {
  requestStop(reason: string): void;
}

export function disposeAppForUnload(
  rehearsal: RehearsalDisposable,
  xrController: AsyncDisposable,
  scene: Disposable,
  socket: Closeable,
): void {
  rehearsal.requestStop('page_unload');
  rehearsal.dispose();
  void xrController.dispose();
  scene.dispose();
  socket.close();
}
