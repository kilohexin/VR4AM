export interface AsyncDisposable {
  dispose(): Promise<void>;
}

export interface Disposable {
  dispose(): void;
}

export interface Closeable {
  close(): void;
}

export function disposeAppForUnload(
  xrController: AsyncDisposable,
  scene: Disposable,
  socket: Closeable,
): void {
  void xrController.dispose();
  scene.dispose();
  socket.close();
}
