# Single Controller Occupancy Feedback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保持单一 WebSocket 控制端，同时让 PC 与 Quest 明确区分“控制端被占用”、后端不可达、连接中断和重连状态。

**Architecture:** FastAPI 继续以现有 owner token 拒绝第二连接，只在拒绝帧中增加稳定的 `reason`。前端传输层解析结构化连接状态，`main.ts` 将状态同步给 HUD 与 `ArmPanel`，VR 状态牌通过安全快照显示占用原因；被占用时采用 2–5 秒低频退避重连，恢复后保持未解锁。

**Tech Stack:** Python 3.11、FastAPI WebSocket、Pydantic v2、pytest、TypeScript 5.9、WebSocket、Vitest 4、Three.js 0.181

## Global Constraints

- 后端继续只允许一个 `/ws/v1/teleop` 控制连接，不增加观察端或状态广播客户端。
- 占用提示固定为“已有控制页面占用，请关闭电脑端网页后重试。”，稳定原因码为 `controller_occupied`。
- 被占用或重连成功均不得自动发送 `arm_request`，且必须保持 Grip 松开和 DISARMED/未解锁。
- 占用重连初始间隔为 `2_000 ms`，最大间隔为 `5_000 ms`；普通网络断线保留现有 `250–2_000 ms` 退避。
- 不增加外部依赖，不修改机器人坐标、安全边界或控制频率。

---

## File Structure

- `backend/app/api/teleop_ws.py`：在已有第二连接拒绝帧中增加原因码与最终中文文案。
- `backend/tests/api/test_teleop_ws.py`：验证拒绝结构、关闭码、owner 不受影响和释放后的新 owner 安全状态。
- `web/src/protocol/messages.ts`：定义并严格校验 `ConnectionRejectedMessage`。
- `web/src/transport/teleopSocket.ts`：拥有连接状态机、拒绝原因记忆和两套退避策略。
- `web/src/ui/armPanel.ts`：把结构化连接状态投影到不可解锁的安全快照。
- `web/src/ui/hud.ts`：显示可读连接状态并在失联时清理陈旧遥测。
- `web/src/scenes/vrSafetyPanel.ts`：在头显内优先显示控制端占用提示。
- `web/src/main.ts`：连接传输层、HUD、ArmPanel 和 VR 状态牌。
- `web/tests/messages.test.ts`、`teleopSocket.test.ts`、`armPanel.test.ts`、`hud.test.ts`、`vrSafetyPanel.test.ts`：对应单元与集成边界测试。

### Task 1: Backend Occupancy Reason

**Files:**
- Modify: `backend/app/api/teleop_ws.py:140-151`
- Test: `backend/tests/api/test_teleop_ws.py:139-188`

**Interfaces:**
- Consumes: 现有 `app.state.teleop_owner` 与关闭码 `4409`。
- Produces: `{"v":1,"type":"connection_rejected","reason":"controller_occupied","message":"已有控制页面占用，请关闭电脑端网页后重试。"}`。

- [ ] **Step 1: Strengthen the existing second-socket test**

在 `test_second_socket_is_rejected_without_affecting_owner` 的 rejection 断言中使用完整结构：

```python
assert rejection == {
    "v": 1,
    "type": "connection_rejected",
    "reason": "controller_occupied",
    "message": "已有控制页面占用，请关闭电脑端网页后重试。",
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cd backend; python -m pytest tests/api/test_teleop_ws.py::test_second_socket_is_rejected_without_affecting_owner -q`

Expected: FAIL because `reason` is absent and the old message differs.

- [ ] **Step 3: Add the stable reason and final copy**

Replace the occupied branch in `teleop_websocket` with:

```python
if not owns_connection:
    message = "已有控制页面占用，请关闭电脑端网页后重试。"
    await websocket.send_json(
        {
            "v": 1,
            "type": "connection_rejected",
            "reason": "controller_occupied",
            "message": message,
        }
    )
    await websocket.close(code=4409, reason=message)
    return
```

- [ ] **Step 4: Run backend WebSocket tests**

Run: `cd backend; python -m pytest tests/api/test_teleop_ws.py -q`

Expected: all tests PASS, including owner continuity and reconnect safety tests.

- [ ] **Step 5: Commit the backend contract change**

```bash
git add backend/app/api/teleop_ws.py backend/tests/api/test_teleop_ws.py
git commit -m "fix: explain occupied teleop connection"
```

### Task 2: Frontend Protocol Guard and Transport State Machine

**Files:**
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/src/transport/teleopSocket.ts`
- Test: `web/tests/messages.test.ts`
- Test: `web/tests/teleopSocket.test.ts`

**Interfaces:**
- Consumes: backend `connection_rejected` frame from Task 1.
- Produces: exported `TeleopConnectionStatus` and callback `(status: TeleopConnectionStatus) => void`.

- [ ] **Step 1: Add failing protocol-guard tests**

Import `isConnectionRejectedMessage` and add:

```typescript
it('accepts only the exact controller-occupied rejection contract', () => {
  const valid = {
    v: 1,
    type: 'connection_rejected',
    reason: 'controller_occupied',
    message: '已有控制页面占用，请关闭电脑端网页后重试。',
  };
  expect(isConnectionRejectedMessage(valid)).toBe(true);
  expect(isConnectionRejectedMessage({...valid, reason: 'busy'})).toBe(false);
  expect(isConnectionRejectedMessage({...valid, extra: true})).toBe(false);
});
```

- [ ] **Step 2: Run the guard test and verify RED**

Run: `cd web; npm test -- messages.test.ts`

Expected: FAIL because the guard is not exported.

- [ ] **Step 3: Define the exact rejection type and guard**

Add to `messages.ts`:

```typescript
export interface ConnectionRejectedMessage {
  v: typeof PROTOCOL_VERSION;
  type: 'connection_rejected';
  reason: 'controller_occupied';
  message: string;
}

export function isConnectionRejectedMessage(value: unknown): value is ConnectionRejectedMessage {
  return isRecord(value)
    && hasExactKeys(value, ['v', 'type', 'reason', 'message'])
    && value.v === PROTOCOL_VERSION
    && value.type === 'connection_rejected'
    && value.reason === 'controller_occupied'
    && typeof value.message === 'string';
}
```

- [ ] **Step 4: Add failing transport-state tests**

Update the test constructor callbacks and add assertions for this exact state union:

```typescript
export type TeleopConnectionStatus =
  | {state: 'connected'}
  | {state: 'occupied'; message: string}
  | {state: 'unreachable'}
  | {state: 'disconnected'}
  | {state: 'reconnecting'};
```

Add tests that:

```typescript
it('preserves occupied state across close and retries at 2 then 4 then 5 seconds', () => {
  const states: TeleopConnectionStatus[] = [];
  const {client, sockets} = setup(() => {}, (status) => states.push(status));
  client.connect();
  sockets[0].open();
  sockets[0].message(JSON.stringify({
    v: 1,
    type: 'connection_rejected',
    reason: 'controller_occupied',
    message: '已有控制页面占用，请关闭电脑端网页后重试。',
  }));
  sockets[0].closeFromServer();

  expect(states.at(-1)).toEqual({
    state: 'occupied',
    message: '已有控制页面占用，请关闭电脑端网页后重试。',
  });
  vi.advanceTimersByTime(1_999);
  expect(sockets).toHaveLength(1);
  vi.advanceTimersByTime(1);
  expect(sockets).toHaveLength(2);
});
```

Also assert factory construction errors emit `{state:'unreachable'}`, ordinary closes emit `disconnected` then `reconnecting`, and a successful later open emits `connected` without `arm_request`.

- [ ] **Step 5: Run transport tests and verify RED**

Run: `cd web; npm test -- teleopSocket.test.ts`

Expected: FAIL because the callback is boolean-only and rejection messages are ignored.

- [ ] **Step 6: Implement structured connection state and occupied backoff**

In `teleopSocket.ts`, export the union above and replace the boolean callback with:

```typescript
private occupied = false;
private reconnectDelayMs = INITIAL_RECONNECT_DELAY_MS;

constructor(
  private readonly url: string,
  private readonly onRobotState: (state: RobotStateMessage) => void,
  private readonly socketFactory: SocketFactory = (socketUrl) => new WebSocket(socketUrl),
  private readonly onConnectionChange: (status: TeleopConnectionStatus) => void = () => {},
  private readonly onArmFeedback: (message: ArmFeedbackMessage) => void = () => {},
) {}
```

Use `OCCUPIED_INITIAL_RECONNECT_DELAY_MS = 2_000` and `OCCUPIED_MAX_RECONNECT_DELAY_MS = 5_000`. On a valid rejection, set `occupied=true`, set the delay to 2 seconds on the first rejection, and emit `occupied`. In `onclose`, do not overwrite an occupied status with `disconnected`; schedule the occupied retry. On ordinary close emit `disconnected`, then `reconnecting`. On factory failure emit `unreachable`, then `reconnecting`. On open set `occupied=false`, restore the normal 250 ms delay, emit `connected`, and send only `hello`.

- [ ] **Step 7: Run protocol and transport tests**

Run: `cd web; npm test -- messages.test.ts teleopSocket.test.ts`

Expected: all selected tests PASS.

- [ ] **Step 8: Commit the transport state machine**

```bash
git add web/src/protocol/messages.ts web/src/transport/teleopSocket.ts web/tests/messages.test.ts web/tests/teleopSocket.test.ts
git commit -m "feat: expose teleop connection reasons"
```

### Task 3: HUD, Arm Safety Snapshot, and VR Occupancy Copy

**Files:**
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Modify: `web/src/main.ts`
- Test: `web/tests/armPanel.test.ts`
- Test: `web/tests/hud.test.ts`
- Test: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Consumes: `TeleopConnectionStatus` from Task 2.
- Produces: `ArmSafetySnapshot.connectionState` used by the VR status presentation.

- [ ] **Step 1: Add failing UI projection tests**

Extend `ArmSafetySnapshot` fixtures with `connectionState`. Add exact expectations:

```typescript
panel.setConnectionStatus({
  state: 'occupied',
  message: '已有控制页面占用，请关闭电脑端网页后重试。',
});
expect(panel.safetyState).toMatchObject({
  phase: 'disconnected',
  connected: false,
  connectionState: 'occupied',
});
expect(panel.requestArm('xr')).toBe(false);
```

```typescript
expect(describeVrSafety({...state('disconnected'), connectionState: 'occupied'}, true))
  .toMatchObject({
    title: '控制端已被占用',
    instruction: '请关闭电脑端网页后重试',
    tone: 'red',
  });
```

```typescript
hud.setConnectionStatus({state: 'occupied', message: '占用'});
expect(document.body.textContent).toContain('请关闭电脑端网页');
```

- [ ] **Step 2: Run the focused UI tests and verify RED**

Run: `cd web; npm test -- armPanel.test.ts hud.test.ts vrSafetyPanel.test.ts`

Expected: FAIL because structured connection state is not yet represented.

- [ ] **Step 3: Add connection state to ArmPanel safety snapshots**

Import `TeleopConnectionStatus`, add:

```typescript
export type ArmConnectionState = TeleopConnectionStatus['state'];

// in ArmSafetySnapshot
connectionState: ArmConnectionState;

// field
private connectionState: ArmConnectionState = 'disconnected';

setConnectionStatus(status: TeleopConnectionStatus): void {
  this.connectionState = status.state;
  this.connected = status.state === 'connected';
  this.resetToLocked();
}
```

Include `connectionState` in every immutable `safetyState`. Keep `setConnected` only as a compatibility wrapper during the change, implemented as `setConnectionStatus({state: connected ? 'connected' : 'disconnected'})`, then migrate `main.ts` and tests to the structured method.

- [ ] **Step 4: Render exact HUD and VR connection messages**

Add `Hud.setConnectionStatus(status)` with the mapping:

```typescript
const labels: Record<TeleopConnectionStatus['state'], string> = {
  connected: '已连接',
  occupied: '控制端已被占用，请关闭电脑端网页后重试',
  unreachable: '后端不可达',
  disconnected: '连接已中断',
  reconnecting: '正在重连',
};
```

All non-connected states call the existing stale-telemetry cleanup. In `describeVrSafety`, test `connectionState` before generic phase/fault logic and return the occupied title/instruction above; map unreachable/disconnected/reconnecting to distinct readable titles instead of “故障/失联”.

- [ ] **Step 5: Wire `main.ts` to the structured callback**

Replace the boolean callback body with:

```typescript
(status) => {
  scene?.resetConnection();
  pendingFrames.clear();
  latency.reset();
  hud.setConnectionStatus(status);
  hud.setLatency(null, null);
  armPanel?.setConnectionStatus(status);
},
```

Initialize with `armPanel.setConnectionStatus({state: 'disconnected'})`. Do not send any arm request on status changes.

- [ ] **Step 6: Run focused UI tests and the frontend build**

Run: `cd web; npm test -- armPanel.test.ts hud.test.ts vrSafetyPanel.test.ts teleopSocket.test.ts`

Expected: all selected tests PASS.

Run: `cd web; npm run build`

Expected: TypeScript check and Vite build PASS.

- [ ] **Step 7: Commit the user-visible occupancy flow**

```bash
git add web/src/ui/armPanel.ts web/src/ui/hud.ts web/src/scenes/vrSafetyPanel.ts web/src/main.ts web/tests/armPanel.test.ts web/tests/hud.test.ts web/tests/vrSafetyPanel.test.ts
git commit -m "feat: show occupied controller guidance"
```

### Task 4: End-to-End Regression Gate

**Files:**
- Verify only; no production file changes expected.

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces: independently releasable single-controller occupancy behavior.

- [ ] **Step 1: Run the complete backend suite**

Run: `cd backend; python -m pytest -q`

Expected: full backend suite PASS.

- [ ] **Step 2: Run the complete frontend suite**

Run: `cd web; npm test`

Expected: full frontend suite PASS.

- [ ] **Step 3: Build the production frontend**

Run: `cd web; npm run build`

Expected: `tsc --noEmit` and `vite build` PASS.

- [ ] **Step 4: Perform the two-browser smoke test**

1. 打开 PC 控制页并等待“已连接”。
2. 在第二浏览器或 Quest 打开相同地址。
3. 确认第二端显示“控制端已被占用，请关闭电脑端网页后重试”，且不能解锁。
4. 关闭 PC 页，等待 Quest 低频重连。
5. 确认 Quest 显示已连接但仍未解锁，必须松开 Grip 后按 A。

Expected: 第二端无高频状态流，owner 不受影响，切换后没有自动运动。

If the regression gate exposes a defect, return to the owning task, add a focused failing test, implement the smallest fix, rerun this gate, and include the exact affected files in that task's commit. If no files changed, do not create an empty commit.
