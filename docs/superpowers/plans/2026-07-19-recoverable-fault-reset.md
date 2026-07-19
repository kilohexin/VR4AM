# Recoverable Fault Reset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让工作空间越界等可恢复仿真故障无需刷新即可安全复位，同时保持显式确认、停止优先和复位后重新解锁。

**Architecture:** 协议新增 `reset_fault` 控制请求与带 `request_id` 的结果消息。`RobotControl` 负责白名单判断、二次停止、后端静止确认和运动状态原子清理；前端 `ArmPanel` 统一 PC 按钮与 Quest B 键，在故障时发送复位、处理 pending/结果并把状态投影给 HUD 和 VR 状态牌。

**Tech Stack:** Python 3.11、FastAPI、Pydantic v2、NumPy/SciPy、pytest、TypeScript 5.9、Vitest 4、WebXR、Three.js

## Global Constraints

- 新请求固定为 `type="reset_fault"`，成功或拒绝均返回 `type="fault_reset_result"` 并原样回传 `request_id`。
- 可恢复白名单仅包含 `workspace_violation`、`ik_unreachable`、`ik_singular`、`joint_safety_window`。
- `control_loop_error`、未知错误、后端保护停机和真实机械臂故障不得在线复位。
- 只有故障已经至少发布一次、停止完成、后端不为 MOVING、控制循环未失败且会话仍为唯一 owner 时才能复位。
- 复位流程必须再次停止后端，并清除 mapper 锚点、目标、滤波、限速器运动历史和 stop episode；失败时保留故障。
- 复位成功后的权威模式固定为 DISARMED，A/B/Grip 必须先松开，用户再按 A 才能重新解锁。
- PC 故障时“停止”按钮改为“复位故障”；Quest 可恢复故障时显示“松开 Grip，按 B 复位”。
- 只要 `fault` 非空，HUD/VR 不得显示 READY 或“等待解锁”。
- 不允许通用复位接口解除未来真实机械臂的急停、保护停机或伺服故障。

---

## File Structure

- `schemas/teleop-v1.json`：共享 `ClientControlMessage` 枚举增加 `reset_fault`，并定义复位成功/拒绝结果。
- `backend/app/schemas/messages.py`：Pydantic 接受新的控制类型。
- `backend/app/control/filters.py`、`safety.py`：提供显式、可测试的运动状态清理方法。
- `backend/app/control/robot_control.py`：定义复位结果和原子复位流程。
- `backend/app/api/teleop_ws.py`：路由请求并返回结构化结果。
- `web/src/protocol/messages.ts`：请求类型、结果类型和严格 guard。
- `web/src/transport/teleopSocket.ts`：将复位结果分发给 UI。
- `web/src/ui/armPanel.ts`：统一 stop/reset 行为、pending 和可恢复性。
- `web/src/ui/hud.ts`、`web/src/scenes/vrSafetyPanel.ts`：修复状态优先级与中文处置提示。
- `web/src/xr/session.ts`、`web/src/main.ts`：Quest B 键调用统一的 `requestStopOrReset('xr')`。
- 相应测试文件覆盖协议、状态清理、后端拒绝、前端 pending、B 键和显示优先级。

### Task 1: Shared Reset-Fault Protocol

**Files:**
- Modify: `schemas/teleop-v1.json`
- Modify: `backend/app/schemas/messages.py`
- Modify: `backend/tests/contract/test_messages.py`
- Modify: `web/src/protocol/messages.ts`
- Modify: `web/tests/messages.test.ts`

**Interfaces:**
- Consumes: v1 `ClientControlMessage` and existing 1–64 code-point request IDs.
- Produces: `reset_fault` request plus strict `FaultResetResultMessage` guard.

- [ ] **Step 1: Add failing backend and JSON-schema contract tests**

Add:

```python
def test_reset_fault_is_a_valid_v1_control_message() -> None:
    message = ClientControlMessage.model_validate(
        {"v": 1, "type": "reset_fault", "request_id": "reset-1"}
    )
    assert message.type == "reset_fault"

    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": "#/$defs/ClientControlMessage"}
    )
    assert not list(validator.iter_errors(message.model_dump(mode="json")))
```

- [ ] **Step 2: Run backend contract test and verify RED**

Run: `cd backend; python -m pytest tests/contract/test_messages.py::test_reset_fault_is_a_valid_v1_control_message -q`

Expected: FAIL because `reset_fault` is not accepted.

- [ ] **Step 3: Extend backend and JSON Schema request enums**

Change both literal/enum lists to exactly:

```python
type: Literal["hello", "arm_request", "disarm", "reset_fault", "ping"]
```

```json
["hello", "arm_request", "disarm", "reset_fault", "ping"]
```

在 `$defs` 中增加 `FaultResetAccepted`、`FaultResetRejected` 和 `FaultResetResultMessage`。成功对象必须且只允许 `v,type,request_id,accepted,mode`，其中 `accepted=true`、`mode="DISARMED"`；拒绝对象必须且只允许 `v,type,request_id,accepted,reason,message`，其中 `accepted=false`，`reason` 枚举与下面的 TypeScript `FaultResetRejectReason` 完全一致。扩展 backend contract test，用 `Draft202012Validator` 分别验证一个成功样例和一个拒绝样例，并拒绝额外字段。

- [ ] **Step 4: Add failing frontend guard tests**

Add `reset_fault` to the valid request-type loop and add:

```typescript
const accepted = {
  v: 1,
  type: 'fault_reset_result',
  request_id: 'reset-1',
  accepted: true,
  mode: 'DISARMED',
};
const rejected = {
  v: 1,
  type: 'fault_reset_result',
  request_id: 'reset-2',
  accepted: false,
  reason: 'unrecoverable_fault',
  message: '该故障无法在线复位，请重启后端并重新检查。',
};
expect(isFaultResetResultMessage(accepted)).toBe(true);
expect(isFaultResetResultMessage(rejected)).toBe(true);
expect(isFaultResetResultMessage({...accepted, mode: 'READY'})).toBe(false);
expect(isFaultResetResultMessage({...rejected, extra: true})).toBe(false);
```

- [ ] **Step 5: Run frontend guard tests and verify RED**

Run: `cd web; npm test -- messages.test.ts`

Expected: FAIL because the request type and result guard are absent.

- [ ] **Step 6: Implement exact frontend result union**

Add:

```typescript
export type ClientControlType = 'hello' | 'arm_request' | 'disarm' | 'reset_fault' | 'ping';

export type FaultResetRejectReason =
  | 'no_fault'
  | 'stop_incomplete'
  | 'backend_moving'
  | 'unrecoverable_fault'
  | 'control_loop_unavailable';

export type FaultResetResultMessage =
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'fault_reset_result';
      request_id: string;
      accepted: true;
      mode: 'DISARMED';
    }
  | {
      v: typeof PROTOCOL_VERSION;
      type: 'fault_reset_result';
      request_id: string;
      accepted: false;
      reason: FaultResetRejectReason;
      message: string;
    };
```

Implement `isFaultResetResultMessage` using `hasExactKeys`: accepted results require exactly `v,type,request_id,accepted,mode`; rejected results require exactly `v,type,request_id,accepted,reason,message` and a reason in the union above.

- [ ] **Step 7: Run both contract suites**

Run: `cd backend; python -m pytest tests/contract/test_messages.py -q`

Expected: all backend contract tests PASS.

Run: `cd web; npm test -- messages.test.ts`

Expected: all frontend protocol tests PASS.

- [ ] **Step 8: Commit the shared request/result contract**

```bash
git add schemas/teleop-v1.json backend/app/schemas/messages.py backend/tests/contract/test_messages.py web/src/protocol/messages.ts web/tests/messages.test.ts
git commit -m "feat: define fault reset protocol"
```

### Task 2: Resettable Filter and Safety Motion State

**Files:**
- Modify: `backend/app/control/filters.py`
- Modify: `backend/app/control/safety.py`
- Modify: `backend/tests/control/test_filters.py`
- Modify: `backend/tests/control/test_safety.py`

**Interfaces:**
- Consumes: existing `PoseFilter.value`, `SafetyLimiter.anchor`, `linear_velocity`, and `angular_velocity`.
- Produces: idempotent `PoseFilter.clear()` and `SafetyLimiter.clear()`.

- [ ] **Step 1: Add failing state-clear tests**

```python
def test_pose_filter_clear_forgets_previous_pose() -> None:
    filter_ = PoseFilter()
    filter_.reset(Pose(p=(1, 2, 3), q=(0, 0, 0, 1)))
    filter_.clear()
    assert filter_.value is None
```

```python
def test_safety_limiter_clear_removes_anchor_and_motion_history() -> None:
    limiter = SafetyLimiter(anchor=(0.3, 0.0, 0.3))
    limiter.linear_velocity[:] = (0.1, 0.2, 0.3)
    limiter.angular_velocity[:] = (0.4, 0.5, 0.6)
    limiter.clear()
    assert limiter.anchor is None
    assert np.allclose(limiter.linear_velocity, 0)
    assert np.allclose(limiter.angular_velocity, 0)
```

- [ ] **Step 2: Run focused tests and verify RED**

Run: `cd backend; python -m pytest tests/control/test_filters.py tests/control/test_safety.py -q`

Expected: FAIL because `clear` methods are absent.

- [ ] **Step 3: Implement idempotent clear methods**

```python
# PoseFilter
def clear(self) -> None:
    self.value = None
```

```python
# SafetyLimiter
def clear(self) -> None:
    self.anchor = None
    self.linear_velocity[:] = 0
    self.angular_velocity[:] = 0
```

- [ ] **Step 4: Run focused tests**

Run: `cd backend; python -m pytest tests/control/test_filters.py tests/control/test_safety.py -q`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit motion-state reset helpers**

```bash
git add backend/app/control/filters.py backend/app/control/safety.py backend/tests/control/test_filters.py backend/tests/control/test_safety.py
git commit -m "feat: clear teleop motion state"
```

### Task 3: RobotControl Atomic Fault Reset

**Files:**
- Modify: `backend/app/control/robot_control.py`
- Modify: `backend/tests/control/test_robot_control.py`

**Interfaces:**
- Consumes: clear methods from Task 2 and current fault/stop state.
- Produces: `async RobotControl.reset_fault() -> FaultResetResult`.

- [ ] **Step 1: Add result type and failing happy-path test**

The planned result type is:

```python
@dataclass(frozen=True)
class FaultResetResult:
    accepted: bool
    reason: str | None = None
    message: str | None = None
```

Add a helper that enters `ik_unreachable`, calls `state_message()` once to publish FAULT and complete the stop into DISARMED, then test:

```python
result = await control.reset_fault()

assert result == FaultResetResult(accepted=True)
assert control.mode == TeleopMode.DISARMED
assert control._fault is None
assert control.last_target is None
assert control.mapper._hand_anchor is None
assert control.filter.value is None
assert control.limiter.anchor is None
assert backend.stops[-1] == StopReason.FAULT
with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
    await control.arm()
```

- [ ] **Step 2: Add failing rejection matrix tests**

Parameterize these outcomes:

```python
[
    (None, "no_fault"),
    ("control_loop_error", "unrecoverable_fault"),
    ("backend_fault", "unrecoverable_fault"),
]
```

Add separate tests for pending stop (`stop_incomplete`), `BackendState.MOVING` (`backend_moving`), `_loop_failed=True` (`control_loop_unavailable`), and a backend `stop()` exception. For the exception case, assert the exception is contained, the result is rejected, and `_fault` remains unchanged.

- [ ] **Step 3: Run RobotControl tests and verify RED**

Run: `cd backend; python -m pytest tests/control/test_robot_control.py -q`

Expected: FAIL because `FaultResetResult` and `reset_fault` do not exist.

- [ ] **Step 4: Implement classification and atomic reset**

Add:

```python
RECOVERABLE_FAULTS = frozenset(
    {"workspace_violation", "ik_unreachable", "ik_singular", "joint_safety_window"}
)
```

Implement `reset_fault` with this order:

```python
async def reset_fault(self) -> FaultResetResult:
    if self._fault is None:
        return FaultResetResult(False, "no_fault", "当前没有可复位故障。")
    if self._fault not in RECOVERABLE_FAULTS:
        return FaultResetResult(
            False,
            "unrecoverable_fault",
            "该故障无法在线复位，请重启后端并重新检查。",
        )
    if self._loop_failed or self._shutdown_started:
        return FaultResetResult(
            False,
            "control_loop_unavailable",
            "控制循环不可用，请重启后端并重新检查。",
        )
    if self._pending_stop_completion or self.machine.mode != TeleopMode.DISARMED:
        return FaultResetResult(False, "stop_incomplete", "停止尚未完成，请稍后重试。")
    state = await self.backend.get_state()
    if state.robot_state == BackendState.MOVING:
        return FaultResetResult(False, "backend_moving", "仿真仍在运动，请稍后重试。")
    try:
        await self.backend.stop(StopReason.FAULT)
    except Exception:
        return FaultResetResult(
            False,
            "stop_incomplete",
            "无法确认仿真已停止，故障保持锁定。",
        )
    self.mapper.clear()
    self.filter.clear()
    self.limiter.clear()
    self.last_target = None
    self._clear_stop_episode()
    self.machine.disarm()
    return FaultResetResult(True)
```

Do not clear `_last_frame_id`: the old held frame must not become a new command. Do not clear Grip release gating; `machine.disarm()` intentionally sets `_grip_released=False`.

- [ ] **Step 5: Run RobotControl and fault-path tests**

Run: `cd backend; python -m pytest tests/control/test_robot_control.py tests/api/test_fault_paths.py -q`

Expected: all selected tests PASS; existing “FAULT is observable before DISARMED” behavior remains intact.

- [ ] **Step 6: Commit atomic backend reset logic**

```bash
git add backend/app/control/robot_control.py backend/tests/control/test_robot_control.py
git commit -m "feat: reset recoverable simulator faults"
```

### Task 4: WebSocket Reset Routing

**Files:**
- Modify: `backend/app/api/teleop_ws.py`
- Modify: `backend/tests/api/test_teleop_ws.py`

**Interfaces:**
- Consumes: `RobotControl.reset_fault()` and `FaultResetResult` from Task 3.
- Produces: exact `fault_reset_result` WebSocket responses.

- [ ] **Step 1: Add failing accepted/rejected API tests**

Mock `app.state.control.reset_fault` and assert:

```python
ws.send_json({"v": 1, "type": "reset_fault", "request_id": "r1"})
assert _receive_until(ws, "fault_reset_result") == {
    "v": 1,
    "type": "fault_reset_result",
    "request_id": "r1",
    "accepted": True,
    "mode": "DISARMED",
}
```

For rejection assert exact `reason` and Chinese `message`. Keep the second-socket test parameter list extended with an intruder `reset_fault`, and assert it never calls `control.reset_fault`.

- [ ] **Step 2: Run API tests and verify RED**

Run: `cd backend; python -m pytest tests/api/test_teleop_ws.py -q`

Expected: FAIL because reset messages are not routed.

- [ ] **Step 3: Route reset results without exposing exceptions**

Add an `elif message.type == "reset_fault"` branch. On accepted, first call `control.state_message()` and send that authoritative `robot_state` (`mode=DISARMED`, `fault=null`), then send only `v,type,request_id,accepted,mode` in `fault_reset_result`. On rejected, send only `v,type,request_id,accepted,reason,message`. If `reset_fault()` unexpectedly raises, contain raw details and return `accepted=false`, `reason="unrecoverable_fault"`, and the standard restart message. The periodic state sender may emit an additional state frame; clients already tolerate repeated authoritative state.

- [ ] **Step 4: Run API and contract tests**

Run: `cd backend; python -m pytest tests/api/test_teleop_ws.py tests/contract/test_messages.py -q`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit WebSocket routing**

```bash
git add backend/app/api/teleop_ws.py backend/tests/api/test_teleop_ws.py
git commit -m "feat: route fault reset requests"
```

### Task 5: Frontend Reset Result Transport

**Files:**
- Modify: `web/src/transport/teleopSocket.ts`
- Modify: `web/tests/teleopSocket.test.ts`

**Interfaces:**
- Consumes: `isFaultResetResultMessage` from Task 1.
- Produces: constructor callback `(message: FaultResetResultMessage) => void`.

- [ ] **Step 1: Add failing transport-delivery test**

Construct the socket with a reset callback, send valid accepted and rejected messages plus malformed variants, and assert only the two valid results are delivered in order.

```typescript
expect(results).toEqual([
  {v: 1, type: 'fault_reset_result', request_id: 'r1', accepted: true, mode: 'DISARMED'},
  {
    v: 1,
    type: 'fault_reset_result',
    request_id: 'r2',
    accepted: false,
    reason: 'unrecoverable_fault',
    message: '该故障无法在线复位，请重启后端并重新检查。',
  },
]);
```

- [ ] **Step 2: Run transport tests and verify RED**

Run: `cd web; npm test -- teleopSocket.test.ts`

Expected: FAIL because reset results are ignored.

- [ ] **Step 3: Parse and deliver result messages**

Add the typed callback as the final optional constructor parameter so existing call sites remain source-compatible. In `onmessage`, add:

```typescript
else if (isFaultResetResultMessage(message)) this.onFaultResetResult(message);
```

Malformed results remain ignored at the transport boundary.

- [ ] **Step 4: Run transport tests**

Run: `cd web; npm test -- teleopSocket.test.ts messages.test.ts`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit reset-result transport**

```bash
git add web/src/transport/teleopSocket.ts web/tests/teleopSocket.test.ts
git commit -m "feat: deliver fault reset results"
```

### Task 6: Unified Desktop and Quest Reset Interaction

**Files:**
- Modify: `web/src/ui/armPanel.ts`
- Modify: `web/src/main.ts`
- Modify: `web/src/xr/session.ts`
- Modify: `web/tests/armPanel.test.ts`
- Modify: `web/tests/xrSession.test.ts`

**Interfaces:**
- Consumes: reset result callback from Task 5 and current `ArmSafetySnapshot`.
- Produces: `requestStopOrReset(source)` and reset-specific snapshot fields.

- [ ] **Step 1: Add failing ArmPanel reset-state tests**

Extend `ArmSafetySnapshot` with:

```typescript
faultRecoverable: boolean;
faultResetPending: boolean;
```

Add tests that set `workspace_violation`, keep Grip pressed and confirm reset is blocked, then release Grip, click the stop button, and assert one `reset_fault` message with a `desktop-reset_fault-` ID, button label `复位中…`, and duplicate clicks suppressed. Feed a matching accepted result and assert fault clears locally only into `stopped`, arm remains disabled until a new released-Grip sample. Feed a rejected result and assert the fault remains, pending clears, and readable feedback is shown without raw backend details.

- [ ] **Step 2: Add failing Quest B routing tests**

Change the session callback name from `onStopRequest` to `onStopOrResetRequest` without changing its B-edge semantics. In integration setup, connect it to `panel.requestStopOrReset('xr')`. Assert B sends `disarm` in normal state and `reset_fault` in a recoverable fault, once per released edge.

- [ ] **Step 3: Run focused tests and verify RED**

Run: `cd web; npm test -- armPanel.test.ts xrSession.test.ts`

Expected: FAIL because stop/reset are not unified and no reset pending exists.

- [ ] **Step 4: Implement recoverability and context-sensitive requests**

Export:

```typescript
export const RECOVERABLE_FAULTS = new Set([
  'workspace_violation',
  'ik_unreachable',
  'ik_singular',
  'joint_safety_window',
]);
```

Add `pendingFaultResetId`, `gripPressed`, reset feedback, and update `observeGrip` so `gripPressed` always mirrors the latest valid sample before other gate calculations. Then add:

```typescript
requestStopOrReset(source: ControlSource = 'desktop'): void {
  if (this.fault && RECOVERABLE_FAULTS.has(this.fault)) {
    if (!this.connected || this.gripPressed || this.pendingFaultResetId) return;
    const message = this.control('reset_fault', source);
    this.pendingFaultResetId = message.request_id;
    this.resetFeedback = null;
    this.syncButtonState();
    this.sendControl(message);
    return;
  }
  this.requestDisarm(source);
}
```

The stop button click and Quest B callback both call this method. In a recoverable fault, reset is ignored until the latest Grip sample is released; in normal operation B still disarms immediately even when Grip is pressed. `handleFaultResetResult` ignores stale request IDs. Accepted results clear local fault, call `resetToLocked()`, and keep the mode DISARMED; authoritative state will confirm. Rejected results keep fault, clear pending, and set a fixed readable local message selected by `reason`.

- [ ] **Step 5: Make the stop button reflect reset state**

Store its `<span>` as `stopLabel`. Render:

- recoverable fault + no pending: `复位故障`;
- reset pending: `复位中…` and disabled;
- unrecoverable fault: `无法在线复位` and disabled;
- no fault: `停止`.

Include `faultRecoverable` and `faultResetPending` in every immutable snapshot. A pending reset makes `eligible=false` and blocks A.

- [ ] **Step 6: Wire `main.ts` and session callbacks**

Pass `(message) => armPanel?.handleFaultResetResult(message)` as the new final `TeleopSocket` callback. Replace `onStopRequest` with:

```typescript
onStopOrResetRequest: () => { armPanel.requestStopOrReset('xr'); },
```

Lifecycle safety (`onDisarm: sendVRDisarm`) must remain a direct disarm, never a reset, because tracking loss and exit must always issue stop.

- [ ] **Step 7: Run focused tests and build**

Run: `cd web; npm test -- armPanel.test.ts xrSession.test.ts teleopSocket.test.ts`

Expected: all selected tests PASS.

Run: `cd web; npm run build`

Expected: TypeScript and Vite build PASS.

- [ ] **Step 8: Commit unified reset interaction**

```bash
git add web/src/ui/armPanel.ts web/src/main.ts web/src/xr/session.ts web/tests/armPanel.test.ts web/tests/xrSession.test.ts
git commit -m "feat: reset faults from desktop and Quest"
```

### Task 7: Truthful HUD and VR Fault Priority

**Files:**
- Modify: `web/src/ui/hud.ts`
- Modify: `web/src/scenes/vrSafetyPanel.ts`
- Test: `web/tests/hud.test.ts`
- Test: `web/tests/vrSafetyPanel.test.ts`

**Interfaces:**
- Consumes: `faultRecoverable`, `faultResetPending`, `fault`, and connection state from prior plans.
- Produces: consistent desktop/VR presentation with connection > fault > stale > active > locked priority.

- [ ] **Step 1: Add failing priority tests**

Assert a robot state `{mode:'READY', fault:'workspace_violation'}` renders `FAULT · 故障`, never `READY · 等待解锁`. Add VR cases:

```typescript
expect(describeVrSafety(recoverableFaultState, true)).toMatchObject({
  title: '目标超出工作空间',
  instruction: '松开 Grip，按 B 复位',
  tone: 'red',
});
expect(describeVrSafety({...recoverableFaultState, faultResetPending: true}, true))
  .toMatchObject({title: '复位中', instruction: '等待仿真确认'});
expect(describeVrSafety(unrecoverableFaultState, true)).toMatchObject({
  title: '无法在线复位',
  instruction: '请重启后端并检查原因',
});
```

- [ ] **Step 2: Run presentation tests and verify RED**

Run: `cd web; npm test -- hud.test.ts vrSafetyPanel.test.ts`

Expected: FAIL because READY currently wins in HUD and all faults collapse to “故障/失联”.

- [ ] **Step 3: Enforce display priority**

In `Hud.setRobotState`, derive:

```typescript
const displayMode: TeleopMode = state.fault ? 'FAULT' : state.mode;
```

Use `displayMode` for heading text/tone while retaining the backend mode in application state. In `describeVrSafety`, preserve the connection-state branches from the occupancy plan first, then handle reset pending, recoverable fault, unrecoverable fault, STALE, and normal phases in that order. Use `readableFault` through a shared exported formatter or an equivalent small `faultLabel` helper so workspace/IK messages match the HUD.

- [ ] **Step 4: Run presentation tests**

Run: `cd web; npm test -- hud.test.ts vrSafetyPanel.test.ts armPanel.test.ts`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit truthful fault presentation**

```bash
git add web/src/ui/hud.ts web/src/scenes/vrSafetyPanel.ts web/tests/hud.test.ts web/tests/vrSafetyPanel.test.ts
git commit -m "fix: keep latched faults visible"
```

### Task 8: Full Regression and Manual Recovery Gate

**Files:**
- Verify only; do not change production code unless a failing automated test first captures the defect.

**Interfaces:**
- Consumes: Tasks 1–7.
- Produces: independently releasable recoverable-fault reset flow.

- [ ] **Step 1: Run the complete backend suite**

Run: `cd backend; python -m pytest -q`

Expected: full backend suite PASS.

- [ ] **Step 2: Run the complete frontend suite and build**

Run: `cd web; npm test`

Expected: full frontend suite PASS.

Run: `cd web; npm run build`

Expected: production build PASS.

- [ ] **Step 3: Desktop workspace-fault acceptance**

1. 解锁仿真并用鼠标把目标移出工作空间。
2. 确认仿真停止，HUD 保持显示“目标超出工作空间”，按钮显示“复位故障”。
3. 点击一次，确认短暂显示“复位中…”，随后进入“已停止/未解锁”。
4. 不刷新页面，松开输入并重新解锁。
5. 首次重新按住 Grip 时确认从当前 TCP 建立新锚点，没有目标跳变。

Expected: 故障清除前不能解锁；复位后不能自动运动。

- [ ] **Step 4: Quest B reset acceptance**

触发可恢复故障后松开 Grip，按一次 B。确认头显显示“复位中”后变为“已复位 · 按 A 重新解锁”；保持 B 不重复请求。松开 B、A、Grip 后再按 A，确认重新解锁。

Expected: B 在正常状态仍是停止，在故障状态才是复位。

- [ ] **Step 5: Unrecoverable fault acceptance**

用测试配置或自动化注入 `control_loop_error`。确认 PC 按钮不可用，VR 显示“无法在线复位 · 请重启后端并检查原因”，任何 B/按钮操作都不会清除故障。

Expected: 只有重启后端并重新连接才能恢复，不暴露内部异常文本。
