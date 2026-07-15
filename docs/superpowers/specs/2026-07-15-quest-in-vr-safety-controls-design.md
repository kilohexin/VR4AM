# Quest 3 头显内安全解锁与停止设计

**日期：** 2026-07-15
**范围：** Milestone 1 纯 VR 仿真遥操作补充设计
**目标设备：** Meta Quest 3 原厂 Touch Plus 右手柄

## 1. 问题与目标

进入 `immersive-vr` 前，系统会按安全规则发送停止并回到锁定状态。进入后，桌面 HTML 的“解锁仿真”按钮不在头显画面中，因此现有流程无法在 VR 内重新执行显式解锁。

本设计补充一个头显内可完成的安全流程：

- 右手柄 **A 键**发起显式解锁请求；
- 右手柄 **B 键**立即停止并回到锁定状态；
- 工作台旁显示文字与形状共同表达的世界空间安全状态牌；
- Grip 仍是运动离合，Trigger 仍是夹爪闭合量；
- 所有追踪丢失、页面隐藏、Socket 断开和会话结束规则保持不变。

## 2. 方案选择

采用 Quest 专用面键方案，而不是摇杆按下或完整射线菜单：

- A/B 的语义清晰，误触风险低于摇杆按下；
- 不需要引入射线、碰撞、菜单焦点和额外 3D 交互状态；
- 第一版只面向 Quest 3 原厂手柄，设备专用映射是可接受的；
- 真机验收必须记录 Quest OS、Browser 和 `XRInputSource.profiles`，用于确认 Touch Plus 暴露的实际映射。

WebXR `xr-standard` 固定 Trigger、Squeeze、Thumbstick 等基础槽位，额外面键属于设备输入配置。Milestone 1 对已识别的 Meta/Oculus Touch 右手柄读取 `buttons[4]` 为 A、`buttons[5]` 为 B；未识别配置或按钮不足时保持锁定并显示中文不支持提示，不使用猜测式回退。

参考：

- [W3C WebXR Gamepads Module](https://www.w3.org/TR/webxr-gamepads-module-1/)
- [WebXR Input Profiles Registry](https://immersive-web.github.io/webxr-input-profiles/)

## 3. 输入与安全状态机

### 3.1 A 键解锁

A 键仅在按下上升沿触发一次。以下条件必须同时满足：

1. 当前 XR 会话与 Socket 均有效；
2. 右手柄追踪有效；
3. 已观察到当前会话中的 `Grip=false`；
4. 当前没有未完成的 `arm_request`；
5. 当前不是 ACTIVE、FAULT、STALE 或断线状态；
6. A 键在本次追踪有效期内先被观察为松开，再发生按下。

请求继续使用现有 `ArmPanel` 门禁与 `ClientControlMessage(type='arm_request')`，不从 XR 代码旁路发送。桌面按钮与 A 键调用同一个 `requestArm()` 行为。

### 3.2 B 键停止

B 键按下上升沿立即调用统一 `requestDisarm()`：

- 同步重置本地解锁资格和 pending 状态；
- 发送 `ClientControlMessage(type='disarm')`；
- 状态牌立即显示“已停止”；
- 长按 B 不重复发送；重新松开再按才产生下一次请求。

Grip 松开仍产生 HOLD/停止运动，但 B 是显式回到 DISARMED 的操作，两者语义不合并。

### 3.3 追踪和会话边界

- 追踪丢失、会话隐藏、Socket 断开、XR `end` 或 dispose 继续立即停止并锁定。
- 追踪恢复时，如果 A/B 仍被按住，不产生边沿事件；必须先松开再重新按下。
- 新 XRSession 重置所有面键 latch、Grip 解锁资格和 pending 请求，不继承上次会话状态。
- 没有右手柄、`gamepad.buttons` 缺失、按钮数量不足或 profile 未识别时，不发送解锁请求。

## 4. 组件与数据流

### `controllerInput.ts`

在现有位姿、Grip、Trigger 样本上增加 A/B 的原始 pressed 状态和 profile 支持结果。读取函数保持纯函数，不自行发送协议消息。

### `XRSessionController`

持有每会话的 A/B 边沿 latch：

1. 读取右手柄样本；
2. 先更新 tracking/Grip/Trigger；
3. 再判断 A/B 上升沿；
4. A 调用 `onArmRequest`，B 调用 `onDisarm`；
5. 无效追踪或会话边界清除资格，但不把“持续按住”误认为新按下。

### `ArmPanel`

公开统一的 `requestArm()` 和 `requestDisarm()`，桌面按钮和 XR 回调共用同一套连接、Grip、pending、故障和模式门禁。它同时发布只读安全状态，供桌面 HUD 和世界空间状态牌消费。

### `SimulationScene`

增加一个随 XR 相机朝向可读的世界空间状态牌。状态牌位于工作台安全侧，不遮挡机械臂、TCP 或工作空间边界。它是信息显示，不接受射线点击。

## 5. 世界空间状态牌

状态牌沿用桌面视觉系统：深色石墨底、青色正常、红色停止/故障、白色中文主文字、细边框。必须同时使用文字与形状，不只依赖颜色。

显示状态：

- `未解锁 · 松开 Grip 后按 A`
- `解锁中 · 等待仿真确认`
- `已解锁 · 按住 Grip 移动`
- `运动中 · 松开 Grip 停止`
- `已停止 · 松开 Grip 后按 A`
- `故障/失联 · 保持 Grip 松开`
- `手柄配置不支持 A/B 安全控制`

底部固定提示：`A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪`。

状态牌使用 CanvasTexture/Sprite 或等价的 Three.js 世界空间纹理实现；不引入 DOM Overlay、passthrough、射线 UI、Cannon-es 或新外部依赖。

## 6. 测试与验收

自动化测试必须覆盖：

- 仅右手柄且 profile/按钮完整时读取 A/B；
- A 只在 Grip 松开、追踪有效、连接有效且无 pending 时触发；
- A/B 长按不重复，松开后再次按下才重新触发；
- A 在追踪丢失期间保持按下，恢复后不会自动解锁；
- B 立即锁定并发送一次 disarm；
- 未识别 profile、缺失按钮或无右手柄时保持锁定；
- 状态牌的每个安全状态显示正确中文文字；
- 桌面按钮与 XR 面键共用相同门禁，没有旁路。

Quest 3 手工验收必须确认：

1. 记录实际 `XRInputSource.profiles` 和按钮数组长度；
2. A/B 映射与 Touch Plus 实物一致；
3. 进入 VR 后先锁定，松开 Grip、按 A 才能解锁；
4. 按住 A 不重复请求；
5. B、追踪丢失、隐藏、Socket 断开和退出 VR 均停止；
6. 状态牌在头显内清晰、不遮挡机器人；
7. 任意故障恢复都要求重新松开 Grip 并按 A。

未完成 Quest 真机清单前，不声明 Quest 验收通过。

## 7. 非目标

- 不增加 MR/passthrough、相机、数据集写入或真实机械臂连接；
- 不增加物体抓取物理；
- 不实现通用多品牌控制器配置；
- 不增加完整 3D 菜单或射线交互；
- 不改变 Grip/Trigger 的现有遥操作语义。
