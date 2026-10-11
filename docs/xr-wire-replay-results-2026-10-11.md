# VR 输入到假 SDK 的离线组合验证

日期：2026-10-11。基线提交：`7df4815cb52e30d825f8772486f890df80e31a35`。

## 本轮完成

增加一份两端共用的合成报文样本，以及前端、后端组合测试。样本是人为构造的数据，不包含现场日志、配置或凭据。本轮没有修改运行时代码、停止策略或现场配置。

| 路径 | 使用的生产代码 | 替身与边界 |
| --- | --- | --- |
| 前端 | XRSessionController → ArmPanel / forwardTeleopFrame → TeleopSocket | jsdom、假 XR session/frame、内存 socket；后端反馈为合成输入 |
| 后端 | WebSocket 接收与状态发送 → LatestVRFrame → RobotControl → RealLebaiAdapter | 内存 JSON 传输、假 SDK、可控单调时钟；未运行网络握手和后台控制循环 |

后端分别覆盖原有 target 策略和显式启用的 transactional 实验策略。两者均使用乐白适配器的 `LEBAI` 选解分支，注入假客户端工厂；这个内部标签不代表连接了真机，也不代表实验策略已获现场验收。

## 验证结果

新增 11 个用例：前端 3 个，后端 8 个。

- 进入 VR 先请求 Disarm；释放 Grip 后 A 按键产生一次相关 ID 的 Arm 请求。
- 正常帧限流期间，5 ms 后的 Grip 释放仍立即序列化；随后重新握住即使覆盖容量为 1 的最新帧，后端仍识别释放事件。
- 正常停止路径先进入 HOLD、清除锚点并执行原停止验证；停止验证消耗时间后，需要新收到的帧才能重新抓握。旧 Trigger 意图不会直接发出夹爪指令，松开后新按下才产生预期 force/amplitude。
- 原 200 ms RPC 回复期限仍生效。超时产生 `stop_unverified`；假客户端持续 MOVING/非零速度产生 `stop_incomplete`。迟到回复、之后的原始 IDLE 和重新 Arm 都不能解除锁存，不能发出新的运动或夹爪指令。
- 阻塞 IK 时，在释放事件已接受并由停止接管后，pump generation 失效；放行旧 IK 的原 admission 得到 `invalidated`、`sdk_outcome=not_started`，不会进入 PVAT SDK 调用。没有开始 PVAT 的分支复用并重新验证已确认的进入停止事务，不发送重复停止 RPC。
- 追踪丢失与退出 VR 撤销输入；退出前保存的旧回调不能再产生报文。前端收到两类停止故障后，A/B 操作和断线重连都不能产生 Arm/Home/复位请求。

扩大定向回归：前端 9 个文件 **191 通过**，后端 7 个文件 **348 通过**，共 **539 通过**，JUnit 无失败、错误或跳过。未重跑后端全集。

TypeScript 检查和 Vite 构建通过。既有 744.20 kB bundle 超过 500 kB 的构建警告仍存在。

独立静态审查未发现本轮剩余可操作的 P0–P2；审查者未运行测试或访问机器人。

## 时序解释与限制

`command_tcp` 返回只代表本地入队。首轮测试错误地假定 tick 返回即意味着旧 PVAT 完成；现在主用例等待原 admission 的明确 `sdk_returned` completion，并核对 command ID，再建立释放前基线。另一个受控用例专门检查停止接管时的 pending IK 撤销。

这些测试证明停止接管后的撤销行为，不能证明 WebSocket 接受释放的瞬间已经撤销所有入队或在途运动。前端与后端通过同一报文样本核对契约，反馈仍为合成数据；这不是一个实际网络闭环。测试也没有覆盖真实头显、Wi-Fi、真实后台调度时延、机器人惯性、夹爪动作或物理抓取。

## 真机主线状态

VR 输入撤销、释放事件保持、Trigger 新意图及两类停止故障锁存，现已补上跨模块离线验证。

此前现场 `stop_move` 完整回复约 **613 ms** 的原因仍未确认，200 ms 内运动停止证据仍缺失；持续 VR 运动、真实 Grip 释放停止和夹爪控制仍未验收。下一项现场定位材料仍是当前控制器版本的已有内部日志入口，以及 `stop_move` 回复语义。旧单次授权已经结束，新访问或动作需另定明确范围与现场条件。

本轮没有新机器人连接、命令、抓包改动、原厂消息或推送。物理仿真抓取继续排在真机 VR 主线之后。

## 文件与本机证据

- `schemas/fixtures/xr-grip-release-wire.json`
- `web/tests/xrWireReplay.test.ts`
- `backend/tests/api/test_xr_wire_replay.py`
- 忽略目录 `.tmp/xr-wire-replay-20261011/`：测试输出、初始失败、类型检查、构建与冻结清单，保留本机。

此前冻结材料、现场配置、SDK 二进制和用户项目概述按大小/散列核对保留。只提交本轮合成样本、测试与本报告。
