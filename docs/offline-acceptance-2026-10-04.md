# 本地改动离线验收汇总（2026-10-04）

## 1. 范围和结论

本轮复核 `codex/offline-evidence-audit` 的 `cd62b72a3fc856f19cf8dacda30ba523d513c5fa..124c738e8e7194515278a8a620b1bd7aef545da2`：14 个本地提交、50 个文件、6448 行新增、28 行删除。基准来自本地保存的 origin 跟踪引用；本轮没有 fetch，不能据此声称刚核对过 GitHub。此报告将另作本地提交。

受影响后端定向回归 **735 passed**，前端六个测试文件 **131 passed**，TypeScript 检查与 Vite 构建通过。没有运行后端全集。源码自查未发现本轮需要追加的运行时修复；这不是独立第二审查人结论，也不是物理运动验收。

机械臂按用户说明已断电。全程仅本地代码、注入 fake SDK、内存会话和离线工具；没有连接设备、启动现场服务、部署或推送。现场配置、原始日志、迁移目录保持原样。Home 按用户本人误触记录，停止/Home 按键行为未改。

## 2. 提交清单

| 提交 | 内容 | 实际适用范围 |
|---|---|---|
| `3264f81` | 控制目标各阶段最多 10 Hz 采样；记录 await 后再次检查代次和新鲜度 | 当前控制路径本地实现 |
| `232ecbd` | IK 背压与恢复实验 | fake SDK |
| `5585f35` | VR 输入帧龄文案；SDK 读取起止边界 | 当前本地后端和 UI |
| `354fcae` | 45 组参数敏感性比较 | 合成输入、假反馈 |
| `4aef34c` | 冻结日志节奏分析、pump 调度对照 | 只读脚本与实验 |
| `8678105` | 快照读取用途标签、争用实验 | 本地诊断字段与 fake SDK |
| `5564b2d` | 被动显示快照 | 尚未接入生产调用 |
| `801c27b` | 显示合成与代次验证 | 尚未接入生产调用 |
| `f917abb` | 拆分状态观察的四个私有步骤 | 保留原 state_message 顺序 |
| `86ab2a6` | 共享观察器生命周期 | 测试夹具 |
| `cc7b59b` | 观察器与会话取消、清理耦合 | 测试夹具 |
| `d94ee1a` | 取消至停止启动时间边界 | fake 时钟与 SDK |
| `e18e698` | 会话生命周期同步普通队列诊断 | 当前会话路径本地实现 |
| `124c738` | 生命周期与停止时间离线汇总 | 标准库只读脚本 |

“当前路径”均表示源代码已经接入，不表示已经部署至现场。

## 3. 兼容性复核

### 控制、快照和停止

- 目标采样记录发生在提交之前，不能当作 IK 接受或设备执行。记录 await 后检查控制代次、模式、故障、当前帧身份和帧龄，避免继续提交旧输入；正常日志不可用仍走原有记录故障处理。
- SDK 快照的 requested/start/completed 是本机读取边界，包含顺序读取和锁等待语义，不是设备传感器采样时间。读取用途标签没有引入额外 SDK 读取。
- `state_message` 仍按读取/故障观察、合成、记录、停止转换的顺序执行。消息在最后转换之前构造，返回模式可能与函数结束时模式不同；已有定向测试保留此语义。
- 静态引用复核显示显示快照/合成模块未被生产发送器调用；共享观察器仍只存在于测试。当前单 owner 拒绝第二控制连接，不能用假想多显示消费者的收益承诺当前降载。
- 保留 `stop_unverified`、RPC 创建后的 200 ms 截止、原锁顺序和停止验证；未加入自动 `stop_sys`。200 ms 不覆盖之前的任务 join 或 SDK 锁等待。

### 记录器和时间关联

- 新同步方法是可选普通事件入队接口，无异步回退。原异步/关键记录接口的错误仍传播；普通队列满、已关闭、写入失败或接口缺失可导致诊断丢失。
- 累计会话事件最多保留八条任务摘要；task_done 时间是回调观察时间。同步序列化与内存存储仍有 CPU 和队列容量成本，不声称零开销。
- 断连清理返回、owner 释放不能证明机器人停止。停止事件尚无 session_trace_id 外键；汇总器只能把唯一时间候选标作推断，缺失、重叠和冲突保留未知。
- 旧冻结日志没有新生命周期字段，不能补造取消或 join 时刻。最近一次已记录的旧日志分析为 0 个生命周期会话、2 个未关联停止；见[汇总器报告](offline-session-lifecycle-summary-2026-10-04.md)。本轮未重新读取原始日志。

### 前后端

- 本轮 14 个提交没有修改 wire schema、依赖清单或锁文件。`sample_age_ms` 保持 VR 输入接收年龄语义，UI 仅改明确文案；不是机器人遥测年龄。
- 更早的基准提交 `cd62b72` 已增加 `tracking_lag`。后端 Literal、JSON schema、前端联合类型/解析器及提示一致。将来若从更旧现场版本升级，须一起部署对应前后端，不能仅发后端新枚举。
- 显示原型 DTO 没有进入 WebSocket 协议，不能直接拿它替换原 RobotStateMessage。

## 4. 本轮验证

后端 32 个指定文件、735 项，28.39 s，退出码 0。覆盖控制采样/状态机/记录失败、映射/滤波/限幅、IK/PVAT/pump、停止事务、快照争用、会话取消与所有新增离线工具；未包含后端其余目录全集。

仓库根目录复现：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/diagnostics/test_session_trace.py backend/tests/api/test_session_diagnostics.py backend/tests/diagnostics/test_recorder.py backend/tests/diagnostics/test_store.py backend/tests/recording/test_commissioning.py backend/tests/api/test_observer_cancel_timing.py backend/tests/api/test_observer_session_lifecycle.py backend/tests/diagnostics/test_shared_observer.py backend/tests/control/test_state_observation.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_recording.py backend/tests/api/test_teleop_ws.py backend/tests/diagnostics/test_display_integration.py backend/tests/diagnostics/test_display_snapshot.py backend/tests/robots/test_snapshot_contention.py backend/tests/robots/test_stop_timing_boundaries.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py backend/tests/scripts/test_analyze_session_lifecycle.py backend/tests/scripts/test_analyze_vr_cadence.py backend/tests/scripts/test_stop_timeline_audit.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_vr_profile_experiments.py backend/tests/robots/test_lebai_pump.py backend/tests/contract/test_messages.py backend/tests/control/test_coordinate_mapper.py backend/tests/control/test_filters.py backend/tests/control/test_safety.py backend/tests/robots/test_lebai_ik_policy.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_pvat_request_lifecycle.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

首次前端命令使用 PATH 中 Node v12.19.0，发生 `node:` ESM 与空值合并语法解析失败；npm 更新检查还遇到本机配置文件 rename EPERM。改用本机已有 Node v24.19.0 直接运行已安装工具后，六文件 131 项通过（31.07 s），类型检查和构建退出码均为 0。未安装依赖、修改全局 PATH 或更改产品代码来绕过错误。

在 web 目录，以本机现有运行时复现：

```powershell
$taskNode = 'C:/Users/Kilo/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe'
& $taskNode node_modules/vitest/vitest.mjs run tests/hud.test.ts tests/diagnosticsPanel.test.ts tests/messages.test.ts tests/vrSafetyPanel.test.ts tests/controllerInput.test.ts tests/robotState.test.ts
& $taskNode node_modules/typescript/bin/tsc --noEmit
& $taskNode node_modules/vite/bin/vite.js build
```

Vite v7.3.6 构建保留约 743.27 kB JS chunk 提示（gzip 197.70 kB）。本轮没有浏览器交互或 VR 设备验证。测试与构建输出留在忽略目录 `.tmp/offline-acceptance-*`，构建产物不提交。历史差异 `git diff --check cd62b72..HEAD` 通过。

## 5. 尚未解决的证据缺口与下一步

| 问题 | 当前能说明什么 | 尚不能说明什么 |
|---|---|---|
| 平移幅度小 | 低增益、限幅、输入节奏、目标覆盖与背压分别有证据 | 各因素对现场末尾约 0.956 mm 读数的精确贡献；正常真机参数 |
| IK 背压 | fake 反馈变慢或停滞可复现目标推进受阻；无背压也可能不跟随 | 真实姿态/控制器反馈下的恢复效果 |
| 运动中停止迟到 | 冻结日志 SDK await 609 ms，另有前置等待；fake 案例保留未确认锁存 | SDK、网络、控制器各自耗时及物理停止时间 |
| 共享观察 | 多消费者模型可减少重复读；单连接对照无降载收益 | 当前现场性能提升；忽略取消的真实 SDK 行为 |
| 现场收尾 | 用户后来明确关闭电源 | 早于 Home 的旧收尾日志不能证明 Home 后收尾 |

下一项离线工作：用同一合成手部轨迹，分别改变 VR 新帧节奏、控制 tick 与发送耗时，量化固定 20 ms 步长对滤波/限幅目标推进的影响；记录每阶段和最新目标覆盖，包含暂停/过期边界。先建立当前行为的对照证据，不直接改积分规则、增益、阈值或默认观察器。现有冻结日志缺失部分消费与目标历史，实验不能冒充精确现场重放。

当前无须用户重新解释 Home 或提供已有目录。真机采集、动作、部署以及 push 仍等待用户分别明确授权。
