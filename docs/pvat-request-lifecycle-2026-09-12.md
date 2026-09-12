# PVAT 请求生命周期诊断（2026-09-12）

## 目的与范围

旧的 `pvat_sent` 在 SDK 返回后仍会检查请求代次；停止操作使代次失效时，会提前返回而不写该事件。因此缺少 `pvat_sent` 不能证明 SDK 没有收到调用。

本次仅补诊断，不调整运动参数、60ms PVAT 等待上限、停止策略、停止超时、关节限制或预检放行条件。不连接真机，不恢复实验室运动测试。

## 新事件

`sdk_request_lifecycle` 针对每次实际进入 `move_pvat` 调用的请求记录：

- `request_id`：适配器实例内单调递增编号，须结合会话识别，不是控制盒运动 ID。
- `command_id`、`generation`：对应控制命令与代次。
- `started_ns`、`completed_ns`：本机单调时钟时间，不是控制盒执行时间。
- `p`、`v`、`a`、`horizon_s`：调用参数。
- `rpc_outcome`：`returned`、`timeout`、`cancelled`、`error`；异常另记录 `error_type`。
- `invalidated`：调用结束时请求是否已失效，独立于 RPC 结果。

调用前只更新内存；无论正常返回、失效提前返回还是异常退出，都在退出 SDK 锁后调度后台记录，不等待记录完成才上报运动错误。每个适配器最多同时保留 16 个诊断任务，满额丢弃并累计计数；后续事件通过 `diagnostics_dropped_total` 暴露满额丢弃数量。50ms 后请求取消慢任务，写入失败不替换运动错误；原有取消继续传播。正常情况下原有 `pvat_sent` 保留。断开连接时最多等待 50ms 收尾诊断任务。

## 证据边界

`returned` 仅表示 SDK 调用返回；`timeout` / `cancelled` 仅表示本机等待结束，不能证明控制盒取消运动。`invalidated=true` 不表示远端撤回成功。

事件是调用结束后的汇总，不是调用前同步落盘。进程强制终止、记录器失败或超时仍可能丢失记录，缺失事件不能证明没有发出请求。该诊断与独立进程观察脚本互补，不能代替物理停止确认。

正常发送路径增加任务调度和日志量，不能宣称完全零开销。控制循环不等待后台回调；吞掉取消的回调会持续占据有限任务槽位，阻塞整个事件循环的同步回调仍不受异步超时约束。取消或写入失败造成的丢失不计入满额丢弃计数。

## 离线覆盖

测试覆盖失效期间的返回、超时、取消和异常；正常发送的独立编号与原事件保留；诊断阻塞时 SDK 锁可获取且原始超时错误仍传播。测试使用本地假 SDK，不证明控制盒停止问题已解决。

### 记录器与进程观察集成验收

新增 `tests/commissioning/test_pvat_lifecycle_recording.py`：假 SDK 挂起 PVAT 返回，随后调用真实 `adapter.stop(GRIP_RELEASED)` 使请求失效，再释放 SDK 返回。使用真实 `CommissioningRecorder` 检查落盘的生命周期事件、停止诊断和会话汇总。该场景没有 `pvat_sent`，但必须有 `sdk_request_lifecycle`，且结果为 `returned / invalidated=true`。

反证检查：仅在独立测试进程中把 `_schedule_sdk_diagnostic` 替换为空操作，测试按预期因缺少事件失败；未改写生产源码。正常运行恢复通过。

通过 `scripts/observe_process.py` 包裹这条集成测试、7 个生命周期测试和 2 个停止超时测试，10 项全部通过。原始输出与进程状态保存在 `artifacts/acceptance/pvat-lifecycle-integration-20260912/`，该目录是本地验收产物，不随 Git 自动发布。

注意：记录器分别维护普通与关键事件队列，文件行序未必等于采集顺序。`session_ended` 不一定是最后一行；分析应结合事件内时间、请求编号和语义，不以单纯行序重建控制盒执行顺序。

回归命令（在 `backend` 目录运行）：

```powershell
python -m pytest tests/robots tests/commissioning tests/prototypes tests/scripts/test_observe_process.py -q --tb=short
```

## 实验室状态

本次不是停止故障修复验收，不据此恢复 B01、B 段或 VR 真机遥操作。源码尚未提交或推送时，实验室不能通过拉取获得这些诊断。后续须先固化版本、核对离线诊断产物，再单独确认现场执行范围。
