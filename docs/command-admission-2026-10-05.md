# 结构化入队回执与命令关联

## 已实现（本地，未部署）

`RealLebaiAdapter.submit_tcp(target, command_id)` 新增显式回执入口，返回不可变 `CommandAdmission`。它将“当前请求进入本地队列”与“之前异步处理留下的约束反馈”分开。检查权限、状态新鲜度、pump 故障/运行状态、入队和生成回执之间没有让出事件循环。

默认 `RobotControl` 继续调用旧 `command_tcp()`。旧入口通过同一检查/入队实现后，仍按原规则抛出旧约束错误，成功时仍返回 None。因此上轮复现的候选速度重置问题**尚未改变**；本次提供区分请求的接口基础，没有切换生产限幅器或取消约束后的重置。

## 回执契约

| 字段 | 含义与边界 |
| --- | --- |
| `command.command_id` | 调用者提供的命令编号；允许重复，不单独用于唯一识别 |
| `command.generation` | pump 的取消代次；不是 RobotControl 的权限代次 |
| `command.submission_id` | 当前 pump 实例内从 1 开始递增的成功入队序号；stop/disconnect/reconnect 不重置；新建适配器/进程不能沿用唯一性假设 |
| `replaced_pending` | 本次入队覆盖的待发请求引用；不包含已经开始执行的请求，没有覆盖时为 None |
| `previous_feedback.command` | 旧反馈对应的原请求引用；直接调用内部约束记录函数而未提供来源时显式为 None，不猜测当前命令 |
| `previous_feedback.code` | 既有约束码，如 ik_tracking_lag、ik_joint_jump、ik_unreachable |
| `previous_feedback.outcome` | catch_up、rejected、interpolated_advance，区分追赶旧目标、拒绝、部分推进 |

回执及嵌套引用均为 frozen dataclass，不随之后的处理而变化。它只证明某时刻本地入队，不证明 IK 接受、SDK 返回、远端执行或机械臂停止，也不赋予未来继续运动的权限。没有旧反馈不代表当前请求成功执行。硬错误仍抛异常，不入队、不产生回执。

回执保留的旧反馈可以来自旧 pump 代次，尤其在停止/重新预检之后；这是显式历史，不应当解释为当前目标的拒绝。成功完整推进会清除适配器最新约束反馈，但已返回的回执仍保留原值。

`PvatRequest` 的直接构造兼容原测试，缺省 submission_id=0；0 为未经过正常入队分配的保留值，不属于正常入队序号。

## 日志关联

保持现有日志种类与字段，增加：

- `pvat_sent`：generation、submission_id，以及实际被接受目标的 accepted_target_generation、accepted_target_submission_id。catch_up 的请求引用与被接受目标引用可以不同。
- `sdk_request_lifecycle` 的 move_pvat 记录：submission_id（原来已有 generation）。SDK request_id 仍是独立的 RPC 编号。
- 本次候选处理产生的 IK 拒绝/背压诊断：generation、submission_id，使用原请求引用；迟到记录仍指向原请求。

这些日志仍遵守原来的记录和失败处理，不作为可靠的逐请求结果总线；诊断可能丢失，入队回执也不会等待日志或 SDK。没有增加后台任务或 SDK 读写。

## 定向验证

新增 `backend/tests/robots/test_command_admission.py` **21 例**：

- 冻结回执、重复外部编号、pending 覆盖与 in-flight 区分；停止与重连代次/序号不复用。
- 回执在 IK 完成前返回；发送事件和 SDK 调用关联同一引用。
- catch_up/拒绝反馈指向旧请求，同编号的新请求仍可恢复；catch_up 的接受引用保持旧目标。
- 旧入口继续先入队后抛旧错误；两入口各覆盖状态过期、预检未就绪、pump 停止、真实停止失败锁存路径、readonly 五类门禁。
- 停止后旧反馈保留原代次，失效 IK 不发布新接受/反馈，缺失来源不伪造引用。
- fake 插值接受明确为 interpolated_advance，不被标成完全拒绝。

测试过程：首先观察到新模块缺失的 RED。首轮故障门禁夹具只赋 `_latched_fault`，未走真实停止失败状态转换，导致 1 例不符合前提；改为注入 stop_move 异常，验证真实 stop_unverified/预检禁用路径，未新增或放宽门禁。新增拒绝日志关联断言先因缺 submission_id 失败，补齐实现后通过。旧日志完整字典断言因新增两个字段失败，保留完整断言并补齐字段。

最终相关 11 个测试文件 **383 passed in 9.19 s**，包括上轮 14 个历史不一致特征案例，证明默认控制行为仍被保留。未运行后端全集，没有前端改动。作者自审，无独立第二审阅。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_command_admission.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_pump.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_lebai_backpressure_experiments.py backend/tests/robots/test_retarget_backpressure.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_braking_supervision.py backend/tests/control/test_robot_control.py backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 下一步

补齐逐请求的最终软件处理结果：覆盖、撤销、IK 拒绝、SDK 返回及失败，避免候选依赖可丢失的诊断日志或仅一份最新反馈。然后再评估候选速度历史何时提交/撤销；本次接口本身尚未解决这个问题。

仅注入 fake SDK 离线验证；没有现场配置/原始日志提交、推送、部署、真机连接、Home 或运动操作。保留 stop_unverified、200 ms 停止截止和禁止自动 stop_sys。
