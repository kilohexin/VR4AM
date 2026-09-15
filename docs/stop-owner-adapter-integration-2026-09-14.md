# 停止请求所有权：适配器级离线集成验证

## 范围

本轮运行真实 `run_smoke → LebaiAdapter → 记录器 → 停止观察` 流程，在 SDK 边界注入测试专用 `OwnedStopClient`。不替换生产停止方法，不连接机器人，不读取现场 YAML，不改变 `backend/app/`。

新增文件：

- `backend/tests/prototypes/owned_stop_client.py`：按 stop_move / stop_sys 分别持有一个请求，调用方取消不取消该请求；关闭测试时才清理本地任务。
- `backend/tests/commissioning/test_owned_stop_integration.py`：四种场景的真实应用流程集成测试。

这不是可用于真机的客户端：不实现跨运动周期重建、SDK 连接退出策略或系统失能的决策，严禁接入生产 client_factory。

## 验证结果

| 场景 | 结果 |
| --- | --- |
| 停止 RPC 迟到成功 | shutdown 复用请求；结果记录为 RETURNED_LATE；原 stop_unverified 不解除 |
| 停止 RPC 一直未返回 | 保持 UNKNOWN；不因 shutdown 新发同名请求 |
| 停止 RPC 迟到异常 | 保存 FAILED 与异常内容；不自动重试、不当作成功 |
| 停止观察中一次连接读取异常 | 后续仍有采样、session 正常收尾；read_errors=1、complete=false，不伪装为完整通过 |

四种场景都经过实际平移控制路径并产生 pvat_sent，随后停止失败继续抛出；停止后没有新增 PVAT 写入。注入的位置变化仍被观察记录捕获，故障锁存保留。

对照实验中，不加请求所有权时可观察到 stop_move、stop_sys 各发两次；加入测试边界后各发一次。两次上层 stop_diagnostics 仍保留，因为两个调用方确实尝试等待了停止结果。生产诊断日志目前不能自动区分“发送新请求”与“复用同一个请求”，后续集成必须补齐这一层映射。

## 本轮没有解决的事项

1. **不同方法仍可能重叠**：stop_move 未返回时，stop_sys 仍会进入执行；测试明确保留并断言这个事实。去重不等于解决系统失能或停止后位移。
2. **重复等待预算仍在**：shutdown 复用请求但上层仍开启新的等待窗口；尚未实现整个停止事务统一 deadline。
3. **连接生命周期尚未覆盖**：一次读取抛 ConnectionError 不是完整断网、SDK 重连或进程退出模拟。真实断连下未决 RPC 的处置仍需设计。
4. **异常优先级线索**：初次注入 ConnectionError 发生在 `_wait_for_stable_state` 的读取而不是观察中，抛出 `sdk_call_failed:get_kin_data` 而非 `smoke_stop_failed:stop_unverified`。这可能掩盖对外主错误口径，未在本轮修改生产处理。正式用例将读取异常放到观察阶段以验证指定边界，不把前一个发现当作已修复。
5. **不验证机械响应**：假客户端的位置变化是人为注入，不能证明真机漂移原因，更不能证明去重后漂移消失。

## 测试纪律

先运行无所有权的透传边界，两项原始集成用例按预期失败：实际四次停止写入，不满足仅两次写入。随后实现测试边界，两项通过，再补迟到异常和观察读取异常。

读取异常用例最初错误地要求 complete=true；核对观察器源码后修正为 complete=false，同时要求 interrupted=false、后续采样、读取错误计数及 session 收尾。这是忠实验证现有不完整判定，不是改变生产代码将错误变成成功。

复核命令（在 backend 目录，用项目 Python）：

```text
python -m pytest tests/commissioning/test_owned_stop_integration.py tests/commissioning/test_stop_timeout_lifecycle.py tests/prototypes/test_stop_request_owner.py tests/prototypes/test_stop_episode_model.py -q --tb=short
```

共 29 项：4 项新增集成 + 2 项原有故障特征 + 5 项异步所有权 + 18 项状态模型。通过含义是这些离线契约符合断言，不是真机验收。

## 下一步

应优先确定生产停止事务的统一身份与截止时间，以及 stop_move 结果未知时如何处理系统失能。不能只把测试包装器移入 app，也不能直接删除 stop_sys 或增加超时。生产设计应将“请求结果未知”“停止已确认”“观察有缺失”“允许再次运动”分开处理，并在诊断中保留 request_id 和复用关系。

实验室继续维持既定停测边界，本轮不需要新增真机测试或恢复 IDLE。
