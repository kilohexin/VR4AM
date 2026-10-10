# 应用停止链路与 native SDK 的本机集成验证

2026-10-10，Asia/Shanghai。BASE：`ce93fb0684b47c2686f6d715481d96fe1d62d252`。

## 本轮解决的证据缺口

前轮回环直接调用 SDK，结果标记 `application_stop_path_exercised=false`。本轮使用实际 `RealLebaiAdapter.stop`、`SdkLock`、`StopTransaction`、`StopRpc` 和已安装 native SDK，验证应用在收到不同模拟回复时的停止判定及锁存。生产控制源码未修改。

新增 `scripts/application_stop_loopback_probe.py`。只能连接脚本自建的 `127.0.0.1` 随机 IPv4 端口；无 target/config 参数，不读取现场配置、不发现设备。模拟配置固定在脚本内。服务器只接受 `stop_move` 及完整状态读取所需的固定空参方法；拒绝 PVAT、夹爪、Home、复位及 `stop_sys`。不创建真实运动，也不绕过 adapter 连接/预飞流程。

SDK 为 `lebai-sdk-asyncio 0.3.13`，二进制 SHA-256 与已记录现场二进制一致：

`ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`

本机客户端固定 `request_timeout=2` 秒，应用自身 200 ms 门限保持；这是回环参数，不是声明与所有现场连接参数相同。所有时序在同一本机进程使用 `perf_counter_ns`，未注入 fake clock。服务器 send 完成也不是客户端接收或控制器处理完成的证据。

## 实测结果

五种固定场景各三次，最终共 **15 组**。每组只发出一次 native `stop_move`，完成后连接均已关闭。下表为最终轮最小～最大值；应用耗时包含后续状态验证或故障处理。

| 模拟场景 | native SDK await / ms | 应用 stop 入口至退出 / ms | 软件判定 |
|---|---:|---:|---|
| 立即回复、固定 IDLE | 0.542～0.563 | 316.938～322.718 | 模拟状态验证通过 |
| 请求延迟 100 ms、固定 IDLE | 105.241～115.941 | 421.053～435.462 | 模拟状态验证通过 |
| 请求延迟 600 ms、始终 IDLE | 593.048～601.199 | 204.693～212.271 | `sdk_timeout:stop_move`、`stop_unverified` |
| 立即返回 RPC 错误 | 0.576～0.766 | 0.903～1.064 | `sdk_call_failed:stop_move`、`stop_unverified` |
| 立即回复、随后 MOVING 600 ms | 0.842～1.084 | 511.082～514.184 | `stop_incomplete` |

关键区别：**200 ms 限制停止 RPC 等待，不是整个 stop 函数的完成期限。** 原有状态验证要求至少 300 ms 连续稳定，并有 500 ms 验证预算。主机调度会让实际退出晚于请求的计时边界；本轮没有放宽这些门限。`asyncio.sleep` 的 100/600 ms 是请求延迟，不能当作 `perf_counter` 的严格下界。

在 600 ms 晚回复场景中，应用已超时退出，完整只读查询仍在停止回复发出前完成；SDK 后来记录 `returned_late`，锁存仍为 `stop_unverified`，预飞仍被拒绝。

在及时 ACK 但仍 MOVING 的场景中，验证阶段没有把零关节速度或 ACK 当作停稳。超过验证预算后锁存 `stop_incomplete`；等待模拟状态变为 IDLE 后再次读取和预飞，也没有清除故障。输出分别记录 `raw_robot_state` 和应用公开状态：后者在锁存后为 FAULT，不能混称为 IDLE。

上述 IDLE/MOVING 和关节/TCP 数据均来自本机模拟服务器，不能用作机械臂物理状态证据。

## 失败、测试与审查

- 初始 17 项协议/固定配置用例为红回归；实现后通过。
- 首次 native 矩阵因 `timely simulated stop failed` 中断，未写出逐 case JSON，错误日志保留。检查发现诊断判定遗漏了及时 ACK 后持续 MOVING 应得到的 `stop_incomplete`；补判定回归后修正诊断脚本，没有修改生产停止策略。最初该新测试因判定函数缺失失败，随后对旧判定的回归明确以 `timely simulated stop failed` 失败，两份日志均保留。
- 修正后完成一轮 15 组结果；补充原始/公开状态字段后，再完成最终 15 组。两轮成功结果均保留，表格仅使用最终轮。
- 独立核对脚本最初误把锁存后的应用公开状态预期为 IDLE。核对现有状态映射后改为原始 IDLE / 公开 FAULT，并补明确的原始状态输出，生产状态映射未改。
- 最终 **8 个定向文件，115 passed，5.33 秒，退出 0**；含新增 32 项协议、CLI/目标保护、输出保留、失败判定及回环读回复用例。pytest 不导入 native SDK；native 矩阵作为单独本机实验运行。未执行后端全集。
- 独立只读静态审查未发现本轮可操作 P0–P2，审查未运行测试、SDK 或网络连接。

## 证据与结论边界

本机 `.tmp/application-stop-loopback-20261010/` 冻结 18 项材料，包含失败记录、两轮成功矩阵、最终 pytest/JUnit、独立核对、审查摘要和源码副本。清单 SHA-256：

`55ada8c6f68452943bcf68f980bb69ec58a817868258968f017ceb695e368db1`

统计、调用数量及时间顺序已独立重算；此前七批共 148 项冻结材料逐项 size/hash 保持。adapter、PVAT、RobotControl、SdkLock 及原现场配置哈希保持。实验原始结果和现场材料不提交，用户既有项目概述保留，暂不推送。

本轮证明的是这些模拟条件下，生产停止路径与已安装 SDK 的超时、稳定验证和锁存行为一致。没有真实运动、PVAT 竞争、无线网络或控制器内部时序，不能证明 SDK 全路径，更不证明物理停稳。

**现场约 613 ms 回复的根因仍未确定。** 现有回环证据不能继续区分控制器处理、网络回复传输或 SDK 的条件路径。进一步归因需要既有控制器内部日志，或具有明确区分变量的新单次现场范围及新鲜授权；本轮没有新增真机连接、动作或复用旧授权。持续真机 VR 和真实 Trigger 抓取仍待验收。
