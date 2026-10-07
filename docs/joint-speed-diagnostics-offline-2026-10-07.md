# 关节速度偏差：离线核对、验收与下一次只读范围

2026-10-07，Asia/Shanghai。接续 [单次准备位姿实机结果](prepare-motion-result-2026-10-07.md)。用户要求继续离线工作，直到需要新真机连接再通知。本轮没有真机连接或机器人动作，未推送。**速度偏差的物理原因仍未确定，运动中停止仍未验证。**

## 1. SDK 源码与实际二进制核对

官方 `lebai-sdk.rs` 的 `v0.3.13` 标签指向 `79bc9bba3cc6d710c9c7d3f40f0fe01742911187`，Cargo.toml 声明 0.3.13。Python 包装直接转发 movej 的 p/a/v/t/r，Rust RPC 将 a/v/t/r 对应到 acc/velocity/time/radius，然后调用 move_joint，没有这段路径中的速度倍增。[固定版本 movej 源码](https://github.com/lebai-robotics/lebai-sdk.rs/blob/79bc9bba3cc6d710c9c7d3f40f0fe01742911187/src/rpc/motion.rs)。运动学转换直接包装 actual_joint_speed、target_joint_speed 原数组；JointPose 包装不换算这些值。[固定版本遥测转换](https://github.com/lebai-robotics/lebai-sdk.rs/blob/79bc9bba3cc6d710c9c7d3f40f0fe01742911187/proto/src/serde/kinematic.rs)。源码核对不等于本次真机请求的网络捕获。

本机保留的 Windows wheel SHA-256 为 `22ceb57ad81e379307b8e63a4ef1c3bd87f674829435eb467aac2ab7d78e946f`，与 [PyPI 0.3.13 发布元数据](https://pypi.org/pypi/lebai-sdk-asyncio/0.3.13/json) 一致。wheel 内二进制与已安装二进制 SHA-256 均为 `ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b`，SDK.version 返回 0.3.13。没有升级或重新安装依赖，也未编译下载源码。

进一步执行**安装二进制到本机伪服务的回环核对**：伪服务只绑定 127.0.0.1 的系统分配端口，SDK 连接目标固定为此地址和端口，不读取现场配置、不扫描设备、不使用真机 bridge 或控制 adapter。首先验证一个合成 movej，捕获 JSON-RPC `move_joint`：`velocity=0.05 / acc=0.2 / time=0 / radius=0`，伪服务返回 motion ID 111。扩展核对的请求列表严格为 move_joint、get_kin_data、get_kin_factor；合成 actual/target 速度 0.105/0.05 原样返回，合成倍率返回整数 100。二者均是伪服务数据，不是当前机器人数据。

这验证了已安装二进制的该组参数序列化与遥测转发，缩小了应用/SDK 参数倍增这一假设的范围；不能补造之前真机的网络载荷，不能确认控制器内部采样、速度计算或物理速度。没有根据结果调整 SDK 版本、阈值或速度参数。

## 2. 保存遥测的跟踪与量化核对

原始 session SHA-256 仍为 `34747046958f96dff74edbe8d8d5caec742fbe5c0bde40f5423036ed2e29629a`，318,099 字节。新正式离线工具对相同日志报告：**186 条 joint_move 采样，0 条无效，50 条实际速度超限，峰值 J5 0.105461 rad/s；规划速度最高约 0.05 rad/s**。这里按用途筛选运动采样，原报告的 200 条包含连接/预检/末尾状态等，两个计数没有矛盾。

保存实际关节角与 `2π/65536` 网格的最大残差约 3.64e-12 个网格步长；实际速度与 `100×2π/65536` 网格的最大残差约 1.78e-15 个速度步长。这是对数值网格的核对，不证明编码器规格或采样周期为 10 ms。

J5 实际减规划关节角范围约 -0.010161 到 +0.000203 rad；199 个相邻观察区间中，21 个实际位置差为零，负位置差为零。按客户端读取完成时间计算，有 75 个区间的正位移速度估算超过 0.05，其中 59 个伴随实际相对规划的落后减少。它支持“落后后追近”的遥测模式，并未确定是实际跟踪响应还是采样/数据更新影响，也不等于诊断出了伺服故障。原较长窗口仍超过请求的结果保留，不能仅用量化解释为无害噪声。

## 3. 已完成的针对性改进

- **共享 prepare/Home 的 movej 调用记录**：加入本地请求 ID、p/a/v/t/r、起止时间、returned/timeout/error/cancelled，以及成功返回的 motion ID。异常仅保存类型，不保存可能敏感的异常文本。使用已有有界、尽力记录机制，错误仍先走原停止处理，不在停止前等待日志；RPC 计时不含停止清理耗时。记录可能丢失，属于本地 SDK 调用证据，不能称远端执行确认。
- **纯离线速度分析器**：`scripts/audit_joint_move_speed.py`，只读已有 JSONL，不导入 SDK、不访问设备。显式提供速度阈值，输出原日志哈希、有效/无效及超限计数、每轴峰值和客户端时间边界。报告用新文件保存，不覆盖原始数据。仅 1e-12 量级的浮点比较容差，未引入物理超速宽容值；异常向量、NaN、布尔值、超大数、时间倒退和无运动记录都不能得到“观察值在限内”的结果。归约本身使用固定内存；CLI 为解析/hash 会读入原文件。
- **prepare 的到位后速度验收**：在原到位和后检结束后，记录关键事件 `joint_move_speed_audit`。速度超限或采样不完整则报错、不返回成功 smoke_result；成功结果携带独立速度摘要。到位稳定与速度判断分开。此处没有增加 SDK 查询或停止，没有把它实现为运动中实时超速停止器，没有修改运行 Home 的入口/策略。物理速度保证与运动中停止字段始终为 false；原停止截止和锁存规则保持。
- **只读倍率选项**：现存只读脚本新增显式 `--include-velocity-factor`，仅初始白名单末尾增加一次 get_velocity_factor，后续仍只轮询状态/急停/运动学/夹爪。默认七查询保持，默认 CLI 仍是离线预览。倍率必须为 SDK 声明的整数，保留原值；没有自行把它解释为批准的速度上限、归一化比例或历史值，没有 setter 或通用 RPC 回退。

## 4. 定向验证与自查

新增 **50 项**：离线速度分析 30、prepare 到位后验收 4、movej 调用记录 9、只读倍率 7。先确认缺少实现的失败，再补实现。prepare 正/负超限和缺失采样在旧实现中仍返回成功，失败被确定性复现。调用记录旧实现的 8 个入口/结果组合都缺少事件。最初一个诊断测试未提供 HomeOptions 必需参数，已修正 fixture；最初“session_ended 必须为文件最后一行”断言忽略关键队列优先写入，已改为核对结束事件恰好一次。无隐瞒失败。

分析器的新增边界用例还暴露超大整数触发 OverflowError、读取完成时间倒退被误判在限内；已分别按无效证据处理并通过回归。

12 个受影响文件 **250 项通过，12.52 秒**；另 3 个 SDK 生命周期/入队/旧日志分析兼容文件 **38 项通过，1.43 秒**。共 **288 项**，未跑后端全集、前端构建、头显或真机。命令和输出保存在本机 `.tmp/joint-speed-audit-regression.txt` 与 `.tmp/joint-speed-compatibility-regression.txt`。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_joint_move_speed_audit.py backend/tests/scripts/test_prepare_speed_acceptance.py backend/tests/robots/test_joint_move_diagnostics.py backend/tests/robots/test_joint_move_speed_limit.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_lebai_adapter_readonly.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_lebai_stop_diagnostics.py backend/tests/scripts/test_real_robot_scripts.py backend/tests/scripts/test_readonly_state.py backend/tests/scripts/test_readonly_velocity_factor.py backend/tests/config/test_low_speed_profile.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_command_admission.py backend/tests/scripts/test_analyze_vr_cadence.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

已做主作者 diff 自查，尤其核对异常停止顺序、有界诊断、请求 ID 共用、默认只读行为和旧日志分析按 method=move_pvat 过滤。没有独立审查。软件回归与本机回环成功均不证明物理限速已修复或停止已验证。

## 5. 下一次需要新真机授权的范围

下一步只提出 **一次连接、约 5 秒只读采集**，沿用原 readonly 配置的独立副本，额外读取一次全局速度倍率。初始八个 SDK 查询为状态、急停原因、运动学、工具 TCP、夹爪反馈、负载、重力、速度倍率；之后仅按原周期轮询前述状态/急停/运动学/夹爪四项。连接等待 5 秒、单查询最多 1 秒且受采集窗口剩余时间约束；异步取消仍不是硬实时保证。查询失败/非 IDLE/急停/配置变化结束，无重试或自动动作。用独立 CLI 进程和新报告，结束后核对进程退出与本机连接收尾。

该范围不构造控制 adapter，没有 prepare/Home/复位/运动/VR/夹爪写入/stop/stop_sys 或服务部署。需要用户另行批准新连接并重新确认当前上电、IDLE、目视静止及现场有人观察。此处只是准备范围，未执行。倍率读数只能代表新采集时的状态，不能证明刚才运动时倍率相同，亦不能单凭一次只读解决运动速度或停止问题。

本机忽略目录 `.tmp/prepare-window-20261007T134429-4nlfxght/` 保存 SDK 源码哈希清单、wheel/二进制比对、量化/跟踪分析、正式速度报告、回环脚本与两个伪服务报告。原始日志、现场配置、原始输出和下载源码均不提交。此前单次 prepare 授权已消费，本轮没有延续使用。
