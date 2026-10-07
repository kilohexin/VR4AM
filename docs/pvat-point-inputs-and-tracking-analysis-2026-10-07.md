# PVAT 输入溯源与保存反馈的关节贡献核对

2026-10-07，Asia/Shanghai，起点 `3f7176444140074094deabce45f50f520ecbe644`。继续用户授权的离线开发。本轮增加诊断字段和定向测试，没有真机连接、部署或推送；原现场授权已消费。停止 610 ms 迟到、`stop_unverified` 与 200 ms 截止保持，没有据此进入 VR／夹爪验收。

## 保存反馈进一步定位

沿用 [上一轮诊断 MDH 与保存工具变换](offline-mdh-path-analysis-2026-10-07.md)，不拟合模型或替换控制运动学。输入仍是早停原日志 SHA-256 `f6bdc1375bc713bfacdf172b868d8e2676815fbee4a091c3eda78a276a70dcb6`。以下时间零点是该日志首个 `robot_kinematics` 的客户端读取开始；**不是停止请求，也不是控制器时钟**。

| 关节 | 首次保存实际位置变化的读取窗口 | 该轴首次变化 |
| --- | --- | ---: |
| J3 | +515–531 ms | -0.00009587 rad |
| J2 | +578–594 ms | +0.00009587 rad |
| J4 | +781–797 ms | +0.00009587 rad |
| J1 / J5 / J6 | 本组 81 个常规运动学记录未变 | 0 |

这是离散报告的位置变化顺序，不能宣称关节实际启动时间不同。首次实际 TCP 的 X 变化为 +0.03755989 mm，只有保存 J3 发生变化；诊断 FK 对应 +0.03756071 mm。该记录的控制器规划 TCP X 相对起点约 +0.00002080 mm。

实际 X 峰值的读取窗口是 +1015–1031 ms。同一记录的保存反馈可精确拆为：

| 项目 | X 分量 |
| --- | ---: |
| 规划 TCP 相对实际起点 | +0.22345697 mm |
| 实际 TCP 相对同记录规划 TCP | +0.36094352 mm |
| 实际 TCP 相对实际起点 | **+0.58440049 mm** |

为了避免单一替换顺序的归因偏差，在诊断模型中对所有关节替换顺序取平均（Shapley 分解），核对峰值处从首个实际关节角到当前实际关节角的贡献：

| 关节 | 关节相对变化 | 数学分解的 X 贡献 |
| --- | ---: | ---: |
| J2 | +0.00210922 rad | -1.41685130 mm |
| J3 | -0.00517719 rad | +2.02812880 mm |
| J4 | +0.00019175 rad | -0.02685116 mm |
| 其它三轴 | 0 | 0 |
| 合计 | — | +0.58442634 mm |

模型贡献之和与保存实际 TCP 的变化接近；这支持关节配合与跟踪对应关系是重点。贡献是模型里的数学分解，没有确定哪个电机、控制器或应用模块造成问题。另在 +656–672 ms 的记录里，规划 X 接近零而实际／规划 X 差达 +0.40782975 mm；后续受限点继承已有实际偏差的结果保持，不能只看最终端点就分配物理原因。

## 反馈网格的扰动对照

81 个保存实际关节角与数值网格 `2π/65536 ≈ 0.0000958738 rad` 一致，最大网格残差约 3.64e-12 格。这是保存数值特征，没有厂家编码器精度／反馈舍入规范。

在每个姿态给六轴分别施加正负半格或一格误差，计算 64 个角点：半格对照最大绝对 X 变化约 0.089259 mm，一格约 0.178520 mm，分别各 5,184 个角点。量级小于 0.584400 mm 峰值；不能单凭网格就把偏差解释为量化噪声。上述是假设误差的角点采样，不是已证明的物理误差界或连续区域最大值，没有把它升级为测量准确度结论。

## 端点速度兼容性：条件性结果

对每条点提出可证伪的条件假设：保存速度对应读取窗口中的某时刻，PVAT 在本次客户端 SDK await 窗口内开始，并在 T 后达到参数 v，整段服从 0.2 rad/s² 的加速度上限。为给该假设最长时间，使用 `Δt = T + (SDK完成 - 快照读取开始)`；必要条件为 `|v端点 - v快照| ≤ 0.2 × Δt`，时间均换算成秒。

- 25 条记录的请求 a 最大绝对值仍为 0.2 rad/s²。
- 从保存 actual_qd 出发，**22/25** 不符合该条件，所需平均绝对加速度最大约 0.526316 rad/s²。
- 从保存 target_qd 出发，同样 **22/25** 不符合，最大约 0.396711 rad/s²；首次为 submission 6。

这是对假设的兼容性失败，不是已证明的真机加速度超限。控制器接收／开始边界、缓存策略、反馈采样时刻和 PVAT 终点语义尚未验证；请求 a 的数值合格不能单独证明整段插补遵守相同上限。下一步离线对照必须明确初始状态、时序与插补假设，再检验速度参考选择；不会凭此修改运动策略、上调速度或延长停止截止。

## 针对证据缺口的代码改动

现有日志能按数值重放 p/v/a，但旧日志不记录构造器实际选择的速度参考来源。同值的 observed / previous 两种来源可能完全不可区分；当前阶段仅靠相邻 SDK 开始时间也不能还原私有决策时刻。

这次限定为诊断补强：

- `PvatPoint` 保存不可变 `reference_qd` 与 `reference_source`。来源标记 `observed_velocity` 或 `previous_command_velocity`，直接取构造器本次传入的分支与最终参考向量。
- `sdk_request_lifecycle` 和 `pvat_sent` 添加 `point_inputs`：快照读取申请／开始／完成时间、actual q/qd/qdd、controller target q/qd/qdd，以及上述参考来源和向量。
- 每条记录生成独立列表，记录器修改一条记录不会污染另一条或冻结点。SDK await 超时、取消、异常、代次失效仍能从现有有界诊断路径保留本次输入；没有伪造成功 pvat_sent。

没有新增 SDK 读取或时钟读取，没有改变 horizon 判定、IK 选择、参考选择、p/v/a 计算、SDK 参数、停止顺序或锁存。增加了固定数量的六轴日志字段和分配；未实测现场日志开销。该元数据只是本地构造输入，不能叫作控制器接收、执行或插补初始状态。

## 验证与本地证据

先写 15 个案例，红灯运行全部因缺少 `reference_qd/reference_source/point_inputs` 失败（两次运行，第二次使用短 traceback 核对全部原因），不是 SDK 或现场失败。实现后 15 项通过，再补两项不可变／可变输入隔离和插值选择覆盖，**17 项通过，1.05 秒**。

受影响的 9 个文件 **236 passed，24.45 秒**：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_pvat_point_inputs.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/robots/test_explicit_catch_up.py backend/tests/robots/test_command_completion.py backend/tests/robots/test_lebai_stop_diagnostics.py backend/tests/robots/test_stop_transaction.py backend/tests/commissioning/test_actions.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

覆盖参考同值来源、负时间／0／79／80／120 ms 边界、advance／interpolated_advance／显式 catch_up、SDK 返回／超时／取消／异常后的代次失效，以及记录隔离。fake SDK 验证无额外运动学读取、IK 和运动写入，原请求参数与真实构造器一致；没有机器人网络或发现调用。按用户要求仅运行相关回归，没有后端全集或独立审查。

修改后的构造器再次对原 25 条保存点重放，p/v/a **逐分量最大残差 0**；原日志哈希不变。复制原重放脚本仅替换输出文件名，保留历史脚本和报告；没有向旧日志补造新增字段。

本机忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/`：

| 输出 | SHA-256 |
| --- | --- |
| `tracking-contribution-analysis.json` | `02a9c2203a02b432af158d280d63bd3c8cb6f61a5959906684752c0105d372aa` |
| `endpoint-velocity-compatibility.json` | `53a079197dd2ec40f8aa6debed64f37ffd161b15db03f558ee3d414ea11f0495` |
| `pvat-stage-replay-after-point-inputs.json` | `d1348acf1f4e05e4e0cd848c4095d2a0a62c905fbd8582de95db378c1060c00a` |

`verify_tracking_analysis.py` 退出 0：再次核对输入／分析源码哈希、三个选中样本的反馈分解和关节贡献、25 条条件核对与 25 点重放；Shapley 另以全部 720 个替换顺序独立核对，最大差约 1.47e-14 mm。原始日志、配置和完整分析输出留在本机，Git 只保存代码、合成测试与摘要。
