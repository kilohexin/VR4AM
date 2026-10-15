# Native Cartesian Jog Region Prediction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan in the current session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将准备位单点分析扩展为含姿态变化和 Jdot 曲率项的纯离线区域预测，为单次 speedl 诊断准备可测试的模型检查。

**Architecture:** 新增一个独立纯数学模块，复用现有理论 MDH 参数和不可变工具定义。保持旧单点函数、真实适配器、只读入口、原生 SDK 与网页/VR 行为不变；区域预测结果不授予任何运动权限。

**Tech Stack:** 现有 Python 3.12、NumPy、pytest；不安装或升级依赖。

**Spec:** [已认可的 speedl 架构](../specs/2026-10-15-native-cartesian-jog-design.md) 第 5 节；算法依据为[本轮区域推导](../../native-cartesian-jog-region-derivation-2026-10-15.md)。这是单次真机阶段之前的模型子任务，未包含真实执行入口或现场验收。

**执行状态：** 用户已审阅并认可，由当前会话直接实施，离线模块及验证已完成（78 项新增合成测试，相关回归 468 passed）；结果见上述推导文档的实现章节。本文件不授权任何机器人连接或写操作。

## Global Constraints

- 输出仅适用于固定工具和理论 MDH 模型；不把个体标定、控制器插补、物理关节速度或制动标记为已验证。
- 输入速度、加速度和关节比较限值为有限正数。关节盒各轴半径须 `0<r<=0.01 rad`；起始偏差各轴须 `0<=e<r`；时长显式为 10–80 ms。拒绝 bool、NaN、Inf、维数错误和溢出。
- η 不可收缩、奇异或非有限结果拒绝计算；不使用伪逆、自动降低速度、扩大区域或改变限值掩盖失败。
- `hardware_verified`、`joint_bounds_verified`、`braking_verified`、`continuous_control_admitted` 固定 false；模型比较满足也不能修改这些字段。
- 不修改现场配置、不导入 SDK、不连接设备、不新增生产运动选择器、不推送。原始姿态和日志不加入测试或 Git。
- 定向测试，保留 RED→GREEN 证据；不例行运行后端全集。仅验证模型，不把采样覆盖表述为均匀区域的物理证明。

## Review Focus

1. 非零速度但平移加速度很小：Jdot 项仍产生非零关节加速度；对应任务 1 的曲率回归。
2. 初始姿态已经接近盒边：速度检查通过也必须检查 `e+Vt<r`，对应区间退出回归。
3. 近奇异、大盒或不同工具：失败必须显式，不能用参考点伪逆；对应奇异/非收缩/工具测试。
4. 旧单点分析低估整个区域：区域参考可能不满足比较限值，不能静默裁剪；对应区域与单点差异回归。
5. 上游数组或返回数组被修改：再次计算不受影响；对应复制和工具不可变回归。

## Files and Interface

| 文件 | 职责 |
| --- | --- |
| `backend/app/commissioning/native_jog_region.py`（新） | 纯离线区域预测和严格输入检查 |
| `backend/tests/commissioning/test_native_jog_region.py`（新） | 合成测试，不含现场姿态 |
| `docs/native-cartesian-jog-region-derivation-2026-10-15.md` | 补充实现验证和限制 |

模块接口：

`assess_translation_region(q, tool: MdhTool, *, joint_radius_rad, initial_offset_rad, duration_s, cartesian_speed_mps, cartesian_acceleration_mps2, joint_speed_limit_radps, joint_acceleration_limit_radps2) -> dict`

q 为六个普通有限数值；radius/offset 为六分量列表或元组。复用 `native_jog_mdh` 的理论 `_A/_D`，不可维护第二份不同的链参数。对当前模型 J(q) 求逆，用推导中 L、M、V、A 计算区域参考；`η>=1−1e-12`、秩不足、线性求解失败或非有限/负参考量抛出 ValueError。

输出包含：`scope='theoretical_region_only_not_controller_or_braking'`、`model='manufacturer_theoretical_mdh_not_factory_calibration'`、`perturbation_contraction`、`joint_velocity_bound_radps`（6 分量）、`joint_acceleration_bound_radps2`（6 分量）、`max_reference_excursion_rad`（6 分量 e+Vt）、`model_rates_within_limits`、`reference_segment_within_region`、`model_reference_eligible`（前两者均满足）、`directed_rounding=false`、`factory_calibration_error_bounded=false`，以及 Global Constraints 的四个固定 false 字段。返回全新普通 JSON 兼容对象，不保留输入可变数组别名。

## Task 1: 区域预测模块与定向验收

- [x] **RED — 基本行为：** 在新测试文件使用既有合成 Q/TOOL，不复制现场数据。`test_region_covers_sampled_directions_and_curvature` 对至少 64 角点、轴向/斜向速度和加速度，用现有独立基本变换或数值 J/Jdot 对照，断言没有超过预测 V/A；明确采样本身不是证明。
- [x] **RED — 必要拒绝：** 参数化非法 q/r/e/速度/加速度/时长、零半径、e 达到半径、超大半径、奇异参考点、非收缩区域。输入拒绝不产生任何权限对象。
- [x] **RED — 反例保护：** `test_curvature_survives_near_zero_cartesian_acceleration`，非零速度时输出 A 仍大于仅含 b·a 的初始参考；`test_reference_can_leave_region_despite_low_rates`，放宽比较限值也不改变 e+Vt 越界结果；`test_model_success_never_admits_hardware`，模型通过仍四个固定 false；`test_excess_reported_without_clipping`，比较限值设为很小，输出明确不满足且输入未被裁剪。
- [x] **运行 RED：** `.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_region.py -q -p no:cacheprovider`。保存本机 XML，确认失败由缺少新模块或行为造成。
- [x] **GREEN：** 按上面接口和推导实现最小模块；曲率项使用区域 V 计算，不用参考点单一 qd 替代。优先 `np.linalg.solve`，普通双精度计算如实标注未定向舍入。实现输入/输出复制和错误原因；不改旧接口。
- [x] **定向回归：** 执行下列完整命令，预期全部通过；保存 JUnit、检查所有失败，不运行全后端。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/commissioning/test_native_jog_region.py backend/tests/commissioning/test_native_jog_mdh.py backend/tests/commissioning/test_native_jog_packet.py backend/tests/commissioning/test_native_jog_candidate.py backend/tests/commissioning/test_readonly_jog_jacobian.py backend/tests/commissioning/test_readonly_kinematics.py backend/tests/scripts/test_native_jog_loopback_probe.py backend/tests/scripts/test_readonly_jog_jacobian_entry.py backend/tests/scripts/test_readonly_kinematics_entry.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_motion_call_ownership.py backend/tests/robots/test_gripper_call_ownership.py backend/tests/robots/test_gripper_sdk_entry.py backend/tests/robots/test_gripper_snapshot_boundary.py -q -p no:cacheprovider --junitxml=.tmp/native-jog-region-design-20261015/targeted-final.xml
```
- [x] **冻结回放：** 仅在本机将本次冻结只读结果传入新函数，核对与原型峰值约 0.0388040 rad/s、0.160208 rad/s² 一致；记录来源和输出哈希，源姿态不提交。该回放不连接真机。
- [x] **构建与作者复核：** `.venv/Scripts/python.exe scripts/run_segmented_demo.py --build-only --node C:/Users/Kilo/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe`，随后 `.venv/Scripts/python.exe scripts/run_segmented_demo.py --check`，预期退出码均为 0；构建不会启动服务。核对旧入口仍隔离、结果不提供运动权限，保留独立审查未执行的限制。用户原未跟踪文档 SHA256 应为 `c023cb2ed8528c277632a320190c7d540fe7c35a3a5f6224fbeb2835e4a2c59f`。
- [x] **提交与报告：** `git diff --check` 后只提交新模块、合成测试和文档，不推送。将本任务标完成，报告实际测试数、冻结回放结果及尚缺的控制器/制动证据。此任务完成不等于 speedl 单次真机入口或网页控制已经完成。

## Self-review and Next Gate

算法、接口、数值范围和测试已明确；没有将普通数值结果标为工厂校准或物理认证。任务只覆盖原架构中的局部模型检查，单次 speedl 的完整现场执行入口、原停止事务与独立松开验收仍需下一阶段实现；没有以本模型输出替换这些要求。

原独立审查因工作区额度不足未执行，不将作者推导或数值抽样称为独立审查通过。本任务可以离线验证，实际机器人访问仍必须有新的单次范围和当天现场条件。
