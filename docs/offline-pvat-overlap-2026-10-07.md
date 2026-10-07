# 覆盖窗口内的跨阶段续接与数学制动分支

2026-10-07，起点 `530c33fdab6bf9df3389cd98d121f6d2eeaa0aa7`。新增内容均在测试目录：滚动参考与制动分支候选、定向测试及注入 SDK 的停止组合。无生产代码／参数／配置／SDK修改，没有连接真机、部署或推送。现场 610 ms 停止迟到未解决；200 ms 截止、`stop_unverified` 与已消费的现场授权保持。

## 滚动续接

[上轮阶段编码](offline-pvat-phase-encoding-2026-10-07.md)的首尾相接列表不能直接用于提前发送。本轮新增 `backend/tests/robots/pvat_overlap_probe.py`，以声明的旧窗口作为当前预测状态来源：

1. 续接时间必须为整型 ns，严格晚于旧窗口起点、严格早于截止；不能从完整未来计划外推过期的已发窗口。
2. 从旧窗口在该时刻的 P/V/A 继承初始状态；从完整参考取下一窗口末端的期望 P/V/A，重建一条声明的五次曲线。
3. 检查整段 v/a 与关节位置极值，并检查续接初始状态、窗口末端的数学制动分支。拒绝不替换旧候选，接受则返回新对象。
4. 目标不变时保留完整参考对象及原始到点时钟；目标改变才从覆盖中的 P/V/A 构造既有有界到点候选。不会将非零速度、加速度置零。

参考起点和窗口起点均为合成声明时间，不是 SDK 接收或控制器执行确认。没有 dispatcher、网络任务或现场反馈估计；没有把该候选接入默认控制。

## 制动分支检查

先在 B=80 ms 内线性消除初始加速度，再用平滑速度曲线将六轴速度降到零：

```text
阶段一 a(t) = a0 * (1-t/B)
q1 = q0 + v0*B + a0*B²/3
v1 = v0 + a0*B/2
阶段二 R = max(B, 1.5*max(abs(v1))/amax)
v(τ) = v1 * (1-3τ²+2τ³)
q2 = q1 + v1*R/2
```

两阶段 P/V/A 连续，末端数学 v/a 归零，全部阶段检查内部极值。初始状态超限、加速度消除引起速度超限、制动越过关节边界均拒绝 `pvat_brake_reserve_limit`；端点仍在关节范围而内部越界的反例亦拒绝。该分支是一个有限轨迹族，不是最短制动证明。

这些检查仅证明续接点及窗口末端的合成分支，不覆盖当前窗口每个中间时刻的制动可行性、碰撞／jerk约束或真实软件停止。分支未发送给 SDK。

## 离线结果与保留的限制

显式合成参考采用 .045 rad/s／.18 rad/s²，实际候选检查上限仍为 .05／.2。该余量只用于本轮实验，未选择为现场 profile，也没有自动降低现场参数或放宽检查。

| 固定目标 | 参考全程 | 20 ms 续接次数 | 窗口速度／加速度峰值 | 超过目标最大幅度 |
| --- | ---: | ---: | --- | ---: |
| ±.5 rad | 11.486111 s | 各 580 | .04507433／.18 | 约 9.26e-7 rad |
| ±.001 rad | .182574 s | 各 15 | .01092139／.18524587 | 约 3.90e-6 rad |

四条共 1190 次续接，均在原窗口覆盖内通过 `WindowGuard` P/V/A 续接检查，最大交接残差约 2.78e-17。±.5 rad 参考前 2 秒位移 ±.0815625 rad；相比上轮无余量巡航的 ±.090625 rad 减小。没有重复重启完整到点计划。

重建窗口**不再精确等于原分段参考**：采样最大 P/V/A 偏差，远目标约 `9.40e-7 / 7.43e-5 / .00830474`，近目标约 `4.00e-6 / .000317857 / .02549576`。表中目标越过量由窗口多项式内部位置极值求得。因此尚不能保证固定目标单调到点，尤其不能将旧完整参考作为实际窗口观察的替代值。终点数学归零不确认真机静止。

满限幅 .05／.2 参考在 240 ms 首次拒绝制动余量：期望窗口末端 320 ms 的 `v=.04708882963 / a=.10012444444`，80 ms 加速度归零后的速度将为 `.05109380741 > .05`。不能据此认定所有制动轨迹都不可行；本候选族无法通过。原跨阶段拟合拒绝仍存在于先前报告。

冻结原日志 25 条 `pvat_sent` 包含 24 个客户端 SDK 开始间隔，首个 110 ms。用该间隔续接 80 ms 合成窗口明确拒绝 `pvat_overlap_time`，没有抹去空档。这些间隔来自本地调用时间，不是控制器接收／执行时间；未将原日志重放称为闭环物理模拟。

## 定向验证

初始 26 项对占位实现均红灯；实现后 25 通过，1 个测试原期望跨阶段拟合拒绝，实际由更早的制动余量检查拒绝。定位上述 240 ms／320 ms 反例后，修正测试原因预期，保留检查并核对拒绝不替换旧候选。随后 27 项通过，再新增内部关节越界及大 epoch 时钟案例。

停止桥接新增过期续接、整段曲线超限、制动分支不可行三个触发 × 原五类停止结果，共 15 项。均走原 `RobotControl._enter_fault → _stop_backend → RealLebaiAdapter.stop`，network/discovery 为零；确认、反馈仍运动、异常、fake 610 ms 迟到、取消均保持既有规则。停止失败后 fake 静止、新帧、Arm 或 reset 不恢复门禁；没有增加 Home／movej／stop_sys 或额外停止写入。

本轮 **45 个新增案例**，11 个相关文件 **348 passed，46.80 s**，退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_pvat_overlap.py backend/tests/robots/test_pvat_phase_encoding.py backend/tests/robots/test_pvat_cruise_candidate.py backend/tests/control/test_pvat_window_supervision.py backend/tests/robots/test_pvat_settling_candidate.py backend/tests/robots/test_pvat_ramp_candidate.py backend/tests/robots/test_pvat_interpolation_probe.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_pvat_point_inputs.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_braking_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

没有后端全集、独立审查、前端构建或现场测试。数学制动分支及上述通过结果均不能解除现场停止阻塞。

## 本机证据

完整分析脚本／输出在忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/`，不入 Git。分析退出 0；既有原日志、只读配置和生产构造器哈希匹配，输出按实际保存的 UTF-8 字节计算哈希：

| 项目 | SHA-256 |
| --- | --- |
| `evaluate_overlap.py` | `994ac40ba91a02e19298d073b4975a1e04c8d05884367ab525aac8f692d98ae6` |
| `overlap-evaluation.json` | `4d4eb30d12958a17ae3b75d9350ab942bcc9849185ac5fd68c25c21c3341eb26` |
| 滚动参考源码 | `eb748f12ed6676520a1c51d25f2fda734a3249e8b5fd14a117ae5b84ab67b609` |
| 滚动参考测试 | `59aa51b5c2a096342f76df1ab0706be1897dc973f149cd863dac2e6482363ee5` |
| 停止监督测试 | `6e590a8ed75e0cad5400eef458411316a12e1c5da9b97f7d319710dd7b4ada54` |

下一步离线处理到点附近的重建越过／参考偏差、参考与真实快照来源以及发送时序门禁。厂家实际插补、窗口执行起点、实际反馈和 610 ms 停止迟到仍未解决，当前不进入真机运行测试。
