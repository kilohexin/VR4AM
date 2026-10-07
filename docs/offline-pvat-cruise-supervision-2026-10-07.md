# PVAT 巡航／制动参考与窗口失效监督

2026-10-07，起点 `a64881925f509e259262e3a735671f0a57511a2f`。本轮完成测试专用分段轨迹和停止桥接夹具，未接入默认控制，未改变生产代码、参数、SDK、现场配置或 200 ms 停止截止。没有真机连接、部署或推送；610 ms 现场停止迟到仍未解决，保留 `stop_unverified`，原现场授权已消费。

## 巡航与到点参考

针对[上一轮单段五次远目标响应偏慢](offline-pvat-settling-candidate-2026-10-07.md)，新增 `backend/tests/robots/pvat_cruise_candidate.py`。只处理声明的零初始速度／加速度；传入非零初始值明确拒绝 `cruise_requires_rest`，不能用它把运动反馈强行置零。

六轴按最大关节位移 D 归一化为共同进度。加速阶段采用平滑速度 `v=V*(3τ²-2τ³)`，积分得四次位置曲线；减速对称，阶段连接的 q/v/a 连续，中间可接匀速巡航：

```text
V = min(vmax, sqrt(D*amax/1.5))
R = 1.5*V/amax
C = max(0, D/V-R)
H = 2R+C
```

每轴速度／加速度不超过共同上限，起终点零 v/a；从静止到固定目标单调，不需要反复重启。阶段间没有 jerk 连续性／jerk 上限保证；共同关节进度不等于 TCP 直线、碰撞安全或厂家插补。窗口跨阶段时返回全部参考分段；最后窗口只覆盖剩余时间，到点后的数学保持不确认真实静止。

| 合成目标（单轴 rad） | 单段五次 H | 巡航参考 H | 前 2 s 五次位移 | 前 2 s 巡航位移 |
| --- | ---: | ---: | ---: | ---: |
| ±.5 | 18.75 s | **10.375 s** | ±.005139 | **±.090625** |
| ±.001 | .169904 s | .173205 s | 已到点 | 已到点 |

以上是精确数学参考，vmax=.05 rad/s、amax=.2 rad/s²。巡航参考全段峰值仍为 .05/.2；没有物理反馈或 SDK 时序模型。没有扩展成任意初始状态／改目标的完整规划器，也没有用它替换现有真机算法。

## 跨阶段端点不能直接当作合格 PVAT

新增 `fit_reference_window`：在明确的五次插补假设下，用窗口起终点 P/V/A 拟合单条曲线，再检查整段速度、加速度和位置极值。厂家实际插补未确认，因此该函数只用于实验，不是 SDK 编码器。

参考轨迹自身限幅合格，拟合窗口内部仍可能超过 .05：

| .5 rad 计划中的窗口起点 | 窗口长度 | 五次拟合速度峰值 rad/s | 处理 |
| ---: | ---: | ---: | --- |
| .35 s | .08 s | .050028863 | 明确拒绝 |
| .37 s | .08 s | .050040196 | 明确拒绝 |
| 9.95 s | .08 s | .050040742 | 明确拒绝 |
| 9.98 s | .08 s | .050012299 | 明确拒绝 |

9 个窗口核对中 4 个拒绝；非跨阶段曲线可精确复现原四次／一次参考。它不是对所有窗口的证明，更不是机器人实测超速。阶段浮点边界另以 1e-12 s 容差处理，避免 .375 s 处产生约 5.55e-17 s 的伪分段。

## 停止桥接验证

新增 `backend/tests/control/pvat_window_supervision_probe.py`，仅测试导入原 `RobotControl`。它在 ACTIVE tick、消费新 VR 帧前检查声明的参考覆盖；没有新 VR 帧时也能检查。没有后台任务或运行时调度接入。

- 以人为指定参考起点和窗口 duration 定义覆盖；到达截止即过期，晚到的续接不能抹掉空档。此起点不是已验证的控制器接收／执行时刻。
- 用显式注入的观察值比较参考 q/qd/qdd。比较阈值 1e-4 rad／1e-3 rad/s／.02 rad/s² **仅为合成测试常量**，不写入现场 profile，也没有快照帧龄或实际反馈估计保证。
- 缺少／无效观察、参考偏离、PVA 不连续、旧窗口、过期和拟合拒绝均保留故障；续接清除旧观察，故障后禁止提交新窗口。
- 故障进入原 `_enter_fault → _stop_backend → RealLebaiAdapter.stop` 路径。夹具注入 SDK；并未让上述新参考产生 SDK 运动命令，SDK 仍使用原生产构造器。
- 普通 Grip 释放沿用原规则并撤销参考，不把旧窗口带入后续 HOLD；清除参考不清除已锁存故障。

6 类触发（过期、q/v/a 偏离、缺少观察、真实拟合函数拒绝）×5 类停止结果（确认、反馈仍运动、异常、注入 610 ms 迟到、取消），共 30 个组合核对：停止失败后，即使 fake 后续反馈静止，仍锁存 `stop_incomplete`／`stop_unverified`；新帧、Arm 和 reset 不能恢复，未增加 Home／movej／stop_sys 写入。迟到例仍使用 200 ms 截止，610 ms 是 fake 时钟注入，不是新的现场测量。

另覆盖过期时阻塞 IK 和 pending 请求，原停止事务先撤销代次，释放旧 IK 后无 PVAT 写入；无新 VR 帧的到期停止与正常 Grip 释放均有独立案例。该夹具没有证明网络时延、真机停止、控制器插补或可直接上线的监督策略。

## 验证与失败记录

首批 52 案例对占位实现红灯，均为缺少实现；实现后 52 通过。补拟合拒绝与续接边界后，16 个红灯来自缺少拟合／拒绝接口、浮点伪分段、空档／不连续续接未拦截，以及无效观察仍保留旧值；实现相应检查后通过。

正常 Grip 释放最初暴露参考未撤销；补撤销后，夹具在 100 ms 不更新输入又触发原 STALE 停止。保留生产 STALE 规则，改为刷新 Grip=false 输入来核对正常 HOLD，未抑制合法停止。非零初始 v/a 两个红灯因新参数尚不存在，补明确拒绝后通过。

最终 **73 个新增案例，9 个相关文件 252 passed，23.51 秒**；提交前复跑仍为 **252 passed，23.75 秒**，均退出 0：

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/robots/test_pvat_cruise_candidate.py backend/tests/control/test_pvat_window_supervision.py backend/tests/robots/test_pvat_settling_candidate.py backend/tests/robots/test_pvat_ramp_candidate.py backend/tests/robots/test_pvat_interpolation_probe.py backend/tests/robots/test_lebai_pvat.py backend/tests/robots/test_pvat_point_inputs.py backend/tests/robots/test_stop_transaction.py backend/tests/control/test_braking_supervision.py -q --tb=short -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

fake 客户端确认 network/discovery 调用均为 0。没有后端全集、独立审查或现场测试。

本机忽略目录 `.tmp/early-stop-window-20261007-l0erdkr4/` 中 `evaluate_cruise_supervision_v2.py` 退出 0：核对原日志、只读配置和生产构造器哈希；4 个方向／幅度、每条 1001 点单调性、阶段 PVA、9 个窗口各 31 点的参考一致性及 4 个拟合拒绝。完整输出不入 Git：

首次独立报告哈希校验失败：v1 按序列化字符串的 LF 字节计算哈希，Windows 文本保存则写入 CRLF。保留 v1 脚本／报告，v2 改为从保存后的文件字节计算哈希；数学结果、测试源码和输入均未改变。v2 独立复核通过：两份实际文件哈希、7 项源码／输入哈希、4 个计划、9 个拟合及 4 个拒绝全部匹配。该失败是报告字节校验问题，没有真机操作。

| 项目 | SHA-256 |
| --- | --- |
| `cruise-supervision-evaluation-v2.json` | `be64455ebca13b38db7ef9215999e0b935998d2de13f0bfaa8c6225e3738eaea` |
| `evaluate_cruise_supervision_v2.py` | `d45eb4dd9f1160c2a9680ce2e687a48e8da0e60d2044550557bb649dc53384cc` |
| 巡航参考源码 | `0f45967486f6d41d858f405d935c822ada79f6f9c245aa055fec21219f5b595b` |
| 监督夹具源码 | `66c1d005d8ae912691b969b57cb18d4286e5f046ab2f37d9cc95895665529c89` |

下一步离线解决跨阶段 PVAT 端点表达及其整段限幅，接着核对任意移动初始状态／改目标制动与真实快照来源。窗口参考起点、SDK 接收时间和反馈时刻仍未对齐；当前结果不能解除停止门禁或成为新的现场授权。
