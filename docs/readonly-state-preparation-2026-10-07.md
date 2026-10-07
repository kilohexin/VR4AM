# 首轮只读状态采集准备

2026-10-07。本轮完成独立查询入口与离线验证，尚未建立现场连接。用户“继续下一步”按既有离线开发授权处理，尚未形成带现场条件的真机批准范围。仍暂不推送。

## 改动与用途

新增 `backend/app/commissioning/readonly_state.py`、CLI 包装 `scripts/real_robot_readonly_state.py` 与 24 项定向用例。既有 `readonly_dynamics.py` 只查询负载/重力，不能代替本次状态采集；既有 `real_robot_smoke.py` 要求 control 并可能执行停止收尾，不能用作本阶段入口。

默认模式读取配置文件并输出 mode/文件哈希/固定查询清单，未调用 SDK 工厂，不含 IP 或完整配置。现场执行需要独立 readonly 配置、一个不存在的新报告路径及显式 `--execute-readonly` 参数。该参数只是执行开关，不能代替用户授权。

一次连接，初始依次 `get_robot_state`、`get_estop_reason`、`get_kin_data`、`get_tcp`、`get_claw`、`get_payload`、`get_gravity`；后续重复前三项和夹爪，循环间最多等待 250 ms，采样窗口 5 秒。实际 TCP 来自运动学数据中的 `actual_tcp_pose`；`get_tcp` 是工具 TCP 设置，二者保留独立原始记录。每项查询附时间与用途；报告保留 JSON 原值和未知字段。

非 IDLE、急停原因、关节/实际 TCP 缺失、非有限 JSON、读取超时、迟到返回、配置变化均结束，保留部分报告。无动作/设置写入、重试、Home、stop_move、stop_sys 或 reset 路径。取消保留部分报告并传播。不会创建控制 adapter、pump、RobotControl 或服务。

报告 `complete` 是采集结果，不代表静止或控制可用；`hardware_stop_verified=false` 固定保留。连接超时 5 秒、每项查询超时最多 1 秒并受剩余窗口限制；异步超时无法保证拒绝取消的 SDK 在硬截止退出，迟到返回会记失败。独立进程退出与现场连接释放需另行确认，报告不伪造已经验证的断连。应用查询白名单也不构成 SDK 内部行为的现场证明。

## 离线证据

- 首次新模块缺失导致导入失败；补实现后首批 14 项通过。
- 扩展到 24 项，覆盖固定白名单、单连接、窗口、报告不覆盖、readonly 门禁、默认离线预览、非 IDLE/后续 MOVING、急停、缺失/非有限数据、失败/超时/迟到不重试、配置变化、取消部分报告及执行必需输出路径。
- 三个相关文件最终定向回归：55 项通过，5.27 秒。测试中的白名单独立固定，不随实现的清单改变。输出 `.tmp/readonly-state-regression.txt`。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_readonly_state.py backend/tests/scripts/test_readonly_dynamics.py backend/tests/robots/test_lebai_adapter_readonly.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

所有 SDK 均注入测试替身。使用保留 IP 段的临时配置运行 CLI 默认预览和 `--help`，均成功；未运行 `--execute-readonly`。未运行后端全集或头显测试。

## 现场待确认

通过本机进程表/监听表被动检查发现候选服务端口仍有监听。进程命令行无法读取，无法判断属于哪个检出/配置，或是否存在机器人连接；没有 HTTP/TCP 探测、SDK 连接或进程结束操作。原始本机库存保存在 `.tmp/readonly-site-inventory-2026-10-07.json`，SHA-256 `7b0da527a7154fa072b224a0166efc04cba026ec7a07a60ef7ef935cfe036baa`，不提交原始记录。

当前检出未发现 `config/real-robot.local.yaml`；没有构造猜测 IP 的现场配置。旧服务收尾、配置来源、通电时机、操作人、工具/夹爪安装、当前姿态与空间必须由现场确认，再批准一次只读连接。若需要结束旧控制服务，需先审查其配置和退出动作，不能因为端口残留直接杀进程或触发服务关闭。

VR 控制源码冻结仍为 `409d18a8d1c74747b5d189f4a362b2fa6c8cf041`；本次只新增采集准备，不修改控制策略、停止截止或门禁。运动测试尚未授权，约 609 ms 的现场停止返回延迟仍未解决。后续顺序仍为只读核对、单独批准非 VR 早停诊断、单独批准短程 VR/Trigger。
