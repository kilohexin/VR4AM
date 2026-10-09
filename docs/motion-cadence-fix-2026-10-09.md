# 诊断动作循环的额外等待修复

2026-10-09，Asia/Shanghai。接续 [发点间隔审计](stop-method-latency-audit-2026-10-09.md)。本轮仅离线开发，不连接真机，不推送。

## 定位与修复范围

本次非 VR 诊断循环使用 `tick → sleep(period) → get_state`。固定睡眠不扣除 tick 与上一轮状态读取耗时，状态读取又和 PVAT handler 的读取竞争同一 SDK 锁。因此诊断目标输入周期包含额外整周期等待，不能把配置的 loop_hz 当作实测输入频率。

仅修改 `backend/app/commissioning/smoke.py`：用 asyncio 事件循环的 monotonic time 维护下一周期截止时间。首周期扣除 tick 耗时，后续等待也扣除上一轮已经消耗的时间；错过的截止时间重定位到本轮起点，等待归零。每轮只发布一份输入，仍执行 `await sleep(0)` 让出调度机会，不积累多份补帧任务。

原 tick、让出调度、状态读取及运动判定顺序保留；状态读取不会被省略或缓存复用。PVAT pump 的频率约束、新鲜度检查、停止事务、200 ms 门限、`stop_unverified`、速度和时域不改。新增 `scheduled_sleep_ms` 是计划等待量，不能代替实际睡眠、真实 PVAT 发点间隔或控制器接收时间。

生产 VR 的 `RobotControl.run` 已采用周期截止时间调度，本轮没有修改它。此次修复减少诊断入口自身引入的额外等待，**不等于已解决生产 VR 平移响应或所有 PVAT 超时域间隔**。完整快照读取、IK、共享 SDK 锁和系统调度仍可能限制发点连续性。

## 定向验证

先增加 6 个确定性周期测试，缺少新 helper 时失败；实现后验证首轮、正常工作耗时、慢 tick、慢读取和跨多个周期迟到的等待计算。

新增 fake-client 集成回归让每次 get_kin_data 延迟 25 ms，确认晚周期计划等待为 0、后续状态读取继续执行、停止后没有新的 PVAT、没有 stop_sys。原动作测试覆盖早停、停止不确认、授权到期及快照刷新等行为。

最终 7 个定向文件 **151 passed，62.57 秒，退出 0**：`test_motion_cadence.py`、`test_actions.py`、`test_wire_stop_admission.py`、`test_motion_wire_capture.py`、`test_readonly_wire_launch.py`、`test_stop_transaction.py`、`test_stop_observation.py`。测试使用 fake / 合成材料，没有运行真机 SDK、抓包或后端全集。

独立静态复核未发现新增 P0–P2；确认仍按原帧数限制运行，没有新增严格总墙钟超时保证。现场数据未改，现有用户项目总结文件保留。

本轮修复尚未真机验证；运动停止的 613 ms 往返问题没有被本修复解决。旧入口材料中的 runtime 哈希随代码改变失效；即使之后重新准备预览，也必须取得新的单次现场授权才能运行。
