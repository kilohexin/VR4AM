# 停止确认：残留运动 ID 的显式核验

## 已确认的范围

第26轮原始 session `20260916T105844Z-b2e971b6` 中，13个验证样本均为 STOP、运动ID 111、速度为零，关节和TCP记录相同。旧代码将 `running_motion is None` 纳入 `stationary` 条件，因此这些样本无法建立300ms静止窗口。

原件显示第一轮 stop_sys **开始于+531ms，返回于+578ms**；不是+578ms才开始。第一轮12个样本，收尾复核1个样本，共13个。RPC均返回，不是本轮200ms应答超时。

这定位的是软件判定失败的直接条件，不证明运动ID 111在控制盒内部已完成，更不解释第20轮约22mm的异常位移。第26轮未采集 `get_motion_state(111)`，不能补写为FINISHED。

## 实现

沿用现有IDLE下的运动状态查询，扩展至明确的 STOP / PAUSED 状态（含既有数值映射）。对于非空运动ID，在同一快照读取预算中查询其状态：

- 仅明确字符串 `FINISHED`（大小写/空白规范化）使内部有效 `running_motion` 为 `None`。
- WAIT、RUNNING、未知/缺失/畸形返回不判完成；RPC错误、超时、整次快照过期仍失败。
- 每个快照重新查询，不能跨快照/跨ID缓存FINISHED。
- 新增 `raw_running_motion` 保留SDK原始ID，`motion_state` 记录状态结果。字段出现在运动学日志、停止验证样本、停止后观察中；只读预检通过已有事件采集机制落盘这些字段。
- `motion_state` 是规范化后的文本，不是SDK返回值原件；非字符串返回记为null并保留有效ID，不将其当作FINISHED，也不由null推断具体SDK原值。
- 无ID时不新增查询；MOVING/STOPPING/TEACHING不使用该查询绕过状态判定。

**本次与上一版不同：STOP/PAUSED且有ID时会额外调用已有只读RPC `get_motion_state`，不能宣称读取调用次数不变。** 查询受已有300ms整次快照预算约束，不扩大预算；现场须核对延迟。

未改变：停止RPC的200ms期限、300ms连续静止窗口、500ms验证预算、速度/位移阈值、故障锁存、停止升级策略。STOP/PAUSED仍映射HOLD并拒绝运动预检；确认停止不清除历史故障、不启用关节、不恢复IDLE、不自动重发运动。

## 证据边界

[官方运动接口文档](https://help.lebai.ltd/sdk/motion.html)区分运动ID与 `get_motion_state` 的 WAIT/RUNNING/FINISHED。文档不能代替本机控制盒的软件行为核验。本机未联系厂商、未连接机械臂。

RPC序列不是原子快照；FINISHED单独不证明物理静止。位置和速度窗口仍必须成立；窗口成立也不能证明未来不会出现迟发位移。现有22mm异常仍保留为独立未闭环问题。

## 验证方法

新增运动ID回归先在旧代码上得到20失败/4通过，失败包括STOP下FINISHED无法完成停止确认和原始ID证据缺失。修正后补齐跨ID、状态变化、急停和无ID分支；只读CLI集成测试验证HOLD仍返回退出码2并完整保存新字段，SDK写调用为零。

使用真实适配器和本地假SDK，不连接设备。现场未知的FINISHED和未完成分支分别作为假设测试，不能把测试fixture说成现场观测。

首轮全量872项通过（108.65s）；随后补入4项只读CLI集成测试。最终全量 **876 passed，106.67s，退出码0**。进程观察器同为退出码0，`process.json`为EXITED、`child_exit_code=0`、`observer_errors=[]`，stderr为0字节。原件保留于本机忽略目录 `artifacts/acceptance/host-motion-id-20260917-001/`。

独立只读审查通过，无阻断项；审查者独立运行新ID测试和脚本测试，46项通过（2.10s）。交接PowerShell代码块经解析器检查无语法错误（未执行）；只运行预检CLI的 `--help` 核对参数，未连接设备。

第26轮核对原件SHA-256：`9adcb3168e5b8ca9b1b4ab8a1d21a9b25eccd68c1ba377581f3eca434b1a460a`。原件未改动、不纳入代码提交。

## 下一步

见 `lab-motion-id-readonly-handoff-2026-09-17.md`：下一项只需读取实际运动ID对应状态，不重复静止stop、不切control。只有拿到这一缺失事实后，才决定是否需要新的停止验证。该修改尚不构成VR真机放行。
