# VR 记录、预检和锚点读取的等待边界修复

## 本轮范围

延续[夹爪返回后校验](vr-gripper-frame-freshness-2026-10-05.md)，修复真实控制层在记录、预检、锚点读取期间继续使用旧输入或旧权限的问题。只进行内存后端/fake SDK验证；没有现场连接、服务启动、部署或推送。Home交互、停止未确认锁存、200 ms停止RPC截止和固定20 ms积分保持原规则。

## 复现和实现

| 边界 | 原问题 | 本次处理 |
|---|---|---|
| VR输入记录await | 输入已更新/过期或权限已撤销，仍可能先发送旧Trigger | 记录前保存代次，返回后复核输入身份、帧龄与权限，再处理夹爪 |
| HOLD恢复预检及其记录 | 返回的预检可能属于旧Grip或旧控制代次 | 每个等待返回后检查原模式、代次和当前输入 |
| Grip锚点读取 | 先进入ACTIVE，再等待状态读取；可能迟到捕获旧锚点或留下缺失锚点的ACTIVE | 保持ARMED/HOLD读取；校验通过后连续完成锚点捕获、滤波/限幅重置和ACTIVE转换，中间无await |
| ACTIVE转换记录 | 等待期间发生新输入或撤权后，仍继续处理旧序号 | 记录失败继续故障处理；记录成功再检查当前帧与ACTIVE权限，失效即返回 |
| 显式夹爪命令预检/记录 | 预检开始时有权限，返回时已解除或重新Arm，仍可能写夹爪 | 每次返回后复核代次和ARMED/ACTIVE权限；撤权抛出`gripper_command_authority_changed` |
| Arm请求预检/两次记录 | 请求可能跨越解除再READY、故障或关闭，迟到使能 | 固定请求开始时的模式和代次，每次等待后复核；失效抛出`arm_authority_changed` |

公共权限检查保留故障、循环失败、关闭、解除武装中、恢复锁及代次条件。Arm/显式夹爪命令是明确请求，不强制绑到某一VR帧对象；tick的动作继续要求同一当前输入且小于100 ms。

### 顺序和失败语义

- 正常Grip激活的状态读取移到ACTIVE转换及其记录之前。正常读取次数没有增加，但若转换记录失败，状态读取现在已经发生；仍没有发送移动目标，原记录故障处理继续执行。
- 激活转换的关键记录等待期间，可能已经是ACTIVE；此时锚点必须已建立。输入随后变化不会反向撤销此前已有效完成的捕获，但不会再用旧输入捕获或覆盖新序号。
- **Grip释放的停止义务保留**。ACTIVE→HOLD的转换记录期间，即使输入变新或过期，后面的`GRIP_RELEASED`停止仍执行；不能套用激活路径的提前返回而跳过停止。
- 原状态记录不等于最终使能证明。Arm的`state_transition`记录等待期间若被撤权，记录可能已写入，但本轮不会执行`machine.arm()`；不能只看一条记录推断已获得权限。
- 后端/记录异常仍按已有失败策略传播或锁存，未为旧请求静默吞掉错误。已发出的SDK命令无法由这些返回后校验撤销；不声称物理停止更快。
- WebSocket既有`arm_rejected`接口捕获RuntimeError，无协议字段变化；现有通用拒绝文案未在本轮修改。

## 定向验证

新增43案例：五个tick等待边界×四种干预（20）；显式夹爪两个等待点×解除/重新Arm（4）；记录失败（2）；真实异步门控验证锚点读取期间不提前ACTIVE（2）；释放停止不跳过（2）；三个40 ms等待累计过期（1）；Arm三个等待点×四种权限变化（12）。

先修正测试导入位置错误，随后得到21 failed/3 passed的行为复现；第一批实现与既有控制/夹爪相关187项通过。扩展边界31项通过、相关452项通过。Arm新增12例在修复前全部失败，补充请求权限检查后控制/API相关233项通过。测试中的重新Arm先消费释放帧，不绕过原Grip释放要求。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/control/test_input_await_boundaries.py backend/tests/control/test_gripper_frame_freshness.py backend/tests/control/test_robot_control.py backend/tests/control/test_state_machine.py backend/tests/control/test_state_observation.py backend/tests/control/test_recording.py backend/tests/control/test_input_cadence_experiments.py backend/tests/control/test_elapsed_step_experiments.py backend/tests/api/test_teleop_ws.py backend/tests/robots/test_lebai_adapter_control.py backend/tests/robots/test_stop_transaction.py backend/tests/robots/test_pvat_request_lifecycle.py backend/tests/diagnostics/test_display_integration.py backend/tests/acceptance/test_fake_lebai.py backend/tests/commissioning/test_actions.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

最终上述15文件定向回归 **511 passed（42.51 s）**，退出码0。所选acceptance/commissioning案例只使用既有fake SDK、注入工厂和临时测试目录，不运行真实调试命令。未运行后端全集或前端构建。主代理自查模式转换顺序、关键记录失败、释放停止义务、公共校验调用方与测试假设；未计作独立第二审查人通过。

## 后续优先级

继续优先真机VR控制与夹爪。下一项是解决已经隔离的目标到点速度归零/抖动下的速度状态问题：先在测试中建立可比较的减速方案，明确不越界、停止及速度连续性的取舍，再决定生产限幅改动。不能直接把积分常量换成时间差。平台浮空与Web物理抓取仍在后续阶段，尚未选型或安装引擎。当前只完成以上等待点的校验，不宣称所有SDK内部排队和取消竞态均已解决。
