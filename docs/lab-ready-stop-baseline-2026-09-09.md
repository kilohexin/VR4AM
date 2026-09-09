# 下一轮受控诊断：准备位停止基线

## 目的与本次范围

Home 的静止 stop + 60 秒观察已得到可用记录。下一步先在此前运动测试所用
的 `teleop_ready_q` 建立同版本的静止停止基线，随后才考虑 PVAT 后停止。
这样减少位形、软件版本、运动方式同时变化造成的混淆。

**本轮只包含：新只读预检 → 一次既有 prepare → 核对准备位 → 一次 stop +
60 秒观察 → 退出只读预检。没有 +x，没有 B 段，没有 VR。**
prepare 会产生真实关节运动；它不是只读诊断。必须由现场具备操作资格的人
确认完整路径、线缆、工具、台面和周围空间安全，观察员及硬件急停就位后执行。
不能因为此前执行过就省略现场确认。

现有程序版本 `68f51a0` 即可运行；本清单不需要修改控制代码或现场参数。
本轮与 Home 基线的对比仍包含一次准备位关节运动历史差异，不是严格单变量
实验，不据此单次定根因。

## 已核对基线及报告口径

会话 `20260909T110155Z-cefa886c`：

- stop_move 16 ms 返回，无 stop_sys；停止诊断实际是 6 个样本，不是 5 个。
- 观察 60.015 秒，288 个样本，原始状态均 IDLE；读取错误、记录器丢失均为 0。
- 相对首个观察样本，最大关节变化约 0.0055°，TCP 平移变化约 0.063 mm。
- 最长成功采样间隔 281 ms；应称无长时间记录中断，而非零空档。
- 9 个样本有非零速度读数，其来源未确定，不能直接认定为噪声或真实微动。
- 停止成功时仍有 singular_configuration，来自 Home 位形，不等于允许运动。

时间疑点已解除：主机端 PowerShell JSON 日期解析与重新序列化转换了显示
时区。原文件为 +00:00，未发现 +08:00 字节；MD5 为
`f5cf7a9b4409d3e13a66de50bb4e42d2`。保留原始日志，不为统一显示而改写文件。

## 0. 新鲜基线与现场门槛

不要直接使用旧截图或当天较早预检代替当前状态。只在现场仍为已核对 Home、
IDLE、静止、无 fault、TCP 匹配且包络有效时继续。如果状态已变化，回传新
预检并停止，不自动 Home、不示教移回、不修改包络来通过检查。

已核对的本地配置参考值：

- `home_q = [0, -1.5707963267948966, 0, -1.5707963267948966, 0, 0]`
- `teleop_ready_q = [0.0004, -1.5681, 0.252, -1.5722, 0.4017, -0.0011]`

以上用于核对，不是要求现场重新写入参数。如果实际配置不同，先报告差异。
不要手动放宽软限位、奇异保护、SDK 超时、速度或加速度。
关闭其它控制程序；同一时刻只运行本清单的一条命令，不能整段批量执行。

笔记本项目根目录，readonly 模式下：

```powershell
git rev-parse HEAD
git status --short
$env:PYTHONPATH = (Resolve-Path .\backend).Path
$cfg = "config/real-robot.local.yaml"
$confirm = "I_UNDERSTAND_REAL_ROBOT_MOTION"
Get-FileHash $cfg -Algorithm SHA256
python .\scripts\real_robot_preflight.py --config $cfg --output logs\ready-stop-before.json
```

保存基线原文件与实际采集时间，不覆盖旧报告；如文件已存在，改用新的带时间戳
文件名。readonly 下 preflight_ready=false 不等于 control 下可以运动，不能
只看这一位；核对 complete、reason、机器人状态、TCP 和实际关节数据。

## 1. 单次进入准备位

通过第 0 节且现场明确同意本次真实运动后，仅切换 `real_robot.mode` 到 control。

```powershell
python .\scripts\real_robot_smoke.py prepare --config $cfg --confirm $confirm
```

**不加 `--observe-stop-seconds`**：prepare 使用独立清理路径，该组合会被拒绝。
它本身有运动阶段日志，但 prepare 结束至下一条命令之间并非无缝采样。

必须等命令结束、保存退出码及 session，并现场确认 IDLE、静止、无异常。
成功要求 EXIT=0，prepare 返回稳定结果，最终关节接近本地 teleop_ready_q；
不自行改变精度容差。如果失败、读取异常、疑似持续位移、状态非预期，立即
停止后续步骤，按现场安全规程处理并回传，不自动重试、启动系统或回 Home。

成功后切回 readonly，单次核对：

```powershell
python .\scripts\real_robot_preflight.py --config $cfg --output logs\ready-stop-prepared.json
```

核对 complete=true、IDLE、TCP 匹配、关节速度/实际姿态、原因仅为
real_robot_readonly。若还有其它门槛失败则本轮停止。
这份探针与 prepare 的时间间隔要记录，不能把它算作连续观察。

## 2. 同准备位单次 stop + 60 秒观察

仅在第 1 节成功且现场再次确认静止、安全条件未变时，切 control，执行一次：

```powershell
python .\scripts\real_robot_smoke.py stop --config $cfg --confirm $confirm --observe-stop-seconds 60
$stopExit = $LASTEXITCODE
Write-Output "stop-observation exit: $stopExit"
```

观察期保持目视监督；不启动第二个探针、不操作 xLab、不碰机械臂。
如有实际位移或停止失败，现场立即按安全规程处理，不等待观察满 60 秒。
所有人工干预、急停操作和准确时间都必须保留，不能因此删除异常 session。
程序只读观察不会自动恢复机器人，也不是安全监督系统。

无论结果如何，都不重复本命令。结束后恢复 readonly，再做一次退出预检：

```powershell
python .\scripts\real_robot_preflight.py --config $cfg --output logs\ready-stop-after.json
```

若程序未正常退出或现场必须紧急处理，以现场安全规程为先；不要为凑齐报告
执行额外机器人系统命令。

## 3. 回传并暂停

必须回传：

1. HEAD、工作区状态、实际使用的 home_q/teleop_ready_q、配置 SHA256及 mode。
2. prepare 和 stop 的完整 session 目录、各自退出码，不只摘最后一条事件。
3. 三份只读预检、实际时间、两条命令之间是否有人工干预。
4. stop_diagnostics 的全部 RPC 结果、samples、initial_fault/latched_fault。
5. stop_observation_summary、session summary.json，以及全部观察样本。
6. 现场观察到的运动、异常或急停处置，原样报告，不先断言噪声/回弹。

主机将比较准备位与 Home 基线的停止耗时、原始状态转换、位置变化和读延迟。
“观察 complete”不是漂移验收，也不是运动许可。只要停止失败、出现长时间
采样空白或异常位置变化，就先分析，不进入下一段。

## 后续候选，不在本轮执行

待准备位基线核对后，单独确认是否安排同版本、同准备位、保留现有低速参数的
一次 +x 5 mm PVAT 诊断并附加 60 秒观察。此处不提供执行命令，以免混入本轮。
选择 5 mm 是为了与已有异常样本的请求幅度对应，不代表已知一定安全。
本轮不放宽任何保护，也不恢复 11 方向 B 段或 VR 真机。
