# 单次静止 IDLE 系统停止诊断：交接草案

状态：主机实现与离线核验完成。本文件随源码发布，但不是实验室立即执行授权；须由后续通知钉扎完整提交号、新隔离目录和一次性输出编号。不得用旧隔离版本拼接执行。

## 目的及范围

单独检验静止 IDLE → `stop_sys` 是否出现实际偏离，区别于已经 STOP/HOLD 上的重复停止，以及运动后的停止升级。系统停止会禁用关节，可能产生非预期运动；本工具不是只读工具，也不是独立保护装置。

不修改 VR/生产停止逻辑、200 ms 超时或保护升级。唯一写调用是一次 `stop_sys`；不发送 prepare、PVAT、stop_move、启动、复位、夹爪或 Home。已有现场风险评估声明保留，不代替本次技术判据。异常时现场处置优先，可中断观察，不为凑满 60 秒等待。

## 先离线核验（不接触设备）

新隔离目录中按项目既定流程确认源码与目标提交一致、实际导入来自该目录。保留配置原件与哈希，不修 Git 引用。运行：

```powershell
& $Python -m pytest -c backend/pyproject.toml backend/tests/scripts/test_static_system_stop.py backend/tests/scripts/test_static_system_stop_cli.py -q
```

主机侧验证为 105 passed（`backend/tests/scripts` 全集）；实验室按通知中钉扎的提交与命令核验。任何失败回传，不重跑至通过。

## 工具内置判据

- 配置必须 control，且同时满足项目控制环境确认与独立系统停止确认。包装脚本先验证原 YAML 的 `real_robot.mode=readonly`，再核对候选 YAML 除该字段外完全一致，并保留原有编码/行尾字节。
- 新鲜只读基线连续跨度至少 300 ms，总预算 2 s；样本间隙不超过 250 ms。每组顺序读取以状态字前后包围，单组预算 300 ms；不是原子快照。
- 前后状态均 IDLE；急停解码无触发；非空运动 ID 必须 FINISHED；实际及目标速度绝对值均不超过 0.02 rad/s。
- 基线关节跨度与目标/实际差不超过 0.001 rad；TCP 平移对应不超过 0.5 mm。核对配置的软限位余量、启动包络与 TCP 设定。力矩通道须可读。
- 发出命令前再次核对配置原字节，末次基线不超过 100 ms。任一失败不写入，退出；不自动恢复 IDLE。
- 写请求与只读采样分开；请求后立即读取首帧，随后 60 s 观察，目标间隔 200 ms。首帧间隙、实际速度、位移、字段缺失或异常状态一旦超限保持失败，不用末帧静止覆盖过程。
- 请求超过 5 s 未返回记为等待超限且本轮不通过，但不补发、不升级其他命令。5 s 是诊断标记，**不是物理安全等待期限**；未返回请求在退出时仅尝试本地取消，远端结果可能未知。
- asyncio 超时依赖 SDK 响应取消；不是硬实时/硬进程截止。控制器读数可能有量化、延迟或冻结；采样不能排除亚 200 ms 瞬态。

## 发布后命令模板（目前禁止直接执行）

`$Source` 必须是通知指定的新隔离 `src`，`$Python` 为实验室已核验解释器；`$Output` 为全新绝对目录。

```powershell
& (Join-Path $Source 'scripts\run_static_system_stop.ps1') `
  -Source $Source -Python $Python `
  -Config 'C:\WorkSpace\VR4AM\config\real-robot.local.yaml' `
  -Output $Output `
  -ExpectedConfigSha256 '97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208' `
  -Confirm 'I_UNDERSTAND_SINGLE_SYSTEM_STOP_MAY_MOVE'
```

mode 范围：包装脚本备份后切 control，独立工具完成或失败后 finally 原字节恢复 readonly。若发现第三方修改配置，不覆盖它，记录恢复失败后停手。进程被强杀/掉电可能不运行 finally，因此必须核对 `execution.json.config_restored` 和当前哈希；切 readonly 不是停臂措施。无回显不代表没执行：先查产物/进程，禁止重复调用。

## 回传最小集

整包输出目录：原配置备份、控制候选文件、实测退出码和恢复记录 `execution.json`、stdout/stderr、`probe/events.jsonl` 与 `probe/result.json`。原件不编辑；逐件哈希、命令原文和实际导入路径随附。Windows PowerShell 5.1 的重定向会将原生进程的 stdout/stderr 写成 UTF-16LE 文本（含 BOM），不能当作子进程原始字节；核心结构化证据以 UTF-8 的 `probe/` 两件为准。事件日志逐条 flush/fsync；磁盘失败会中止而不补发。

只回答：是否发送了唯一 stop_sys；返回/迟到/未知；基线及观察是否完整；实际/目标关节、TCP、力矩与原始状态时间线；最大相对基线位移；人工干预或截断。`complete=true` 仅指本次诊断准则满足，不代表运动后停止可靠、不放行 VR。
