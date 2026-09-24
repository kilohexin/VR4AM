# 低速 VR 候选配置：仅离线准备

本文件供实验室 AI 在下一轮真机 VR 演练前准备候选配置。它不授权连接设备、切换 `control` 或发送运动指令。

从现场现有的 `readonly` 配置生成**新文件**，原配置逐字节保持不变；候选文件也保持 `readonly`。工具使用 `x` 独占创建，目标已存在即失败，不覆盖历史文件。只收紧现有限值，不提高任何限值，不改 IP、TCP、准备姿态、关节限位或夹爪设置。

在实验室仓库根目录用 PowerShell：

```powershell
$env:PYTHONPATH = (Resolve-Path .\backend).Path
New-Item -ItemType Directory -Force .\artifacts\acceptance | Out-Null
$source = (Resolve-Path .\config\real-robot.local.yaml).Path
$output = Join-Path (Resolve-Path .\artifacts\acceptance).Path 'vr-low-speed-candidate-001.yaml'
$before = (Get-FileHash -Algorithm SHA256 -LiteralPath $source).Hash
python .\scripts\prepare_vr_low_speed_profile.py --source $source --output $output
if ($LASTEXITCODE -ne 0) { throw 'PROFILE_GENERATION_FAILED' }
$after = (Get-FileHash -Algorithm SHA256 -LiteralPath $source).Hash
if ($before -ne $after) { throw 'SOURCE_CONFIG_CHANGED' }
```

候选上限（9 项）：TCP 平移 `0.005 m/s`、旋转 `0.05 rad/s`、关节 `0.05 rad/s`；平移/旋转加速度 `0.02 m/s²` / `0.1 rad/s²`、关节加速度 `0.2 rad/s²`；单步 TCP 平移 `0.5 mm`、旋转 `0.2°`；映射比例 `0.2`。候选保留来源文件的 `max_relative_translation_m` 与 `max_relative_rotation_deg` 原值，但新版真机 `LEBAI` 运行路径不再使用 Grip 锚点相对位移／旋转边界；假 SDK 路径仍使用这两个字段。**这不取消速度、单步、关节、IK 等检查，也不是实测速度或运动安全保证。**

旧版候选／control 副本含额外收紧的 `20 mm`／`5°` 字段，不能沿用为新版候选；必须在新隔离源码中从现场原始 `readonly` 配置重新生成并核验，且不能接续此前已闩锁故障的会话。

生成后回传原/新 SHA-256、配置差异、测试退出码，并核对候选仍为 `readonly`。真机启动与动作须另行明确安排；此工具不执行任何现场步骤。

在仍为 `readonly` 时立即运行静态核对（不连接设备）：

```powershell
python .\scripts\verify_vr_low_speed_profile.py --source $source --candidate $output
if ($LASTEXITCODE -ne 0) { throw 'LOW_SPEED_PROFILE_VERIFY_FAILED' }
```

核对器逐字段拒绝预期之外的配置修改，也拒绝候选限值被调高或原/候选指向同一文件。若未来现场**另行授权**将候选文件切到 `control`，须在启动后端之前用项目确认词运行 `--expect-mode control` 再核对一次；核对器本身不会切换模式、连接设备或放行运动。

现场前置通过、收到明确的真机试验安排后，可以创建**第三份、独立的** `control` 文件；这不是修改前两份文件，也不会启动后端：

```powershell
$env:VR4ARM_REAL_ROBOT_CONFIRM = 'I_UNDERSTAND_REAL_ROBOT_MOTION'
$controlOutput = Join-Path (Resolve-Path .\artifacts\acceptance).Path 'vr-low-speed-control-001.yaml'
python .\scripts\prepare_vr_low_speed_control.py --source $source --readonly $output --output $controlOutput
if ($LASTEXITCODE -ne 0) { throw 'CONTROL_COPY_FAILED' }
python .\scripts\verify_vr_low_speed_profile.py --source $source --candidate $controlOutput --expect-mode control
if ($LASTEXITCODE -ne 0) { throw 'CONTROL_COPY_VERIFY_FAILED' }
```

以上两条命令均只读/写本地配置文件，不连接 SDK。`control` 文件不得替代现场原始 `readonly` 配置；试验结束时关闭服务、撤销本会话的确认词环境变量，不自动运行后续动作。

主机端离线测试还会把生成的假配置装入 `LEBAI_FAKE` 的实际运行时限速器，以连续 50 个 20 ms 周期的超限目标核验每周期输出。该测试只覆盖**配置 → 限速器**，不覆盖 Quest 网络传输、PVAT、真机实测速度或现场停止行为。
