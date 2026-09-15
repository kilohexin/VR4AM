# 发给实验室：停止事务生产集成离线核验及回执模板

> 第 9 轮后更新：ACK 测试新增立即/延迟发送两个场景，同一命令最新预期为 **173 项**，见 [ACK 复核说明](lab-ack-race-response-2026-09-15.md)。下方 172 项记录属于上一修订，不覆盖实验室该轮失败。

> 后续修订：实验室原 `d10219c` 版本为 171 项且实测 170/1。线程启动竞态修正后同一命令为 172 项；见 [失败复核说明](lab-rehearsal-race-response-2026-09-15.md)。以主机端新提供的完整 HEAD 为准，不把旧版失败改记为通过。

## 可直接转发正文

主机端已完成停止事务生产集成，代码基准提交为 `8beaa119cf254b8b70e4fc756f21247349eb56ea`。本文件随后的提交只增加交接文档；请以主机端消息中给出的完整推送 HEAD 为更新目标，分支为 `codex/offline-rehearsal`。

与上一轮原型不同，本版已修改生产停止链路：同一停止事务内重复调用不重复发送 `stop_move`/`stop_sys`，固定首次截止时间，记录迟到结果；修复断连和网页关闭的取消时序。首轮停止升级仍保留，未放宽运动限位或现场配置。

原生产集成版主机端全量后端测试为 **773 passed / 0 failed**。线程启动竞态修正后，下方实验室指定子集已在主机端另行执行，结果 **172 passed / 0 failed，退出码 0**。两者版本与范围不同，不混用数量。

**本轮只做离线核验，不连接或操作机械臂。真机停止后位置变化与抖动仍未证实消除。通过本轮不自动放行任何真机动作。**

## 1. 更新及版本核对

在仓库根目录执行，保留原始输出；若命令报错、输出异常或 SHA 不符，停止并回传，不手改引用、不执行 `reset --hard`、不修改索引标记。

```powershell
git status --short
git branch --show-current
git fetch origin
git merge --ff-only origin/codex/offline-rehearsal
git rev-parse HEAD
git rev-parse 'HEAD^{tree}'
git diff --exit-code HEAD --
git diff --cached --exit-code HEAD --
```

仅当当前分支是 `codex/offline-rehearsal`、没有已跟踪修改且能正常快进时执行更新。保留既有 logs、配置备份及其它本地文件，不清理它们。不要根据 fetch 退出码单独断言版本到位，必须核对 HEAD。

若 Git 不在 PATH，可使用本机已确认的 git.exe 绝对路径，并如实记录。不要在回执或命令中暴露访问令牌。

## 2. 配置保全（只读文件，不连接机器人）

确认 `config/real-robot.local.yaml` 仍为 `readonly`，计算并记录原始字节哈希和文件大小；本轮不修改该文件，包括换行和编码。

```powershell
Get-FileHash -Algorithm SHA256 -LiteralPath config/real-robot.local.yaml
(Get-Item -LiteralPath config/real-robot.local.yaml).Length
.\.venv\Scripts\python.exe --version
```

既有现场基线 SHA-256：`97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208`，1567 字节。若不同，保留当前原件并回报，不自动恢复或覆盖。

## 3. 单次离线测试

保持机器人停测，不启动项目服务、不打开 VR 控制，不执行真机预检脚本。本命令使用假 SDK 和测试配置，不使用现场机器人连接。解释器缺失或依赖缺失时先回报，不临时替换为真机脚本。

在仓库根目录用 PowerShell 执行（下面是一行命令）：

```powershell
.\.venv\Scripts\python.exe scripts/observe_process.py --output artifacts/acceptance/lab-stop-transaction-20260915-001 --warn-after 120 -- .\.venv\Scripts\python.exe -m pytest -c backend/pyproject.toml backend/tests/robots/test_stop_transaction.py backend/tests/commissioning/test_owned_stop_integration.py backend/tests/commissioning/test_stop_timeout_lifecycle.py backend/tests/api/test_teleop_ws.py backend/tests/control/test_robot_control.py -q --tb=short
$ObserverExit = $LASTEXITCODE
$ObserverExit
```

输出目录必须不存在，不覆盖上轮产物。若已存在，请使用新编号并记录实际目录。`--warn-after 120` 只告警、不自动杀进程；它不是机器人停止保障。输出为空时先查原始产物和进程状态，不再次启动同一测试。

当前修订正常预期：**173 passed / 0 failed**；`process.json` 中 `status=EXITED`、`child_exit_code=0`、`observer_errors=[]`。观察器退出码与子进程退出码分别记录。若失败，保留原始失败，不反复运行直到通过，不删除失败日志。

测试验证请求去重、固定期限、迟到/取消结果、断连竞争、日志完整性及网页清理。假 SDK 中注入的漂移不是对真实物理根因的模拟证明。

## 4. 归档与暂停

原样复制以下四件产物至本次交付目录，记录源/副本 SHA-256，不改写时间字段，不只提供终端截图：

- `stdout.bin`
- `stderr.bin`
- `events.jsonl`
- `process.json`

附版本核对原始输出、配置哈希、下方回执。测试后再次核对配置哈希与 `git status --short`。无需运行机器人状态探针；现场状态未重新观察时写“本轮未采集”，不要复制历史 HOLD/IDLE 作为当前事实。

**收到主机端复核和单独的受控安排前，不恢复旧 A/B/C、B01 清单，不执行 prepare、平移、旋转、夹爪、Home、stop 真机诊断或 VR 真机控制；不为本次核验点击“启动机械臂”或调用 start_sys。** 现场离场或异常处置由操作员按既有安全规程进行，不为等待回执而延误安全处置。

## 实验室回执模板（执行后填写，不预填通过）

```text
【停止事务新版离线核验回执】
执行日期/时区：
执行人：
仓库路径：
分支：
完整 HEAD：
HEAD tree：
git status --short 原文：
工作区/暂存区 diff 退出码：
配置 mode：
配置大小及 SHA-256（测试前）：
配置大小及 SHA-256（测试后）：
Python 版本/解释器路径：
实际完整测试命令：
开始/结束时间：
测试通过/失败/跳过数量：
观察器退出码：
process.json.status：
process.json.child_exit_code：
process.json.observer_errors：
stderr 内容或字节数：
产物目录及四件文件完整 SHA-256：
是否存在异常/是否做过额外操作（如实列出）：
确认本轮未连接或操作机器人、未修改现场配置：是/否（说明）
机器人当前状态：本轮未采集 / 现场目视结果及观察时间（不推断）
交付目录：
结论：离线核验通过/失败/未完成；不代表真机故障已修复。
后续：保持停测，等待主机端复核，不自行追加运动测试。
```
