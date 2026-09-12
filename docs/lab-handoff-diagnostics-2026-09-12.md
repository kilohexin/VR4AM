# 实验室交接：停止异常诊断版（2026-09-12）

## 当前状态

**本次交接只安排版本核对和离线自检，不安排真机动作。** 不需要机器人上电或连接机器人网络。

- 已完成 PVAT 请求生命周期诊断、独立进程观察脚本和离线集成验收。
- 相关离线回归：290 项通过（32.87 秒）；进程观察脚本包裹的重点验收：10 项通过，子进程退出码 0，观测错误为空。
- B01 的停止超时、STOP/HOLD 和停止后位置变化仍未查明根因。单次成功及离线假 SDK 结果不能解除限制。
- 保持现场配置 `readonly`；不运行 prepare、translate、rotate、gripper、Home、stop 或 start_sys；不进入 VR 真机控制；不重复猜测 RPC 接口。
- 用户要求自行排查，不联系厂商，不以厂商答复作为继续工作的前置条件。

## 1. 主机发布前核对

编写本文时，仓库 HEAD 是 `d1c5eb5562e0196a437dd1bdadda36c5a77b2889`，诊断代码尚在未提交工作区。**这个哈希不是本次新版本号。** 用户提交、推送后，应把实际完整提交号发给实验室；在此之前不要通知实验室“新版本已可拉取”。

本次诊断交接需要包含：

- `backend/app/robots/lebai_adapter.py`
- `scripts/observe_process.py`
- `backend/tests/robots/test_pvat_request_lifecycle.py`
- `backend/tests/commissioning/test_pvat_lifecycle_recording.py`
- `backend/tests/commissioning/test_stop_timeout_lifecycle.py`
- `backend/tests/scripts/test_observe_process.py`
- `backend/tests/prototypes/` 中的离线模型和测试（仅测试目录，不进入生产控制路径）
- 本文，以及 `docs/pvat-request-lifecycle-2026-09-12.md`、`docs/process-observer-2026-09-12.md`
- 排查记录：`docs/b01-stop-analysis-2026-09-10.md`、`docs/sdk-stop-cancellation-probe-2026-09-12.md`、`docs/observation-exit-investigation-2026-09-12.md`、`docs/offline-stop-episode-prototype-2026-09-12.md`

不要把 `real-robot.local.yaml`、现场日志、SDK 探测产物或未授权的文件一起提交。`docs/group-meeting-progress-2026-09-10.md` 是独立组会材料，不属于本次诊断提交范围。不要使用 `git add .` 混入无关文件。

## 2. 实验室收到已发布的准确提交号后

先核对分支和本地修改，不使用 `git reset --hard`，不自行重建引用或回退分支。若拉取异常、HEAD 不符或存在未解释修改，停止版本更新并回传原始输出。

在仓库根目录运行这些只读命令，并保留输出：

```powershell
git branch --show-current
git rev-parse HEAD
git status --short
Get-FileHash -Algorithm SHA256 -LiteralPath .\config\real-robot.local.yaml
```

人工确认配置仍为 `readonly`。哈希不同只说明字节不同，不能直接推断哪个参数变化；需要原件才能比较。此前存档只读基线哈希为 `97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208`，不要求为了匹配哈希修改当前配置。

## 3. 唯一安排的执行项：离线诊断自检

以下命令从仓库根目录开始，使用已有 Windows 虚拟环境；若路径或依赖缺失，先回传错误，不自行改成真机脚本。

```powershell
Set-Location .\backend
..\.venv\Scripts\python.exe ..\scripts\observe_process.py --output ..\artifacts\acceptance\lab-offline-diag-001 --warn-after 15 -- ..\.venv\Scripts\python.exe -m pytest tests/commissioning/test_pvat_lifecycle_recording.py tests/robots/test_pvat_request_lifecycle.py tests/commissioning/test_stop_timeout_lifecycle.py -q --tb=short
```

该命令测试本地假 SDK，不加载现场配置，不访问机械臂。预期 10 项通过；如果环境错误或测试失败，保存全部文件后暂停，不据此调整控制参数。输出目录必须不存在，已有目录不可覆盖；如确需另一次离线运行，换一个新目录并保留旧证据。

`--warn-after 15` 只提示等待过长，不杀进程、不重试。没有输出不等于命令未执行。不要附加 `tail`、`Select-Object -Last` 或其它截断输出的管道。该观察脚本不是急停工具，`robot_stop_confirmed` 为 null 是预期行为。

## 4. 回传内容与判断

回传完整提交号、分支、工作区状态、配置哈希、Python 版本，以及完整的 `artifacts/acceptance/lab-offline-diag-001/`：

- `stdout.bin` 和 `stderr.bin`：原始字节，不重排、转码或截断。
- `events.jsonl`：观察器启动与退出过程。
- `process.json`：应核对 `status=EXITED`、`child_exit_code=0`、`observer_errors=[]`，不能只看外层退出码。

简要回执模板：

> 完整提交号：____；分支：____；工作区：____。配置 readonly，SHA-256：____。Python：____。离线重点测试：____ 项通过 / ____ 项失败。观察器状态：____，子进程退出码：____，观测错误：____。原始文件已传输至：____。本次未连接或操作机械臂，B01/B 段仍暂停。

通过本节仅表示诊断版本在实验室笔记本上可用，不是物理停止验收。后续真机复测必须另行明确单次动作、现场监督、停止手段和证据采集范围；本文不构成该复测安排。
