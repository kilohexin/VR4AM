# 独立进程观测脚本（本机验证阶段）

入口：`scripts/observe_process.py`。不加载机器人 SDK，不修改配置，不自动重试、不启停机器人；没有接入真机测试入口。本轮只运行本机假命令。

## 本机使用示例

在项目根目录执行（使用本机现有 Python）：

```powershell
python .\scripts\observe_process.py --output artifacts\acceptance\observer-demo-001 --warn-after 1 -- python -u -c "import time; print('begin', flush=True); time.sleep(2); print('end', flush=True)"
```

目录必须是新目录，已存在则拒绝启动，避免覆盖证据。不要把多条命令、管道、shell 运算符拼成一条命令；脚本通过参数数组启动一个非交互程序，`shell=False`。标准输入关闭。

## 输出

- `stdout.bin`、`stderr.bin`：子进程直接写入的原始字节，不截断、不合并两个流。扩展名强调未转码，可用对应编码的文本查看器打开。
- `events.jsonl`：UTF-8 观测事件，包含 UTC、单调经过时间、PID、状态、原始子进程退出码、超阈值标志和观测错误。
- `process.json`：最近一次成功保存的事件快照。外部终止后可能陈旧，不能仅凭它断言进程现在仍活着。

启动前记录 PREPARED，启动后 STARTED；超过 `--warn-after` 只记录 WAIT_THRESHOLD_EXCEEDED，仍是 RUNNING，继续等实际退出。终态 EXITED；启动失败为 START_FAILED，没有伪造 PID 或子进程退出码。

终端约每 20 ms 轮询镜像已有输出，不保证硬实时。Python 子进程设置 PYTHONUNBUFFERED=1；其他程序自身的缓冲无法由观测器保证消除。两个流分开保存，不承诺精确的跨流全序。

终端镜像由独立有界队列执行。终端堵塞或输出速度过高时可以省略终端片段，并记录 terminal_mirror_backpressure / terminal_mirror_incomplete；原始 stdout.bin、stderr.bin 不因此截断。子进程退出后只额外等待镜像最多 0.5 秒，不因终端无人读取而卡住退出状态记录。

## 超时、取消和安全边界

`--warn-after` **不是硬超时**，不会结束程序。如果子进程挂起，观测器也会继续运行；没有自动 kill 或重试。操作员必须理解这一行为，不能把告警当作已停止。

在监测循环中收到 Ctrl+C 时记录 OBSERVER_INTERRUPT_CONTINUING，继续观测，不把它转换成机械臂停止命令。子进程使用独立进程组/会话（Windows 隐藏窗口），但不同外层工具仍可能强制结束整个进程树。

不能保证在观测器、终端工具或 OS 被强制终止时完成收尾；子进程可能继续运行。不要因没有输出或观察器退出就重跑命令。启动窗口内的异常中断也不能保证完整记录。此脚本不是机器人监督控制器、急停工具或安全保证。

数据只覆盖直接子进程的生命周期。不支持通过 shell 再启动长期后台孙进程；子进程退出不等于其派生进程已退出。`robot_stop_confirmed` 始终为 null；不读取或推断机械臂状态。

## 错误和退出码

正常完成观测时返回子进程退出码，并在 JSON 中保留原值。观测层记录/镜像错误返回 2，原始子进程码仍尽量保存在 JSON；启动失败也返回 2。因此不能只看 shell 码而不看事件类型。

元数据写入失败时仍不终止子进程，内存记录错误并继续等退出；如果之后恢复可写，后续快照会携带错误。磁盘写满、断电等情况下不能保证错误记录、终端告警或完整证据；JSON 中没有错误不证明所有底层 I/O 已持久化。

命令参数和工作目录会存入日志，勿在参数中放密码、令牌或其它不应留存的秘密。日志不自动上传。

## 验证

测试使用真实本机子进程，覆盖完整双流、大于末尾摘要长度的输出、非零退出码、无输出、超阈值仍完成且只启动一次、运行中输出可见、启动失败、非法阈值及现有目录保护。

审查发现并修复终端背压阻塞问题：增加“不读取观测器 stdout、子进程输出 2 MB 后退出”的测试，旧实现因等待超时失败，新实现保留完整原始输出和子进程退出码，并将镜像不完整单独报告。最终执行 `python -m pytest tests/scripts/test_observe_process.py tests/prototypes tests/commissioning tests/robots/test_lebai_stop_evidence.py -q --tb=short`，103 passed（29.51 s），其中新增脚本 12 项。CLI `--help` 正常退出。未运行全后端套件。

真机测试尚未获准使用本脚本；本机验证通过不能解释或修复 B01 停止后的位移变化。
