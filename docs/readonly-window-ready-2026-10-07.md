# 只读窗口执行准备与旧服务收尾

2026-10-07。用户已确认上轮提出的一次 5 秒只读范围，并要求需要通电时通知。当前尚未连接机械臂，现场通电、静止、工具安装与人员/空间条件待确认。运动、Home、复位、prepare、夹爪动作与运动停止诊断未获本轮授权。

## 确认并处理旧服务

原工作目录在 `C:/WorkSpace/VR4AM`。原现场配置目前确为 readonly，SHA-256 `97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208`，与旧关停记录一致；其地址与冻结 readonly 候选一致，没有猜测或修改配置。

被动读取进程表和监听表后，通过 `round29v-backend-launch-run4.txt`、`round29v-frontend-launch-run4.txt` 及后端启动日志确认残留服务来自旧轮次。后端日志明确记录了当前端口所属 PID；前端由 Node 名称、5173 端口、创建时间与启动记录关联。16:46 的关停发生在 16:49 重启之前，不能替代 Home 事件后的最终收尾。

先保留 Home 会话原日志、旧服务输出和配置到独立忽略目录，再按已记录的 `Stop-Process -Force` 流程结束旧前后端。执行前再次比对 PID/进程名/创建时间/端口归属，避免 PID 重用或误杀其他进程。硬终止不运行应用退出回调，没有调用 SDK、服务 API 或机器人控制命令。没有直接复用旧关停脚本中的 HTTP 请求、宽范围 PID 兜底、日志覆盖和环境变量修改。

之后复查：两个旧工作进程及两个启动器均消失，8000/5173 均无监听；原始日志与配置和采集前副本逐字节一致。这是本机资源收尾证据，不是机械臂静止或控制柜侧连接释放证据。

保留目录：`.tmp/readonly-window-20261007T131732`。Home 原会话末件 9,473,839 字节，SHA-256 `2f0bfe00560aafe060b710975208b85b22eea856470b6a950109fe140bffe587`。早先 6,521,229 字节冻结件 SHA-256 `99e99d108081490258152f5fe8388f2483bb53261b41d31aeb303696be449bc3` 已核实，且确为末件前缀。两件均保留，未覆盖原件。配置、副本、进程记录和精确执行清单均留在忽略目录，不提交原始文件。

## SDK 与离线验证

新环境原先缺 SDK，现已安装 `lebai-sdk-asyncio==0.3.13`，与旧环境版本一致，原生二进制 SHA-256 `ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b` 与旧环境相同。当前 Python 是 3.12.14；wheel 标签 cp37-abi3-win_amd64，原生导入成功，七项查询方法均存在。只验证导入与方法存在，没有调用 SDK init、connect 或任何设备方法，也没有变更旧环境。

使用现场 readonly 配置的保留副本完成默认离线预览，哈希一致。安装后四个相关文件定向回归 **58 项通过，6.05 秒**，SDK 桥接与查询测试均注入/替换连接，不是实机验证。没有重跑后端全集。输出 `.tmp/readonly-window-environment-regression.txt`。

```powershell
.venv/Scripts/python.exe -m pytest backend/tests/scripts/test_readonly_state.py backend/tests/scripts/test_readonly_dynamics.py backend/tests/robots/test_lebai_adapter_readonly.py backend/tests/robots/test_lebai_sdk_bridge.py -q -p no:cacheprovider -o asyncio_default_fixture_loop_scope=function
```

## 下一步

已保存 `execution-plan.json`：确切源码提交、配置路径/哈希、新报告路径、单次 CLI 参数、已接受范围与待确认现场事实。尚未执行 `--execute-readonly`。现场先由操作人通电并观察静止，确认夹爪/工具安装、工作区清空、急停可达和其他控制客户端关闭。只允许自然进入 IDLE；若进入 IDLE 需要复位、Home 或其他操作，暂停并报告，不扩展范围。

事实齐备后仅执行已接受的一次只读连接。IDLE、采集成功和 SDK 返回均不能证明运动中的停止可靠。约 609 ms 现场停止返回问题仍未解决，B/C 不因这一步完成而自动放行。未推送、未部署 VR 服务。
