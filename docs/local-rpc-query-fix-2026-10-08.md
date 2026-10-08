# 停止诊断前置连接表查询修复

2026-10-08，Asia/Shanghai。接续 [工具兼容修复](motion-stop-tool-compatibility-2026-10-08.md)。本轮用户批准的范围仍为条件化清理唯一失败筛选器，以及一次 +Z 3 mm / 运动证据后早停 / 2 秒只读尾段诊断。**本轮实际未清理筛选器、未连接机器人、未运动、未启动捕获。**

## 实际中止与根因

06:32 UTC，管理员临时入口在第一次本机连接表查询处中止：`Get-NetTCPConnection` 子进程超过 5 秒限制。`cleanup-result/result.json` 记录 `cleanup_attempted=false`、`robot_connected=false`、`capture_started=false`、`tool_commands=[]`；`broker.json` 记录 `motion_launcher_started=false`。另一次普通本机查询也复现超时。失败没有被转换成零连接。

这次失败发生在 Pktmon 查询之前；之前留下的长名称筛选器尚未清理。本轮候选采集目录及启动器记录目录均未创建。一次机器人测试额度未使用，但 06:49 UTC 的启动期限已经到期，旧材料不能再启动。

## 改动

- 新增 `scripts/local_rpc_connections.py`，统一读取 `netstat.exe -ano` 的本机数字连接表。`-n` 使用数字地址、`-o` 包含 PID、`-a` 包含活动连接与监听端口；参数含义见 [Microsoft Netstat 文档](https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/netstat)。读取不会创建机器人 SDK、socket 或发起地址探测。
- 只计入远端端口严格等于 3031 的 TCP `ESTABLISHED` 行；处理 IPv4、IPv6 与 IPv6 scope。本地端口 3031、远端 30311、非 Established 状态、UDP 均不计入。
- 本机实测：普通进程输出英文表头，`CREATE_NO_WINDOW` 隐藏进程输出 OEM 936 编码的中文表头。新代码捕获原始 bytes，按系统 OEM 页严格解码，仅接受实测的完整中英文表头配对。未知语言、坏编码、缺列、未知状态、非数字地址、查询失败或超时均拒绝；不声称覆盖所有 Windows 本地化格式。
- 本机表中存在带数字远端的已连接 UDP 行；完整校验该行，但不将其算作 TCP RPC。
- 运动入口与共享 PowerShell 启动器使用同一查询。新增 helper 纳入两种入口的 runtime 哈希，旧批准预览因源码变化不能复用。默认离线预览仍不查询连接表。
- 审查发现启动器后置查询失败时可能保留子进程的成功退出码。已修复：保留原始 `python_exit`，后置状态显式为 `unknown`，启动器返回非零并记录 `launcher_exit`；已有非零子进程结果保持。

机器人控制、200 ms 门限、`stop_unverified` 和禁止自动 `stop_sys` / 复位 / 重试的边界未修改。

## 验证

新增回归先复现失败：旧查询委托、隐藏进程中文解码、连接 UDP 解析，以及后置查询错误报告成功。修复后最终运行 **6 个定向文件，128 passed，30.49 秒，退出 0**。范围为本机连接表、运动捕获、SDK 准入、Windows 启动器、哈希绑定与只读捕获；没有例行后端全集或现场重跑。Windows 启动器失败测试使用隔离的标准库子进程和查询 stub，不使用机器人 SDK 或实际系统表。

实际 helper CLI 退出 0、计数 0；随后三次只读测量分别为 **71.600 / 67.310 / 66.852 ms**，三次 TCP 3031 Established 均为 0。该计数描述查询瞬间的本机表，不证明机器人物理停止或未来连接状态。独立静态复核确认上述退出码问题闭合，未发现新增 P0–P2。

## 证据与下一步

失败材料保留在 `.tmp/motion-stop-admin-20261008T0629/`。11 文件清单为 `.tmp/local-rpc-fix-20261008/failed-stage-manifest.json`，SHA-256 `5d439b23fc81fa1ccdf9a05a16ae8a1e7f06f668b5ebe47c9fba48add74b33c5`。本机三次测量记录 `measurement.json` SHA-256 `2a76daf52dab7918eb35c14f6c6af9462b034b1441c7a3ad10571d11414c7ec5`。现场配置、工具原始输出与失败材料留在本机忽略目录。

此前冻结只读证据 23 文件逐项大小与完整哈希核对一致。PVAT、RobotControl、低速配置哈希与上一轮一致。旧管理员进程已退出，没有新的待执行 UAC、有效运动材料或机器人会话。

下一次只需刷新一次现场条件与同一诊断范围：IDLE 且目视静止、工具空载、机械臂全程净空、用户观察并掌握急停、没有其它控制或捕获任务。随后用新目录与当前源码准备材料，核对 idle、RPC 0、唯一筛选器完整显示指纹与冻结失败记录一致后，清理该失败筛选器一次并复核；当前工具基线、完整动作预览与截止时间绑定后，执行一次固定 +Z 3 mm 早停抓包。若条件不符或停止未确认，保持拒绝与锁存，不自动重跑。

真机 VR 的主要阻塞仍是运动中约 610 ms 的 `stop_move` 等待，需要取得运动请求/回复证据后定位；本次工具修复没有证明该问题解决，也没有完成 J5 实际速度与轨迹精度验收。暂不推送。
