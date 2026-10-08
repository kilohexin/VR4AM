# 只读诊断启动失败与修正

2026-10-08，Asia/Shanghai。接续 [收发工具](readonly-wire-tooling-2026-10-08.md)。**用户已授权一次只读连接/正常 5 秒读取窗口/名义 15 秒指定网卡捕获；旧 Windows 确认超过启动有效期，没有执行该连接或抓包。本轮完成启动入口修正，没有真机命令、部署或推送。**

## 实际记录

旧 `.tmp/stop-wire-tools-20261007/approved-readonly-launch-20261008.json` 显示管理员进程于 `2026-10-08T01:10:27Z` 开始，原入口要求早于 `2026-10-07T19:21:29Z`。首个过期检查先于 SDK/捕获模块加载。启动记录仅留下 RemoteException，stdout/stderr 为空；目标采集目录未创建，本机 TCP 3031 established 数为 0。没有只读 state-report、ETL 或 PCAP；不能称这次诊断完成。原现场范围不重复申请，但重新开始前必须刷新当前 IDLE/目视静止、观察急停与无其它控制/采集客户端条件。

此前临时 PowerShell 直接运行 Python 并重定向 stderr，配合 ErrorActionPreference=Stop 将原生错误变成 RemoteException，跳过退出码保存。无害退出 7 的 Python 对照又发现 Start-Process 后仅 WaitForExit 会得到空 ExitCode；改为 Start-Process -Wait 后，退出 7、进程已退出和固定 stderr 均保存并核对。初始失败、对照 v1/v2 和原预检文件均保留在本机忽略目录。

## 可复用修正

- `scripts/launch_readonly_wire_capture.ps1` 固定启动仓库 `.venv` Python 和 `readonly_wire_capture.py`；默认仅预览，不提权、不连接、不查询网络连接表。新启动记录目录与采集目录必须分开且在仓库 `.tmp` 下；已存在记录拒绝覆盖。使用隐藏进程、-Wait、独立 stdout/stderr 和 launch.json，保留实际退出码；不自动 UAC、不重试。
- `-Execute` 必须提供原确认常量、审阅工具基线、批准配置哈希、批准预览文件及其哈希、UTC 启动有效期。先检查本机既有 TCP 3031 established 连接，查询失败或存在连接则退出；子进程退出后再记录连接数。此检查只看本地表，不发网络探测，也不能单独证明任意外部控制器客户端已关闭。
- 捕获 CLI 在构造 Native 或创建采集目录前核对预览字节 SHA-256、源码哈希、配置、端口、组件以及**唯一采集输出路径**；复用预览不能改成另一轮目录。预览源码清单包含新 PowerShell 入口。开始捕获前和启动只读子进程前再次查有效期；已启动的本轮采集照原归属检查收尾，人工筛选器清理仍明确保留。
- UTC 有效期传入固定只读子进程，在实际进入 SDK connection factory 前复核，防止 Python 启动/导入延迟跨过有效期。过期保存 `authorization_expired=true / connection_attempted=false / reads=[]`；不连接。Standalone readonly 旧接口仍支持省略此可选字段；捕获 CLI 的执行入口强制需要批准文件和有效期。

有效期仅限制**开始连接**，不是硬实时网络截止，也不替代现场观察；后台调度/系统时钟调整/SDK cancellation 的限制仍在。采集 `acquisition_complete` 不等于人工收尾完成，hardware_stop_verified 始终 false。原 200 ms、stop_unverified、禁自动 stop_sys/复位/重试及速度/加速度参数均保持。

## 定向验证

新增启动/授权测试 19 项。最初 13 项全红为入口/新参数缺失；实现后 41 通过。准备中和捕获开始后过期补测先得到 2 红（记录缺少过期字段）后修复。独立静态审查指出子进程没有收到有效期、批准文件未绑定唯一目录；3 项红灯复现后修复，复核确认已解决，无其它此次范围内可操作发现。审查仅读源码，没有执行测试或现场动作。

实际 Windows 测试只运行有效/无效配置的离线预览，核对空格路径、原始错误输出、真实退出码、进程结束和没有创建捕获目录；其余入口执行使用 FakeNative，不调用 SDK/网络。与原只读、配置哈希、停止时序/锁存/诊断共 **9 文件 118 passed，19.14 秒，退出 0**。没有后端全集、前端构建或真机运行。

## 下一步

旧冻结预览因运行源码变化已经失效，原件保留。修正版本的新预览已冻结：`.tmp/stop-wire-tools-20261007/readonly-wire-reviewed-preview-launch-v2-20261008.json`，SHA-256 `f14520d340b0664144bf0d83765e945c888773a811ee608b99c948000609b317`，76 个运行源码文件、组件 3、绑定尚未创建的原采集目录。获得**当前**现场状态确认后，再为原一次只读范围生成新的启动有效期并经 Windows 管理员进程执行。没有实际连接前，不认为原一次额度已使用；如果开始后失败或停止未确认，则不自动重跑。本次只读数据用于验证证据链路，不能解决或验收运动中约 610 ms 的 stop_move 延迟。厂家版本/回复和真实限速验收仍缺证据。
