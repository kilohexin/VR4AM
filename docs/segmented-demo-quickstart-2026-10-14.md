# 分段控制离线演示 — 启动与交付检查

## 当前交付能力

这是本机 PC 的完整网页演示入口：实际前端构建、WebSocket、RobotControl、分段适配器与本地 LM3 模型共同运行。可演示解锁、Grip 平移输入、有限短段及反馈等待、Trigger 开合、松开 Grip 和停止。它不加载原生乐白 SDK，不连接机器人；不包含物理引擎，也没有真实头显验收。

真机 VR 主线代码已经接入，现场仍需通过真实 FK/IK 与方向、单段运动与原停止、连续 Grip/Trigger 三组验收。历史约 621 ms 停止回复和真实 FK 超时未被本次假客户端结果解决。每段 0.2 mm 是当前保守上限，不能作为已完成可见跟随标定或交付速度的承诺。物理抓取继续后置。

## 启动

在仓库目录打开 PowerShell。依赖使用本机已有 `.venv`、`web/node_modules` 和 Node.js 22+；这不是可脱离依赖运行的独立 exe。

```powershell
Set-Location C:/Users/Kilo/Documents/ChatGPT/Agent4ARM/vr4arm-offline
.venv/Scripts/python.exe scripts/run_segmented_demo.py --build
```

等待控制台 `Application startup complete`，然后用这台电脑的浏览器打开：

**http://127.0.0.1:8768/**

以后源码没有变化时可直接运行：

```powershell
.venv/Scripts/python.exe scripts/run_segmented_demo.py
```

只构建或核对包而不启动服务：

```powershell
.venv/Scripts/python.exe scripts/run_segmented_demo.py --build-only
.venv/Scripts/python.exe scripts/run_segmented_demo.py --check
```

脚本优先使用本机 bundled Node。其他机器可通过 `--node <node.exe绝对路径>` 指定已有 Node.js 22+。缺依赖会明确失败，不自动安装。构建保留每个独立版本，不覆盖旧输出；包在 `artifacts/delivery/segmented-demo/`，`latest.json` 指向最新成功版本。

入口固定为 loopback 和 Fake 工厂，没有 IP、host、live、config 或 execute-once 参数。存在真机确认环境变量、非 Fake 配置或 WebSocket 环境覆盖时会拒绝启动；如使用过真机终端，请另开没有这些变量的终端，不要修改现场配置。`delivery_sources_changed` 或 `delivery_assets_changed` 表示旧包已失效，重新 `--build`。

## 桌面操作与检查

1. 只打开一个控制页面。等待“已连接”、`LEBAI_FAKE`、顶部“仿真模式，不是真机”和“分段候选 · 未真机验收”。
2. 保持鼠标左键松开，点“解锁仿真”，确认进入 ARMED。
3. 在机械臂画布上按住鼠标左键，等 ACTIVE，再缓慢拖动几像素。左键模拟 Grip；画布中的标记表示输入，机械臂短段反映后端反馈。滚轮改变另一个输入轴，右键拖动调整视角。
4. 保持左键按住时按空格模拟 Trigger；松开空格，并等待夹爪反馈回到 0%，再松开左键。Grip 松开后不会继续发送旧目标；点“停止”关闭权限。
5. 若出现 STALE 或故障，先松开输入并停止，查看显示与日志。启动加载阶段本次曾触发 STALE，随后按原流程重新解锁；没有放宽超时。停止未确认仍锁存，不在网页中自动复位。
6. “开始演练”在该模式禁用，因为原自动演练依赖 Home；B/停止不执行 Home。首版不提供旋转、准备位姿或在线故障复位。
7. 用完先松开输入、点停止，再关闭页面；终端 Ctrl+C 结束本地服务。脚本只有正常启动、非强制退出、应用收尾完成且无收尾错误才判成功。连续按 Ctrl+C 强制退出不算确认收尾；本地成功收尾也不代表硬件停止。

`/demo/identity` 提供 Fake 身份、SDK 未加载、物理引擎未启用、头显未验证和日志目录；`/health` 的 `real_robot_enabled` 必须为 false。`real_robot_mode=control` 是本地假客户端软件模式，不能解读为已连接真机。

此入口只服务本机浏览器。头显中的 127.0.0.1 指向头显自身；当前 HTTP loopback 演示不提供头显远程接入。真实 WebXR、HTTPS 和现场部署需后续专项验收。

## 本轮验证与材料

- **136 项后端定向测试、92 项前端定向测试通过**；TypeScript 与 Vite 构建通过，保留已有大型 chunk 警告，未重跑后端全集。
- 完整构建页面使用已有 Edge / Playwright 验证；Browser 插件不可用且 bundled Chromium 不存在，未下载浏览器。所有 HTTP / WebSocket 限定 `127.0.0.1:8768`。
- 最终页面记录 148 条状态，经过 READY、ARMED、ACTIVE、HOLD 及回执、时间窗口、反馈等待；关节最大变化约 `0.001742 rad`，夹爪反馈从 0 到 1 再回到 0。1440×1000 和 390×844 页面检查通过，无控制台错误、警告或外部请求。它是合成运动，没有接触、力、重力或真实抓取证明。
- 初次关闭页面时取消假读取，被缺省原生 RPC 规则判成未知结果，后续连接失败。只在新的纯本地客户端声明原生读取次数为 0、无原生结果未知；有效取消用例复现并验证修复。真实适配器、所有权、停止超时和锁存未改。
- Fake 采用本地数值 IK、原适配器四次 FK 准入和零起止速度/加速度的五次曲线。演示关节参数沿用已有响应型 Fake 配置，不能复制为真机参数。SDK 回执或假关节动画不等于控制器轨迹验收。
- 初始构建、包审查和浏览器失败记录保留在本机。最终 Fake 日志在 `logs/commissioning/20261014T103732Z-dabd4df1/`，summary 为 475 条普通、20 条关键事件、0 条普通丢失、无致命错误；页面“Dropped events”还有独立诊断显示窗口，不能直接当日志写入缺失数量。正常应用收尾已观察到，PTY Ctrl+C 外层返回码为 1，未把它写成进程退出码 0。

测试与冻结材料：`.tmp/segmented-delivery-package-20261014/`。完整页面截图、原始 WS 状态和测试脚本在仓库外 `../.tmp/segmented-demo-20261014/`，保留本机，不提交 Git。日志目录内的 `session.jsonl`、`summary.json`、`commissioning-report.md` 可供本机排查；本轮没有增加网页下载接口或上传现场日志。

下一步优先处理真机运行前的离线准入与延迟预算，再在用户重新授权和现场条件满足时开展上述三组验收。当前没有准备新的现场执行授权，也没有推送本轮代码。
