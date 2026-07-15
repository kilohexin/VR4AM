# Quest 3 开发与安全操作指南

## 适用范围

本指南适用于 Milestone 1 的纯 VR LM3 仿真。浏览器页面只连接本机 FastAPI 仿真后端，从不连接 LM3 IP，不导入 Lebai SDK，也不产生真实机械臂动作。

Milestone 1 不是 MR/passthrough 应用，不采集相机画面，不录制数据集，不包含 Cannon-es 或物体抓取物理。

## 准备条件

- Quest Browser 使用 WebXR 本身**不需要**开启 Quest Developer Mode。
- PC 与 Quest 使用同一 WLAN；路由器必须关闭客户端隔离、AP isolation 或访客网络设备隔离。
- 仅可选的 USB ADB 反向转发流程需要 Developer Mode、ADB 和已授权的 USB 调试。
- 后端必须保持 `127.0.0.1:8000` 回环监听；局域网只开放 Vite 的 5173。

## 同一 WLAN 启动

### 1. 获取 PC 的局域网 IPv4

在 PowerShell 中运行：

```powershell
ipconfig
```

找到与 Quest 位于同一 WLAN 的活动网卡，记录其 IPv4 地址。不要使用 `127.0.0.1`、VPN 地址或已断开网卡的地址。下文以 `<PC-LAN-IP>` 表示该地址。

### 2. 启动回环后端

首次安装步骤见仓库根目录的 [README](../README.md)。安装后，从仓库根目录运行：

```powershell
.\.venv\Scripts\Activate.ps1
Set-Location .\backend
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

在 PC 上打开 `http://127.0.0.1:8000/health`，确认响应报告 `SIMULATOR` 且 `real_robot_enabled` 为 `false`。

### 3. 启动 Vite HTTPS

在另一个仓库根目录终端中运行：

```powershell
Set-Location .\web
npm.cmd run dev
```

Vite 监听 `0.0.0.0:5173`。如果 Windows 防火墙询问，只允许 TCP 5173 用于受信任的专用网络；不要放开后端 8000。

### 4. 在 Quest Browser 打开页面

1. 打开 `https://<PC-LAN-IP>:5173`。
2. 首次访问时，Vite `basicSsl` 自签名证书会触发浏览器警告；核对地址确为自己的开发 PC 后，选择继续访问。
3. 等待页面显示连接成功和 `SIMULATOR`。
4. 选择“进入 VR”，同意浏览器所需的沉浸式 VR 权限。

HTTPS 页面会建立同源 `wss://<PC-LAN-IP>:5173/ws/v1/teleop`。Vite 再把 `/ws` 代理至 PC 回环的 `ws://127.0.0.1:8000`；Quest 不直接连接 8000。

## 可选：USB ADB 反向转发

此流程不是 WebXR 或日常同 WLAN 开发的前置条件。仅在已启用 Developer Mode、Quest 已通过 USB 连接并在头显内授权“USB 调试”时使用。

在安装了 Android platform-tools 的 PC 上运行：

```powershell
adb devices
adb reverse tcp:5173 tcp:5173
adb reverse --list
```

设备状态必须是 `device`，不能是 `unauthorized`。后端和 Vite 仍按本指南启动，然后在 Quest Browser 尝试打开：

```text
https://127.0.0.1:5173
```

仍需处理 `basicSsl` 的证书警告。只反向转发 5173；同源 WSS/Vite 代理会继续把 8000 保留在 PC 回环。USB 连接必须保持，且 Quest OS/Browser 对本地证书和 WebXR 安全上下文的处理可能随版本变化；若此路径不可用，应回到上述同一 WLAN 流程，不要改为暴露明文 8000。

测试结束后移除转发：

```powershell
adb reverse --remove tcp:5173
```

## 右手柄映射

仅使用原厂 Meta Quest 3 Touch Plus **右手柄**。左手柄不参与遥操作。

| 输入 | 行为 |
| --- | --- |
| A | 显式发送一次解锁/使能请求 |
| B | 立即停止并解除使能，优先于同一帧 A |
| Grip | 运动离合；按下时建立相对锚点并移动，松开时停止运动 |
| Trigger | 归一化 `0..1` 夹爪闭合量 |

系统只对识别到的 `meta-quest-touch*` 或 `oculus-touch*` profile 使用 A=`buttons[4]`、B=`buttons[5]`。如果 profile 未识别、按钮数量不足、右手柄或 gamepad 数据缺失，系统保持锁定，并在 VR 中显示“手柄不受支持 / 当前配置不支持 A/B 安全控制”；它不会猜测其他按钮映射。

## 安全操作顺序

1. 确认页面已连接 `SIMULATOR` 后进入 VR。进入 VR 会立即走本地停止路径并自动回到锁定状态。
2. 完全松开 Grip、A 和 B，让系统先观察到各按钮的松开状态。
3. 保持 Grip 松开，短按一次 A。
4. 等待世界空间中文状态牌从“解锁中”变为后端确认的“已解锁”。没有确认时不得移动。
5. 按住 Grip 建立当前手柄与仿真 TCP 的相对锚点，然后缓慢移动或转动右手柄。
6. 松开 Grip；运动必须停止。再次按住 Grip 会在当前位姿建立新锚点，不应跳变。
7. 按 B 显式停止并解除使能。再次移动前，必须重新松开 Grip/A/B、按 A，并等待确认。

### 首次松开门闩

A 和 B 都使用“先观察到松开、再接受按下上升沿”的门闩。新 XR 会话、追踪丢失或恢复边界会重置门闩；恢复时持续按住的 A/B 不会触发命令，必须先松开再按。系统没有自动解锁或恢复后自动使能路径。

### 世界空间状态牌

VR 中的中文状态牌显示未解锁、解锁中、已解锁、运动中、已停止、故障/失联或手柄不受支持，并固定提示 `A 解锁 · B 停止 · Grip 移动 · Trigger 夹爪`。状态判断同时依赖后端确认，不应仅凭手柄按键推测已解锁。

## 中断与恢复

以下任一事件都必须立即触发本地停止/解除使能并锁定：

- Quest XR 会话的可见性进入 `hidden` 或 `visible-blurred`；
- 右手柄/位姿追踪丢失；
- WebSocket 关闭、断开或重连；
- 页面刷新、关闭或卸载；
- XR 会话结束、退出 VR 或清理。

恢复连接、追踪或可见性后，不会恢复先前的使能状态。先确认 Grip、A、B 全部松开，再按一次 A，并等待状态牌重新确认“已解锁”；随后才能按 Grip 移动。若状态牌仍显示故障/失联或不支持，保持 Grip 松开并停止操作。

桌面模式另有 `window blur` 输入复位，用于清除桌面 Grip/Trigger；它不是一项独立的 Quest XR 真机事件。Quest 真机验收以 XR 会话报告的 `hidden` / `visible-blurred` 为准。
