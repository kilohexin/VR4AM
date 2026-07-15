# VR4Arm Milestone 1

> **仅限仿真（SIMULATOR）**
>
> Milestone 1 不安装、导入或调用 `lebai_sdk` / `lebai_sdk_asyncio`，不配置或连接 LM3 的 IP/网络，也不会驱动真实机械臂。仓库中的真实适配器只是硬禁用占位实现；任何非 `simulator` 后端配置都会在启动时被拒绝。

本项目在浏览器和 Meta Quest 3 中显示 LM3 机械臂的纯 VR 仿真，并通过右侧 Touch Plus 手柄进行带安全门禁的相对位姿控制。它不是实机控制程序。

## 架构

正常的同一局域网开发链路如下：

```text
Quest Browser
  https://<PC-LAN-IP>:5173
  wss://<PC-LAN-IP>:5173/ws/v1/teleop
                │ 同源 WSS
                ▼
Vite HTTPS 开发服务器（监听 0.0.0.0:5173）
                │ /ws WebSocket 代理
                ▼
FastAPI（仅监听 127.0.0.1:8000）
                │
                ▼
SimRobotAdapter（内存中的 LM3 仿真）
```

浏览器默认从当前 HTTPS 页面的同源地址建立 WSS，不直接访问后端的明文 WebSocket。Vite 将 `/ws` 代理到 `ws://127.0.0.1:8000`。

## Windows PowerShell 启动

需要 Python 3.11 或更高版本，以及与 `web/package-lock.json` 兼容的 Node.js/npm。以下命令均从仓库根目录开始。

### 后端

```powershell
python --version
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".\backend[dev]"
Set-Location .\backend
python -m pytest -q
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```

保持此终端运行。PC 本机可检查：

- 健康检查：`http://127.0.0.1:8000/health`
- 后端原始 WebSocket：`ws://127.0.0.1:8000/ws/v1/teleop`

`/health` 应报告 `backend: "SIMULATOR"` 和 `real_robot_enabled: false`。

### 前端

在另一个位于仓库根目录的 PowerShell 终端中运行：

```powershell
Set-Location .\web
npm ci
npm.cmd test
npm.cmd run build
npm.cmd run dev
```

保持开发服务器终端运行，然后在 Quest Browser 打开：

```text
https://<PC-LAN-IP>:5173
```

Vite 使用本地自签名证书。首次访问需要在 Quest Browser 中明确接受证书警告，随后页面建立的有效遥操作连接是：

```text
wss://<PC-LAN-IP>:5173/ws/v1/teleop
```

该地址由 Vite 代理到回环后端，不表示 FastAPI 的 8000 端口已对局域网开放。

## 局域网与防火墙

- PC 与 Quest 应连接同一 WLAN，并关闭客户端隔离/AP isolation。
- Windows 防火墙只需允许 Vite 的 TCP 5173 入站（建议仅限“专用网络”配置文件）。
- FastAPI 必须继续绑定 `127.0.0.1:8000`；正常开发不要向局域网开放 8000。
- 只有在另一个部署端点本身能够正确终止 TLS/WSS 时，才设置 `VITE_TELEOP_WS_URL` 覆盖默认同源地址。不要把它指向 Quest 页面中的 `ws://` 明文地址。

## Quest 控制摘要

Milestone 1 只读取原厂 Touch Plus **右手柄**：

- A：显式解锁/使能仿真；
- B：立即停止并解除使能；
- Grip：运动离合，按住时相对移动，松开即停止运动；
- Trigger：`0..1` 的夹爪闭合量。

进入 VR 会自动停止并锁定。先松开 Grip、A、B，再按一次 A；必须等世界空间中文状态牌确认“已解锁”后，才能按住 Grip 移动。系统不会自动解锁，B 的停止优先于同一帧的 A 解锁。

详细操作与故障恢复见 [Quest 开发与操作指南](docs/quest-development.md)。真机验收必须使用 [Milestone 1 验收清单](docs/milestone-1-acceptance.md)；在该清单由实际 Quest 测试人员完成之前，Quest 真机验收状态始终为 **PENDING**。
