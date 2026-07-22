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

Milestone 1 使用原厂 Touch Plus 双手柄。右手控制机械臂末端和夹爪：

- A：显式解锁/使能仿真；
- B：立即停止；发生可恢复硬故障时，松开 Grip 后按 B 开始复位；
- 右 Grip：运动离合。每次按下都会锁定当前“手柄—TCP”零位，此后手柄相对零位的前后、左右、上下直接映射到末端；手柄姿态按夹爪双目摄像头工具坐标系映射，手柄绕自身前向轴翻滚时，夹爪绕摄像头光轴翻滚；松开即停止；
- 右 Trigger：`0..1` 的夹爪闭合量。

左手柄只调整 VR 中工作台的观察位置，不会向后端发送机械臂目标：

- 左摇杆：前后、左右移动工作台；
- 左 Grip + 左摇杆 Y：升降工作台；
- 按下左摇杆：恢复工作台默认三维位置。

进入 VR 会自动停止并锁定。先松开 Grip、A、B，再按一次 A；必须等世界空间中文状态牌确认“已解锁”后，才能按住右 Grip 移动。系统不会自动解锁，B 的停止优先于同一帧的 A 解锁。

琥珀色“已到达操作边界/当前方向暂时不可达”是软约束：末端停在最后有效位置，将手柄退回可达区域后会自动继续，不需要复位。红色硬故障仍会停止并锁定；在仿真模式中点击复位后，机械臂会依次确认停止、低速返回 Home、确认稳定，成功后才清除故障并允许重新解锁。真机后端仍为硬禁用状态，不会执行自动回 Home。

每次 Grip 松开或仿真停止时，连续控制参考都会同步到机械臂实际关节状态；再次按下 Grip 后从当前真实仿真姿态继续，不沿用停止前尚未执行到的旧目标。微分 IK 每个控制周期都从实际关节角计算一个受限小步，普通的姿态受限或单步关节裁剪不会触发需要复位的硬故障。

当前仿真的 FK、雅可比矩阵、微分 IK 和浏览器渲染共同读取 `config/lm3_visual_kinematics_v1.json` 中的 GLB 关节链，避免数学末端与画面末端方向不一致。伺服器优先满足 TCP 位置；姿态接近关节边界时允许暂时降低姿态跟随精度，以保持平移仍然跟手。桌面橙色方块用于最小抓取演示：将夹爪 TCP 移到方块附近，按下右 Trigger 闭合即可抓住，移动机械臂时方块跟随，松开 Trigger 后方块回落到桌面。该交互仅用于浏览器仿真，真机后端仍保持禁用。

### 本次改动的人工验收

1. 按住右 Grip 锁定零位后，手柄向前、后、左、右、上、下移动，确认 TCP 视觉方向一致。
2. 将手柄移回零位，确认 TCP 回到锁定时位置。
3. 松开右 Grip，再次按下，确认末端目标没有跳变。
4. 分别做手柄俯仰、偏航和绕自身前向轴滚转，确认夹爪以双目摄像头光轴为前向且旋转符合直觉。
5. 在停止或锁定状态下，用左摇杆调整工作台前后左右，用左 Grip + 摇杆 Y 调整高度，并按下摇杆复位位置。
6. 将目标移动到相对工作边界，确认只出现琥珀色提示和一次短振动，不进入故障。
7. 将手柄退回有效区域，确认机械臂自动恢复跟随，无需点击复位。
8. 将夹爪靠近橙色方块，按下 Trigger 抓住方块；移动末端后松开 Trigger，确认方块回到桌面。
9. 触发一个真正硬故障，确认系统停止并保持锁定。
10. 松开右 Grip 后点击复位，确认能看到“确认停止—返回 Home—确认稳定”的过程，完成后才可重新解锁。
11. 确认项目没有物体动力学、Cannon-es 或浏览器物理仿真行为。

详细操作与故障恢复见 [Quest 开发与操作指南](docs/quest-development.md)。真机验收必须使用 [Milestone 1 验收清单](docs/milestone-1-acceptance.md)；在该清单由实际 Quest 测试人员完成之前，Quest 真机验收状态始终为 **PENDING**。
