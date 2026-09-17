# 工具负载与重力配置：一次只读采集

## 发布边界

本工具新增于第38轮接口核对之后。必须先取得包含 `scripts/real_robot_readonly_dynamics.py` 和 `backend/app/commissioning/readonly_dynamics.py` 的新提交并验证隔离导入。旧7383287不含本工具。当前文档不代表已推送或真机验证通过。

本轮仅采集当前配置，不修改参数、不判断机器人是否可运动、不运行preflight/smoke。现场无需称重、拆装、猜重心。LMG-90自重0.46kg仅作参考，不自动写入。

## 行为

配置必须readonly，输出JSON必须不存在；均在连接前检查。按配置IP连接一次，调用get_payload一次、get_gravity一次，任一失败即退出，不重试。连接等待5秒、单次配置读取等待2秒，仅适用于本工具，不改变生产停止期限。

这些是协作式取消期限，不是进程级硬实时保证：SDK同步初始化阻塞或忽略取消时，进程可能晚于期限退出。返回后再次核对单调时钟，迟到结果保留为失败、不得继续下一项读取或标记complete。若进程仍在，不重跑；保存终端情况并回传，不把本地超时当作远端取消确认。

不构造控制适配器，不调用其disconnect/stop收尾；独立CLI进程退出释放本进程连接。取消本地只读等待不等于取消控制器请求。若进程未退出，记录情况，不再启动第二个实例。

`complete=true`仅代表两次返回值已成功记录且配置字节未变，不代表字段合理、已标定或运动安全。原始JSON值保留；重力单位/归一化及cog参考系未知，不自行推断。非JSON/非有限数值返回记录失败类型，不伪造数值。

## 发布后执行原文

先将 `$Src` 改为本次新提交的已核验隔离源码目录。仅这一路径需要填写，不得沿用旧7383287冒充新版本。输出目录已存在则停止，不清理覆盖。

```powershell
$ErrorActionPreference='Stop'
$Src='REPLACE_WITH_VERIFIED_NEW_ISOLATED_SRC'
$Py='C:\WorkSpace\VR4AM\.venv\Scripts\python.exe'
$Cfg='C:\WorkSpace\VR4AM\config\real-robot.local.yaml'
$Baseline='97a2c87c218fce7613029a3487dfb312a4a02ed4488ed556b99708fb0f4ce208'
Set-Location -LiteralPath $Src
if((Get-FileHash -LiteralPath $Cfg -Algorithm SHA256).Hash.ToLowerInvariant() -ne $Baseline){throw 'CONFIG_BASELINE_MISMATCH'}
$Out=Join-Path $Src 'artifacts\acceptance\lab-readonly-dynamics-001'
if(Test-Path -LiteralPath $Out){throw 'OUTPUT_ALREADY_EXISTS'}
$OldPythonPath=$env:PYTHONPATH
try {
  $env:PYTHONPATH=(Resolve-Path .\backend).Path
  & $Py -c "from pathlib import Path; import app.commissioning.readonly_dynamics as m; p=Path(m.__file__).resolve(); assert p == (Path.cwd()/'backend/app/commissioning/readonly_dynamics.py').resolve(), p; print(p)"
  if($LASTEXITCODE -ne 0){throw 'IMPORT_NOT_ISOLATED'}
  New-Item -ItemType Directory -Path $Out | Out-Null
  $Started=(Get-Date).ToString('o')
  & $Py .\scripts\real_robot_readonly_dynamics.py --config $Cfg --output (Join-Path $Out 'dynamics.json') 1> (Join-Path $Out 'stdout.txt') 2> (Join-Path $Out 'stderr.txt')
  $Code=$LASTEXITCODE
  [ordered]@{started=$Started; ended=(Get-Date).ToString('o'); exit_code=$Code; config_sha256_after=(Get-FileHash -LiteralPath $Cfg -Algorithm SHA256).Hash} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $Out 'execution.json') -Encoding UTF8
} finally {
  $env:PYTHONPATH=$OldPythonPath
}
```

不切mode、不启动或恢复机器人、不调用set_payload/set_gravity、stop、prepare、Home、平移、旋转、夹爪；不探测443、不切网络。若现场正在处置异常，先按现场安全规程处置，不为采集继续等待。

## 回传

只需 `dynamics.json`、`execution.json`、stdout/stderr四件和使用的完整提交号。退出2是采集失败或配置变化；其它非零码按失败保留原件。输出不见/无回显先查本次进程和目录，不重跑。本轮不要求新的长篇回执或运动日志。
