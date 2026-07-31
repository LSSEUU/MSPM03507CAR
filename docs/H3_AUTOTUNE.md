# H3 免烧录自动调参

固件身份为 `M6-runtime-20260801-r148-stop`，协议能力为
`m6-h3-tune-v1`（bit 31）。首次烧录本版本后，H3 参数只在 RAM 中
Stage/Apply；重启恢复编译默认值，接口没有 Save 命令，也不会写参数 Flash。

## 运行

在仓库根目录执行：

```powershell
$env:PYTHONPATH='python/src'
python scripts/h3_autotune.py run --port COM10
```

脚本预检固件、Safety、系统 IDLE、支架回水平、视觉流和 revision。球在
`0±3 mm`、速度低于 `5 mm/s` 连续稳定 1 秒后倒计时 3 秒。每轮完成后把球
放回中心即可；单批最多 8 轮。

运行时可从另一个终端读取状态或补充目视结果，这些命令只写会话文件，
不会打开串口：

```powershell
python scripts/h3_autotune.py status
python scripts/h3_autotune.py feedback --run 1 --result no_roll --note '目视未动'
python scripts/h3_autotune.py feedback --run 2 --result rolled
python scripts/h3_autotune.py abort --reason operator_abort
```

目视结果可为 `no_roll/rolled/overshoot/oscillation/normal/unsafe`。
`unsafe` 与 `abort` 会通知串口所有者发送原始 `S` 并撤销授权；脚本不会
自动清 fault。

## 恢复与离线分析

```powershell
python scripts/h3_autotune.py resume --port COM10 --session captures/h3_autotune/<会话>
python scripts/h3_autotune.py analyze --session captures/h3_autotune/<会话>
```

恢复前会核对固件身份、Safety、boot/profile revision 和 golden 回滚结果。
离线分析不打开串口。会话目录保存 golden/active profile、每轮原始 CSV、
事件、目视反馈、指标、候选决策、回滚证据与最终报告。

授权最多 8 次 START、30 分钟，单轮最长 12 秒。运行中主机心跳超过
300 ms、断连、急停或真实故障时，固件停止 H3、撤销剩余额度，并在 RAM
中恢复授权开始时锁存的 golden。正常批次 REVOKE 保留验证后的最佳 active
参数，仍不会永久保存。
