# H3 PID 串口示波器

该界面用于手拨球测试 0 点闭环。固件目标固定为 `0 mm`，不会执行原来的
`+50 → -50 mm` 比赛轨迹；参数只在 RAM 中 Stage/Apply，不写 Flash。

## 首次准备

1. 如果 `h3_autotune.py run` 仍在 COM10 上运行，先回到它的终端按
   `Ctrl+C`，等待正常停止并释放串口。空闲时不要用 `abort` 代替
   `Ctrl+C`，因为 `abort` 会发送原始急停 `S`。
2. 首次烧录 r86：
   `keil/Artifacts/MSPM03507CAR-r86-h3-pid-scope.hex`。
3. 核对启动身份为 `M6-runtime-20260801-r148-stop`。

## 启动界面

在仓库根目录执行：

```powershell
$env:PYTHONPATH='python/src'
python scripts/h3_pid_scope.py --port COM10
```

浏览器会自动打开 <http://127.0.0.1:8766/>。终端必须保持运行；同一时间
不要再运行 autotune、dashboard 或其他会打开 COM10 的程序。

## 基线测试

1. 确认页面显示 `SERIAL ONLINE`、`FAULT 0`，视觉年龄持续较小。
2. 保持 `Kp=105`、`Kd=36`，把球放在中心。
3. 点击“开始 0 点保持”。
4. 向一侧轻拨约 `10 mm` 后立即松手；界面会记录峰值、反向过冲、稳定
   时间、过零次数、最大倾角和遥测 gap。
5. 等待本轮 8 秒自动停止并回水平，再做相反方向。

四张波形分别显示：

- A：位置、目标 0 mm 与 `±3 mm` 稳定带。
- B：球速度与 `±5 mm/s` 稳定带。
- C：PD 原始请求、实际倾角、强制倾角；CENTER 模式的强制倾角应为 0。
- D：执行器 current/target count；两条线的差值表示执行器跟随滞后。

## 手动修改 PID

必须先“正常停止”并等待状态回到 `idle`。一次只修改一项：

- Kp 范围 `20..300`，点击“单独应用 Kp”。
- Kd 范围 `0..120`，点击“单独应用 Kd”。

Apply 后 profile revision 会增加，但固件身份不变，说明免烧录 RAM 参数已
生效。建议 Kp 每次变化约 5，Kd 每次变化约 4：多次穿越 0 且衰减慢时先
增加 Kd；无明显过冲但回零慢时再小幅增加 Kp。

“暂停画面”只冻结显示，不停止串口记录；CSV 和完整原始记录仍持续写入
`captures/h3_pid_scope/<时间>/`。按终端 `Ctrl+C` 会先请求正常停止、等待
回水平、撤销授权，再关闭串口。界面绝不自动清 fault。
