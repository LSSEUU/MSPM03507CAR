# H3 冲刺版球位发送端

`demo_uart` 使用现有 YOLO26 模型检测钢球，将球心映射到摆杆物理坐标，并在每个检测周期通过 UART 发送一帧固定 9 字节测量。摄像头只负责测量；控制闭环仍由主控负责。

## 坐标和标定

- 原点为摆杆中心 `O=0 mm`。
- 沿摆杆指向活动端为正方向。
- 当前安装下像素坐标增大对应正方向；安装变化后必须重新确认方向和标定。
- `main.py` 中 `CALIBRATION_MODE="multi"` 默认复用现有 25 点实测表。
- 改为 `CALIBRATION_MODE="three"` 时，只使用 `(127,-50 mm)`、`(218,0 mm)`、`(331,+50 mm)` 做分段线性标定。实机位置变化后应更新这三个像素值。

有效控制范围默认是 `[-115,+115] mm`。三点模式在标定区间外沿最近线段外推，随后仍执行该物理范围检查。

## 测量有效性

最高置信度检测框必须同时满足：

- 置信度不低于 `0.50`；
- 球心到标定轴线的垂距不超过 `20 px`；
- 框宽和框高都在 `6..64 px`；
- 宽高比在 `0.5..2.0`；
- 映射位置在 `-115..+115 mm`。

丢检或任一条件不满足时，本周期仍发送帧，但 `valid=0` 且位置、速度全部为零。异常样本不会更新毫米域位置/速度滤波器。

## UART 帧

串口保持 `/dev/ttyS0, 115200`，每次写入恰好 9 字节，不带换行：

```text
偏移  大小  字段          说明
0     2     A5 5A         固定帧头
2     1     seq:u8        每周期递增，255 后回到 0
3     1     flags:u8      bit0 valid；其余位固定为 0
4     2     x_mm_x10:i16  小端，0.1 mm/LSB
6     2     vx_mm_s:i16   小端，mm/s
8     1     crc8          覆盖偏移 0..7
```

CRC 固定为 CRC-8/ATM：多项式 `0x07`、初值 `0x00`、输入/输出均不反射、结果不异或（`xorout=0x00`）。标准检查串 `123456789` 的结果为 `0xF4`。

冻结的 MCU/Python 对照向量：

```text
A5 5A 2A 01 F4 01 EC FF 73  # seq=42, valid, x=+50.0 mm, vx=-20 mm/s
A5 5A 2B 00 00 00 00 00 44  # seq=43, invalid
```

## 性能默认值

YOLO26 模型、屏幕显示和 `LOW_LATENCY_MODE=True` 保持启用。以下高开销功能默认关闭：

```python
DEBUG_LOG = False
USE_RTSP = False
USE_JPEG = False
USE_WEBRTC = False
```

## 串口回放

回放 CSV 必须包含以下表头；`delay_ms` 表示发送该行前相对上一帧的等待时间：

```csv
delay_ms,valid,x_mm,vx_mm_s
0,1,0,0
40,1,50,-20
40,1,-50,20
40,0,0,0
```

先检查编码结果，不打开串口：

```powershell
python demo_uart/replay_serial.py scenario.csv --dry-run --seq 0
```

发送一次或循环发送：

```powershell
python demo_uart/replay_serial.py scenario.csv --port COM5
python demo_uart/replay_serial.py scenario.csv --port COM5 --loop
```

实时发送使用仓库已经声明的 `pyserial`，没有新增依赖。
