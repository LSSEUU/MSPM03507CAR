# H3 混合识别球位发送端

`demo_uart` v1.3.0 面向当前安装的 MaixCAM Pro。程序使用 YOLO26、模板跟踪和亮度自适应识别钢球，将球心换算为摆杆物理坐标，并在每个识别周期通过 UART 发送固定 9 字节测量。摄像头只负责测量，控制闭环仍由 MSPM0 主控负责。

## 混合识别顺序

1. `Y`：在原始画面运行 YOLO26，正常情况下只执行这一阶段。
2. `T`：YOLO 瞬时漏检时，在预测位置附近运行归一化模板匹配。
3. `E`：模板失败时，根据画面亮度执行 gamma 或 CLAHE 处理，再用 YOLO 复检。
4. `G`：连续丢失期间，每隔三帧运行一次全画面模板重定位，并用钢球内部纹理排除端帽等误匹配。

跟踪器先完成各来源对应的置信度和时序判断，再由当前项目的候选校验器检查框尺寸、宽高比、标定轴线误差和物理范围。未通过校验的候选不会更新跟踪状态。

屏幕框颜色和状态栏中的 `Y/T/E/G` 表示本帧来源。默认关闭额外调试绘制和所有网络流，避免增加识别到 UART 的延迟。

## 坐标和标定

- 原点为摆杆中心 `O=0 mm`，沿摆杆指向活动端为正方向。
- 当前活动端位于画面左侧，因此像素 X 减小时物理坐标增大。
- 默认使用 `CALIBRATION_MODE="three"`。
- 当前三点实测值为 `(142,+50 mm)`、`(247,0 mm)`、`(345,-50 mm)`。
- `CALIBRATION_MODE="multi"` 仅保留给旧摄像头安装姿态。
- 有效控制范围为 `[-115,+115] mm`；轴线误差不超过 `20 px`，检测框宽高均为 `6..64 px`，宽高比为 `0.5..2.0`。

混合识别没有引入参考程序中的 `0..25 cm` 示例标定、二维像素滤波或像素速度。发送位置仍为毫米，速度仍由毫米域 alpha-beta 滤波器输出为 `mm/s`。

## UART 帧

串口保持 `/dev/ttyS0, 115200`，每个识别周期写入恰好 9 字节且不带换行：

```text
偏移  大小  字段          说明
0     2     A5 5A         固定帧头
2     1     seq:u8        每周期递增，255 后回到 0
3     1     flags:u8      bit0 valid；其余位固定为 0
4     2     x_mm_x10:i16  小端，0.1 mm/LSB
6     2     vx_mm_s:i16   小端，mm/s
8     1     crc8          覆盖偏移 0..7
```

CRC 使用 CRC-8/ATM：多项式 `0x07`、初值 `0x00`、输入/输出均不反射、`xorout=0x00`。丢检、候选不合格或数值越界时立即发送 `valid=0`，位置和速度固定为零。

冻结的 MCU/Python 对照向量：

```text
A5 5A 2A 01 F4 01 EC FF 73  # seq=42, valid, x=+50.0 mm, vx=-20 mm/s
A5 5A 2B 00 00 00 00 00 44  # seq=43, invalid
```

## 运行与性能默认值

在 MaixVision 中关闭旧应用后，打开本目录并运行全部文件。启动日志应包含：

```text
Hybrid detector v1.3: 480x160, model=models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud
```

当前 MaixCAM 模型与混合参考程序中的模型 SHA-256 一致，因此 v1.3.0 继续使用原模型。默认设置为：

```python
DEBUG_LOG = False
DRAW_DEBUG = False
LOW_LATENCY_MODE = True
USE_RTSP = False
USE_JPEG = False
USE_WEBRTC = False
```

本版本仅打包当前 MaixCAM/Pro 模型，不包含 MaixCAM2 的 AXModel。

## 串口回放

回放 CSV 表头为：

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

## 实机验收重点

- 已知三点标定位置与正方向保持不变。
- 稳定发送频率不低于 `20 Hz`，主控 UART 无 CRC、溢出或硬件错误。
- 遮挡后 `150 ms` 内进入无效，恢复后 `200 ms` 内重新连续有效。
- 正常画面以 `Y` 为主，短暂漏检时允许出现 `T/E/G` 恢复结果。
