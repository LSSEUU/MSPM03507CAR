# r35 循迹与固定赛道基线

本检查点用于在后续 H 题第二题速度调试中随时回退。它不是最终验收，也没有把参数保存到 MCU 参数 Flash。

## 固件与验证

- 固件：`M6-runtime-20260730-r35`
- 主机回归：169 项通过
- Keil Rebuild：0 error / 0 warning
- 已烧录并通过实车定向段及 160/240 mm/s 整圈

关键录制：

- `recordings/2026-07-30/20260730T161433-828Z-r35-exit-cd-160-240-b893e8`
- `recordings/2026-07-30/20260730T161555-339Z-r35-full-lap-160-240-a8c4a9`

r35 将固定半圆到直线的交接点从 175° 提前到 170°。160/240 定向测试中，首弯超过计划出口航向由 r34 的 3.04° 降为 0°；无丢线、反转、故障、轮速饱和或目标钳制。

## RAM 参数基线

以下值只通过运行时 Apply 使用，尚未 Save：

```text
line.speed_min = 160
line.speed_max = 240
line.position_alpha = 0.35
line.derivative_alpha = 0.25
line.kp = 55
line.kd = 10
line.correction_limit = 60
line.correction_slew = 300
line.curve_gain = 2.0
line.lost_search_speed = 105
heading.straight.kp = 3
heading.straight.kd = 0.88
wheel.filter_alpha = 0.1
```

## 已拒绝的速度组合

`line.speed_min = 180`、`line.speed_max = 240` 在第二弯产生持续最外侧占用并被保护取消，因此第二题继续调速时先保持弯道 160，只提高或设计直线速度规划。
