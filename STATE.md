# MSPM03507CAR 当前状态

更新时间：2026-07-27

本文记录当前可开发基线、不可破坏的控制边界、已完成验证和下一步。历史过程见 `docs/`。

## 1. 当前基线

- 平台为 MSPM0G3507-64P 两轮差速小车，32 MHz，Keil ARM Compiler 6.22；入口为 `Source/main.c`，冻结板级配置修订号 `20260727`。
- `m6link.Session` 是 UART0/DAPLink 的唯一主机串口所有者。网页仪表盘、控制、参数、录制和回放均复用该会话。
- M6 V2 冻结 registry 已加入 `MANUAL_DRIVE` bit 25、GET/SET RPC 和状态事件；旧固件仍可连接，但不得启用网页遥控。
- 本地遥测会话保存在启动目录的 `recordings/`，该目录不进入 Git。

## 2. 已实现能力

- 实时仪表盘同时支持滚动总览和 1920×1080 固定驾驶舱，显示电机、轮速、灰度、循迹、陀螺仪、链路健康和故障原因。
- 支持两阶段动作控制、持续循迹、急停、清故障、运行参数暂存/应用/保存，以及车辆重新上电后的自动重连。
- 支持流式遥测录制、动作前 5 秒/后 2 秒联动录制、标记、会话库、ZIP 下载和同页离线回放。会话以 UTF-8 JSON/JSONL、schema、索引和摘要保存，适合 AI 分析。
- 支持网页 WASD 实时遥控：W/S 前后、A/D 原地转向，组合键为弧线；最高速度 50–400 mm/s，转向力度 10–100%。按键变化立即发送，按住每 100 ms 续租。
- `AppManualDrive` 统一网页、红外和未来视觉控制的左右轮目标与停车逻辑。优先级为急停 > HOST_RPC > UART1 视觉 > PB11 红外。
- 网页指令租约为 300 ms，由 20 ms 任务检查；松键、失焦、断线或租约到期执行普通停车，不锁存 Safety 故障。500 ms Safety deadline 仅作为调度异常的最后保护。
- UART1 已在 PB6/TX、PB7/RX 配置为 115200 8N1，提供独立 RX/TX 环形缓冲、IRQ 和溢出统计。当前不解析摄像头坐标，也不接受视觉运动指令。

## 3. 安全与控制边界

- `System/safety.c` 仍是电机输出的最终门控；故障、急停、电机/编码器保护始终高于手动驾驶来源。
- 裸 `S` 或空格急停会立即制动并锁存故障；普通松键和手动驾驶租约超时不得产生 Safety 故障。
- 手动驾驶期间禁止新动作 Arm、参数 Apply/Save；第一个非零 HOST_RPC 指令可以正常取消正在执行的动作并接管，不走急停路径。
- 回放页面保持车辆控制组件可见但只读；进入回放、车辆重启或链路断开后不得自动恢复 WASD 控制。
- UART1 接口可向模块提供 5 V 电源，但 PB6/PB7 信号必须为 3.3 V TTL 并可靠共地，板上没有电平转换。

## 4. 有效验证

- 2026-07-27：用户完成当前版本实车验证，WASD 遥控功能确认成功。
- Python/C/协议自动化测试：`164 passed, 14 subtests passed`。
- 冻结 M6 V2 registry、schema、生成向量及 Python/C 常量一致性检查通过。
- Keil Rebuild：0 error、0 warning；`Code=63324, RO-data=9748, RW-data=12, ZI-data=29580`。
- 固件普通镜像最高字节 `0x00011D7F`，低于 `0x0001F800` 参数保留区。
- 手动驾驶核心覆盖启动、续租、来源优先级、普通停车、租约到期和故障；原 PB11 红外回归测试通过。
- 仪表盘接口覆盖鉴权、单网页控制权、输入序号、轮速混合、300 ms 租约、心跳去重、录制事件和重连不重放。

## 5. 下一步

1. 确定摄像头/视觉模块型号、输出数据和定位方案，再为 UART1 增加带版本、长度、序号和校验的解析协议。
2. 将视觉识别结果先转换为统一左右轮目标，通过 `AppManualDrive_SubmitVision()` 接入，不新增直接 PWM 路径。
3. 在固定 1920×1080 显示器上继续检查驾驶舱排版，并根据实车调参频率优化图表窗口和信号组合。
4. 为视觉接口补充目标丢失、坐标超界、数据超时和来源切换的实车验收。

## 6. 常用入口

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
m6-tune --port COM10 dashboard
```

- Keil 工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- 接线说明：`docs/接线说明.md`
- V2 合同：`Tests/vectors/m6_v2_wire_v1.json`
