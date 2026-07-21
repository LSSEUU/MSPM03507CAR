# MSPM03507CAR 当前状态

更新时间：2026-07-22

本文是仓库当前实现状态的唯一事实来源。设计意图见 `docs/superpowers/specs/`，台架证据见 `docs/2026-07-21-M0-M1验证记录.md`，历史状态通过 Git 追溯。

## 1. 当前结论

- M0“工程基线”：完成。
- M1“外设诊断”：完成。
- 当前入口：`Source/main.c`；上电初始化后保持两路电机制动，只运行 UART 诊断循环。
- 下一节点：M2（确定性调度、双轮速度闭环、灰度黑白标定与循迹位置算法）。尚未实现自动行驶。
- JY61P 已重新安装：TX 接 PA22，RX 不接。2026-07-22 当前固件被动复核持续收到有效角度帧且无校验/帧错误。
- 灰度模块使用 5 V 供电、与主控共地；当前实物 OUT 已确认不超过 3.3 V。仓库通过 PA25/A0_2 读取模拟量，不使用旧数字位图算法。

## 2. 固定硬件与接线

主控是立创·天猛星 MSPM0G3507-64P，固件系统时钟 32 MHz。当前开发板原理图是 `docs/立创·天猛星MSPM0G3507开发板原理图.pdf`。

| 功能 | MCU 引脚 | 当前配置/接线 |
|---|---|---|
| UART0 诊断 | PA10 TX / PA11 RX | 115200 8N1；独立 USB-UART 当前为 COM5；HC-04 尚未单独验收 |
| JY61P | PA22 UART2_RX | 9600；JY61P TX→PA22，RX 不接 |
| PA21 / VREF− | PA21 | 无上下拉高阻；不启用 UART2_TX，避免经 C44 耦合灰度 AD0 |
| 电机 A PWM/方向 | PB3 / PA14 / PA13 | TIMA1 CCP1，4 kHz，比较值 0–999；AIN1/AIN2 |
| 电机 B PWM/方向 | PB2 / PA16 / PA17 | TIMA1 CCP0，4 kHz，比较值 0–999；BIN1/BIN2 |
| 编码器 A | PA8 / PA9 | E1A/E1B，双相 GPIO 上升沿中断 |
| 编码器 B | PB9 / PB8 | E2A/E2B，双相 GPIO 上升沿中断 |
| 灰度地址 | PA23 / PA24 / PA26 | AD0/AD1/AD2；PA23 高电平通过释放引脚和内部上拉形成 |
| 灰度 OUT | PA25 / ADC0 A0_2 | ADC0 MEM0，12 bit，VDDA 参考；每通道 8 次平均，扫描顺序为偶数通道后奇数通道 |
| 灰度供电 | 5 V / GND | 厂家要求 5–12 V；当前实物 OUT 已测≤3.3 V，更换模块后必须重测 |
| OLED | PA28 SDA / PA31 SCL | 软件 I²C，探测 0x3C/0x3D |
| 状态 LED | 无 | 载板只有电源指示灯，诊断按能力缺失报告 `DEGRADED` |
| BSL/SWD 保留 | PA18 / PA19 / PA20 | 应用不配置；PA18 为默认 BSL 调用输入，PA19/PA20 为 SWDIO/SWCLK |

PA23/VREF+ 与 PA21/VREF− 在天猛星-64P 上通过 C44（1 µF）耦合，因此 PA21 必须高阻，PA23 不得以普通推挽方式快速拉高。完整接线见 `docs/接线说明.md`。

## 3. 当前软件结构

| 路径 | 职责 |
|---|---|
| `Source/main.c` | 唯一启动入口和 5 ms 诊断轮询 |
| `Source/ti_msp_dl_config.c/.h` | 唯一冻结板级配置，修订号 `20260722` |
| `Source/board.c/.h` | SysTick 延时与标准输出重定向 |
| `Hardware/` | 电机、编码器、JY61P、灰度 ADC、OLED、LED 能力和 UART0 诊断驱动 |
| `System/diagnostics.c` | 安全自检、单字符命令和人工台架测试 |
| `System/diagnostics_health.c/.h` | 与硬件无关的状态模型 |
| `Tests/` | 板级不变量、Keil 可移植性、HEX 和状态模型回归 |

代码清理结果：未参与当前目标的旧 `Source/empty.c` 和 `Control/` 演示实现已删除，Keil include path 也不再包含 `Control`。灰度公开接口仅保留 `Init/Scan/GetSnapshot`；电机、编码器和 JY61P 不再公开无调用方的可变全局状态。Git 历史仍可追溯旧演示代码。

启动流程：

1. 初始化冻结板级配置和 SysTick。
2. 初始化电机并立即制动，再初始化编码器、灰度、JY61P、LED 能力和 OLED。
3. 等待约 750 ms 收集串口传感器数据，输出自检报告和命令帮助。
4. 主循环轮询补充接收 JY61P 字节、处理诊断命令并延时 5 ms。

电机只有命令 `m` 会进入测试，且必须在 10 s 内再次输入大写 `A` 解锁。测试按 A 正/反、B 正/反各 500 ms、PWM 250 执行；每段监听 `S` 或空格急停。

## 4. UART0 诊断命令

| 命令 | 作用 |
|---|---|
| `h/H` | 帮助 |
| `s` | 全部状态报告 |
| `d/D` | 编码器、JY61P、灰度和通信原始数据 |
| `e/E` | 编码器累计计数和边沿 |
| `y/Y` | JY61P 角度、帧计数和最近字节 |
| `i/I` | 临时探测 PA21/PA22 跳变并恢复 UART2；仅用于接线诊断 |
| `q/Q` | 执行一次八路灰度 ADC 扫描并输出 min/max/span/错误 |
| `o/O` | 显示 OLED 棋盘格，等待 `p/f` 人工确认 |
| `p/f` | 记录/清除 OLED 目视通过状态 |
| `l/L` | LED 能力信息 |
| `c/C` | UART0 收发统计 |
| `m/M` | 电机测试；仍需大写 `A` 二次解锁 |
| `S` / 空格 | 电机急停 |

## 5. 功能与验证状态

| 功能 | 当前证据 | 状态 |
|---|---|---|
| Keil 工程/HEX | ARM Compiler 6.22 全量重建 0 error、0 warning；AXF/HEX 非空 | 已编译，M0 完成 |
| 下载链路 | 新 DAPLink 未接 RST；当前复核 `Erase Done / Programming Done / Verify OK / Application running` | 已上板 |
| UART0 | 独立 CH340 COM5，115200 双向命令和输出正常 | 已上板 |
| 电机与急停 | 历史台架完成 A/B 正反转，dEdgeA=1543、dEdgeB=1402，`S` 急停有效；本次清理未重复运动测试 | 已上板 |
| 编码器 | 手转 A count=1076/edge=1084，B count=-1041/edge=1041；电机联测两路边沿均增长 | 已上板 |
| JY61P | 最终固件有效帧 41→89，yaw/pitch/roll 有效，checksum=0、frame=0 | 已上板 |
| 灰度 ADC | 最终固件 `[3204,3200,3246,3249,68,73,74,74]`，span=3181、errors=0；此前连续四轮 span=3182–3189 | 已上板；标定/循迹属 M2 |
| OLED | 本次 I²C ACK 正常；此前棋盘格已目视确认 | 已上板 |
| LED | 硬件无 MCU 可控状态 LED | 预期 `DEGRADED` |
| HC-04 | UART0 接口已验证，模块无线透传未单独验收 | 未单独验证 |

本次代码整理的最新验证：13 项主机回归全部通过；Keil V6.22 全量构建 `Code=10514, RO-data=1602, RW-data=4, ZI-data=17956`，0 error、0 warning，AXF 190272 字节、HEX 34161 字节；下载与上述 JY61P/灰度被动测试通过。没有触发电机测试，开机报告中的 MOTOR/ENCODER `DEGRADED` 表示本次上电尚未执行人工运动验收，不是回归故障。

## 6. 构建与仓库规则

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
```

- Keil 工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，ARM Compiler 6.22。
- `Source/ti_msp_dl_config.c/.h` 是手工维护且由测试保护的冻结配置；仓库不保留 `.syscfg`，避免错误的 LaunchPad 配置覆盖当前载板。
- 新克隆在仓库根目录运行 `scripts/setup_git_hooks.ps1`，启用要求同步 `STATE.md` 的 pre-commit hook。
- `keil/Objects`、构建日志和临时下载日志不纳入版本控制。
- 出现问题时先定位原因，只有高可能性时才修复；同一问题两次尝试仍失败就停止并总结，不盲目重试。

## 7. 已知风险与 M2 边界

1. 灰度模块更换或供电变化后必须重新测量 OUT；PA25 不是 5 V 容忍输入。M2 还需黑白标定、阈值、位置和失线判定。
2. JY61P M2 控制链路需要毫秒时间戳、超时停车、校零、错误率窗口和掉线保护；当前 M1 只证明接收与解析可用。
3. UART0 输出和 OLED 软件 I²C 都是阻塞式；接入确定性控制周期前必须隔离或限流。
4. 当前只有人工诊断级急停，尚无全局调度优先级、堵转保护、任务超期和看门狗闭环。
5. PA18 与载板蜂鸣器网络复用且是默认 BSL 输入；应用和板级配置均不得占用 PA18/PA19/PA20。
6. OLED 软件 I²C 尚无总线恢复和时钟拉伸超时。
