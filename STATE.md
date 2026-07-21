# MSPM03507CAR 当前状态

更新时间：2026-07-22

本文是仓库当前实现状态的唯一事实来源。设计意图见 `docs/superpowers/specs/`，台架证据见 `docs/2026-07-21-M0-M1验证记录.md`，历史状态通过 Git 追溯。

## 1. 当前结论

- M0“工程基线”：完成。
- M1“外设诊断”：完成。
- M2“调度与安全”：软件实现完成；15 项主机回归和 Keil ARM Compiler 6.22 全量构建通过。尚未下载本版固件执行急停、传感器断开、动作超时和 WWDT 复位的实车故障注入，因此当前结论不是硬件验收通过。
- 当前入口：`Source/main.c`；系统按 1 ms 时基调度 5/10/20/100 ms 任务，电机输出只能由 `System/safety.c` 仲裁。没有动作时保持两路电机制动。
- 下一节点：M3“电机闭环”。尚未实现速度 PI、自动行驶、航向或循迹控制。
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
| `Source/main.c` | 唯一启动入口、周期任务分派和硬件看门狗健康门控 |
| `Source/ti_msp_dl_config.c/.h` | 唯一冻结板级配置，修订号 `20260722` |
| `Source/board.c/.h` | 1 ms SysTick 下的短延时与标准输出重定向 |
| `Config/vehicle_config.h` | 任务周期、传感器超时、动作超时和诊断动作参数 |
| `Hardware/` | 电机、编码器、JY61P、异步灰度 ADC、分页面 OLED、LED、中断式 UART0 和 WWDT0 驱动 |
| `System/system_time.c` / `scheduler.c` | 1 ms 单调时间、ISR 任务释放、漏期统计和关键任务健康窗口 |
| `System/system_state.c` / `safety.c` | `BOOT/SELF_TEST/IDLE/RUN/FAULT` 状态机、故障锁存和唯一 PWM 安全仲裁 |
| `System/diagnostics.c` | 非阻塞单字符命令、启动自检和人工电机动作状态机 |
| `System/diagnostics_health.c/.h` | 与硬件无关的状态模型 |
| `Tests/` | 板级不变量、Keil 可移植性、HEX、诊断状态和 M2 调度/安全模型回归 |

代码清理结果：未参与当前目标的旧 `Source/empty.c` 和 `Control/` 演示实现已删除，Keil include path 也不再包含 `Control`。灰度公开接口仅保留 `Init/Scan/GetSnapshot`；电机、编码器和 JY61P 不再公开无调用方的可变全局状态。Git 历史仍可追溯旧演示代码。

启动与运行流程：

1. 初始化冻结板级配置并立即制动电机；初始化调度、状态和安全模型。
2. 在周期调度启动前初始化 UART、编码器、灰度、JY61P、LED、OLED 和 WWDT0，避免初始化耗时被误判为任务漏期。
3. 启动 1 ms SysTick，进入 `SELF_TEST`；约 750 ms 后输出自检报告并进入 `IDLE`。该等待由状态和时间戳实现，不阻塞主循环。
4. SysTick ISR 只更新时间并释放任务。主循环执行 5 ms 安全检查、10 ms 传感器服务、20 ms 预留运动状态机和 100 ms OLED/诊断任务。
5. WWDT0 周期约 31 ms；每 20 ms 只有当 5/10/20 ms 三个关键任务均完成且没有漏期时才喂狗。调试器暂停 CPU 时 WWDT 停止。

急停字符 `S`/空格由 UART0 RX ISR 截获并立即调用安全层制动和锁存 `FAULT`。传感器超时和动作超时在 5 ms 任务中检查。当前陀螺仪超时为 250 ms、灰度超时为 300 ms；只对动作声明为必需的传感器生效。动作必须声明截止时间，超时后制动并锁存故障。`x` 只能把 `FAULT` 清到 `IDLE`，不会恢复先前动作。

电机仍只有命令 `m` 会进入人工测试，且必须在 10 s 内再次输入大写 `A` 解锁。测试现在是非阻塞状态机：A 正/反、B 正/反各 500 ms，段间制动 150 ms，PWM 250，总动作看门狗 3500 ms；所有 PWM 经过 `Safety_RequestMotor()`。

## 4. UART0 诊断命令

| 命令 | 作用 |
|---|---|
| `h/H` | 帮助 |
| `s` | 全部状态报告 |
| `d/D` | 编码器、JY61P、灰度和通信原始数据 |
| `e/E` | 编码器累计计数和边沿 |
| `y/Y` | JY61P 角度、帧计数和最近字节 |
| `q/Q` | 请求一次非阻塞八路灰度 ADC 扫描，完成后输出 min/max/span/错误 |
| `o/O` | 请求 OLED 棋盘格分页面刷新，再用 `p/f` 人工确认 |
| `p/f` | 记录/清除 OLED 目视通过状态 |
| `l/L` | LED 能力信息 |
| `c/C` | UART0 收发统计 |
| `m/M` | 电机测试；仍需大写 `A` 二次解锁 |
| `A` | 在 `m` 解锁窗口内启动非阻塞电机测试 |
| `v/V` | 系统状态、故障位、动作截止时间、PWM 和调度统计 |
| `x/X` | 清除锁存故障并仅返回 `IDLE` |
| `S` / 空格 | UART RX 中断级全局急停并进入 `FAULT` |

## 5. 功能与验证状态

| 功能 | 当前证据 | 状态 |
|---|---|---|
| Keil 工程/HEX | ARM Compiler 6.22 全量重建 0 error、0 warning；AXF/HEX 非空 | 已编译，M0 完成 |
| 下载链路 | 新 DAPLink 未接 RST；当前复核 `Erase Done / Programming Done / Verify OK / Application running` | 已上板 |
| UART0 | 独立 CH340 COM5，115200 双向命令和输出正常 | 已上板 |
| 电机与急停 | 历史台架完成 A/B 正反转，dEdgeA=1543、dEdgeB=1402，`S` 急停有效；本次清理未重复运动测试 | 已上板 |
| 编码器 | 手转 A count=1076/edge=1084，B count=-1041/edge=1041；电机联测两路边沿均增长 | 已上板 |
| JY61P | 最终固件有效帧 41→89，yaw/pitch/roll 有效，checksum=0、frame=0 | 已上板 |
| 灰度 ADC | 最终固件 `[3204,3200,3246,3249,68,73,74,74]`，span=3181、errors=0；此前连续四轮 span=3182–3189 | 已上板；标定/循迹属 M5 |
| OLED | 本次 I²C ACK 正常；此前棋盘格已目视确认 | 已上板 |
| LED | 硬件无 MCU 可控状态 LED | 预期 `DEGRADED` |
| HC-04 | UART0 接口已验证，模块无线透传未单独验收 | 未单独验证 |
| M2 状态机/安全模型 | 主机 C 回归覆盖合法状态迁移、调度漏期、动作超时、陀螺仪超时、急停锁存、PWM 否决和仅回到 IDLE 的清故障 | 主机测试通过 |
| M2 目标构建 | ARM Compiler 6.22：`Code=12786, RO-data=1546, RW-data=4, ZI-data=18988`，0 error、0 warning；AXF 229008 字节、HEX 40400 字节 | 已编译，未上板 |

最新软件验证：15 项主机回归全部通过；Keil V6.22 全量构建 0 error、0 warning，尺寸见上表。M0/M1 的下载与台架证据仍来自上一版固件；M2 改动尚未下载，不能据此前记录声称 M2 已上板或实车通过。

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

## 7. 已知风险与 M3 边界

1. 灰度模块更换或供电变化后必须重新测量 OUT；PA25 不是 5 V 容忍输入。黑白标定、阈值、位置和失线判定属于 M5，当前异步扫描只提供有时间戳的原始 ADC 快照。
2. JY61P 已有有效帧时间戳和按动作声明的 250 ms 超时保护；校零、角度归一化和错误率窗口属于 M4。
3. UART0 已改为 RX/TX 中断环形缓冲；缓冲满会丢弃数据并计数，不阻塞关键周期。OLED 仍是软件 I²C，但运行期每 100 ms 最多刷新一页；其总线恢复和时钟拉伸超时仍未实现。
4. M2 的故障延迟上界来自源码结构、主机模型和约 31 ms WWDT 配置，尚缺示波器/逻辑分析仪或串口时间戳的上板故障注入证据。
5. 编码器堵转/物理不合理跳变保护依赖 M3 的速度估计和目标速度，目前尚未实现。
6. PA18 与载板蜂鸣器网络复用且是默认 BSL 输入；应用和板级配置均不得占用 PA18/PA19/PA20。
