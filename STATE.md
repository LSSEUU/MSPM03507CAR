# MSPM03507CAR 当前状态

更新时间：2026-07-26

本文是当前实现状态的唯一事实来源，只保留继续工程工作所需的事实、约束和未完成项。详细验证过程和 M6 工程记录在 `docs/` 的日期记录中，历史状态通过 Git 追溯；它们都不能覆盖本文的当前结论。

## 1. 新对话快速启动

- 项目：立创·天猛星 MSPM0G3507-64P 两轮差速小车固件，系统时钟 32 MHz，Keil ARM Compiler 6.22。
- 当前入口：`Source/main.c`；`Source/ti_msp_dl_config.c/.h` 是受回归测试保护的唯一冻结板级配置，修订号 `20260725`，不得用旧 `.syscfg` 覆盖。
- 里程碑：M0–M5 已完成；PB11 红外遥控阶段一完成，阶段二已完成全部架空路径和 100 mm/s 落地前进/制动，用户自行测试按键功能无异常，正式落地后退/左右转延后到算法定型后；M6 软件基础设施、架空轮速矩阵和 100 mm/s 落地直行/制动/航向筛选已取证，巡线分级、220 mm/s×5 圈和掉电保持验收尚未完成；M7 尚未开始。
- 当前运行镜像：`M6-runtime-20260726-r12`，已在误选根目录旧 M5 工程后重新手动下载精确 `.worktrees\m6-runtime-tuning` 正式工程，并经 COM10 确认 schema/revision 1、source `defaults`、generation 0；帮助中不存在 `TEST ONLY`/`FAULT INJECTION`。短按复位报告 raw `0x0C external_boot`、boot 5、coarse 恢复有效；Safety `IDLE`，故障/PWM/目标、四级任务漏期、UART/遥测丢包和格式错误均为 0。
- 当前验证基线：V2 `m6-v2-wire-v1` registry revision 1 与 C/Python golden vectors 已按 registry/schema canonical SHA-256 `23fbd056db28c1fedc7987a34d0893594fddb417f34058851667ffc2d5dca0a2` / `95191cda4ea5ed0691126af1fa027ff37b3dd6f920ede9c099cad0b16fce7ede` 冻结，104 项主机测试通过。该切片只改文档、JSON、测试和检查脚本，未改目标源码/工程，故最近目标构建仍是待上板 `M6-runtime-20260726-r13` 的 Keil ARMCLANG 6.22 Rebuild 0 error、0 warning，`Code=43868, RO-data=9612, RW-data=8, ZI-data=26184`，最高普通镜像字节 `0x0000D0EF`，低于参数区 `0x0001F800`；专用 256 B `UNINIT` 区仍位于 `0x20207F00`。当前 MCU 仍运行已核对的正式 r12，r13 尚未下载或上板；硬件已验证的 power、短/长 NRST、受控 SYSRST 和 WWDT 分类/保持边界仍属于 r12/r12-fi 证据。真实 HardFault 异常入口、r13 日志负载和长期运行仍待上板验证。
- PB11 阶段二进行中：架空的三档、100/220 mm/s 四方向、251 ms 失联制动、`OK`/`0`、直行/±90°/巡线门控均通过；落地以 100 mm/s 完成遥控前进、航向保持和松键制动，用户确认车辆前进且停车正常，并自行确认其余按键功能无异常。正式落地后退/左右转仍保留为 PB11 最终验收项，但按用户决定不再阻塞使用串口急停与物理中止保护的 M6 算法调试。
- 开始任何工作前先执行 `git status --short`，保留所有不属于当前任务的改动。需要细节时按第 9 节只读取对应文档，不要一次加载全部验证记录或 PDF。

## 2. 当前待办与停止条件

### P0：M6 复位可观测性、V2 通信与高速运行

当前优先目标依次为高速直线、平滑左右弧线、高速陀螺仪角度/角速度控制和系统长期无复位。后续顺序固定为：

1. `r12` 的 power、短/长 NRST 与 `r12-fi` 的受控 SYSRST/WWDT 矩阵均已取证；正式 r12 已恢复，并以精确身份及帮助中无 `TEST ONLY` 双重核对测试入口移除。不得延长、禁用或旁路 WWDT，也不得把合成 scratch 写成真实 HardFault 异常入口通过。
2. 现有 UART 热路径诊断与 R2 清理已在待上板 r13 源码完成，精确 V2 wire registry、消息字段和 C/Python golden vectors 也已冻结；下一步实现 R3 C/Python codec、分帧、CRC、序号和单帧优先 TX，再接优先队列与背压。UART RX ISR 的 `S`/空格急停必须独立保留。
3. 抽出 Python `m6link`，由唯一 Session 串口所有者供 CLI、GUI、自动调参和 AI 调车共用，并保持现有 V1 单次写入、参数事务和急停语义。
4. 最小桌面 GUI 已实现：默认要求 V2、V1 仅能显式选择；提供 identity/reset/任务/UART/link 诊断、轮速/航向/八路灰度与质心曲线、参数事务、独立 Arm/Confirm/Cancel、常驻急停、录制/回放/评分和逐步重新 Arm 的测试向导。未发布 capability 对应的页签、按钮和动作组会隐藏；当前 15 项 headless/fake Session 测试通过，但尚未连接真实 r14 串口或完成 GUI 长时运行。
5. GUI/统一链路具备可观测性后，验证日志关闭/开启及不同频率下均无复位、漏期和异常丢包。
6. 完成架空轮速复核后，落地按 `160→180→200→220→250 mm/s` 依次验证直线与左右弧线；任一级出现故障、复位、持续饱和、漏期或明显振荡立即回退。
7. 随后验证高速直线保持和可配置角度/角速度动作；赛题/赛道公布前不继续试未知直角弯的纯巡线搜索参数。
8. 最后才做参数保存、复位/断电保持和 PB11 整体回归。

遥控与串口从 `WheelSpeed_SetTarget` 之后共享轮速 PI、Safety 仲裁和电机输出。赛题/赛道公布前，保留 r11 飞行记录器供未来取证，但不再把未知直角弯的纯巡线搜索调参列为当前入口。证据入口见 `docs/2026-07-26-M6-V2通信与复位可观测性验证记录.md`，当前架构和 R0–R7 切片见 `docs/2026-07-26-M6-V2通信架构设计与实施计划.md`，V2 数值合同见 `docs/2026-07-26-M6-V2-wire-registry-v1.md` 及其指向的机器 JSON；历史 M6 证据仍见 `docs/2026-07-25-M6在线调参与行驶姿态优化验证记录.md`。同一问题两次有依据的修复仍失败时停止试验并总结证据边界。

### P1：下载链路可靠性

- 正式脚本为 `scripts/flash.ps1`：先检查 Keil 6.22、MSPM0G3507 工程、相邻 `.uvoptx` 中的 Use Target/CMSIS-AGDI/Horco 序列号/SWD 1 MHz/Flash 算法、探针枚举和最新构建证据，再用 CMSIS-DAP/SWD 下载并要求日志包含 `Verify OK`，最后通过 UART 核对固件 ID、schema、revision 和参数来源。缺失或选错 Debug 配置会在擦写前停止。
- 2026-07-25 已枚举 Horco CMSIS-DAP v2（父设备序列号 `4041840e`）和 COM10。一次正式脚本调用按规则执行了两次完全相同的下载命令，两次均报 `Internal DLL Error`、`Flash Download failed - Target DLL has been cancelled`，没有 `Verify OK`，也没有新固件运行证据。
- 高概率根因是脚本选择了隔离工作树中的 M6 工程，而该目录缺少被 Git 忽略的 `.uvoptx`；主目录工程虽有已验证的 Horco Debug 配置，但当前固件仍是旧主线，不能用它替代 M6 镜像下载。`UV4 -f` 的下载语义正确，不应改成启动调试会话的 `-d`。
- 当前 SWD 下载偶发 `RDDI-DAP Error`、`Erase Failed`、`Flash Timeout` 或校验不一致；问题早于 Horco 无线 DAP，不能归因于当前应用 WWDT。
- 本轮没有进行第三次命令行下载；用户在 Keil 中手动打开精确的 M6 工作树工程并完成下载，随后 COM10 返回精确 M6 固件身份，证明新固件实际运行。后续新的下载周期仍遵守首次失败后只原样重试一次的上限。
- r12 下载前的只读预检再次确认 Keil 6.22、精确工作树、Use Target/CMSIS-AGDI、Horco `4041840e`、SWD 1 MHz 和最新 AXF 均匹配；正式命令第一次失败后只原样重试一次，两次都在 `DebugPortStart` 报 `E310 Debug access failed`、`Cannot access Memory`、`Target DLL has been cancelled`，没有 `Verify OK`。已按上限停止；随后 COM10 的精确 r11 probe 超时，当时固件身份未确认，必须手动下载，不得再发第三次命令或切换自动化方法。
- 用户随后在精确工作树工程中手动下载 r12；COM10 已确认精确身份与首次 reset/Safety/任务/UART 快照。手动成功不改变同一命令行下载周期已达两次上限的事实。
- Keil 手动配置曾把 IROM 显示上限改为 `0x20000`、SWD 时钟改为 5 MHz；已恢复为 `0x1F800` 和 1 MHz，并重新全量构建。随后 `scripts/flash.ps1 -PreflightOnly` 通过，且明确没有发出擦除/编程命令。
- 本次手动下载 `r11` 后，工作树工程的 IROM 显示 Size 再次漂移到 `0x20000`，但 OCR 区与实际 scatter 文件仍为 `0x1F800`，既有 r11 MAP/HEX 边界证据不受影响；已在下一次构建前只恢复该安全值，保留工程文件的其他既存改动。
- 修复后的 `.uvoptx` 配置与脚本防护已经完成两次正式自动下载闭环：`r2` 首次命令成功；`r3` 首次失败后仅原样重试一次并成功，均得到 `Verify OK` 且经 COM10 核对精确身份。日志为未纳入版本控制的 `keil/flash-20260725-140338.log` 和 `keil/flash-20260725-140947.log`。这证明所选工程和 Debug 配置正确，但历史偶发 SWD 故障仍要求每个新下载周期遵守一次原样重试上限。
- 无线 UART ROM BSL 的手动 BSL+RST 握手已验证，自动 Invoke/Reset 未验证且暂缓。完整证据见 `docs/2026-07-24-下载链路诊断记录.md`。

### 后续里程碑

- M6：运行期参数模型、V1 文本事务、A/B Flash 保存、50 Hz 文本遥测、主机评分/保守轮速搜索和正式下载脚本均已实现；r12 reset diagnostics 已完成 power、外部 BOOTRST/POR、受控 SYSRST/WWDT 上板矩阵并恢复正式镜像。待上板 r13 已完成 R2 周期热路径清理，V2 wire registry/golden vectors 已冻结；下一入口是 R3 C/Python codec 与单帧优先 TX，然后依次进入共享 `m6link`、GUI 和日志负载矩阵。尚不能记为 r13/V2 上板、高速或长期运行硬件完成。
- M7：至少 30 分钟连续运行、低电压、急停物理响应、编码器断开、人为堵转、传感器/通信中断和任务超期验收，并形成可复查记录。

## 3. 已实现主流程与安全边界

启动和调度：

1. 启动时只读一次 `RSTCAUSE` 并立即冻结 reset 快照/恢复 breadcrumb，再初始化冻结板级配置并制动，随后初始化调度、状态、安全、UART、编码器、灰度、JY61P、红外、OLED 和 WWDT0。
2. 启动 1 ms SysTick；约 750 ms 非阻塞自检后由 `SELF_TEST` 进入 `IDLE`。
3. 5 ms 任务完成参数诊断边界提交、编码器测速、滤波、目标整形、双轮 PI、编码器健康检查和安全输出；后台服务事务回复、M6 遥测队列、红外队列与单通道灰度扫描；10 ms 处理 JY61P/灰度快照；20 ms 只采集固定大小 M6 快照并更新航向或巡线目标；100 ms 只采集紧凑诊断事件，格式化与发送在周期任务之后的后台执行。
4. WWDT0 周期约 31 ms；每 20 ms 仅在 5/10/20 ms 三个关键任务都完成且无漏期时先喂狗，再提交最近健康窗口摘要，调试器暂停 CPU 时 WWDT 停止。该顺序未放宽原健康门控。

不可破坏的安全约束：

- 系统状态为 `BOOT/SELF_TEST/IDLE/RUN/FAULT`；所有电机请求必须经 `System/safety.c`，没有动作时两路制动，应用/诊断/遥控都不得直接写 PWM。
- UART `S` 或空格在 RX ISR 中立即请求制动并锁存 `FAULT`；`x` 只清到 `IDLE`，不会恢复先前动作。红外没有清故障入口。
- 普通参数 `apply` 仅允许无故障 `IDLE`；轮速诊断只允许在 5 ms 边界更新 Kp/Ki/目标并重置对应控制状态。`save` 还要求动作结束、轮速环关闭且电机制动；最终轮目标始终硬限幅在 ±700 mm/s。
- 传感器、动作、遥控心跳、编码器异常和堵转均有超时/故障门控。编码器速度超过 1500 mm/s、相对滤波值跳变超过 1000 mm/s，或目标≥150 mm/s、PWM≥250 且连续 500 ms 无计数时锁存故障。
- 人工电机、闭环、航向和巡线诊断均为“先选择、10 s 内大写 `A` 二次解锁”的非阻塞动作；无线链路不能替代可立即触达的物理断电手段。

## 4. 当前硬件与接线摘要

完整接线见 `docs/接线说明.md`；芯片与载板约束见 `docs/MSPM0最小系统板解析.md` 和 `docs/天猛星V2_小车主控板_工程参考.md`。

| 功能 | MCU 引脚 | 当前配置/约束 |
|---|---|---|
| UART0 诊断 | PA10 TX / PA11 RX | 115200 8N1；2026-07-25 已重新枚举 Horco 虚拟串口 `COM10`。任何时刻只允许一个发送端驱动 PA11 |
| Horco DAP | PA19 SWDIO / PA20 SWCLK / NRST | Horco CMSIS-DAP v2 已枚举，父设备序列号 `4041840e`。车端由 5 V/GND 供电，3.3 V 不接；正式 M6 下载走 CMSIS-DAP/SWD，不走 BSL |
| JY61P | PA22 UART2_RX | 9600；仅接模块 TX→PA22，模块 PCB 平面与地面平行 |
| PA21 / VREF− | PA21 | 必须保持无上下拉高阻，不启用 UART2_TX |
| VS1838B 红外 | PB11 TIMG0_CCP1 | 4 MHz、16 位、双边沿捕获；与旋转编码器按压开关不可同时使用 |
| 电机 A | PB3 PWM / PA14 / PA13 | TIMA1 CCP1，4 kHz，比较值 0–999 |
| 电机 B | PB2 PWM / PA16 / PA17 | TIMA1 CCP0，4 kHz，比较值 0–999 |
| 编码器 A | PA8 / PA9 | 双相 GPIO 上升沿中断；530.0 count/rev，前进符号 `+1` |
| 编码器 B | PB9 / PB8 | 双相 GPIO 上升沿中断；531.4 count/rev，前进符号 `-1` |
| 驱动轮 | 左/右 | 实测外径约 46 mm，与各轮 CPR 共同用于 mm/s 换算 |
| 灰度地址 | PB1 / PA24 / PA26 | AD0/AD1/AD2；AD0 接 H13-6，PA23/VREF+ 与 PA21/VREF− 均保持高阻 |
| 灰度 OUT | PA25 / ADC0 A0_2 | 感为无 MCU 模拟 OUT；12 位软件触发单次转换、VDDA 参考、每通道 8 次平均，PA25 数字输入关闭 |
| 灰度供电 | 5 V / GND | 与主控共地，当前 OUT 已实测≤3.3 V；PA25 不耐 5 V，模块或接线变化后必须重测 |
| OLED | PA28 SDA / PA31 SCL | 软件 I²C，探测 0x3C/0x3D，分页面后台刷新 |
| BSL/SWD 保留 | PA18 / PA19 / PA20 | 应用不配置；PA18 是高有效默认 BSL 调用输入并与载板蜂鸣器网络复用 |

PA23/VREF+ 与 PA21/VREF− 通过 C44（1 µF）耦合，因此两者必须保持高阻；灰度 AD0 已迁移到 PB1/H13-6。载板只有电源指示灯，没有 MCU 可控状态 LED；诊断对此按能力缺失报告 `DEGRADED`。

## 5. 软件结构与当前参数

| 路径 | 职责 |
|---|---|
| `Source/` | 唯一启动入口、冻结板级配置、1 ms 时基和标准输出 |
| `Config/` | 任务/超时、机械参数、PI/滤波/巡线参数和实测红外键表 |
| `Hardware/` | 电机、编码器、JY61P、模拟灰度 ADC/地址扫描、OLED、UART0、WWDT0、NEC 解码和红外事件队列 |
| `Control/` | 无硬件依赖的 PI、滤波、轮速、航向与巡线数值控制 |
| `App/` | 巡线生命周期和红外人工驾驶/演示动作，不直接控制 PWM |
| `System/` | 时间、调度、状态机、安全仲裁、reset diagnostics、参数模型/协议/持久化、M6 遥测、巡线飞行记录、诊断和健康模型 |
| `Tests/` | 板级不变量、可移植性、HEX/MAP、M2–M6、NEC、遥控和主机工具回归 |
| `scripts/` | 主机回归/Keil 构建、M6 参数事务/记录/评分/保守轮速搜索、CMSIS-DAP 下载和镜像边界检查 |

| 能力 | 当前参数/行为 |
|---|---|
| 参数模型/持久化 | 35 个公开键按 wheel/heading/line/remote 分组；维护默认/活动/暂存、revision 和 source。顶部 A/B 两个 1 KiB 扇区为 `0x1F800`/`0x1FC00`，采用 schema、generation、CRC32 和最后提交标记；无有效记录时使用默认值并报告 `DEGRADED`，不锁存运动故障 |
| 双轮速度闭环 | 默认 A/B Kp=0.70、Ki=2.50，滤波 α=0.25，加/减速 600/900 mm/s²，jerk=20000 mm/s³，前馈为 0；最大 jerk 是 M5 兼容斜坡，调低后启用 jerk 限制，反向强制过零。PI/PWM/误差/dt/delta/饱和快照已开放 |
| 航向 | JY61P `0x52` 角速度在 100 ms 内优先用于 D 项，随后回退角度差分；20 帧校零要求新鲜角速度≤3°/s、跨度≤1°、标准差≤0.35°。默认增益兼容 M4，近目标 smoothstep 和遥控直行航向保持可由参数启用 |
| 巡线 | 新模拟输入的白/黑常量已固化并验证，进入阈值 `[1002,920,1080,975,443,1091,1211,823]`、退出阈值 `[1903,1734,2053,1842,777,2077,2141,1538]`；滞回 0/1 只负责 mask/count/cluster 与有效性，位置由有效簇及相邻通道的连续归一化黑度计算，低于 8% 的弱反射忽略；位置/导数滤波、动态基速、修正变化率和失线搜索保持参数化 |
| 红外人工驾驶 | 地址 `0x00`、17 键表已冻结；100/160/220 mm/s 三档可调，方向心跳 250 ms，Safety 后备期限 500 ms；航向保持启用时前进/后退动作强制依赖新鲜陀螺，转向键仍为人工差速 |

首个感为模拟灰度模块 CH7/CH8 已确认损坏并只保留为历史记录；临时 YB-MVX05-V1.0 数字模块已经退役。当前只支持新到货的感为模拟八路模块；供电、模块、安装高度或赛道材料变化后必须重新采集白/黑数据并构建。

## 6. UART0 诊断命令

| 分组 | 命令 | 作用 |
|---|---|---|
| 帮助/总览 | `h`、`s`、`d`、`v` | 帮助、完整状态、原始诊断、分时系统快照 |
| 传感器 | `e`、`y`、`g`、`q`、`r` | 编码器、JY61P、航向、单次灰度扫描、M5 快照 |
| 通信/显示 | `c`、`o`、`p/f`、`l` | UART 统计、OLED 棋盘格、目视确认/清除、LED 能力 |
| 红外采集 | `t` | 无故障 `IDLE` 下进入 30 s 只读 NEC 采集；采集期间禁止红外车辆动作 |
| 电机/闭环 | `m`、`j/k/b`、`n` | 人工电机测试、A/B/双轮闭环选择、轮速参数快照 |
| 闭环调参 | `+/-`、`[/]`、`,/.` | 调整诊断目标、Kp、Ki；只在 RAM 中生效 |
| 航向动作 | `z`、`w`、`u/i` | 重新校零、250 mm/s 直行 5 s、相对 `+90°/-90°` |
| 巡线 | `1`、`2`、`3`、`T` | `1` 输出白/黑常量、跨度和滞回阈值；`2` 提示极性由标定固定；`3` 仅在标定有效且当前有线时选择巡线；`T` 仅在空闲时分时导出最近一次巡线首个失线事件的 `G6` 飞行记录 |
| 动作控制 | `A`、`0` | 二次解锁；正常停止已选择或运行中的巡线/红外非故障动作 |
| 故障控制 | `S`/空格、`x` | ISR 级急停并进入 `FAULT`；清锁存故障且只回 `IDLE` |
| M6 参数事务 | `:<id>,get/set/apply/revert/load/defaults/save/log,...` | 小写、无空格、CR/LF 结尾；正文最多 58 字节，回复固定为 `ok,<id>,...` 或 `err,<id>,...`；`S`/空格在任何解析状态仍由 ISR 抢占 |

`s`/`v` 使用冻结快照并每 100 ms 最多输出一行，`v` 在动作 CSV 期间延后，M5 运行中的 `r` 只返回短提示；r11 的既有分时路径已上板复测无 WWDT 复位。r12 为 `v` 增加 `RESET/RESETCTX/RESETIO` 三行并把它们放到所有周期任务之后的后台。待上板 r13 进一步把 M3/M4/M5 与红外通知移出 100/20 ms 周期任务，以三槽紧凑事件队列和固定通知标志交给后台；动作 CSV 连续性优先于旧文本报告，普通/self-test 后续行在动作期间可延后，最长受 300000 ms 动作超时限制。队满累计 `diag100_drop`，V1 TX 拥塞仍由 `tx_drop` 暴露；当前尚无 r13 日志压力下的无复位证据。

`M2_FAULT_INJECTION_TEST` 当前源码使用独立身份 `M6-runtime-20260726-r13-fi`；已完成的硬件复位矩阵使用历史 `r12-fi`。只有测试构建开放大写 `Z`（合成 HardFault scratch 后请求 SYSRST）和大写 `W`（停止喂狗并持续 Safety 制动等待 WWDT）。两者仅在 IDLE、无故障、无活动动作且尚未停喂狗时接受，并先经 Safety 直接制动；`W` 接受后不再返回主循环或处理动作命令，UART ISR 的 `S`/空格仍可抢占。正式 r12/r13 均不含这些入口。`Z` 传入空栈帧，PC/LR/xPSR 预期为 0，只能验证 SYSRST 保持和 scratch 恢复，不能宣称真实 HardFault 向量/栈捕获通过。

## 7. 功能与验证结论

| 范围 | 当前结论 | 证据入口 |
|---|---|---|
| M0 工程基线 / M1 外设诊断 | 完成；工程、UART、电机、编码器、JY61P、OLED 与能力降级路径已建立 | `docs/2026-07-21-M0-M1验证记录.md` |
| M2 调度与安全 | 完成；主机回归、目标构建、架空故障注入、急停/超时/WWDT 路径通过；PWM 引脚物理响应时间留 M7 | `docs/2026-07-22-M2验证记录.md` |
| M3 双轮速度闭环 | 完成；300 mm/s 架空稳态平均绝对误差 A/B 均低于 2%，无持续振荡；轮径与 CPR 已实测校准 | `docs/2026-07-22-M3验证记录.md` |
| M4 航向 | 完成；最终 `+90°/-90°` 误差 `+0.09°/-1.20°`；250 mm/s×5 s 落地约 125 cm，直行最大瞬时航向误差 0.99° | `docs/2026-07-22-M4验证记录.md` |
| M5 巡线 | 新感为模拟模块已完成 PB1 接线、64 帧白/黑标定、解锁构建/下载、全黑/全白和八探头静态复核；方向、架空动作、1500 ms 失线与落地圈数仍需在 M6 组合镜像上复验 | `docs/2026-07-25-感为模拟灰度模块恢复记录.md` |
| PB11 红外阶段一 | 完成；17 个 NEC 键实测唯一且反码正确，键表冻结；连续长按和运行期 `s` 复位缺陷已关闭 | `docs/2026-07-24-PB11红外遥控接入验证记录.md` |
| PB11 红外阶段二 | 进行中；架空的 100/220 mm/s 四方向、三档/长按、`OK`/`0`、251 ms 失联停车和演示门控通过；落地 100 mm/s 遥控前进/航向保持/松键制动通过，仍缺落地后退/左右转 | 同上 |
| M6 软件基础设施 | 运行期参数、V1 文本事务、A/B Flash、50 Hz 文本遥测、连续模拟位置、弯道方向锁存和有界失线飞行记录均已实现；100 mm/s 连续模拟位置已实车显著改善。r12 reset diagnostics 已通过主机模型、目标构建，以及 power、短按 NRST/BOOTRST、长按 NRST/POR、受控 SYSRST/WWDT 上板验证。待上板 r13 已完成 R2：周期任务只采集紧凑事件/通知，RX 扩至 256 B、每轮预算 32 字节且最多完成一个事务/旧命令，RX 积压时不 WFI。V2 codec/优先 TX、`m6link`、GUI、真实 HardFault、r13 日志压力与高速长期运行仍未实现/验证 | `docs/2026-07-25-M6在线调参与行驶姿态优化验证记录.md`、`docs/2026-07-26-M6-V2通信与复位可观测性验证记录.md` |
| 当前组合构建/上板证据 | 当前 r13 源树 98 项主机回归通过；正式 r13 Rebuild 为 `Code=43868, RO-data=9612, RW-data=8, ZI-data=26184`，0 error、0 warning，最高普通字节 `0xD0EF`，低于 `0x1F800`，`RW_NOINIT` 仍为 `0x20207F00`/256 B。正式 r12 已验证 raw `0x01` power、`0x0C` external BOOTRST 与 `0x02` external POR；历史测试镜像 r12-fi 已验证 raw `0x1B` SYSRST 恢复 coarse/detail/合成 scratch，以及 raw `0x0E` WWDT 仅恢复 coarse并锁存 Safety `0x20`。当前 MCU 仍是正式 r12，最终快照为 Safety IDLE、PWM/目标/任务 miss/UART/遥测 drop 0；r13 尚未下载，尚未做其运动、日志压力或长期运行验证 | 同上 |

这些是已有证据的摘要。只有实际重新运行主机测试、Keil 构建或硬件测试后，才能更新相应证据；源代码检查不能写成目标编译或上板验证。

## 8. 构建、下载与仓库规则

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Mode Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/flash.ps1 -PreflightOnly
python scripts/m6_tune.py probe --json
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build_fault_injection.ps1
```

- Keil 工程：`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
- 新克隆先运行 `scripts/setup_git_hooks.ps1`；pre-commit 要求工程状态变化同步 `STATE.md`。
- 本仓库不使用 OpenSpec；不得创建或调用 `openspec/`、`.openspec.yaml`、OpenSpec CLI 或项目级 OpenSpec 技能。设计、实施计划和日期证据统一保存在 `docs/`。
- `scripts/build_fault_injection.ps1` 是纯构建脚本，不提供下载开关；它只在临时测试工程加入 `M2_FAULT_INJECTION_TEST`，并在本机存在正式 `.uvoptx` 时生成同名测试 options，复用正式下载脚本的完整断言核对 current target、Use Target、CMSIS-AGDI、Horco 序列号、Flash sequence/算法、SWD 协议与 1 MHz。正式 target 不含故障注入入口；既有命令行下载周期已经耗尽，本轮没有为 r13 发起新的命令行下载，只允许用户手动下载精确生成工程。
- `keil/Objects`、`keil/ObjectsFaultInjection`、临时工程、构建/下载日志不纳入版本控制。
- 命令行固件下载必须遵守第 2 节 P1 的一次原样重试上限。

## 9. 按需读取地图

| 任务 | 只需继续读取 |
|---|---|
| 当前接线/换模块 | `docs/接线说明.md`，必要时再看对应原理图/手册 |
| 红外使用或验收 | `docs/PB11红外遥控器使用指南.md`、`docs/2026-07-24-PB11红外遥控接入验证记录.md` |
| 下载失败 | `docs/2026-07-24-下载链路诊断记录.md` |
| M6 历史参数、遥测、下载或实车调参 | `docs/2026-07-25-M6在线调参与行驶姿态优化验证记录.md` |
| M6 reset/V2/`m6link`/GUI/高速与长期运行 | `docs/2026-07-26-M6-V2通信与复位可观测性验证记录.md`、`docs/2026-07-26-M6-V2通信架构设计与实施计划.md`、`docs/2026-07-26-M6-V2-wire-registry-v1.md` |
| M2–M5 回归或复核 | 对应的单个日期验证记录 |
| 里程碑范围/新赛题设计 | `docs/superpowers/specs/2026-07-21-competition-car-platform-design.md`；其中历史路径不代表当前代码结构 |
| 载板接口或 MCU 约束 | `docs/天猛星V2_小车主控板_工程参考.md`、`docs/MSPM0最小系统板解析.md` |

更完整的文档分类见 `docs/README.md`。不要把历史计划当作当前待办，也不要为了“建立上下文”批量读取 PDF、全部验证记录或构建产物。

## 10. 当前风险

1. 新模拟灰度的编译期标定只适用于当前模块、供电、安装高度和赛道材料；任一条件变化后必须重新确认 OUT≤3.3 V、采集 64 帧白/黑数据、更新常量并重新构建。当前几何为离地约 25 mm、传感器行到轮轴约 240 mm、黑线宽约 17 mm；r12 已核对身份/空闲状态，尚未运动验证。
2. JY61P 接近竖直时会因 pitch≈±90° 产生欧拉角失真；当前必须平放。磁场变化和 30 分钟温漂尚未量化。
3. OLED 尚未实现总线恢复和时钟拉伸超时；UART 缓冲满会丢数据并计数，运行期不得恢复长同步报告。r13 的诊断事件队满由 `diag100_drop` 计数，但旧 V1 `debug_printf()` 在 TX 拥塞时仍可能形成残行并只体现为 `tx_drop`；这要由后续单帧 TX 仲裁/V2 关闭。
4. 急停到实际 PWM 引脚归零尚缺示波器/逻辑分析仪测量；编码器断开、人为堵转和保护阈值仍需 M7 架空实测。
5. M5 没有起终点标志识别，圈数由现场人工判断；现有赛道验收不能外推到不同材料或安装条件。
6. PB11 与编码器按压开关不可共用。阶段二正式验收仍缺 100 mm/s 落地后退/左右转，但已按用户决定延后且不阻塞串口控制的 M6 算法调试。本轮落地长按中累计 `timing=81`且出现多次心跳正常制动，说明遥控验收时必须保持接收器视距，不能用加长心跳超时换取连续性。
7. SWD 自动下载已有一次完整成功闭环，但历史仍有偶发故障；不得用无依据的配置变体扩大诊断范围，失败时必须遵守一次原样重试上限。
8. `r9` 的连续模拟位置已解决 100 mm/s 直道二值量化极限环，但 160 mm/s 的失线恢复仍有明显摆动。动态降速候选未改善综合评分；`r10` 前向圆弧搜索在约 75° 后仍未找到 90° 弯线并于 1520 ms 安全停车。`r11` 已恢复旧搜索并保留飞行记录器；赛题/赛道公布前不再试第三个纯巡线搜索参数变体，高速工作先限定为普通直线、平滑弧线和陀螺仪动作。
9. M6 Flash A/B 保存仅通过主机模型、源码约束和目标链接验证，尚未进行真实复位/断电/写入中断测试；调试器整片擦除不承诺保留参数。
10. M6 轮速参数已通过架空四档矩阵筛选，但这不代表落地航向或巡线参数已调好。220 mm/s 硬目标和 250 mm/s 扩展目标必须按分级实车证据判定。
11. `r3` 已关闭 160 mm/s/20 ms 文本遥测 WWDT，并在双轮 300 mm/s 架空最高负载下保持稳定；用户观察到的另一次无征兆停车仍无当时复位证据。r12/r12-fi 已实际区分 raw `0x01` power、`0x0C` external BOOTRST、`0x02` external POR、`0x1B` software SYSRST 和 `0x0E` WWDT；SYSRST 恢复 detailed，BOOTRST 只恢复 coarse，POR 全部失效。`Z` 仍只是 PC/LR/xPSR 为 0 的合成 scratch，不等于真实 HardFault；`RSTCAUSE` 也只给出自上次读取以来最低层级原因，raw `0x01` 还合并 POR/PMU trim parity/SHUTDNSTORE parity。编码器输入缺少硬件滤波/明确 IRQ 优先级仍是次级 EMI 风险；若再现复位，不得盲目试修，必须先取得 r12 reset 报告、任务执行时间和 GROUP1 IRQ 频率。

## 11. 本次文档整理记录

2026-07-24：将当前摘要、活动待办、安全边界和按需读取地图前置，把下载故障的长诊断证据迁入独立记录，并删除与日期验证记录重复的历史叙述。无功能变化；未重新运行主机测试、Keil 构建或硬件验证，所有功能结论均沿用第 7 节列出的既有证据。

2026-07-25：实现 M6 运行期参数、事务协议、A/B Flash 持久化、控制优化分支、50 Hz 非阻塞遥测、主机调参与正式 SWD 下载工具；76 项主机回归和 Keil 6.22 Rebuild 通过，镜像边界与 RAM Flash 写入器通过 MAP/HEX 检查。Horco 探针未枚举，预检在下载前停止；无上板、掉电或高速赛道功能结论。

2026-07-25：删除项目内全部 OpenSpec 工件，并改由 `docs/` 保存 M6 工程记录；禁用规则已写入 `AGENTS.md`。无功能变化；未因本次流程清理重新运行主机测试、Keil 构建或硬件验证。

2026-07-25：修正复合 USB 设备下 Horco 父子拓扑识别，并强制下载前校验当前工程的 `.uvoptx` Debug/探针/SWD/Flash 算法配置；整库 77 项主机测试通过。Horco 与 COM10 已连接，但工作树此前缺少该选项文件，两次相同命令均以 Target DLL 取消失败；没有 `Verify OK` 或新固件运行证据，本轮按重试上限停止并转人工下载。

2026-07-25：用户手动下载 M6 工作树工程后，COM10 确认 `M6-runtime-20260725`、schema 1、revision 1、source `defaults`、generation 0；35 项活动参数读取完整。空闲快照为 Safety IDLE、PWM 0/0、故障 0、任务漏期 0、UART 丢包 0；JY61P 当前有效帧与校验正常。恢复 Keil 自动改动的 IROM `0x1F800` 和 SWD 1 MHz 后重新 Rebuild，0 error、0 warning，镜像边界通过；只读下载预检通过且未擦写。

2026-07-25：首次双轮 160 mm/s + 20 ms M6 遥测在启动约 100 ms 后发生 WWDT 复位；截断的 35 列行是复位结果，不是 schema 不一致。关闭 M6 遥测的同速对照完整运行并正常减速，左右稳态平均绝对误差 2.52%/3.64%、无调度漏期。修复版 `M6-runtime-20260725-r2` 抑制 M6 开启时的 M3/M4/M5 重复 CSV、把后台排空移到周期任务之后，并缓存背压待发送行避免重复格式化；78 项主机测试与 Keil Rebuild 通过，尚未上板。

2026-07-25：正式脚本首次命令成功下载 `r2`，日志包含 `Verify OK`，COM10 身份核对通过；同一双轮架空 160 mm/s/20 ms 遥测运行 4.1 s、取得 206 个完整样本后仍发生 WWDT，复位后 Safety 为 `0x21`。第二个也是最后一个软件修复 `r3` 以有界整数 CSV 编码器替换后台通用 `snprintf`；78/78 主机测试和 Keil Rebuild 通过，最高镜像地址 `0xB537`，尚未下载。

2026-07-25：`r3` 在首次命令失败后按规则原样重试成功，日志包含 `Verify OK`，COM10 身份核对通过。双轮架空 160 mm/s/20 ms 遥测完整取得 366 样本/7.32 s，硬失败、故障、任务漏期、UART/遥测丢包、持续饱和均为 0；延迟复核仍为 IDLE、PWM 0/0、固件 ID 不变，诊断目标恢复 300 mm/s。同工况 WWDT 缺陷关闭。

2026-07-25：完成左右单轮和双轮 100/160/220/300 mm/s 共 12 个架空基线，稳态误差 1.10%–4.67%、超调 3.33%–8%，全部无硬失败。每轮四点 PWM 拟合候选使综合评分下降 2.24%且 100 mm/s 超调升至 26%，故前馈保持 0；α=0.20 在 220 单点改善 12.55%，但四档综合下降 0.89%，故恢复 0.25。最终活动/暂存 PI、滤波、前馈均回到已验证基线，未保存候选。

2026-07-25：完成 PB11 阶段二架空安全路径：100/220 mm/s 四方向、220 mm/s 两次额外长按前进、遮挡接收器 251 ms 停车、`OK`/`0` 制动、直行/±90°选择确认与停止、10 s 确认超时、巡线无效拒绝与有效运行停止均通过。所有记录无故障、WWDT 复位、调度漏期或 UART/遥测丢包；最终为 `r3`、100 mm/s、IDLE、PWM 0/0。本次未改固件、未重新编译；PB11 仅剩 100 mm/s 无障碍短距离落地人工驾驶。

2026-07-25：完成 PB11 的 100 mm/s 落地人工前进和 M6 首轮落地航向筛选。无航向保持的 3.04 s 直行用户确认逐渐向右弯，相对动作起点的 yaw RMS/峰值 `3.09°/5.37°`；开启遥控航向保持后，6.94 s/2.14 s 两个人工段为 `0.86°/1.40°` 和 `0.24°/0.45°`，用户确认总体直线、停车正常，但仍感到轻微右弯。右轮 `Ki=2.75` 重复后超调达 22%，航向 `Kp=4.0` 目视摇摆，均拒绝并回滚。最终 revision 8、IDLE、PWM 0/0、故障/漏期/丢包 0，只保留 `remote.heading_hold=1`；未保存、未改固件、未重新编译。PB11 阶段二仍需落地后退/左右转，不提前关闭。

2026-07-25：用户确认自行测试 PB11 按键功能无异常，并决定先完成算法调优、参数定型后再做遥控器正式复核。代码核对确认 PB11 与串口在输入/心跳/动作入口上不同，但从轮速目标之后共享控制与 Safety 链路；因此将 M6 主线切换为串口控制的 100 mm/s 巡线基线，PB11 正式落地后退/左右转保留为后续验收项。随后把巡线最低/最高速度仅在 RAM 原子应用为 100/100 mm/s，revision 9；活动值/暂存值一致，Safety IDLE、目标/速度 0/0、应用 PWM 0/0、故障/漏期 0，未保存。本次无固件功能变化。

2026-07-25：将主线已验证的新感为模拟八路采集、PB1 地址选通和最终白/黑滞回标定合入 M6，保留 `r7` 的弯道方向锁存、运行期参数、遥测、1500 ms 失线停车、Safety、急停、WWDT 和红外入口；组合身份为 `M6-runtime-20260725-r8`。整库 88 项主机测试通过，Keil 6.22 Rebuild 为 0 error、0 warning，最高普通镜像字节 `0xBD57`，未下载或上板。

2026-07-26：完成 `r8`–`r10` 的新模拟巡线实车诊断。`r9` 将二值质心改为局部连续模拟质心；100 mm/s 代表性直道 4 s 窗口的位置 RMS、修正 RMS、修正 jerk 相对 `r8` 分别下降 56.9%、52.7%、75.3%，用户确认无右偏。160 mm/s 固定速度仍明显摆动；100–160 mm/s 动态基速候选综合评分更差，已回滚。`r10` 前向圆弧搜索连续 1520 ms 未重新获线，触发 `SAFETY_FAULT_LINE_LOST=0x100`，用户观察为大弯未回线即停车，故拒绝。两次候选未解决同一搜索问题，停止继续盲调。

2026-07-26：源码推进到待下载的 `M6-runtime-20260726-r11`：恢复 `r9` 已知能重新获线的搜索行为，不再改变车辆控制输出；新增 48 帧、20 ms 间隔的首失线飞行记录器，保留约 220 ms 事件前与 720 ms 事件后数据，并在空闲时用 `T` 分时输出 `G6` 原始 ADC/拓扑/搜索方向/位置/修正。主机评分器新增 `action_not_started` 硬失败，防止记录命令把未启动的零输出误判为通过。91 项主机测试通过；Keil 6.22 Rebuild 为 0 error、0 warning，`Code=41348, RO-data=8488, RW-data=8, ZI-data=25272`，最高普通镜像字节 `0xC2B7`。车辆已由用户关机，本轮按要求没有下载或上车测试；`r11` 硬件状态明确为待验证。

2026-07-26：用户手动下载 `r11` 后，COM10 精确确认 `M6-runtime-20260726-r11`、schema 1、revision 1、source `defaults`、generation 0；只读空闲快照为 Safety IDLE、故障/PWM/目标 0、约 357 s 四级任务漏期 0、UART 丢包/格式错误 0。本轮未启动电机、改参数、写 Flash 或执行命令行下载。源码审计确认 `RSTCAUSE` 为 read-to-clear，而 r11 仅把 WWDT0 映射为 Safety `0x20` 后丢弃原值；本次故障 0 只能排除当前启动的未清除 WWDT 复位，不能区分上电、外部、软件或调试器复位，也不能恢复复位前 breadcrumb。后续目标改为 reset diagnostics、V2、`m6link`/GUI、无复位日志矩阵和 160–250 mm/s 直线/平滑弧线/高速航向分级验证；未知直角弯搜索暂停。

2026-07-26：实现待上板的 `M6-runtime-20260726-r12` reset diagnostics：启动只读一次 raw `RSTCAUSE`，加入分类/boot ID、4 B `SHUTDNSTORE` 粗 checkpoint、256 B `UNINIT` 双槽 CRC 健康摘要及 HardFault PC/LR/xPSR scratch；Safety、急停、TX 编码/提交、参数 Flash、WWDT 与后台阶段均有有界 checkpoint。健康窗口保持“先通过原门控并喂狗，再记录”的顺序；`v` 的三条 reset 报告移到所有周期任务后的后台且每 100 ms 至多一行。96 项主机测试通过；Keil 6.22 Rebuild 为 0 error、0 warning，`Code=43580, RO-data=9416, RW-data=8, ZI-data=25704`，最高普通字节 `0xCF0F`，保留区在 `0x20207F00` 且为 `UNINIT`。本轮未下载，车上仍是 r11；跨 SYSRST/BOOTRST/BOR/WWDT/NRST/断电保持、HardFault 和日志负载均不能据源码或构建写成硬件通过，下一步先做 r12 上板身份与复位矩阵。

2026-07-26：提交 r12 源码后，以只读工程文件重新 Rebuild 保持 IROM `0x1F800`，随后 `scripts/flash.ps1 -PreflightOnly` 通过且未擦写。正式命令行下载第一次失败后仅原样重试一次，两次均在 TI pack `DebugPortStart` 阶段报 `E310 Debug access failed`、无法写 AP `0x00000001` 寄存器 `0x0C`、`Cannot access Memory` 和 Target DLL 取消；日志无 `Verify OK`，故按规则停止。下载后的 COM10 精确 r11 probe 超时，当前 MCU 身份未知；没有第三次命令、GUI 自动化、BSL 或配置变体。下一步必须由用户手动打开精确 r12 工作树工程下载，完成后再核对 r12 身份和 reset 报告。

2026-07-26：用户手动下载 r12 后，COM10 精确确认 `M6-runtime-20260726-r12`、schema 1、revision 1、source `defaults`、generation 0。`v/c` 冻结快照报告 raw `0x01 por_or_parity`、class `power`、boot 1、旧 coarse/detail/HardFault 均无效，符合 POR 边界；Safety IDLE、故障/PWM/目标 0，约 44.7 s 的 5/10/20/100 ms 任务漏期 0，UART TX/RX drop、format 和遥测 drop 均为 0。本次只读核对未启动电机、应用/保存参数或触发额外复位；raw `0x01` 不能进一步区分冷上电与 PMU trim/SHUTDNSTORE parity，其他复位保持仍待验证。

2026-07-26：用户随后短按 NRST；COM10 再次精确确认 r12，`v/c` 报 raw `0x0C external_boot`、class `external`、boot 2，上一启动的 4 B coarse 记录有效且为 boot 1/`idle`，详细 SRAM 记录与 HardFault scratch 无效。TI G3x0x DriverLib 将 `0x0C` 定义为 `<1 s` NRST 触发 BOOTRST，并只保证 `SHUTDNSTORE` 跨 BOOTRST 保持；TI 用户手册明确 BOOTRST 会 power-cycle 系统 SRAM，因此 `.noinit` detailed 丢失符合硬件边界，不据此修改固件。复位后约 31 s 仍为 Safety IDLE、故障/PWM/目标 0、四级任务漏期 0、UART/遥测丢包和格式错误 0；本轮只读验证未运动、改参、写 Flash 或再次下载。

2026-07-26：用户按指引长按 NRST 约 2 s；COM10 精确身份不变，`v/c` 报 raw `0x02 external_por`、class `external`、boot 1，coarse/detail/HardFault 历史均无效，符合 TI 对 `>1 s` NRST 触发 POR、POR 清 shutdown memory 和 SRAM 的定义。约 31.1 s 空闲快照仍为 Safety IDLE、故障/PWM/目标 0、四级任务漏期 0、UART/遥测丢包和格式错误 0；本轮只读验证未运动、改参、写 Flash 或再次下载。

2026-07-26：新增仅 `M2_FAULT_INJECTION_TEST` 可用的受控复位验收路径：测试身份独立为 `M6-runtime-20260726-r12-fi`；大写 `Z` 在 Safety 制动后写合成 HardFault scratch 并请求 SYSRST，大写 `W` 在同一安全门控后停止喂狗并进入持续制动等待，拒绝复位前再次处理动作命令。两者均拒绝非 IDLE、有故障、有活动动作或已停喂狗状态，正式 r12 无入口；`Z` 不模拟真实异常栈。故障注入脚本已移除下载入口，并复用正式完整断言验证生成的 Debug options。首轮测试构建因 `diagnostics.c` 缺少 `__WFI` 声明而失败；定位后仅在测试宏下引入现有 TI 配置头，复测成功。97 项主机测试通过；正式 r12 Rebuild 仍为 `Code=43580, RO-data=9416, RW-data=8, ZI-data=25704`、最高 `0xCF0F`，测试镜像 Rebuild 为 `Code=43620, RO-data=9432, RW-data=8, ZI-data=25696`、最高 `0xCF47`，两者均 0 error、0 warning且低于 `0x1F800`。本轮未下载，车上仍是正式 r12。

2026-07-26：用户手动下载精确 `r12-fi` 后，COM10 确认 schema/revision 1、defaults/generation 0、测试帮助标记、Safety IDLE、PWM/目标/漏期/丢包 0。受控 `Z` 得到 raw `0x1B software_sys`，boot `1→2`，coarse/detail/合成 HardFault scratch 均恢复，PC/LR/xPSR 按空栈设计为 0；受控 `W` 得到 raw `0x0E wwdt0`，boot `2→3`，只恢复 coarse，Safety 锁存 `0x20`，PWM/目标/漏期/丢包仍为 0。该结果验证 SYSRST 与 WWDT 实际分类/保持边界，但不代表真实 HardFault 栈捕获；取证前未清故障。当前测试镜像仍在 MCU 上，必须手动恢复正式 r12 并确认帮助中无 `TEST ONLY` 后才能继续。

2026-07-26：首次恢复正式镜像后，COM10 的精确 r12 probe 连续超时；短按复位后裸 `h` 返回旧 M5 菜单，存在 `2=active-level` 且没有 M6 transaction。旧固件 `v/c` 为 Safety IDLE、故障/PWM/目标/漏期 0，UART `tx_drop=337`。二进制字符串比对确认仓库根目录 AXF 恰含 `M5 LINE FOLLOWING`/`2=active-level`，而 `.worktrees\m6-runtime-tuning` AXF 含精确 r12 身份与 M6 transaction，因此根因是手动打开了根目录旧工程，不是 r12 事务失效。已要求重新下载精确工作树工程；当前不能记为正式 r12 已恢复。

2026-07-26：用户重新手动下载精确工作树正式工程；下载后目标暂未运行，短按 NRST 后 COM10 精确确认 `M6-runtime-20260726-r12`、schema/revision 1、defaults/generation 0。帮助含 M6 transaction 且无 `TEST ONLY`/`FAULT INJECTION`，证明测试入口已移除；`v/c` 报 raw `0x0C external_boot`、boot 5、coarse 有效，Safety IDLE、故障/PWM/目标 0、四级任务漏期 0、UART/遥测 drop 和格式错误 0。受控复位矩阵至此完成并安全恢复正式 r12；本轮没有命令行下载、动作、改参、保存或清除测试故障。

2026-07-26：待上板固件推进为 `M6-runtime-20260726-r13` 并完成 V2 R2 热路径清理：UART RX 环由 64 B 扩为 256 B，主循环每轮最多消费 32 个事务字节且最多完成一个事务或执行一个旧单字节命令，RX 积压时不进入 WFI；100 ms 诊断任务只采集 92 B 紧凑事件，三槽队列占 276 B，M3/M4/M5 指标累计仍在周期任务内且不随输出丢弃；20 ms 红外任务只置固定通知标志。格式化和发送移到后台，动作 CSV 优先于普通报告，队满由 `diag100_drop` 暴露。98 项主机测试通过；Keil 6.22 Rebuild 为 0 error、0 warning，`Code=43868, RO-data=9612, RW-data=8, ZI-data=26184`，最高普通字节 `0xD0EF`，`RW_NOINIT` 仍为 `0x20207F00`/256 B，正式 AXF 仅含 r13 且无测试入口字符串。构建时一度发现 Keil 将 UI IROM 漂移为 `0x20000`，边界测试拦截后只恢复为 `0x1F800`，保留既有 OCR 大小写差异。Safety、电机唯一所有者、ISR 裸 `S`/空格急停、WWDT 健康门控和参数 apply/save 边界均未改变；本轮未下载，当前 MCU 仍是正式 r12，故无 r13 上板或日志负载结论。

2026-07-26：用户再次报告按键后执行只读 COM10 复核；精确身份仍为正式 r12、schema/revision 1、defaults/generation 0，`v/c` 仍报前次 `boot=5/raw=0x0C external_boot`，5 ms release 已连续到 450482（约 2252.4 s），故本次观测窗口没有形成新的 MCU reset。Safety IDLE、故障/PWM/目标、四级任务漏期、UART/遥测 drop 和格式错误均为 0；所按对象没有额外证据，不将其写成失败 NRST。本次只读复核未下载、运动、改参、save 或 clear，无功能变化。

2026-07-26：冻结 V2 `m6-v2-wire-v1` registry revision 1、JSON schema、C/Python golden vectors 和只读一致性检查器，registry/schema canonical SHA-256 分别为 `23fbd056db28c1fedc7987a34d0893594fddb417f34058851667ffc2d5dca0a2` / `95191cda4ea5ed0691126af1fa027ff37b3dd6f920ede9c099cad0b16fce7ede`；覆盖 CRC 3、escape 1、session 2、legacy ASCII 1、resync 1、frame 10、stream 22 组向量。终审补齐 HELLO 固定 required/required-optional 重叠拒绝、未知 wire version 的 ARMED/ESTABLISHED 分支及普通 async `0xFFFFFFFD`/专用 terminal `0xFFFFFFFE` 边界。检查器、6 项聚焦测试和全量 104 项主机测试均通过，`git diff --check` 无错误；冻结守卫会拒绝 schema/registry 摘要漂移、duplicate key、非有限数、布局/枚举/语义漂移和试图用 `--write-inc` 为 revision 1 重新背书。本切片仅改文档、JSON、测试和工程检查脚本，没有改目标源码、Keil 工程或硬件结论，因此没有重新构建、下载、运动或执行硬件动作；待上板 r13 的既有 Keil 证据不变，当前 MCU 仍为正式 r12。Safety、电机唯一所有者、ISR 裸 `S`/空格急停、WWDT 健康门控和参数事务边界没有功能变化；下一入口为 R3 C/Python codec 与单帧优先 TX。

2026-07-26：完成赛前工作区合并。整理前在仓库外 `Archive/2026-07-26-pre-merge` 保存全引用 bundle、全部分支 HEAD、HC04 独有提交补丁、用户文件 SHA-256、大小写工程差异备份和清单；`git bundle verify` 通过且 `git fsck --full --strict` 无损坏对象。`main` 仅以 fast-forward 前进到 `bd701e5`，随后将 `codex/m6-runtime-tuning` 切换到唯一活动目录；analog、HC04 和旧 M6 三个附加 worktree 已移除，HC04 历史分支保留，两个已完全合并的冗余分支已删除。两份相同灰度例程 ZIP 经 SHA-256 `08E49D34AAAE0BA82827AFF2942716BD42DB51CF9780D421FBD6015B52A17EB2` 复核后只保留一份到仓库外 `References`，重复副本进入回收站且从未暂存；KEIL、Seekfree/红外/DAP 及灰度厂商资料保持目录完整移入 `References`。精确删除既有 88 个只读 `tmp_obj_*`（4,715,270,144 B，约 4.39 GiB）和可重建 `.codex-tmp`；本轮中新出现的第 89 个 `tmp_obj_NGS0DB`（55,566,336 B）不属于该冻结集合，已保留。未执行广域 `git clean`、prune 或 aggressive gc。当前目录结构只剩 `User_Project/MSPM03507CAR`、`References` 和 `Archive`；本次无固件、工程配置或硬件功能变化，未重新运行主机测试、Keil 构建、下载或上车验证。
