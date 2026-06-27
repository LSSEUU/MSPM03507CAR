# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 这是什么

电赛智能小车工程，主控 **TI MSPM0G3507**（Cortex-M0+, 32 MHz），两轮差速驱动。
开发板：立创·天猛星 MSPM0G3507；电机驱动板：WHEELTEC D153C（TB6612）。
裸机（nortos），C11，Keil MDK5 (ARMClang) 编译，TI SysConfig 配置外设。

---

## ⚠️ 首要认知：文档与代码已不同步

`CODEREADME.md` 是一份详细的代码解析文档，但**它描述的是较早状态，代码已经走在前面**。
动手前务必以**源码 + `docs/接线说明.md`** 为准，不要照搬 CODEREADME 的描述。已知偏差：

- **`Source/empty.c`（main）已是完整状态机**：走方形轨迹（前进定距 → 陀螺仪 PID 原地转向 90° → 重复 4 边），
  状态 `ST_IDLE/MOVING/PAUSE/TURNING/SETTLE/DONE`。CODEREADME 里“main 只做 UART 回显、Control_Loop 被注释”的说法**已过时**。
- **存在 CODEREADME 未提及的驱动**：`Hardware/gyro.c/.h`（JY61P 陀螺仪，UART2）、`Hardware/grayscale.c/.h`（感为 8 路灰度，ADC 分时复用）。
- **引脚表**：CODEREADME 第 2.2 节大量 `TBD` 且与实物不符。**权威引脚分配见 `docs/接线说明.md`**（日期最新）。
- **蓝牙**：实际走 **UART0**（HC-04，115200，与调试串口共用），不是 CODEREADME 写的 UART1/HC-05/9600。

CODEREADME 仍有价值的是：分层架构、模块职责、PID/差速运动学/蓝牙协议帧格式（`Control/uart_callback.c`）的设计说明。

---

## 构建命令

Keil 安装路径硬编码为 `C:\Keil_v5`（脚本和 `.vscode/tasks.json` 都依赖它）。

| 操作 | 命令 |
|------|------|
| VS Code 构建（默认任务） | `Ctrl+Shift+B` → "build (Keil)" |
| 命令行构建 | `scripts\build.bat`（等价于下面这条） |
| 直接调 Keil | `C:\Keil_v5\UV4\UV4.exe -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx -j0 -o keil\build.log` |
| 打开 IDE | `scripts\open_keil.bat`（F7 构建 / F8 下载 / Ctrl+F5 调试） |
| 清理中间文件 | VS Code "clean" 任务，或删 `keil\Objects\*.o`、`keil\*.lst`、`keil\*.dep` |

- 构建产物：`keil\Objects\empty_LP_MSPM0G3507_nortos_keil.axf`（调试用）与 `.hex`（烧录用）。
- 构建日志：`keil\build.log`。`.bat` 退出码 0 = 成功。
- `.bat` 脚本含中文 `echo`，本机 cmd（GBK）下可能乱码或报错；优先用 VS Code 任务或直接调 UV4。

## 烧录 / 调试

OpenOCD + CMSIS-DAP，SWD（SWDIO+SWCLK），适配器 100 kHz，目标 MSPM0G3507。
配置文件：`.vscode\openocd.cfg`、`.vscode\mspm0g3507.cfg`、`.vscode\openocd_test.cfg`。
VS Code 用 `cortex-debug` 扩展，配置 "MSPM0 DAPlink Debug"（`launch.json`，`preLaunchTask` 会先构建，
SVD 在 `include\ti\devices\msp\peripherals\MSPM0G3507.svd`，`runToEntryPoint: main`）。
日常烧录多用 Keil 的 F8 (Download)。

---

## 架构（三层 + driverlib）

```
应用控制层 Control/   control.c(PID+差速) · uart_callback.c(蓝牙协议) · show.c(OLED显示管理)
硬件抽象层 Hardware/  motor · encoder · oled · led · uart_debug · gyro · grayscale
板级支持层 Source/    board.c/.h(枢纽头·printf重定向·延时) · ti_msp_dl_config.c/.h(SysConfig生成) · empty.c(main)
底层              TI driverlib (include/ 头 + lib/driverlib.a) + CMSIS
```

**关键约定：**
- **`Source/board.h` 是全局枢纽头文件**，几乎所有 `.c` 都 `#include "board.h"`。新增模块头文件要在 `board.h` 里挂上。
- **`Source/ti_msp_dl_config.{c,h}` 由 SysConfig 生成，禁止手改**。改外设要回到 SysConfig 工程重新生成。
  SysConfig 源工程：`Source/empty.syscfg` 与 `syscfg/untitled.syscfg`（用 TI SysConfig v1.26.2 打开）。
- 命名规范（见 CODEREADME 附录 B）：函数 `模块_动作`（`Motor_SetPWM`）、宏全大写下划线（`WHEEL_PERIMETER`）、
  变量驼峰（`Target_Velocity`）；缩进 4 空格，不用 Tab。

**main 流程（`Source/empty.c`）**：`SYSCFG_DL_init()` → 外设初始化 → 主循环驱动方形轨迹状态机；
转向 PID 读 `Gyro.yaw`（`PID_Compute`），直行用编码器计数 `Encoder_GetCount(0)` 测距，输出 `Motor_SetPWM/Stop`。

## 物理参数
轮周长 210 mm、轮距 161 mm（见 `docs/接线说明.md`）。

---

## 参考资料（`参考资料/`，只读，不参与编译）

第三方/历史资料已统一收纳在 `参考资料/`，详见 `参考资料/README.md`：
- 电机驱动板 WHEELTEC C12H/D153C 厂商包、天猛星开发板厂商包；
- `参考资料/STM32参考工程/`：两份 **STM32** 工程（`car_control_stm32_project-master` 通用模板；`-51` 是 STM32H750+FreeRTOS 小车，疑似本工程前身）——**不是 MSPM0 目标，仅作算法移植参考**。
  注意 `-51` 目录名以 `-` 开头，命令行需用 `--` 或引号转义。

MCU 数据手册/用户手册/开发板原理图在 `docs/`。

## 仓库说明
本仓库**不是 git 仓库**（根目录无 `.git`）。`参考资料/STM32参考工程/-51/` 内部自带独立的 `.git`。
