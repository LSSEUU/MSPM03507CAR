# MSPM03507CAR 项目代码解析文档

> 基于 TI MSPM0G3507 微控制器的两轮差速小车项目
> 开发板：WHEELTEC C07A / 立创·天猛星 MSPM0G3507
> 更新日期：2026-05-29

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件平台](#2-硬件平台)
3. [目录结构](#3-目录结构)
4. [系统架构](#4-系统架构)
5. [主程序流程](#5-主程序流程)
6. [模块详细解析](#6-模块详细解析)
   - 6.1 [Board 支持层](#61-board-支持层)
   - 6.2 [SysConfig 外设配置](#62-sysconfig-外设配置)
   - 6.3 [Debug UART 驱动](#63-debug-uart-驱动)
   - 6.4 [LED 驱动](#64-led-驱动)
   - 6.5 [电机驱动](#65-电机驱动)
   - 6.6 [编码器驱动](#66-编码器驱动)
   - 6.7 [OLED 显示驱动](#67-oled-显示驱动)
   - 6.8 [PID 控制逻辑](#68-pid-控制逻辑)
   - 6.9 [蓝牙协议解析](#69-蓝牙协议解析)
   - 6.10 [显示管理器](#610-显示管理器)
7. [数据流与控制链路](#7-数据流与控制链路)
8. [关键参数与常量](#8-关键参数与常量)
9. [构建与调试工具链](#9-构建与调试工具链)
10. [开发状态总览](#10-开发状态总览)
11. [待完成事项](#11-待完成事项)

---

## 1. 项目概述

本项目是一个**两轮差速驱动机器人小车**，使用 TI MSPM0G3507 (ARM Cortex-M0+) 微控制器，配合 WHEELTEC C07A 开发板进行开发。

**核心功能：**

- 双路直流电机驱动（TB6612/AT8236 双 H 桥）
- AB 相正交编码器测速
- 增量式 PI 速度闭环控制
- 差速运动学模型（线速度 + 角速度分解）
- 蓝牙 APP 遥控（WHEELTEC 协议）
- OLED 状态显示（SSD1306 128x64）

**技术栈：**

| 组件 | 版本/规格 |
|------|-----------|
| MCU | TI MSPM0G3507 (Cortex-M0+, 32 MHz) |
| SDK | MSPM0 SDK v2.09.00.01 |
| IDE | Keil MDK v5 (ARMClang 编译器) |
| 配置工具 | TI SysConfig v1.26.2 |
| 调试器 | OpenOCD + CMSIS-DAP (SWD) |
| 语言标准 | C11 |

---

## 2. 硬件平台

### 2.1 MCU 规格

```
型号:       MSPM0G3507
内核:       ARM Cortex-M0+
主频:       32 MHz
Flash:      128 KB  @ 0x00000000
SRAM:       32 KB   @ 0x20200000
```

### 2.2 引脚分配

```
功能          引脚     方向     说明
─────────────────────────────────────────────
UART0 TX      PA10     输出     调试串口发送 (9600 baud)
UART0 RX      PA11     输入     调试串口接收 (9600 baud)
UART1         TBD      -        蓝牙模块通信 (待配置)
电机 PWM      TBD      输出     TB6612/AT8236 PWM 输入
电机方向      TBD      输出     TB6612/AT8236 AIN1/AIN2, BIN1/BIN2
编码器 A相    TBD      输入     定时器编码器模式或 GPIO 中断
编码器 B相    TBD      输入     同上
OLED SCL      TBD      输出     I2C 时钟线
OLED SDA      TBD      双向     I2C 数据线
LED x4        TBD      输出     状态指示灯 (低电平有效)
```

### 2.3 小车物理参数

```
轮子周长 (WHEEL_PERIMETER):   0.210 m  (210 mm)
轮间距   (WHEEL_SPACING):     0.161 m  (161 mm)
控制频率 (CONTROL_FREQ):      200 Hz   (5 ms 周期)
```

---

## 3. 目录结构

```
MSPM03507CAR/
│
├── Source/                         # 核心系统文件
│   ├── empty.c                     # ★ 主入口 main()
│   ├── board.c                     #   板级支持: printf 重定向, 延时函数
│   ├── board.h                     #   全局头文件: 类型别名, 宏, 统一 include
│   ├── ti_msp_dl_config.c          #   [SysConfig 生成] 外设初始化代码
│   └── ti_msp_dl_config.h          #   [SysConfig 生成] 引脚映射, 寄存器定义
│
├── Hardware/                       # 硬件抽象层 (HAL)
│   ├── uart_debug.c / .h           #   调试串口驱动 (UART0)
│   ├── led.c / .h                  #   LED GPIO 驱动
│   ├── motor.c / .h                #   直流电机驱动 (PWM + 方向)
│   ├── encoder.c / .h              #   正交编码器驱动 (AB 相)
│   └── oled.c / .h                 #   SSD1306 OLED 驱动 (I2C)
│
├── Control/                        # 应用/控制层
│   ├── control.c / .h              #   ★ PID 速度闭环 + 差速运动学
│   ├── uart_callback.c / .h        #   蓝牙协议解析 (DMA + 状态机)
│   └── show.c / .h                 #   OLED 显示管理 + 蓝牙数据上报
│
├── keil/                           # Keil MDK 工程文件
│   ├── *.uvprojx                   #   工程文件
│   ├── *.uvoptx                    #   工程选项
│   └── mspm0g3507.sct              #   分散加载文件 (链接脚本)
│
├── syscfg/                         # TI SysConfig 配置
│   └── untitled.syscfg             #   外设配置工程
│
├── .vscode/                        # VS Code 配置
│   ├── c_cpp_properties.json       #   IntelliSense 配置
│   ├── launch.json                 #   调试配置 (OpenOCD)
│   ├── tasks.json                  #   构建任务 (Keil UV4)
│   ├── openocd.cfg                 #   OpenOCD 连接配置
│   └── mspm0g3507.cfg              #   OpenOCD 目标芯片定义
│
├── include/                        # TI SDK 头文件
├── lib/                            # TI DriverLib 预编译库
├── scripts/                        # 工具脚本
│   ├── build.bat                   #   命令行构建
│   └── ...
│
└── docs/                           # 文档资料
    ├── mspm0g3507数据手册.pdf
    ├── mspm0g3507用户手册.pdf
    └── ...
```

---

## 4. 系统架构

### 4.1 分层架构

```
┌─────────────────────────────────────────────────┐
│              应用控制层 (Control/)               │
│  ┌──────────┐  ┌──────────────┐  ┌──────────┐  │
│  │ control.c│  │uart_callback.c│  │  show.c  │  │
│  │ PID 闭环 │  │ 蓝牙协议解析  │  │ 显示管理 │  │
│  └────┬─────┘  └──────┬───────┘  └────┬─────┘  │
│       │               │               │         │
├───────┼───────────────┼───────────────┼─────────┤
│       │    硬件抽象层 (Hardware/)      │         │
│  ┌────┴──┐  ┌────┴──┐  ┌────┴──┐  ┌──┴───┐   │
│  │motor.c│  │encoder│  │ oled.c│  │ led.c│   │
│  │ PWM   │  │ AB相  │  │ I2C   │  │ GPIO │   │
│  └────┬──┘  └────┬──┘  └────┬──┘  └──┬───┘   │
│       │          │          │         │        │
├───────┼──────────┼──────────┼─────────┼────────┤
│       │    板级支持层 (Source/)        │        │
│  ┌────┴──────────┴──────────┴─────────┴────┐   │
│  │         board.c / board.h               │   │
│  │    printf 重定向 | 延时 | 类型定义       │   │
│  ├─────────────────────────────────────────┤   │
│  │     ti_msp_dl_config.c/.h               │   │
│  │    [SysConfig 生成] 外设寄存器初始化     │   │
│  └─────────────────────────────────────────┘   │
│                                                 │
├─────────────────────────────────────────────────┤
│            TI MSPM0 DriverLib + CMSIS           │
│         (include/ + lib/driverlib.a)            │
└─────────────────────────────────────────────────┘
```

### 4.2 头文件依赖关系

`board.h` 是**全局枢纽头文件**，所有模块都通过它获取公共依赖：

```
board.h
├── ti_msp_dl_config.h     ← SysConfig 生成的外设定义
├── uart_debug.h           ← 调试串口 API
├── led.h                  ← LED 驱动 API
│
│  (以下通过条件编译或注释控制)
├── oled.h                 ← OLED 显示 API
├── motor.h                ← 电机驱动 API
├── encoder.h              ← 编码器 API
├── control.h              ← 控制算法 API
├── show.h                 ← 显示管理 API
└── uart_callback.h        ← 蓝牙协议 API
```

### 4.3 模块间调用关系

```
main (empty.c)
 │
 ├── SYSCFG_DL_init()          ← 硬件初始化 (SysConfig 生成)
 │
 ├── debug_printf/println()    ← 启动信息输出
 │
 ├── [Control_Init()]          ← 控制系统初始化 (内部调用 Motor_Init + Encoder_Init)
 │
 └── 主循环 (10ms 周期)
     ├── debug_kbhit()         ← 检查串口输入
     ├── [BTBufferHandler()]   ← 蓝牙数据轮询 → bt_protocol() 解析指令
     └── [Control_Loop()]      ← PID 闭环控制
         ├── Encoder_GetCount()   ← 读取编码器
         ├── Incremental_PI()     ← PI 计算
         └── Motor_SetPWM()       ← 输出 PWM
```

---

## 5. 主程序流程

### 5.1 main() 函数执行序列

```c
// 文件: Source/empty.c

int main(void)
{
    // ===== 阶段 1: 系统初始化 =====
    SYSCFG_DL_init();          // SysConfig 生成的初始化
                                 // - 使能 GPIOA, GPIOB, UART0 时钟
                                 // - 配置 PA10=TX, PA11=RX
                                 // - 设置系统时钟 32 MHz
                                 // - 初始化 UART0 (9600 baud, 8N1)

    // ===== 阶段 2: 启动信息输出 =====
    debug_printf("Chip: MSPM0G3507\r\n");
    debug_printf("Clock: %d MHz\r\n", CPUCLK_FREQ / 1000000);
    debug_printf("UART0: PA10(TX) PA11(RX) 9600 baud\r\n");

    // ===== 阶段 3: 外设初始化 (当前已注释) =====
    // Control_Init();           // 内部调用 Motor_Init() + Encoder_Init()
    // OLED_Init();

    // ===== 阶段 4: 主循环 =====
    while (1)
    {
        // 当前生效: UART0 回显测试
        if (debug_kbhit()) {
            char c = debug_getc();
            debug_printf("Received: 0x%02X '%c'\r\n", c, c);
        }

        // 已注释: 蓝牙指令处理
        // BTBufferHandler();

        // 已注释: PID 控制环
        // Control_Loop();

        delay_ms(10);            // 10 ms 循环周期
    }
}
```

### 5.2 初始化时序图

```
上电/复位
    │
    ▼
SYSCFG_DL_init()
    ├── Power: 使能 GPIOA, GPIOB, UART0 时钟
    ├── SYSCTL: SYSOSC = 32 MHz, MCLK = 32 MHz
    ├── GPIO: PA10 → UART0_TX, PA11 → UART0_RX
    └── UART0: 9600 baud, 16x 过采样, IBRD=208, FBRD=21
    │
    ▼
启动 Banner 输出
    │
    ▼
[Control_Init()]          ← 当前已注释
    ├── Motor_Init()      ← PWM 定时器 + 方向 GPIO
    └── Encoder_Init()    ← 编码器定时器/中断
    │
    ▼
进入主循环 (10ms)
```

---

## 6. 模块详细解析

### 6.1 Board 支持层

**文件**: `Source/board.c`, `Source/board.h`

#### 6.1.1 类型定义

```c
// board.h 中定义的类型别名，简化嵌入式编程中的类型声明
typedef signed   int   s32;
typedef signed   short s16;
typedef signed   char  s8;
typedef unsigned int   u32;
typedef unsigned short u16;
typedef unsigned char  u8;

#define ABS(x) ((x) > 0 ? (x) : -(x))
```

#### 6.1.2 printf 重定向

标准 C 库的 `printf` 通过 `fputc` 重定向到调试串口，使得可以直接使用 `printf()` 输出到 UART0：

```c
// board.c
int fputc(int ch, FILE *f)
{
    debug_putc((char)ch);       // 将每个字符发送到 UART0
    return ch;
}
```

> **注意**: 此实现不依赖 Keil 的 MicroLIB，使用标准 C 库即可。

#### 6.1.3 延时函数

基于 SysTick 递减计数器实现微秒级延时：

```c
// 延时原理:
// SysTick 配置为 32 MHz 下的自由运行递减计数器
// 1 μs = 32 个时钟周期

void delay_us(uint32_t us)
{
    uint32_t start = SysTick->VAL;           // 读取当前计数值
    uint32_t ticks = us * (CPUCLK_FREQ / 1000000);  // 计算目标 tick 数
    // 等待计数器递减到目标值 (处理计数器回绕)
    while ((start - SysTick->VAL) < ticks);
}

void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}
```

---

### 6.2 SysConfig 外设配置

**文件**: `Source/ti_msp_dl_config.c`, `Source/ti_msp_dl_config.h`

> **注意**: 此文件由 TI SysConfig 工具自动生成，不建议手动修改。

#### 6.2.1 初始化函数

`SYSCFG_DL_init()` 完成以下配置：

| 配置项 | 具体内容 |
|--------|----------|
| 电源管理 | 使能 GPIOA, GPIOB, UART0 外设时钟 |
| GPIO | PA10 (PINCM21) → UART0 TX, PA11 (PINCM22) → UART0 RX |
| SYSCTL | SYSOSC 基频 32 MHz, MCLK 不分频, ULPCLK 不分频 |
| UART0 | 正常模式, TX+RX, 9600 baud, 16x 过采样 |

#### 6.2.2 波特率计算

```
目标波特率: 9600
参考时钟:   32 MHz (32,000,000 Hz)
过采样:     16x

BRD = 32,000,000 / (16 × 9600) = 208.333...

IBRD (整数部分) = 208
FBRD (小数部分) = round(0.333... × 64) = 21

实际波特率 = 32,000,000 / (16 × (208 + 21/64)) ≈ 9596.5 baud
误差: 0.036% (可接受)
```

---

### 6.3 Debug UART 驱动

**文件**: `Hardware/uart_debug.c`, `Hardware/uart_debug.h`

这是一个**完整可用**的调试串口驱动，提供阻塞式和非阻塞式的收发功能。

#### 6.3.1 API 函数

```c
// ========== 发送函数 ==========

// 阻塞发送单个字节
void debug_putc(char c)
{
    DL_UART_Main_transmitData(UART0_INST, c);  // 写入发送寄存器
    while (DL_UART_isBusy(UART0_INST));         // 等待发送完成
}

// 发送字符串 (不含换行)
void debug_print(const char *str);

// 发送字符串 (自动追加 \r\n)
void debug_println(const char *str);

// printf 风格格式化输出 (内部使用 256 字节缓冲区)
void debug_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);  // 格式化到缓冲区
    va_end(args);
    debug_print(buf);                         // 逐字节发送
}

// ========== 接收函数 ==========

// 阻塞接收单个字节 (轮询 RXFIFO)
char debug_getc(void)
{
    while (DL_UART_isRXFIFOEmpty(UART0_INST));  // 等待数据到达
    return (char)DL_UART_Main_receiveData(UART0_INST);
}

// 非阻塞检查是否有数据可读 (1=有数据, 0=无数据)
int debug_kbhit(void)
{
    return !DL_UART_isRXFIFOEmpty(UART0_INST);
}
```

#### 6.3.2 UART0 硬件框图

```
            ┌─────────────────────────────────┐
            │           UART0 模块            │
            │                                 │
 PA10 (TX) ─┤→ TX FIFO ←── [数据寄存器]      │
            │                                 │  ← debug_putc() 写入
            │   TX 状态: DL_UART_isBusy()     │
            │                                 │
 PA11 (RX) ─┤→ RX FIFO ──→ [数据寄存器]      │
            │                                 │  ← debug_getc()/debug_kbhit() 读取
            │   RX 状态: isRXFIFOEmpty()      │
            │                                 │
            │   波特率: 9600 (IBRD=208+FBRD=21)│
            │   过采样: 16x                   │
            └─────────────────────────────────┘
```

---

### 6.4 LED 驱动

**文件**: `Hardware/led.c`, `Hardware/led.h`

支持最多 4 个 LED，通过 GPIO DOUT 寄存器直接位操作控制（低电平点亮，active-low）。

#### 6.4.1 数据结构

```c
typedef struct {
    GPIO_Regs *port;    // GPIO 端口 (GPIOA / GPIOB)
    uint32_t   pin;     // 引脚掩码 (DL_GPIO_PIN_x)
} LED_t;

static const LED_t leds[] = {
    // {GPIOA, DL_GPIO_PIN_0},   // LED0 - 待填写实际引脚
    // {GPIOA, DL_GPIO_PIN_1},   // LED1 - 待填写实际引脚
};
```

#### 6.4.2 API 函数

```c
void LED_Init(void);                    // 配置 GPIO 输出, 默认高电平 (熄灭)
void LED_On(uint8_t index);             // 点亮指定 LED (拉低)
void LED_Off(uint8_t index);            // 熄灭指定 LED (拉高)
void LED_Toggle(uint8_t index);         // 翻转指定 LED 状态
void LED_Flash(uint32_t period_ms);     // 定时翻转 (需在定时器 ISR 中调用)
```

> **状态**: `leds[]` 引脚映射表为空，需要根据实际 PCB 原理图填写引脚。

---

### 6.5 电机驱动

**文件**: `Hardware/motor.c`, `Hardware/motor.h`

支持 TB6612 / AT8236 / L298N 双 H 桥驱动器，控制两路直流电机。

#### 6.5.1 数据结构

```c
typedef struct {
    int32_t  Current_Encoder;    // 当前编码器值 (速度反馈)
    int32_t  Motor_Pwm;          // 当前 PWM 输出值
    int32_t  Target_Encoder;     // 目标编码器值
    float    Velocity;           // 当前实际速度 (mm/s)
} MotorParam_t;

MotorParam_t MotorA;             // 左电机
MotorParam_t MotorB;             // 右电机
```

#### 6.5.2 API 函数

```c
void Motor_Init(void);                          // 初始化 PWM 定时器 + 方向 GPIO
void Motor_SetPWM(int32_t pwm_a, int32_t pwm_b); // 设置两路 PWM 占空比
void Motor_SetDir(uint8_t motor, uint8_t dir);   // 设置电机方向
void Motor_Stop(void);                           // 停止所有电机 (PWM=0)
```

#### 6.5.3 TB6612/AT8236 驱动逻辑

```
               ┌──────────────┐
               │   MCU (MSPM0)│
               │              │
    PWM_A ────┤→ PWMA        │
    AIN1  ────┤→ AIN1   TB6612│──→ 电机 A (左轮)
    AIN2  ────┤→ AIN2        │
               │              │
    PWM_B ────┤→ PWMB        │
    BIN1  ────┤→ BIN1        │──→ 电机 B (右轮)
    BIN2  ────┤→ BIN2        │
               └──────────────┘

    方向控制真值表:
    ┌──────┬──────┬────────┐
    │ AIN1 │ AIN2 │  动作  │
    ├──────┼──────┼────────┤
    │  0   │  0   │  制动  │
    │  1   │  0   │  正转  │
    │  0   │  1   │  反转  │
    │  1   │  1   │  制动  │
    └──────┴──────┴────────┘
    PWM 占空比控制转速
```

> **状态**: 所有硬件相关代码为 TODO 存根。

---

### 6.6 编码器驱动

**文件**: `Hardware/encoder.c`, `Hardware/encoder.h`

支持 AB 相正交编码器，用于测量电机转速。

#### 6.6.1 全局变量

```c
volatile int32_t EncoderA_Count = 0;   // 左轮编码器计数
volatile int32_t EncoderB_Count = 0;   // 右轮编码器计数
```

> 使用 `volatile` 修饰，因为编码器计数在中断中更新，在主循环中读取。

#### 6.6.2 API 函数

```c
void    Encoder_Init(void);                     // 初始化定时器编码器模式或 GPIO 中断
int32_t Encoder_GetCount(uint8_t channel);       // 读取编码器计数 (channel: 0=左, 1=右)
void    Encoder_ClearCount(uint8_t channel);     // 清零编码器计数
```

#### 6.6.3 编码器工作原理

```
         A 相           B 相
         ┌──┐   ┌──┐   ┌──┐   ┌──┐
    ─────┘  └───┘  └───┘  └───┘  └───
              ┌──┐   ┌──┐   ┌──┐   ┌──
    ──────────┘  └───┘  └───┘  └───┘

    正转: A 相超前 B 相 90° → 计数递增
    反转: B 相超前 A 相 90° → 计数递减

    实现方式 (二选一):
    1. 定时器编码器模式 (硬件自动计数，推荐)
    2. GPIO 外部中断 + 软件解码
```

> **状态**: 无实际硬件配置，所有函数为空实现。

---

### 6.7 OLED 显示驱动

**文件**: `Hardware/oled.c`, `Hardware/oled.h`

SSD1306 128×64 像素 OLED 显示屏驱动，I2C 接口，使用帧缓冲区管理显示内容。

#### 6.7.1 帧缓冲区

```c
static uint8_t OLED_Buffer[1024];  // 128 × 64 / 8 = 1024 字节
                                    // 每个 bit 对应一个像素
                                    // 8 个页面 (page)，每页 128 列 × 8 行
```

**帧缓冲区布局:**

```
Page 0: [0x000 - 0x07F]  → 像素行 0-7
Page 1: [0x080 - 0x0FF]  → 像素行 8-15
Page 2: [0x100 - 0x17F]  → 像素行 16-23
Page 3: [0x180 - 0x1FF]  → 像素行 24-31
Page 4: [0x200 - 0x27F]  → 像素行 32-39
Page 5: [0x280 - 0x2FF]  → 像素行 40-47
Page 6: [0x300 - 0x37F]  → 像素行 48-55
Page 7: [0x380 - 0x3FF]  → 像素行 56-63
```

#### 6.7.2 API 函数

```c
void OLED_Init(void);                            // 初始化 SSD1306 (发送初始化命令序列)
void OLED_Clear(void);                           // 清空帧缓冲区
void OLED_Refresh(void);                         // 将帧缓冲区数据刷新到屏幕
void OLED_SetPos(uint8_t x, uint8_t y);          // 设置光标位置 (页地址 + 列地址)
void OLED_DrawPoint(uint8_t x, uint8_t y);       // 在帧缓冲区中绘制一个像素
void OLED_ShowChar(uint8_t x, uint8_t y, char c);         // 显示单个字符
void OLED_ShowString(uint8_t x, uint8_t y, const char *s); // 显示字符串
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num);     // 显示数字
```

#### 6.7.3 SSD1306 I2C 通信协议

```
I2C 起始信号
    │
    ▼
设备地址: 0x78 (SA0=0) 或 0x7A (SA0=1)
    │
    ▼
控制字节:
    0x00 = 后续为命令字节
    0x40 = 后续为数据字节
    │
    ▼
数据字节 (命令或显示数据)
    │
    ▼
I2C 停止信号
```

> **状态**: I2C 通信函数 (`OLED_WriteCmd`, `OLED_WriteData`) 为空 TODO 存根，字库缺失。

---

### 6.8 PID 控制逻辑

**文件**: `Control/control.c`, `Control/control.h`

这是小车的**核心运动控制模块**，实现双轮独立增量式 PI 闭环控制。

#### 6.8.1 控制参数

```c
// 全局可调参数 (支持运行时修改)
float Velocity_KP = 0.5f;       // 速度环比例系数
float Velocity_KI = 0.02f;      // 速度环积分系数
float Turn_KP     = 0.3f;       // 转向环比例系数
float Turn_KI     = 0.01f;      // 转向环积分系数

// 目标值 (由蓝牙协议或上位机设定)
float Target_Velocity = 0.0f;   // 目标线速度 (mm/s)
float Target_Angular  = 0.0f;   // 目标角速度
```

#### 6.8.2 增量式 PI 控制器

```c
/**
 * 增量式 PI 控制器
 *
 * 增量式 vs 位置式:
 *   - 增量式输出的是"变化量"，而非绝对值
 *   - 无需积分项清零，切换时无冲击
 *   - 更适合电机控制场景
 *
 * @param target   目标值
 * @param current  当前实际值
 * @param kp       比例系数
 * @param ki       积分系数
 * @param integral 积分累加器 (调用者维护)
 * @return         PWM 增量
 */
int32_t Incremental_PI(int32_t target, int32_t current,
                       float kp, float ki, float *integral)
{
    float error = (float)(target - current);

    // 积分累加 (带抗积分饱和限幅)
    *integral += error;
    if (*integral >  500.0f) *integral =  500.0f;   // 上限
    if (*integral < -500.0f) *integral = -500.0f;   // 下限

    // 增量 = Kp × 误差 + Ki × 积分
    float increment = kp * error + ki * (*integral);

    return (int32_t)increment;
}
```

**PI 控制框图:**

```
                      ┌─────────────────────┐
  Target ──→ (⊕) ──→ │  Kp                 │
             ↑ -      │       ┌───┐         │──→ ΔPWM ──→ [累加] ──→ PWM 输出
  Current ──→         │  Ki ──│ Σ │ (积分)  │       ↑
                      │       └───┘         │       │
                      │  积分限幅: ±500     │       │
                      └─────────────────────┘       │
                                                    │
                                    Motor_SetPWM() ─┘
```

#### 6.8.3 差速运动学

将整车的目标线速度 `V` 和角速度 `ω` 分解为左右轮的独立速度目标：

```
已知:
  V = 目标线速度 (mm/s)
  ω = 目标角速度
  L = 轮间距 = 161 mm

计算:
  V_left  = V - ω × L/2    (左轮速度)
  V_right = V + ω × L/2    (右轮速度)

物理含义:
  - ω = 0:    直行,  V_left = V_right = V
  - ω > 0:    左转,  V_left < V_right
  - ω < 0:    右转,  V_left > V_right
  - V = 0:    原地旋转
```

#### 6.8.4 控制主循环

```c
// 建议调用频率: 200 Hz (5 ms 周期)
void Control_Loop(void)
{
    // 步骤 1: 读取编码器反馈
    int32_t encoder_left  = Encoder_GetCount(0);   // 左轮脉冲数
    int32_t encoder_right = Encoder_GetCount(1);   // 右轮脉冲数
    Encoder_ClearCount(0);
    Encoder_ClearCount(1);

    // 步骤 2: 解析遥控指令 → 更新目标值
    Get_RC();  // 从蓝牙标志位计算 Target_Velocity 和 Target_Angular

    // 步骤 3: 差速运动学分解
    float target_left  = Target_Velocity - Target_Angular * WHEEL_SPACING / 2.0f;
    float target_right = Target_Velocity + Target_Angular * WHEEL_SPACING / 2.0f;

    // 步骤 4: 双轮独立 PI 闭环
    static float integral_left  = 0.0f;
    static float integral_right = 0.0f;

    int32_t pwm_left  = Incremental_PI((int32_t)target_left,  encoder_left,
                                        Velocity_KP, Velocity_KI, &integral_left);
    int32_t pwm_right = Incremental_PI((int32_t)target_right, encoder_right,
                                        Velocity_KP, Velocity_KI, &integral_right);

    // 步骤 5: 输出到电机
    Motor_SetPWM(pwm_left, pwm_right);
}
```

**控制回路信号流:**

```
┌──────────┐    Target_Velocity    ┌──────────────┐
│ 蓝牙 APP │──────────────────────→│  Get_RC()    │
│ (遥控器) │    Target_Angular     │  目标解析    │
└──────────┘──────────────────────→└──────┬───────┘
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     ▼                     │
                    │  ┌─────────────────────────────────────┐  │
                    │  │        差速运动学分解                │  │
                    │  │  V_L = V - ω×L/2  V_R = V + ω×L/2  │  │
                    │  └────────────┬──────────┬─────────────┘  │
                    │               │          │                │
                    │        ┌──────┴──┐  ┌────┴────┐          │
                    │        │ PI (左) │  │ PI (右) │          │
                    │        │ Kp=0.5  │  │ Kp=0.5  │          │
                    │        │ Ki=0.02 │  │ Ki=0.02 │          │
                    │        └────┬────┘  └────┬────┘          │
                    │             │            │               │
                    │        PWM_L│       PWM_R│               │
                    │             ▼            ▼               │
                    │        ┌────────┐  ┌────────┐           │
                    │        │Motor A │  │Motor B │           │
                    │        │ (左轮) │  │ (右轮) │           │
                    │        └───┬────┘  └───┬────┘           │
                    │            │           │                │
                    │       ┌────┴──┐   ┌────┴──┐            │
                    │       │Encoder│   │Encoder│            │
                    │       │  A    │   │  B    │            │
                    │       └───┬───┘   └───┬───┘            │
                    │           │           │                │
                    │           └─────┬─────┘                │
                    │                 │                       │
                    │           反馈信号                      │
                    └─────────────────┘                       │
                                                              │
                        控制环路 (200 Hz) ◄──────────────────┘
```

---

### 6.9 蓝牙协议解析

**文件**: `Control/uart_callback.c`, `Control/uart_callback.h`

实现 WHEELTEC 蓝牙 APP 的通信协议解析，使用 DMA + 字节级状态机。

#### 6.9.1 通信架构

```
蓝牙 APP (手机)
    │
    ▼ (无线)
蓝牙模块 (HC-05/06)
    │
    ▼ (UART1, 9600 baud)
UART1 RX 引脚
    │
    ▼ (DMA Channel 0, 循环模式)
gBTPacket[200]  ← DMA 接收缓冲区
    │
    ▼ (BTBufferHandler() 轮询)
bt_protocol()   ← 字节级状态机解析
    │
    ▼ (设置控制标志)
Flag_Direction / Flag_Left / Flag_Right / Target_Velocity
```

#### 6.9.2 WHEELTEC APP 指令集

```
┌─────────┬──────────────┬──────────────────────────────┐
│ 指令码  │     含义     │           处理逻辑           │
├─────────┼──────────────┼──────────────────────────────┤
│  0x41   │   前进       │ Flag_Direction = 1 (前)      │
│  0x42   │   后退       │ Flag_Direction = 2 (后)      │
│  0x43   │   左转       │ Flag_Direction = 3 (左)      │
│  0x44   │   右转       │ Flag_Direction = 4 (右)      │
│  0x45   │   左前       │ Flag_Direction = 5 (左前)    │
│  0x46   │   右前       │ Flag_Direction = 6 (右前)    │
│  0x47   │   左后       │ Flag_Direction = 7 (左后)    │
│  0x48   │   右后       │ Flag_Direction = 8 (右后)    │
├─────────┼──────────────┼──────────────────────────────┤
│  0x49   │ 方向模式     │ Turn_Flag = 0 (方向键控制)   │
│  0x4A   │ 方向模式     │ Turn_Flag = 0                │
│  0x4B   │ 转向模式     │ Turn_Flag = 1 (左右键转向)   │
├─────────┼──────────────┼──────────────────────────────┤
│  0x58   │ 加速         │ Target_Velocity += 100 mm/s  │
│  0x59   │ 减速         │ Target_Velocity -= 100 mm/s  │
├─────────┼──────────────┼──────────────────────────────┤
│  0x7B   │ PID 调参     │ [TODO]                       │
│  0x7C   │ PID 调参     │ [TODO]                       │
│  0x7D   │ PID 调参     │ [TODO]                       │
└─────────┴──────────────┴──────────────────────────────┘
```

#### 6.9.3 状态机解析流程

```c
/**
 * bt_protocol - 字节级协议解析状态机
 *
 * WHEELTEC APP 数据帧格式:
 *   帧头(0xAA) + 指令(1字节) + 数据(N字节) + 帧尾(0x55)
 *
 * 状态机状态:
 *   STATE_IDLE  → 等待帧头 0xAA
 *   STATE_CMD   → 解析指令字节
 *   STATE_DATA  → 读取数据字段
 *   STATE_END   → 等待帧尾 0x55
 */
void bt_protocol(uint8_t recv)
{
    static uint8_t state = STATE_IDLE;

    switch (state) {
        case STATE_IDLE:
            if (recv == 0xAA) state = STATE_CMD;  // 检测帧头
            break;
        case STATE_CMD:
            parse_command(recv);                    // 解析指令
            state = STATE_DATA;
            break;
        // ...
    }
}
```

> **状态**: 协议解析逻辑完整，但 DMA 和 UART1 硬件配置代码已注释，需要配置 UART1 后启用。

---

### 6.10 显示管理器

**文件**: `Control/show.c`, `Control/show.h`

管理 OLED 显示内容和蓝牙数据上报。

#### 6.10.1 显示布局

```
┌────────────────────────────────┐  ← OLED 128×64
│ MSPM0G3507                     │  ← 标题行 (Page 0)
│                                │
│ Speed: 500 mm/s               │  ← 目标速度 (Page 2)
│                                │
│ ...                            │  ← 可扩展区域
│                                │
└────────────────────────────────┘
```

#### 6.10.2 API 函数

```c
void Show_Init(void);     // 调用 OLED_Init()
void oled_show(void);     // 更新 OLED 显示内容
void APP_Show(void);      // 通过蓝牙发送状态数据 (TODO)
```

---

## 7. 数据流与控制链路

### 7.1 完整数据流图

```
                        ┌───────────────────┐
                        │  蓝牙 APP (手机)  │
                        │  WHEELTEC 协议    │
                        └────────┬──────────┘
                                 │ 无线
                                 ▼
                        ┌───────────────────┐
                        │   蓝牙模块        │
                        │   (HC-05/06)      │
                        └────────┬──────────┘
                                 │ UART1
                                 ▼
┌────────────────────────────────────────────────────────────┐
│                    MSPM0G3507 MCU                          │
│                                                            │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │ DMA (CH0)   │───→│ BTBuffer     │───→│ bt_protocol  │  │
│  │ UART1 RX    │    │ Handler      │    │ 状态机解析   │  │
│  └─────────────┘    └──────────────┘    └──────┬───────┘  │
│                                                 │          │
│                         设置标志位              │          │
│                    ┌────────────────────────────┘          │
│                    ▼                                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                   Get_RC()                           │  │
│  │  Flag_Direction → Target_Velocity / Target_Angular   │  │
│  └──────────────────────────┬───────────────────────────┘  │
│                              │                             │
│                              ▼                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Control_Loop() (200Hz)                  │  │
│  │                                                      │  │
│  │  差速分解:  V_L = V - ω×L/2                         │  │
│  │             V_R = V + ω×L/2                         │  │
│  │                                                      │  │
│  │  PI 控制:   PWM_L = Incremental_PI(V_L, enc_L)     │  │
│  │             PWM_R = Incremental_PI(V_R, enc_R)     │  │
│  └────────────┬──────────────────┬─────────────────────┘  │
│               │                  │                         │
│               ▼                  ▼                         │
│  ┌─────────────────┐  ┌─────────────────┐                 │
│  │  Motor_SetPWM   │  │  Motor_SetPWM   │                 │
│  │  (左轮 PWM)     │  │  (右轮 PWM)     │                 │
│  └────────┬────────┘  └────────┬────────┘                 │
│           │                    │                           │
│           ▼                    ▼                           │
│  ┌─────────────────┐  ┌─────────────────┐                 │
│  │ TB6612 + MotorA │  │ TB6612 + MotorB │                 │
│  └────────┬────────┘  └────────┬────────┘                 │
│           │                    │                           │
│           ▼                    ▼                           │
│  ┌─────────────────┐  ┌─────────────────┐                 │
│  │ Encoder A (左轮) │  │ Encoder B (右轮) │                 │
│  └────────┬────────┘  └────────┬────────┘                 │
│           │                    │                           │
│           └────────┬───────────┘                           │
│                    │                                       │
│                    ▼                                       │
│           编码器反馈计数                                    │
│           (Control_Loop 读取)                              │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### 7.2 定时任务分配

```
┌──────────────┬───────────┬───────────────────────────────┐
│    任务      │  周期     │           说明                │
├──────────────┼───────────┼───────────────────────────────┤
│ Control_Loop │  5 ms     │ PID 闭环控制 (200 Hz)         │
│ BTBuffer     │  10 ms    │ 蓝牙 DMA 数据轮询             │
│ oled_show    │  100 ms   │ OLED 显示刷新 (10 Hz)         │
│ LED_Flash    │  500 ms   │ LED 状态指示闪烁              │
│ APP_Show     │  200 ms   │ 蓝牙状态上报 (5 Hz)           │
└──────────────┴───────────┴───────────────────────────────┘
```

---

## 8. 关键参数与常量

### 8.1 系统参数

| 参数 | 值 | 定义位置 |
|------|-----|----------|
| CPU 时钟 | 32 MHz | `ti_msp_dl_config.h` |
| Flash 大小 | 128 KB | 链接脚本 |
| RAM 大小 | 32 KB | 链接脚本 |
| UART0 波特率 | 9600 | `ti_msp_dl_config.h` |
| 调试缓冲区 | 256 字节 | `uart_debug.c` |

### 8.2 小车物理参数

| 参数 | 符号 | 值 | 单位 | 定义位置 |
|------|------|-----|------|----------|
| 轮子周长 | `WHEEL_PERIMETER` | 0.210 | m | `control.h` |
| 轮间距 | `WHEEL_SPACING` | 0.161 | m | `control.h` |
| 控制频率 | `CONTROL_FREQ` | 200 | Hz | `control.h` |

### 8.3 PID 参数

| 参数 | 初始值 | 范围 | 说明 |
|------|--------|------|------|
| `Velocity_KP` | 0.5 | 可调 | 速度环比例增益 |
| `Velocity_KI` | 0.02 | 可调 | 速度环积分增益 |
| `Turn_KP` | 0.3 | 可调 | 转向环比例增益 |
| `Turn_KI` | 0.01 | 可调 | 转向环积分增益 |
| 积分限幅 | ±500 | 固定 | 防止积分饱和 |

### 8.4 蓝牙缓冲区

| 参数 | 值 | 说明 |
|------|-----|------|
| `gBTPacket` | 200 字节 | DMA 环形接收缓冲区 |
| 速度调节步长 | 100 mm/s | 加速/减速每次变化量 |

---

## 9. 构建与调试工具链

### 9.1 构建方式

**方式一: VS Code 任务**

```bash
# 在 VS Code 中按 Ctrl+Shift+B 运行构建任务
# 实际执行:
"D:/keil_5/UV4/UV4.exe" -b "keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx" -o build.log
```

**方式二: 命令行脚本**

```bash
scripts/build.bat
```

### 9.2 调试配置

```
调试器:       OpenOCD
适配器:       CMSIS-DAP
传输协议:     SWD (SWDIO + SWCLK)
适配器速度:   100 kHz
目标芯片:     MSPM0G3507
烧录文件:     keil/Objects/empty_LP_MSPM0G3507_nortos_keil.axf
```

**OpenOCD 配置** (`.vscode/openocd.cfg`):
```tcl
source [find interface/cmsis-dap.cfg]
transport select swd
adapter speed 100
source [find target/mspm0g3507.cfg]
```

### 9.3 内存映射

```
┌──────────────────────────────────────┐ 0x00000000
│                                      │
│           Flash (128 KB)             │
│         代码 + 常量数据              │
│                                      │
├──────────────────────────────────────┤ 0x00020000
│         (保留)                       │
├──────────────────────────────────────┤ 0x20200000
│                                      │
│            SRAM (32 KB)              │
│      堆 + 栈 + 全局变量             │
│                                      │
├──────────────────────────────────────┤ 0x20208000
│         (保留)                       │
├──────────────────────────────────────┤ 0x41C00000
│       BCR 配置区                     │
├──────────────────────────────────────┤ 0x41C00100
│       BSL 配置区                     │
└──────────────────────────────────────┘
```

---

## 10. 开发状态总览

| 模块 | 文件 | 状态 | 说明 |
|------|------|------|------|
| SysConfig 初始化 | `ti_msp_dl_config.c` | ✅ 完成 | UART0 + GPIO + 时钟配置 |
| 调试串口 | `uart_debug.c` | ✅ 完成 | 完整的收发 + printf 支持 |
| UART0 回显 | `empty.c` | ✅ 完成 | 主循环中回显收到的字符 |
| printf 重定向 | `board.c` | ✅ 完成 | fputc → debug_putc |
| 延时函数 | `board.c` | ✅ 完成 | SysTick 微秒级延时 |
| PID 控制算法 | `control.c` | ✅ 逻辑完成 | 增量式 PI + 差速分解 |
| 蓝牙协议解析 | `uart_callback.c` | ✅ 逻辑完成 | WHEELTEC APP 状态机 |
| LED 驱动 | `led.c` | ⚠️ 骨架 | API 定义，引脚表待填写 |
| 电机驱动 | `motor.c` | ⚠️ 骨架 | API 定义，PWM + GPIO 待配置 |
| 编码器驱动 | `encoder.c` | ⚠️ 骨架 | API 定义，定时器模式待配置 |
| OLED 驱动 | `oled.c` | ⚠️ 骨架 | 帧缓冲 + API，I2C 通信待实现 |
| 蓝牙 DMA | `uart_callback.c` | ⚠️ 已注释 | UART1 + DMA 待配置 |
| 显示管理 | `show.c` | ⚠️ 骨架 | 基本框架，待完善 |

---

## 11. 待完成事项

### 优先级 P0 (核心功能)

- [ ] **电机驱动**: 配置 PWM 定时器，填写 TB6612/AT8236 方向控制 GPIO
- [ ] **编码器驱动**: 配置定时器编码器模式或 GPIO 中断解码
- [ ] **启用控制环路**: 取消 `main()` 中 `Control_Loop()` 的注释

### 优先级 P1 (通信功能)

- [ ] **蓝牙 DMA**: 配置 UART1 引脚和波特率，启用 DMA 循环接收
- [ ] **启用蓝牙处理**: 取消 `BTBufferHandler()` 的注释
- [ ] **PID 调参协议**: 实现 0x7B-0x7D 指令的 PID 参数在线调整

### 优先级 P2 (显示与指示)

- [ ] **LED 引脚**: 根据原理图填写 `leds[]` 引脚映射表
- [ ] **OLED I2C**: 实现 `OLED_WriteCmd()` 和 `OLED_WriteData()` 的 I2C 通信
- [ ] **字库**: 添加 16×8 或 12×6 ASCII 字库数据
- [ ] **OLED 内容**: 完善 `oled_show()` 显示速度、编码器、PID 状态等信息

### 优先级 P3 (高级功能)

- [ ] **APP 数据上报**: 实现 `APP_Show()` 通过蓝牙回传速度和状态
- [ ] **转向控制**: 完善 `Get_RC()` 中方向标志到角速度的映射
- [ ] **安全保护**: 添加电机堵转检测、编码器异常处理
- [ ] **参数持久化**: 将 PID 参数存储到 Flash 或 EEPROM

---

## 附录 A: 文件快速索引

| 功能 | 头文件 | 源文件 |
|------|--------|--------|
| 主入口 | - | `Source/empty.c` |
| 板级支持 | `Source/board.h` | `Source/board.c` |
| SysConfig | `Source/ti_msp_dl_config.h` | `Source/ti_msp_dl_config.c` |
| 调试串口 | `Hardware/uart_debug.h` | `Hardware/uart_debug.c` |
| LED | `Hardware/led.h` | `Hardware/led.c` |
| 电机 | `Hardware/motor.h` | `Hardware/motor.c` |
| 编码器 | `Hardware/encoder.h` | `Hardware/encoder.c` |
| OLED | `Hardware/oled.h` | `Hardware/oled.c` |
| 控制 | `Control/control.h` | `Control/control.c` |
| 蓝牙 | `Control/uart_callback.h` | `Control/uart_callback.c` |
| 显示 | `Control/show.h` | `Control/show.c` |

## 附录 B: 编码规范

- **缩进**: 4 空格，不使用 Tab
- **命名**:
  - 函数: `模块名_动作` (如 `Motor_SetPWM`, `Encoder_GetCount`)
  - 宏/常量: 全大写下划线 (如 `WHEEL_PERIMETER`)
  - 变量: 驼峰命名 (如 `Target_Velocity`, `MotorA`)
  - 类型: 驼峰 + `_t` 后缀 (如 `MotorParam_t`)
- **注释**: 使用 `//` 行注释为主，函数头部使用 `/** */` 文档注释
- **头文件保护**: 使用 `#ifndef` + `#define` + `#endif` 模式
