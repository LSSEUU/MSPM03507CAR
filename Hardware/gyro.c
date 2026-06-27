/*
 * gyro.c  —  JY61P 陀螺仪 (UART2 中断接收)
 */
#include "gyro.h"

Gyro_t Gyro;

/* 接收缓冲区 + 索引 */
static uint8_t  rx_buf[11];
static uint8_t  rx_idx;

void Gyro_Init(void)
{
    Gyro.yaw   = 0.0f;
    Gyro.pitch = 0.0f;
    Gyro.roll  = 0.0f;
    Gyro.updated = 0;
    rx_idx = 0;

    /* UART2 Extended 初始化 */
    DL_UART_Extend_reset(UART_2_INST);
    DL_UART_Extend_enablePower(UART_2_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_2_TX_IOMUX, GPIO_UART_2_TX_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_2_RX_IOMUX, GPIO_UART_2_RX_IOMUX_FUNC);

    {
        DL_UART_Extend_ClockConfig clk = { DL_UART_CLOCK_BUSCLK,
                                           DL_UART_CLOCK_DIVIDE_RATIO_1 };
        DL_UART_Extend_setClockConfig(UART_2_INST, &clk);
    }
    {
        DL_UART_Extend_Config cfg = { DL_UART_MODE_NORMAL,
                                       DL_UART_DIRECTION_TX_RX,
                                       DL_UART_FLOW_CONTROL_NONE,
                                       DL_UART_PARITY_NONE,
                                       DL_UART_WORD_LENGTH_8_BITS,
                                       DL_UART_STOP_BITS_ONE };
        DL_UART_Extend_init(UART_2_INST, &cfg);
    }
    DL_UART_Extend_setOversampling(UART_2_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Extend_configBaudRate(UART_2_INST, 32000000, 9600);
    DL_UART_Extend_enableFIFOs(UART_2_INST);

    /* 使能接收中断 */
    DL_UART_enableInterrupt(UART_2_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART2_INT_IRQn);

    DL_UART_Extend_enable(UART_2_INST);
}

void Gyro_Update(void)
{
    /* 中断里已更新 yaw/pitch/roll */
}

/* UART2 中断服务函数 */
void UART2_IRQHandler(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_2_INST)) {
        uint8_t b = (uint8_t)DL_UART_receiveData(UART_2_INST);

        /* 帧解析 */
        if (rx_idx == 0 && b != 0x55) continue;
        rx_buf[rx_idx++] = b;

        if (rx_idx >= 11) {
            rx_idx = 0;
            if (rx_buf[1] == 0x53) {
                uint8_t sum = 0;
                for (uint8_t i = 0; i < 10; i++) sum += rx_buf[i];
                if (sum == rx_buf[10]) {
                    int16_t roll  = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[2]);
                    int16_t pitch = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[4]);
                    int16_t yaw   = (int16_t)((uint16_t)rx_buf[7] << 8 | rx_buf[6]);
                    Gyro.roll  = (float)roll  * 180.0f / 32768.0f;
                    Gyro.pitch = (float)pitch * 180.0f / 32768.0f;
                    Gyro.yaw   = (float)yaw   * 180.0f / 32768.0f;
                    Gyro.updated = 1;
                }
            }
        }
    }
}
