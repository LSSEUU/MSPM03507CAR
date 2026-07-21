/*
 * gyro.c  —  JY61P 陀螺仪 (UART2 中断接收)
 */
#include "gyro.h"

Gyro_t Gyro;

/* 接收缓冲区 + 索引 */
static uint8_t  rx_buf[11];
static uint8_t  rx_idx;
static volatile uint32_t rx_bytes;
static volatile uint32_t valid_frames;
static volatile uint32_t checksum_errors;
static volatile uint32_t frame_errors;

void Gyro_Init(void)
{
    Gyro.yaw   = 0.0f;
    Gyro.pitch = 0.0f;
    Gyro.roll  = 0.0f;
    Gyro.updated = 0;
    rx_idx = 0;
    rx_bytes = valid_frames = checksum_errors = frame_errors = 0U;

    /* UART2 pinmux/baud/power are owned by the frozen board configuration. */
    DL_UART_enableInterrupt(UART_2_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART2_INT_IRQn);
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
        rx_bytes++;

        /* 帧解析 */
        if (rx_idx == 0 && b != 0x55) continue;
        rx_buf[rx_idx++] = b;

        if (rx_idx >= 11) {
            rx_idx = 0;
            if (rx_buf[0] != 0x55U) {
                frame_errors++;
            } else {
                uint8_t sum = 0;
                for (uint8_t i = 0; i < 10; i++) sum += rx_buf[i];
                if (sum != rx_buf[10]) {
                    checksum_errors++;
                } else if (rx_buf[1] == 0x53U) {
                    int16_t roll  = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[2]);
                    int16_t pitch = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[4]);
                    int16_t yaw   = (int16_t)((uint16_t)rx_buf[7] << 8 | rx_buf[6]);
                    Gyro.roll  = (float)roll  * 180.0f / 32768.0f;
                    Gyro.pitch = (float)pitch * 180.0f / 32768.0f;
                    Gyro.yaw   = (float)yaw   * 180.0f / 32768.0f;
                    Gyro.updated = 1;
                    valid_frames++;
                }
                /* Other valid JY61P frame types are expected and ignored. */
            }
        }
    }
}

void Gyro_GetSnapshot(GyroSnapshot_t *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->sample = Gyro;
    snapshot->rx_bytes = rx_bytes;
    snapshot->valid_frames = valid_frames;
    snapshot->checksum_errors = checksum_errors;
    snapshot->frame_errors = frame_errors;
    if (primask == 0U) __enable_irq();
}
