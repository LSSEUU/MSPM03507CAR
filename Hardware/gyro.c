#include "gyro.h"
#include "board.h"
#include <stddef.h>
#include <string.h>

static Gyro_t gyro_sample;

static uint8_t  rx_buf[11];
static uint8_t  rx_idx;
static volatile uint32_t rx_bytes;
static volatile uint32_t valid_frames;
static volatile uint32_t checksum_errors;
static volatile uint32_t frame_errors;
static uint8_t recent_bytes[32];
static uint8_t recent_write;

static void Gyro_DrainRx(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_2_INST)) {
        uint8_t b = (uint8_t)DL_UART_receiveData(UART_2_INST);
        rx_bytes++;
        recent_bytes[recent_write] = b;
        recent_write = (uint8_t)((recent_write + 1U) % sizeof(recent_bytes));

        if (rx_idx == 0U && b != 0x55U) continue;
        rx_buf[rx_idx++] = b;

        if (rx_idx >= 11U) {
            uint8_t sum = 0U;
            rx_idx = 0U;
            if (rx_buf[0] != 0x55U) {
                frame_errors++;
                continue;
            }
            for (uint8_t i = 0U; i < 10U; i++) sum += rx_buf[i];
            if (sum != rx_buf[10]) {
                checksum_errors++;
            } else if (rx_buf[1] == 0x53U) {
                int16_t roll  = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[2]);
                int16_t pitch = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[4]);
                int16_t yaw   = (int16_t)((uint16_t)rx_buf[7] << 8 | rx_buf[6]);
                gyro_sample.roll  = (float)roll  * 180.0f / 32768.0f;
                gyro_sample.pitch = (float)pitch * 180.0f / 32768.0f;
                gyro_sample.yaw   = (float)yaw   * 180.0f / 32768.0f;
                valid_frames++;
            }
            /* Other valid JY61P frame types are expected and ignored. */
        }
    }
}

void Gyro_Init(void)
{
    memset(&gyro_sample, 0, sizeof(gyro_sample));
    rx_idx = 0;
    recent_write = 0U;
    memset(recent_bytes, 0, sizeof(recent_bytes));
    rx_bytes = valid_frames = checksum_errors = frame_errors = 0U;

    /* UART2 pinmux/baud/power are owned by the frozen board configuration. */
    DL_UART_enableInterrupt(UART_2_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART2_INT_IRQn);
}

void Gyro_Update(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Gyro_DrainRx();
    if (primask == 0U) __enable_irq();
}

/* UART2 中断服务函数 */
void UART2_IRQHandler(void)
{
    Gyro_DrainRx();
}

void Gyro_GetSnapshot(GyroSnapshot_t *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->sample = gyro_sample;
    snapshot->rx_bytes = rx_bytes;
    snapshot->valid_frames = valid_frames;
    snapshot->checksum_errors = checksum_errors;
    snapshot->frame_errors = frame_errors;
    snapshot->recent_count = (rx_bytes < sizeof(recent_bytes)) ?
        (uint8_t)rx_bytes : (uint8_t)sizeof(recent_bytes);
    for (uint8_t i = 0U; i < snapshot->recent_count; i++) {
        uint8_t oldest = (uint8_t)((recent_write + sizeof(recent_bytes) -
            snapshot->recent_count) % sizeof(recent_bytes));
        snapshot->recent_bytes[i] = recent_bytes[
            (uint8_t)((oldest + i) % sizeof(recent_bytes))];
    }
    if (primask == 0U) __enable_irq();
}

void Gyro_ProbePins(GyroPinProbe_t *probe)
{
    uint32_t previous;
    uint32_t current;
    if (probe == NULL) return;
    memset(probe, 0, sizeof(*probe));

    NVIC_DisableIRQ(UART2_INT_IRQn);
    DL_UART_Main_disable(UART_2_INST);
    DL_GPIO_initDigitalInput(GPIO_UART_2_TX_IOMUX);
    DL_GPIO_initDigitalInput(GPIO_UART_2_RX_IOMUX);
    previous = DL_GPIO_readPins(GPIOA, GPIO_UART_2_TX_PIN | GPIO_UART_2_RX_PIN);

    for (uint32_t i = 0U; i < 100000U; i++) {
        current = DL_GPIO_readPins(GPIOA, GPIO_UART_2_TX_PIN | GPIO_UART_2_RX_PIN);
        if (((current ^ previous) & GPIO_UART_2_TX_PIN) != 0U) probe->pa21_transitions++;
        if (((current ^ previous) & GPIO_UART_2_RX_PIN) != 0U) probe->pa22_transitions++;
        previous = current;
        delay_cycles(32U);
    }
    probe->pa21_level = ((previous & GPIO_UART_2_TX_PIN) != 0U) ? 1U : 0U;
    probe->pa22_level = ((previous & GPIO_UART_2_RX_PIN) != 0U) ? 1U : 0U;

    SYSCFG_DL_UART_2_init();
    rx_idx = 0U;
    DL_UART_enableInterrupt(UART_2_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART2_INT_IRQn);
}
