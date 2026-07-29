#include "vision_uart.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define VISION_UART_RX_BUFFER_SIZE 256U
#define VISION_UART_TX_BUFFER_SIZE 256U

static volatile uint8_t rx_buffer[VISION_UART_RX_BUFFER_SIZE];
static volatile uint8_t tx_buffer[VISION_UART_TX_BUFFER_SIZE];
static volatile uint16_t rx_read;
static volatile uint16_t rx_write;
static volatile uint16_t tx_read;
static volatile uint16_t tx_write;
static volatile uint8_t receive_enabled;
static volatile VisionUartStats_t uart_stats;

static uint16_t RingNext(uint16_t index, uint16_t size)
{
    return (uint16_t)((index + 1U) % size);
}

static uint16_t RingQueued(uint16_t read, uint16_t write, uint16_t size)
{
    return (write >= read) ? (uint16_t)(write - read) :
        (uint16_t)(size - read + write);
}

static void FillTxFifo(void)
{
    while ((tx_read != tx_write) && !DL_UART_isTXFIFOFull(UART_1_INST)) {
        DL_UART_transmitData(UART_1_INST, tx_buffer[tx_read]);
        tx_read = RingNext(tx_read, VISION_UART_TX_BUFFER_SIZE);
        uart_stats.tx_bytes++;
    }
    if (tx_read == tx_write)
        DL_UART_disableInterrupt(UART_1_INST, DL_UART_INTERRUPT_TX);
}

static void DrainRxFifo(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t byte = (uint8_t)DL_UART_receiveData(UART_1_INST);
        uint16_t next;
        uart_stats.rx_bytes++;
        if (receive_enabled == 0U) continue;
        next = RingNext(rx_write, VISION_UART_RX_BUFFER_SIZE);
        if (next == rx_read) {
            uart_stats.rx_overflow++;
            continue;
        }
        rx_buffer[rx_write] = byte;
        rx_write = next;
    }
}

void VisionUart_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset((void *)&uart_stats, 0, sizeof(uart_stats));
    rx_read = rx_write = 0U;
    tx_read = tx_write = 0U;
    receive_enabled = 1U;
    DL_UART_enableInterrupt(UART_1_INST,
        DL_UART_INTERRUPT_RX |
        DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_FRAMING_ERROR |
        DL_UART_INTERRUPT_PARITY_ERROR |
        DL_UART_INTERRUPT_NOISE_ERROR);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    if (primask == 0U) __enable_irq();
}

void VisionUart_SetReceiveEnabled(uint8_t enabled)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    receive_enabled = (enabled != 0U) ? 1U : 0U;
    if (receive_enabled != 0U) {
        DL_UART_enableInterrupt(UART_1_INST,
            DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    } else {
        DL_UART_disableInterrupt(UART_1_INST,
            DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
        rx_read = rx_write = 0U;
        DrainRxFifo();
    }
    if (primask == 0U) __enable_irq();
}

size_t VisionUart_Read(uint8_t *data, size_t capacity)
{
    uint32_t primask;
    size_t length = 0U;
    if ((data == NULL) || (capacity == 0U)) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    while ((length < capacity) && (rx_read != rx_write)) {
        data[length++] = rx_buffer[rx_read];
        rx_read = RingNext(rx_read, VISION_UART_RX_BUFFER_SIZE);
    }
    if (primask == 0U) __enable_irq();
    return length;
}

size_t VisionUart_Write(const uint8_t *data, size_t length)
{
    uint32_t primask;
    size_t accepted = 0U;
    if ((data == NULL) || (length == 0U)) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    while (accepted < length) {
        uint16_t next = RingNext(tx_write, VISION_UART_TX_BUFFER_SIZE);
        if (next == tx_read) {
            uart_stats.tx_overflow += (uint32_t)(length - accepted);
            break;
        }
        tx_buffer[tx_write] = data[accepted++];
        tx_write = next;
    }
    FillTxFifo();
    if (tx_read != tx_write)
        DL_UART_enableInterrupt(UART_1_INST, DL_UART_INTERRUPT_TX);
    if (primask == 0U) __enable_irq();
    return accepted;
}

void VisionUart_GetStats(VisionUartStats_t *stats)
{
    uint32_t primask;
    if (stats == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *stats = uart_stats;
    stats->rx_queued = RingQueued(rx_read, rx_write,
                                  VISION_UART_RX_BUFFER_SIZE);
    stats->tx_queued = RingQueued(tx_read, tx_write,
                                  VISION_UART_TX_BUFFER_SIZE);
    if (primask == 0U) __enable_irq();
}

void UART1_IRQHandler(void)
{
    uint32_t interrupt = DL_UART_getPendingInterrupt(UART_1_INST);
    while (interrupt != (uint32_t)DL_UART_IIDX_NO_INTERRUPT) {
        switch (interrupt) {
            case DL_UART_IIDX_RX:
                DrainRxFifo();
                break;
            case DL_UART_IIDX_RX_TIMEOUT_ERROR:
                /* A timeout is the normal way to flush a partial FIFO when
                 * the threshold is above one byte; it is not a line error. */
                DrainRxFifo();
                break;
            case DL_UART_IIDX_TX:
                FillTxFifo();
                break;
            case DL_UART_IIDX_OVERRUN_ERROR:
            case DL_UART_IIDX_BREAK_ERROR:
            case DL_UART_IIDX_PARITY_ERROR:
            case DL_UART_IIDX_FRAMING_ERROR:
            case DL_UART_IIDX_NOISE_ERROR:
                uart_stats.hardware_errors++;
                DrainRxFifo();
                break;
            default:
                break;
        }
        interrupt = DL_UART_getPendingInterrupt(UART_1_INST);
    }
}
