#include "zdt_stepper_uart.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define ZDT_STEPPER_UART_MAX_FRAME_SIZE 16U
#define ZDT_STEPPER_UART_TX_FRAME_COUNT 12U
#define ZDT_STEPPER_UART_URGENT_COUNT   2U
#define ZDT_STEPPER_UART_RX_BUFFER_SIZE 128U

typedef struct {
    uint8_t length;
    uint8_t data[ZDT_STEPPER_UART_MAX_FRAME_SIZE];
} TxFrame_t;

static volatile TxFrame_t tx_frames[ZDT_STEPPER_UART_TX_FRAME_COUNT];
static volatile TxFrame_t urgent_frames[ZDT_STEPPER_UART_URGENT_COUNT];
static volatile TxFrame_t active_frame;
static volatile uint8_t rx_buffer[ZDT_STEPPER_UART_RX_BUFFER_SIZE];
static volatile uint8_t tx_read;
static volatile uint8_t tx_write;
static volatile uint8_t tx_count;
static volatile uint8_t urgent_count;
static volatile uint8_t urgent_index;
static volatile uint8_t active_position;
static volatile bool active_valid;
static volatile uint16_t rx_read;
static volatile uint16_t rx_write;
static volatile ZdtStepperUartStats_t uart_stats;

static uint16_t RingNext(uint16_t index, uint16_t size)
{
    return (uint16_t)((index + 1U) % size);
}

static uint16_t RingQueued(uint16_t read, uint16_t write, uint16_t size)
{
    return (write >= read) ? (uint16_t)(write - read) :
        (uint16_t)(size - read + write);
}

static uint16_t TxQueuedBytes(void)
{
    uint16_t queued = active_valid ?
        (uint16_t)(active_frame.length - active_position) : 0U;
    uint8_t index;
    for (index = 0U; index < tx_count; index++) {
        uint8_t slot = (uint8_t)((tx_read + index) %
                                 ZDT_STEPPER_UART_TX_FRAME_COUNT);
        queued = (uint16_t)(queued + tx_frames[slot].length);
    }
    for (index = urgent_index; index < urgent_count; index++)
        queued = (uint16_t)(queued + urgent_frames[index].length);
    return queued;
}

static bool SelectNextFrame(void)
{
    uint8_t index;
    if (active_valid) return true;
    if (urgent_index < urgent_count) {
        active_frame.length = urgent_frames[urgent_index].length;
        for (index = 0U; index < active_frame.length; index++)
            active_frame.data[index] = urgent_frames[urgent_index].data[index];
        urgent_index++;
        if (urgent_index == urgent_count) urgent_index = urgent_count = 0U;
    } else if (tx_count != 0U) {
        active_frame.length = tx_frames[tx_read].length;
        for (index = 0U; index < active_frame.length; index++)
            active_frame.data[index] = tx_frames[tx_read].data[index];
        tx_read = (uint8_t)((tx_read + 1U) % ZDT_STEPPER_UART_TX_FRAME_COUNT);
        tx_count--;
    } else {
        return false;
    }
    active_position = 0U;
    active_valid = true;
    return true;
}

static void FillTxFifo(void)
{
    while (!DL_UART_isTXFIFOFull(UART_3_INST) && SelectNextFrame()) {
        DL_UART_transmitData(UART_3_INST,
                             active_frame.data[active_position++]);
        uart_stats.tx_bytes++;
        if (active_position == active_frame.length) active_valid = false;
    }
    if (!active_valid && (tx_count == 0U) && (urgent_count == 0U))
        DL_UART_disableInterrupt(UART_3_INST, DL_UART_INTERRUPT_TX);
}

static void DrainRxFifo(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_3_INST)) {
        uint8_t byte = (uint8_t)DL_UART_receiveData(UART_3_INST);
        uint16_t next = RingNext(rx_write, ZDT_STEPPER_UART_RX_BUFFER_SIZE);
        uart_stats.rx_bytes++;
        if (next == rx_read) {
            uart_stats.rx_overflow++;
            continue;
        }
        rx_buffer[rx_write] = byte;
        rx_write = next;
        {
            uint16_t queued = RingQueued(rx_read, rx_write,
                                         ZDT_STEPPER_UART_RX_BUFFER_SIZE);
            if (queued > uart_stats.rx_high_water)
                uart_stats.rx_high_water = queued;
        }
    }
}

void ZdtStepperUart_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset((void *)&uart_stats, 0, sizeof(uart_stats));
    tx_read = tx_write = tx_count = 0U;
    urgent_count = urgent_index = active_position = 0U;
    active_valid = false;
    rx_read = rx_write = 0U;
    DL_UART_enableInterrupt(UART_3_INST,
        DL_UART_INTERRUPT_RX |
        DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_FRAMING_ERROR |
        DL_UART_INTERRUPT_PARITY_ERROR |
        DL_UART_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
    if (primask == 0U) __enable_irq();
}

bool ZdtStepperUart_Write(const uint8_t *data, size_t length)
{
    uint32_t primask;
    size_t index;

    if ((data == NULL) || (length == 0U) ||
        (length > ZDT_STEPPER_UART_MAX_FRAME_SIZE)) return false;

    primask = __get_PRIMASK();
    __disable_irq();
    if (tx_count == ZDT_STEPPER_UART_TX_FRAME_COUNT) {
        uart_stats.tx_overflow++;
        if (primask == 0U) __enable_irq();
        return false;
    }

    tx_frames[tx_write].length = (uint8_t)length;
    for (index = 0U; index < length; index++)
        tx_frames[tx_write].data[index] = data[index];
    tx_write = (uint8_t)((tx_write + 1U) % ZDT_STEPPER_UART_TX_FRAME_COUNT);
    tx_count++;
    {
        uint16_t used = TxQueuedBytes();
        if (used > uart_stats.tx_high_water) uart_stats.tx_high_water = used;
    }
    FillTxFifo();
    if (active_valid || (tx_count != 0U) || (urgent_count != 0U))
        DL_UART_enableInterrupt(UART_3_INST, DL_UART_INTERRUPT_TX);
    if (primask == 0U) __enable_irq();
    return true;
}

bool ZdtStepperUart_WriteEmergencyBatch(const uint8_t *const frames[],
                                        const uint8_t lengths[],
                                        size_t count)
{
    uint32_t primask;
    size_t frame;
    size_t byte;
    if ((frames == NULL) || (lengths == NULL) || (count == 0U) ||
        (count > ZDT_STEPPER_UART_URGENT_COUNT)) return false;
    for (frame = 0U; frame < count; frame++) {
        if ((frames[frame] == NULL) || (lengths[frame] == 0U) ||
            (lengths[frame] > ZDT_STEPPER_UART_MAX_FRAME_SIZE)) return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    uart_stats.preempted_frames += tx_count;
    tx_read = tx_write = tx_count = 0U;
    urgent_index = 0U;
    urgent_count = (uint8_t)count;
    for (frame = 0U; frame < count; frame++) {
        urgent_frames[frame].length = lengths[frame];
        for (byte = 0U; byte < lengths[frame]; byte++)
            urgent_frames[frame].data[byte] = frames[frame][byte];
    }
    {
        uint16_t used = TxQueuedBytes();
        if (used > uart_stats.tx_high_water) uart_stats.tx_high_water = used;
    }
    FillTxFifo();
    if (active_valid || (urgent_count != 0U))
        DL_UART_enableInterrupt(UART_3_INST, DL_UART_INTERRUPT_TX);
    if (primask == 0U) __enable_irq();
    return true;
}

size_t ZdtStepperUart_Read(uint8_t *data, size_t capacity)
{
    uint32_t primask;
    size_t length = 0U;
    if ((data == NULL) || (capacity == 0U)) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    while ((length < capacity) && (rx_read != rx_write)) {
        data[length++] = rx_buffer[rx_read];
        rx_read = RingNext(rx_read, ZDT_STEPPER_UART_RX_BUFFER_SIZE);
    }
    if (primask == 0U) __enable_irq();
    return length;
}

void ZdtStepperUart_GetStats(ZdtStepperUartStats_t *stats)
{
    uint32_t primask;
    if (stats == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *stats = uart_stats;
    stats->tx_queued = TxQueuedBytes();
    stats->rx_queued = RingQueued(rx_read, rx_write,
                                  ZDT_STEPPER_UART_RX_BUFFER_SIZE);
    if (primask == 0U) __enable_irq();
}

void UART3_IRQHandler(void)
{
    uint32_t interrupt = DL_UART_getPendingInterrupt(UART_3_INST);
    while (interrupt != (uint32_t)DL_UART_IIDX_NO_INTERRUPT) {
        switch (interrupt) {
            case DL_UART_IIDX_RX:
            case DL_UART_IIDX_RX_TIMEOUT_ERROR:
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
        interrupt = DL_UART_getPendingInterrupt(UART_3_INST);
    }
}
