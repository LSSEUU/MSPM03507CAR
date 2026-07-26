#include "uart_debug.h"
#include "reset_diag.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_TX_BUFFER_SIZE  1024U
#define DEBUG_RX_BUFFER_SIZE    64U

static DebugStats_t debug_stats;
static volatile uint8_t tx_buffer[DEBUG_TX_BUFFER_SIZE];
static volatile uint16_t tx_read;
static volatile uint16_t tx_write;
static volatile uint8_t rx_buffer[DEBUG_RX_BUFFER_SIZE];
static volatile uint8_t rx_read;
static volatile uint8_t rx_write;
static volatile uint32_t emergency_epoch;
static DebugEmergencyCallback_t emergency_callback;

static uint16_t Debug_TxNext(uint16_t index)
{
    return (uint16_t)((index + 1U) % DEBUG_TX_BUFFER_SIZE);
}

static uint8_t Debug_RxNext(uint8_t index)
{
    return (uint8_t)((index + 1U) % DEBUG_RX_BUFFER_SIZE);
}

static void Debug_FillTxFIFO(void)
{
    while ((tx_read != tx_write) && !DL_UART_isTXFIFOFull(UART_0_INST)) {
        DL_UART_transmitData(UART_0_INST, tx_buffer[tx_read]);
        tx_read = Debug_TxNext(tx_read);
        debug_stats.tx_bytes++;
    }
    if (tx_read == tx_write) {
        DL_UART_disableInterrupt(UART_0_INST, DL_UART_INTERRUPT_TX);
    }
}

static void Debug_DrainRxFIFO(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte = (uint8_t)DL_UART_receiveData(UART_0_INST);
        uint8_t next;
        debug_stats.rx_bytes++;
        if ((byte == (uint8_t)'S') || (byte == (uint8_t)' ')) {
            emergency_epoch++;
            if (emergency_callback != NULL) emergency_callback();
            continue;
        }
        next = Debug_RxNext(rx_write);
        if (next == rx_read) {
            debug_stats.rx_dropped++;
            continue;
        }
        rx_buffer[rx_write] = byte;
        rx_write = next;
    }
}

void debug_init(void)
{
    memset(&debug_stats, 0, sizeof(debug_stats));
    tx_read = tx_write = 0U;
    rx_read = rx_write = 0U;
    emergency_epoch = 0U;
    emergency_callback = NULL;
    DL_UART_enableInterrupt(UART_0_INST,
        DL_UART_INTERRUPT_RX |
        DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_FRAMING_ERROR |
        DL_UART_INTERRUPT_PARITY_ERROR |
        DL_UART_INTERRUPT_NOISE_ERROR);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

int debug_try_write(const char *data, size_t length)
{
    uint32_t primask;
    uint16_t used;
    uint16_t free_bytes;

    if ((data == NULL) || (length == 0U) ||
        (length >= DEBUG_TX_BUFFER_SIZE)) return 0;

    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TX_SUBMIT);
    primask = __get_PRIMASK();
    __disable_irq();
    used = (tx_write >= tx_read) ? (uint16_t)(tx_write - tx_read) :
        (uint16_t)(DEBUG_TX_BUFFER_SIZE - tx_read + tx_write);
    free_bytes = (uint16_t)(DEBUG_TX_BUFFER_SIZE - 1U - used);
    if (length > (size_t)free_bytes) {
        if (primask == 0U) __enable_irq();
        return 0;
    }
    for (size_t i = 0U; i < length; i++) {
        tx_buffer[tx_write] = (uint8_t)data[i];
        tx_write = Debug_TxNext(tx_write);
    }
    Debug_FillTxFIFO();
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_TX);
    if (primask == 0U) __enable_irq();
    return 1;
}

void debug_set_emergency_callback(DebugEmergencyCallback_t callback)
{
    emergency_callback = callback;
}

void debug_putc(char c)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t next;
    __disable_irq();
    next = Debug_TxNext(tx_write);
    if (next == tx_read) {
        debug_stats.tx_dropped++;
    } else {
        tx_buffer[tx_write] = (uint8_t)c;
        tx_write = next;
        Debug_FillTxFIFO();
        DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_TX);
    }
    if (primask == 0U) __enable_irq();
}

char debug_getc(void)
{
    uint32_t primask = __get_PRIMASK();
    char result = '\0';
    __disable_irq();
    if (rx_read != rx_write) {
        result = (char)rx_buffer[rx_read];
        rx_read = Debug_RxNext(rx_read);
    }
    if (primask == 0U) __enable_irq();
    return result;
}

void debug_print(const char *text)
{
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TX_SUBMIT);
    while (*text != '\0') debug_putc(*text++);
}

void debug_println(const char *text)
{
    debug_print(text);
    debug_newline();
}

void debug_newline(void)
{
    debug_putc('\r');
    debug_putc('\n');
}

void debug_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int length;

    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TX_ENCODE);
    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0 || length >= (int)sizeof(buffer)) {
        debug_stats.format_errors++;
    }
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TX_SUBMIT);
    for (int i = 0; i < length && i < (int)sizeof(buffer) - 1; i++) {
        debug_putc(buffer[i]);
    }
}

int debug_kbhit(void)
{
    return (rx_read != rx_write);
}

int debug_tx_idle(void)
{
    return (tx_read == tx_write) && !DL_UART_isBusy(UART_0_INST);
}

uint32_t debug_get_emergency_epoch(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t result;
    __disable_irq();
    result = emergency_epoch;
    if (primask == 0U) __enable_irq();
    return result;
}

void debug_get_stats(DebugStats_t *stats)
{
    if (stats != NULL) *stats = debug_stats;
}

void UART0_IRQHandler(void)
{
    uint32_t interrupt = DL_UART_getPendingInterrupt(UART_0_INST);
    while (interrupt != (uint32_t)DL_UART_IIDX_NO_INTERRUPT) {
        switch (interrupt) {
            case DL_UART_IIDX_RX:
                Debug_DrainRxFIFO();
                break;
            case DL_UART_IIDX_TX:
                Debug_FillTxFIFO();
                break;
            case DL_UART_IIDX_OVERRUN_ERROR:
            case DL_UART_IIDX_BREAK_ERROR:
            case DL_UART_IIDX_PARITY_ERROR:
            case DL_UART_IIDX_FRAMING_ERROR:
            case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            case DL_UART_IIDX_NOISE_ERROR:
                debug_stats.format_errors++;
                /* Any RX hardware error can invalidate an in-flight M6 frame.
                 * Count it as receive loss so the parser resets and tuning
                 * acceptance fails conservatively instead of using bad data. */
                debug_stats.rx_dropped++;
                Debug_DrainRxFIFO();
                break;
            default:
                break;
        }
        interrupt = DL_UART_getPendingInterrupt(UART_0_INST);
    }
}
