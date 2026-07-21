#include "uart_debug.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

static DebugStats_t debug_stats;

void debug_putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST));
    DL_UART_transmitData(UART_0_INST, (uint8_t)c);
    debug_stats.tx_bytes++;
}

char debug_getc(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_0_INST));
    debug_stats.rx_bytes++;
    return (char)DL_UART_receiveData(UART_0_INST);
}

void debug_print(const char *text)
{
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
    static char buffer[256];
    va_list args;

    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0 || length >= (int)sizeof(buffer)) {
        debug_stats.format_errors++;
    }
    for (int i = 0; i < length && i < (int)sizeof(buffer) - 1; i++) {
        debug_putc(buffer[i]);
    }
}

int debug_kbhit(void)
{
    return !DL_UART_isRXFIFOEmpty(UART_0_INST);
}

void debug_get_stats(DebugStats_t *stats)
{
    if (stats != NULL) *stats = debug_stats;
}
