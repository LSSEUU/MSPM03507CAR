
#include "uart_debug.h"
#include <stdio.h>      
#include <string.h>



void debug_putc(char c) {
    while (DL_UART_isBusy(UART_0_INST));
    DL_UART_transmitData(UART_0_INST, (uint8_t)c);
}

char debug_getc(void) {
    while (DL_UART_isRXFIFOEmpty(UART_0_INST));
    return (char)DL_UART_receiveData(UART_0_INST);
}



void debug_print(const char *s) {
    while (*s) debug_putc(*s++);
}

void debug_println(const char *s) {
    debug_print(s);
    debug_putc('\r');
    debug_putc('\n');
}

void debug_newline(void) {
    debug_putc('\r');
    debug_putc('\n');
}



void debug_printf(const char *fmt, ...) {
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    for (int i = 0; i < n && i < (int)sizeof(buf) - 1; i++) {
        debug_putc(buf[i]);
    }
}



int debug_kbhit(void) {
    return !DL_UART_isRXFIFOEmpty(UART_0_INST);
}
