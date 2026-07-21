#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdint.h>

typedef void (*DebugEmergencyCallback_t)(void);

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t format_errors;
    uint32_t tx_dropped;
    uint32_t rx_dropped;
} DebugStats_t;

void debug_init(void);
void debug_set_emergency_callback(DebugEmergencyCallback_t callback);
void debug_putc(char c);
char debug_getc(void);
void debug_print(const char *text);
void debug_println(const char *text);
void debug_newline(void);
void debug_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
int debug_kbhit(void);
int debug_tx_idle(void);
void debug_get_stats(DebugStats_t *stats);

#endif
