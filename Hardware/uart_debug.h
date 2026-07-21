
#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdarg.h>


static inline void debug_init(void) { }

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t format_errors;
} DebugStats_t;


void debug_putc(char c);
char debug_getc(void);                   


void debug_print(const char *s);
void debug_println(const char *s);       
void debug_newline(void);                


void debug_printf(const char *fmt, ...)  __attribute__((format(printf,1,2)));


int  debug_kbhit(void);                  
void debug_get_stats(DebugStats_t *stats);

#endif
