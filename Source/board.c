#include "board.h"
#include "uart_debug.h"
#include <stdbool.h>
#include <stdio.h>

int fputc(int ch, FILE *f)
{
    (void)f;
    debug_putc((char)ch);
    return ch;
}

uint32_t systick_get(void)
{
    return SysTick->VAL;
}

void delay_us(uint32_t us)
{
    if (us > SYSTICK_MAX_COUNT / (CPUCLK_FREQ / 1000000))
        us = SYSTICK_MAX_COUNT / (CPUCLK_FREQ / 1000000);
    uint32_t target = us * (CPUCLK_FREQ / 1000000);
    uint32_t start  = systick_get();
    uint32_t elapsed = 0;
    bool     wrapped = false;
    while (1) {
        uint32_t now = systick_get();
        if (now > start) wrapped = true;
        elapsed = wrapped ? (start + SYSTICK_PERIOD - now) : (start - now);
        if (elapsed >= target) break;
    }
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; i++) delay_us(1000U);
}
