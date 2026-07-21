#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define SYSTICK_PERIOD     (CPUCLK_FREQ / 1000U)
#define SYSTICK_MAX_COUNT  (SYSTICK_PERIOD - 1U)

uint32_t systick_get(void);

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif
