#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define SYSTICK_MAX_COUNT  0xFFFFFFU
#define SYSTICK_PERIOD     (SYSTICK_MAX_COUNT + 1U)

uint32_t systick_get(void);

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif
