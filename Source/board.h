#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "ti_msp_dl_config.h"

typedef int32_t   s32;
typedef int16_t   s16;
typedef int8_t    s8;
typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

#define ABS(a)  ((a) > 0 ? (a) : (-(a)))

#define SYSTICK_MAX_COUNT  0xFFFFFF
uint32_t systick_get(void);

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif
