/*
 * led.h  —  LED 指示灯驱动
 * 参考: WHEELTEC_C07A_Bluetooth/Hardware/led.h
 */
#ifndef LED_H
#define LED_H
#include "board.h"

void LED_Init(void);                       /* 初始化 LED GPIO */
void LED_On(uint8_t index);                /* 点亮第 index 个 LED */
void LED_Off(uint8_t index);               /* 熄灭 */
void LED_Toggle(uint8_t index);            /* 翻转 */
void LED_Flash(uint16_t period_ms);        /* 闪烁任务（放定时器中断） */
uint8_t LED_GetConfiguredCount(void);
uint32_t LED_GetErrorCount(void);

#endif
