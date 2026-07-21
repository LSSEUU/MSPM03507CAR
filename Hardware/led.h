#ifndef LED_H
#define LED_H

#include <stdint.h>

void LED_Init(void);
void LED_On(uint8_t index);
void LED_Off(uint8_t index);
void LED_Toggle(uint8_t index);
void LED_Flash(uint16_t period_ms);
uint8_t LED_GetConfiguredCount(void);
uint32_t LED_GetErrorCount(void);

#endif
