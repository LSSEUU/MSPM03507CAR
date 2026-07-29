#ifndef M6_VISION_UART_H
#define M6_VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

void M6VisionUart_Init(void);
void M6VisionUart_SetEnabled(bool enabled);
void M6VisionUart_GetStatus(uint8_t *payload, uint16_t *payload_length);
void M6VisionUart_Service(uint32_t now_ms);

#endif
