#ifndef VISION_UART_H
#define VISION_UART_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_overflow;
    uint32_t tx_overflow;
    uint32_t hardware_errors;
    uint16_t rx_queued;
    uint16_t tx_queued;
} VisionUartStats_t;

/* UART1 transport only. No vision protocol is parsed at this layer. */
void VisionUart_Init(void);
void VisionUart_SetReceiveEnabled(uint8_t enabled);
size_t VisionUart_Read(uint8_t *data, size_t capacity);
size_t VisionUart_Write(const uint8_t *data, size_t length);
void VisionUart_GetStats(VisionUartStats_t *stats);

#endif
