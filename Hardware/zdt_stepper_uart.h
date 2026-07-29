#ifndef ZDT_STEPPER_UART_H
#define ZDT_STEPPER_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_overflow;
    uint32_t rx_overflow;
    uint32_t hardware_errors;
    uint32_t preempted_frames;
    uint16_t tx_queued;
    uint16_t rx_queued;
    uint16_t tx_high_water;
    uint16_t rx_high_water;
} ZdtStepperUartStats_t;

void ZdtStepperUart_Init(void);
/* Enqueue the complete frame or reject it without changing the TX queue. */
bool ZdtStepperUart_Write(const uint8_t *data, size_t length);
/*
 * Replace every not-yet-started normal frame with an emergency batch.  The
 * active frame is always allowed to finish, then the batch is transmitted in
 * order without byte-level truncation or interleaving.
 */
bool ZdtStepperUart_WriteEmergencyBatch(const uint8_t *const frames[],
                                        const uint8_t lengths[],
                                        size_t count);
size_t ZdtStepperUart_Read(uint8_t *data, size_t capacity);
void ZdtStepperUart_GetStats(ZdtStepperUartStats_t *stats);

#endif
