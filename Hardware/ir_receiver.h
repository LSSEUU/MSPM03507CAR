#ifndef IR_RECEIVER_H
#define IR_RECEIVER_H

#include "ir_nec.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    IrNecStats_t nec;
    uint32_t queue_overflow;
    uint32_t queued_events;
    uint32_t capture_edges;
    uint32_t capture_high_edges;
    uint32_t capture_low_edges;
    uint16_t capture_min_ticks;
    uint16_t capture_max_ticks;
    uint16_t capture_last_ticks;
    bool pin_high;
} IrReceiverStats_t;

void IrReceiver_Init(void);
bool IrReceiver_Read(IrNecEvent_t *event);
void IrReceiver_Service(uint32_t now_ms);
void IrReceiver_GetStats(IrReceiverStats_t *stats);

#endif
