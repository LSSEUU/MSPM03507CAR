#ifndef IR_RECEIVER_H
#define IR_RECEIVER_H

#include "ir_nec.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    IrNecStats_t nec;
    uint32_t queue_overflow;
    uint32_t queued_events;
} IrReceiverStats_t;

void IrReceiver_Init(void);
bool IrReceiver_Read(IrNecEvent_t *event);
void IrReceiver_Service(uint32_t now_ms);
void IrReceiver_GetStats(IrReceiverStats_t *stats);

#endif
