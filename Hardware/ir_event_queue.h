#ifndef IR_EVENT_QUEUE_H
#define IR_EVENT_QUEUE_H

#include "ir_nec.h"

#include <stdbool.h>
#include <stdint.h>

#define IR_EVENT_QUEUE_CAPACITY 7U
#define IR_EVENT_QUEUE_STORAGE  8U

typedef struct {
    IrNecEvent_t entries[IR_EVENT_QUEUE_STORAGE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint32_t overflow;
    volatile uint32_t queued;
} IrEventQueue_t;

void IrEventQueue_Init(IrEventQueue_t *queue);
bool IrEventQueue_Push(IrEventQueue_t *queue, const IrNecEvent_t *event);
bool IrEventQueue_Pop(IrEventQueue_t *queue, IrNecEvent_t *event);

#endif
