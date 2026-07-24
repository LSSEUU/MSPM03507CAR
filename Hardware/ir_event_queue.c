#include "ir_event_queue.h"

#include <string.h>

void IrEventQueue_Init(IrEventQueue_t *queue)
{
    if (queue == 0) return;
    memset(queue, 0, sizeof(*queue));
}

bool IrEventQueue_Push(IrEventQueue_t *queue, const IrNecEvent_t *event)
{
    uint8_t next;
    if ((queue == 0) || (event == 0)) return false;
    next = (uint8_t)((queue->head + 1U) % IR_EVENT_QUEUE_STORAGE);
    if (next == queue->tail) {
        queue->overflow++;
        return false;
    }
    queue->entries[queue->head] = *event;
    queue->head = next;
    queue->queued++;
    return true;
}

bool IrEventQueue_Pop(IrEventQueue_t *queue, IrNecEvent_t *event)
{
    if ((queue == 0) || (event == 0) ||
        (queue->tail == queue->head)) return false;
    *event = queue->entries[queue->tail];
    queue->tail =
        (uint8_t)((queue->tail + 1U) % IR_EVENT_QUEUE_STORAGE);
    return true;
}
