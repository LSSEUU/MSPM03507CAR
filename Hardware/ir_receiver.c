#include "ir_receiver.h"

#include "ir_event_queue.h"
#include "system_time.h"
#include "ti_msp_dl_config.h"

#define IR_RECEIVER_PARTIAL_TIMEOUT_MS    20U

static IrNecDecoder_t ir_decoder;
static IrEventQueue_t ir_queue;
static uint16_t ir_previous_capture;
static volatile uint32_t ir_last_edge_ms;
static volatile bool ir_capture_valid;

static void IrReceiver_QueueFromIsr(const IrNecEvent_t *event)
{
    (void)IrEventQueue_Push(&ir_queue, event);
}

void IrReceiver_Init(void)
{
    IrNec_Init(&ir_decoder);
    IrEventQueue_Init(&ir_queue);
    ir_previous_capture = 0U;
    ir_last_edge_ms = SystemTime_Now();
    ir_capture_valid = false;
    NVIC_ClearPendingIRQ(IR_CAPTURE_INST_INT_IRQN);
    NVIC_EnableIRQ(IR_CAPTURE_INST_INT_IRQN);
}

bool IrReceiver_Read(IrNecEvent_t *event)
{
    return IrEventQueue_Pop(&ir_queue, event);
}

void IrReceiver_Service(uint32_t now_ms)
{
    if (ir_capture_valid &&
        ((uint32_t)(now_ms - ir_last_edge_ms) >=
            IR_RECEIVER_PARTIAL_TIMEOUT_MS)) {
        NVIC_DisableIRQ(IR_CAPTURE_INST_INT_IRQN);
        if (ir_capture_valid &&
            ((uint32_t)(now_ms - ir_last_edge_ms) >=
                IR_RECEIVER_PARTIAL_TIMEOUT_MS)) {
            IrNec_Timeout(&ir_decoder);
            ir_capture_valid = false;
        }
        NVIC_EnableIRQ(IR_CAPTURE_INST_INT_IRQN);
    }
}

void IrReceiver_GetStats(IrReceiverStats_t *stats)
{
    if (stats == 0) return;
    NVIC_DisableIRQ(IR_CAPTURE_INST_INT_IRQN);
    IrNec_GetStats(&ir_decoder, &stats->nec);
    stats->queue_overflow = ir_queue.overflow;
    stats->queued_events = ir_queue.queued;
    NVIC_EnableIRQ(IR_CAPTURE_INST_INT_IRQN);
}

void IR_CAPTURE_INST_IRQHandler(void)
{
    DL_TIMER_IIDX interrupt =
        (DL_TIMER_IIDX)DL_TimerG_getPendingInterrupt(IR_CAPTURE_INST);
    if (interrupt == DL_TIMERG_IIDX_CC1_DN) {
        uint16_t capture = (uint16_t)DL_TimerG_getCaptureCompareValue(
            IR_CAPTURE_INST, IR_CAPTURE_CC_INDEX);
        bool level_high = (DL_GPIO_readPins(IR_CAPTURE_PORT,
            IR_CAPTURE_PIN) & IR_CAPTURE_PIN) != 0U;
        uint32_t now_ms = SystemTime_Now();
        uint16_t elapsed = (uint16_t)(ir_previous_capture - capture);
        IrNecEvent_t event;

        ir_last_edge_ms = now_ms;
        if (IrNec_ProcessEdge(&ir_decoder,
                ir_capture_valid ? elapsed : 0U,
                level_high, now_ms, &event)) {
            IrReceiver_QueueFromIsr(&event);
        }
        ir_previous_capture = capture;
        ir_capture_valid = true;
    }
}
