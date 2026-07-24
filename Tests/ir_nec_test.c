#include "ir_event_queue.h"
#include "ir_nec.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

static bool edge(IrNecDecoder_t *decoder, uint32_t ticks, bool high,
                 uint32_t now_ms, IrNecEvent_t *event)
{
    return IrNec_ProcessEdge(decoder, (uint16_t)ticks, high, now_ms, event);
}

static bool send_frame(IrNecDecoder_t *decoder, uint32_t raw,
                       uint32_t now_ms, uint32_t mark_ticks,
                       uint32_t zero_ticks, uint32_t one_ticks,
                       IrNecEvent_t *event)
{
    uint32_t bit;
    bool produced = false;
    (void)edge(decoder, 0U, false, now_ms, event);
    (void)edge(decoder, 36000U, true, now_ms, event);
    (void)edge(decoder, 18000U, false, now_ms, event);
    for (bit = 0U; bit < 32U; bit++) {
        (void)edge(decoder, mark_ticks, true, now_ms, event);
        (void)edge(decoder,
            ((raw & (1UL << bit)) != 0U) ? one_ticks : zero_ticks,
            false, now_ms, event);
    }
    produced = edge(decoder, mark_ticks, true, now_ms, event);
    return produced;
}

static bool send_repeat(IrNecDecoder_t *decoder, uint32_t now_ms,
                        IrNecEvent_t *event)
{
    (void)edge(decoder, 0U, false, now_ms, event);
    (void)edge(decoder, 36000U, true, now_ms, event);
    (void)edge(decoder, 9000U, false, now_ms, event);
    return edge(decoder, 2240U, true, now_ms, event);
}

static void test_full_frame_lsb_and_boundaries(void)
{
    IrNecDecoder_t decoder;
    IrNecEvent_t event;
    IrNecStats_t stats;
    IrNec_Init(&decoder);
    assert(send_frame(&decoder, 0xFD02FF00UL, 100U,
        2240U, 2240U, 6760U, &event));
    assert(event.type == IR_NEC_EVENT_FULL);
    assert(event.address == 0x00U);
    assert(event.command == 0x02U);
    assert(event.raw == 0xFD02FF00UL);

    /* Inclusive lower timing boundaries. */
    assert(send_frame(&decoder, 0x9D62FF00UL, 200U,
        1600U, 1600U, 5200U, &event));
    assert(event.command == 0x62U);
    /* Inclusive upper timing boundaries. */
    assert(send_frame(&decoder, 0x57A8FF00UL, 300U,
        2800U, 3200U, 7600U, &event));
    assert(event.command == 0xA8U);
    IrNec_GetStats(&decoder, &stats);
    assert(stats.full_frames == 3U);
}

static void test_checksum_noise_truncation_and_repeat(void)
{
    IrNecDecoder_t decoder;
    IrNecEvent_t event;
    IrNecStats_t stats;
    IrNec_Init(&decoder);

    /* Noise and a truncated leader never produce a command. */
    assert(!edge(&decoder, 0U, false, 0U, &event));
    assert(!edge(&decoder, 1000U, true, 1U, &event));
    assert(!edge(&decoder, 0U, false, 2U, &event));
    IrNec_Timeout(&decoder);
    assert(!send_repeat(&decoder, 5U, &event));

    /* Command inverse is wrong. */
    assert(!send_frame(&decoder, 0xFC02FF00UL, 10U,
        2240U, 2240U, 6760U, &event));
    assert(send_frame(&decoder, 0x9D62FF00UL, 100U,
        2240U, 2240U, 6760U, &event));
    assert(send_repeat(&decoder, 280U, &event));
    assert(event.type == IR_NEC_EVENT_REPEAT);
    assert(event.command == 0x62U);
    /* Each accepted repeat renews the 180 ms inheritance window. */
    assert(send_repeat(&decoder, 460U, &event));
    assert(event.type == IR_NEC_EVENT_REPEAT);
    assert(event.command == 0x62U);
    assert(!send_repeat(&decoder, 641U, &event));

    IrNec_GetStats(&decoder, &stats);
    assert(stats.full_frames == 1U);
    assert(stats.repeat_frames == 2U);
    assert(stats.checksum_errors == 1U);
    assert(stats.timing_errors >= 2U);
}

static void test_queue_overflow(void)
{
    IrEventQueue_t queue;
    IrNecEvent_t event = { IR_NEC_EVENT_FULL, 0U, 0x62U, 0U, 0U };
    IrNecEvent_t output;
    uint32_t i;
    IrEventQueue_Init(&queue);
    for (i = 0U; i < IR_EVENT_QUEUE_CAPACITY; i++)
        assert(IrEventQueue_Push(&queue, &event));
    assert(!IrEventQueue_Push(&queue, &event));
    assert(queue.overflow == 1U);
    for (i = 0U; i < IR_EVENT_QUEUE_CAPACITY; i++)
        assert(IrEventQueue_Pop(&queue, &output));
    assert(!IrEventQueue_Pop(&queue, &output));
}

int main(void)
{
    test_full_frame_lsb_and_boundaries();
    test_checksum_noise_truncation_and_repeat();
    test_queue_overflow();
    return 0;
}
