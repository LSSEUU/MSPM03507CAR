#include "line_trace.h"

#include <assert.h>
#include <stdint.h>

static LineTraceFrame_t make_frame(uint32_t time_ms)
{
    LineTraceFrame_t frame = {0};
    uint32_t i;
    frame.time_ms = time_ms;
    frame.position_x1000 = (int16_t)time_ms;
    for (i = 0U; i < LINE_TRACE_CHANNEL_COUNT; i++)
        frame.adc[i] = (uint16_t)(time_ms + i);
    return frame;
}

int main(void)
{
    LineTraceInfo_t info;
    LineTraceFrame_t frame;
    uint32_t i;

    LineTrace_Init();
    LineTrace_GetInfo(&info);
    assert(info.count == 0U);
    assert(!info.active);

    LineTrace_Start();
    for (i = 0U; i < 60U; i++) {
        frame = make_frame(i);
        LineTrace_Capture(&frame, false);
    }
    LineTrace_GetInfo(&info);
    assert(info.count == LINE_TRACE_CAPACITY);
    assert(info.active);
    assert(LineTrace_GetFrame(0U, &frame));
    assert(frame.time_ms == 12U);

    frame = make_frame(60U);
    LineTrace_Capture(&frame, true);
    for (i = 61U; i <= 96U; i++) {
        frame = make_frame(i);
        LineTrace_Capture(&frame, false);
    }
    LineTrace_GetInfo(&info);
    assert(info.triggered);
    assert(info.frozen);
    assert(!info.active);
    assert(info.post_trigger_remaining == 0U);
    assert(LineTrace_GetFrame(0U, &frame));
    assert(frame.time_ms == 49U);
    assert(LineTrace_GetFrame((uint8_t)(LINE_TRACE_CAPACITY - 1U), &frame));
    assert(frame.time_ms == 96U);

    frame = make_frame(97U);
    LineTrace_Capture(&frame, false);
    assert(LineTrace_GetFrame((uint8_t)(LINE_TRACE_CAPACITY - 1U), &frame));
    assert(frame.time_ms == 96U);
    assert(!LineTrace_GetFrame(LINE_TRACE_CAPACITY, &frame));

    LineTrace_Start();
    frame = make_frame(200U);
    LineTrace_Capture(&frame, false);
    LineTrace_Freeze();
    LineTrace_GetInfo(&info);
    assert(info.count == 1U);
    assert(info.frozen);
    return 0;
}
