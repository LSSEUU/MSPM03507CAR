#include "line_trace.h"

#include <stddef.h>
#include <string.h>

static LineTraceFrame_t trace_frames[LINE_TRACE_CAPACITY];
static uint8_t trace_write_index;
static uint8_t trace_count;
static uint8_t trace_post_trigger_remaining;
static bool trace_active;
static bool trace_triggered;
static bool trace_frozen;

void LineTrace_Init(void)
{
    memset(trace_frames, 0, sizeof(trace_frames));
    trace_write_index = 0U;
    trace_count = 0U;
    trace_post_trigger_remaining = 0U;
    trace_active = false;
    trace_triggered = false;
    trace_frozen = false;
}

void LineTrace_Start(void)
{
    trace_write_index = 0U;
    trace_count = 0U;
    trace_post_trigger_remaining = 0U;
    trace_active = true;
    trace_triggered = false;
    trace_frozen = false;
}

void LineTrace_Capture(const LineTraceFrame_t *frame, bool search_active)
{
    if ((frame == NULL) || !trace_active || trace_frozen) return;

    trace_frames[trace_write_index] = *frame;
    trace_write_index++;
    if (trace_write_index >= LINE_TRACE_CAPACITY) trace_write_index = 0U;
    if (trace_count < LINE_TRACE_CAPACITY) trace_count++;

    if (!trace_triggered && search_active) {
        trace_triggered = true;
        trace_post_trigger_remaining = LINE_TRACE_POST_TRIGGER_SAMPLES;
    } else if (trace_triggered && (trace_post_trigger_remaining > 0U)) {
        trace_post_trigger_remaining--;
        if (trace_post_trigger_remaining == 0U) {
            trace_active = false;
            trace_frozen = true;
        }
    }
}

void LineTrace_Freeze(void)
{
    trace_active = false;
    trace_frozen = (trace_count != 0U);
}

void LineTrace_GetInfo(LineTraceInfo_t *info)
{
    if (info == NULL) return;
    info->count = trace_count;
    info->capacity = LINE_TRACE_CAPACITY;
    info->post_trigger_remaining = trace_post_trigger_remaining;
    info->active = trace_active;
    info->triggered = trace_triggered;
    info->frozen = trace_frozen;
}

bool LineTrace_GetFrame(uint8_t chronological_index, LineTraceFrame_t *frame)
{
    uint8_t oldest;
    uint8_t physical_index;
    if ((frame == NULL) || (chronological_index >= trace_count)) return false;
    oldest = (trace_count < LINE_TRACE_CAPACITY) ? 0U : trace_write_index;
    physical_index = (uint8_t)(oldest + chronological_index);
    if (physical_index >= LINE_TRACE_CAPACITY)
        physical_index = (uint8_t)(physical_index - LINE_TRACE_CAPACITY);
    *frame = trace_frames[physical_index];
    return true;
}
