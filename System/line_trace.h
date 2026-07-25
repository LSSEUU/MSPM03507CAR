#ifndef LINE_TRACE_H
#define LINE_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_TRACE_CHANNEL_COUNT         8U
#define LINE_TRACE_CAPACITY             48U
#define LINE_TRACE_POST_TRIGGER_SAMPLES 36U

typedef struct {
    uint32_t time_ms;
    uint16_t adc[LINE_TRACE_CHANNEL_COUNT];
    int16_t position_x1000;
    int16_t correction_mm_s;
    uint8_t active_mask;
    uint8_t active_count;
    uint8_t cluster_count;
    uint8_t state;
    int8_t search_direction;
    uint8_t flags;
} LineTraceFrame_t;

typedef struct {
    uint8_t count;
    uint8_t capacity;
    uint8_t post_trigger_remaining;
    bool active;
    bool triggered;
    bool frozen;
} LineTraceInfo_t;

void LineTrace_Init(void);
void LineTrace_Start(void);
void LineTrace_Capture(const LineTraceFrame_t *frame, bool search_active);
void LineTrace_Freeze(void);
void LineTrace_GetInfo(LineTraceInfo_t *info);
bool LineTrace_GetFrame(uint8_t chronological_index, LineTraceFrame_t *frame);

#endif
