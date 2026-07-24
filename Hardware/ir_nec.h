#ifndef IR_NEC_H
#define IR_NEC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    IR_NEC_EVENT_NONE = 0,
    IR_NEC_EVENT_FULL,
    IR_NEC_EVENT_REPEAT
} IrNecEventType_t;

typedef struct {
    IrNecEventType_t type;
    uint8_t address;
    uint8_t command;
    uint32_t raw;
    uint32_t timestamp_ms;
} IrNecEvent_t;

typedef struct {
    uint32_t full_frames;
    uint32_t repeat_frames;
    uint32_t checksum_errors;
    uint32_t timing_errors;
    uint8_t last_address;
    uint8_t last_command;
    uint32_t last_raw;
    uint32_t last_full_ms;
    bool last_full_valid;
} IrNecStats_t;

typedef struct {
    uint8_t state;
    uint8_t bit_index;
    uint32_t raw;
    IrNecStats_t stats;
} IrNecDecoder_t;

void IrNec_Init(IrNecDecoder_t *decoder);
bool IrNec_ProcessEdge(IrNecDecoder_t *decoder, uint16_t elapsed_ticks,
                       bool new_level_high, uint32_t now_ms,
                       IrNecEvent_t *event);
void IrNec_Timeout(IrNecDecoder_t *decoder);
void IrNec_GetStats(const IrNecDecoder_t *decoder, IrNecStats_t *stats);

#endif
