
#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

typedef struct {
    int32_t count_a;
    int32_t count_b;
    uint32_t edge_count_a;
    uint32_t edge_count_b;
    uint32_t error_count;
} EncoderSnapshot_t;


extern volatile int32_t EncoderA_Count;
extern volatile int32_t EncoderB_Count;


void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetCount(uint8_t channel);
int16_t Encoder_GetDelta(uint8_t channel);
void Encoder_ClearCount(uint8_t channel);
void Encoder_GetSnapshot(EncoderSnapshot_t *snapshot);

#endif
