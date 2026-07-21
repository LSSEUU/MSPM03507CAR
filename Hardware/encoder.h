
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t count_a;
    int32_t count_b;
    uint32_t edge_count_a;
    uint32_t edge_count_b;
} EncoderSnapshot_t;

void Encoder_Init(void);
void Encoder_GetSnapshot(EncoderSnapshot_t *snapshot);

#endif
