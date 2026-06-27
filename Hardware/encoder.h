
#ifndef ENCODER_H
#define ENCODER_H
#include "board.h"


extern volatile int32_t EncoderA_Count;
extern volatile int32_t EncoderB_Count;


void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetCount(uint8_t channel);
int16_t Encoder_GetDelta(uint8_t channel);
void Encoder_ClearCount(uint8_t channel);

#endif
