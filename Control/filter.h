#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>

typedef struct {
    float alpha;
    float value;
    bool initialized;
} LowPassFilter_t;

void LowPassFilter_Init(LowPassFilter_t *filter, float alpha);
void LowPassFilter_Reset(LowPassFilter_t *filter);
float LowPassFilter_Update(LowPassFilter_t *filter, float input);

#endif
