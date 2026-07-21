#include "filter.h"

#include <stddef.h>

static float Filter_ClampAlpha(float alpha)
{
    if (alpha < 0.0f) return 0.0f;
    if (alpha > 1.0f) return 1.0f;
    return alpha;
}

void LowPassFilter_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) return;
    filter->alpha = Filter_ClampAlpha(alpha);
    filter->value = 0.0f;
    filter->initialized = false;
}

void LowPassFilter_Reset(LowPassFilter_t *filter)
{
    if (filter == NULL) return;
    filter->value = 0.0f;
    filter->initialized = false;
}

float LowPassFilter_Update(LowPassFilter_t *filter, float input)
{
    if (filter == NULL) return input;
    if (!filter->initialized) {
        filter->value = input;
        filter->initialized = true;
    } else {
        filter->value += filter->alpha * (input - filter->value);
    }
    return filter->value;
}
