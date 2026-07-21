#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    WHEEL_SPEED_LEFT = 0,
    WHEEL_SPEED_RIGHT,
    WHEEL_SPEED_COUNT
} WheelSpeedWheel_t;

typedef enum {
    WHEEL_SPEED_FAULT_NONE            = 0U,
    WHEEL_SPEED_FAULT_ENCODER_INVALID = (1UL << 0),
    WHEEL_SPEED_FAULT_STALL           = (1UL << 1)
} WheelSpeedFault_t;

typedef struct {
    bool enabled;
    bool sample_valid;
    uint32_t last_update_ms;
    uint32_t update_count;
    uint32_t invalid_sample_count;
    uint32_t fault_bits;
    int32_t encoder_count[WHEEL_SPEED_COUNT];
    int32_t encoder_delta[WHEEL_SPEED_COUNT];
    float requested_mm_s[WHEEL_SPEED_COUNT];
    float ramped_mm_s[WHEEL_SPEED_COUNT];
    float raw_mm_s[WHEEL_SPEED_COUNT];
    float filtered_mm_s[WHEEL_SPEED_COUNT];
    int16_t pwm[WHEEL_SPEED_COUNT];
    bool saturated[WHEEL_SPEED_COUNT];
    uint32_t stall_ms[WHEEL_SPEED_COUNT];
} WheelSpeedSnapshot_t;

void WheelSpeed_Init(void);
void WheelSpeed_Enable(void);
void WheelSpeed_Disable(void);
void WheelSpeed_SetTarget(float left_mm_s, float right_mm_s);
void WheelSpeed_SetGains(WheelSpeedWheel_t wheel, float kp, float ki);
void WheelSpeed_GetGains(WheelSpeedWheel_t wheel, float *kp, float *ki);
void WheelSpeed_Update(int32_t encoder_a, int32_t encoder_b,
                       uint32_t now_ms);
void WheelSpeed_ClearFaults(void);
void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot);

#endif
