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
    float kp[WHEEL_SPEED_COUNT];
    float ki[WHEEL_SPEED_COUNT];
    float filter_alpha;
    float accel_mm_s2;
    float decel_mm_s2;
    float jerk_mm_s3;
    float static_feedforward_pwm[WHEEL_SPEED_COUNT];
    float velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_COUNT];
} WheelSpeedConfig_t;

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
    float ramp_accel_mm_s2[WHEEL_SPEED_COUNT];
    float raw_mm_s[WHEEL_SPEED_COUNT];
    float filtered_mm_s[WHEEL_SPEED_COUNT];
    float error_mm_s[WHEEL_SPEED_COUNT];
    float proportional_pwm[WHEEL_SPEED_COUNT];
    float integral_pwm[WHEEL_SPEED_COUNT];
    float feedforward_pwm[WHEEL_SPEED_COUNT];
    float unsaturated_pwm[WHEEL_SPEED_COUNT];
    uint32_t actual_dt_us;
    int16_t pwm[WHEEL_SPEED_COUNT];
    bool saturated[WHEEL_SPEED_COUNT];
    uint32_t saturation_ms[WHEEL_SPEED_COUNT];
    uint32_t stall_ms[WHEEL_SPEED_COUNT];
} WheelSpeedSnapshot_t;

void WheelSpeed_Init(void);
void WheelSpeed_Enable(void);
void WheelSpeed_Disable(void);
void WheelSpeed_SetTarget(float left_mm_s, float right_mm_s);
void WheelSpeed_SetGains(WheelSpeedWheel_t wheel, float kp, float ki);
void WheelSpeed_GetGains(WheelSpeedWheel_t wheel, float *kp, float *ki);
void WheelSpeed_GetDefaultConfig(WheelSpeedConfig_t *config);
void WheelSpeed_GetConfig(WheelSpeedConfig_t *config);
bool WheelSpeed_ApplyConfig(const WheelSpeedConfig_t *config,
                            bool reset_controllers);
void WheelSpeed_Update(int32_t encoder_a, int32_t encoder_b,
                       uint32_t now_ms);
void WheelSpeed_ClearFaults(void);
void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot);

#endif
