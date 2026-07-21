#include "wheel_speed.h"

#include "control_config.h"
#include "filter.h"
#include "pid.h"

#include <stddef.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

typedef struct {
    PIController_t pi;
    LowPassFilter_t filter;
} WheelController_t;

static WheelSpeedSnapshot_t speed;
static WheelController_t wheel[WHEEL_SPEED_COUNT];
static bool have_encoder_baseline;

static float WheelSpeed_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float WheelSpeed_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float WheelSpeed_MoveTowards(float current, float target,
                                    float maximum_delta)
{
    float delta = target - current;
    if (delta > maximum_delta) return current + maximum_delta;
    if (delta < -maximum_delta) return current - maximum_delta;
    return target;
}

static int16_t WheelSpeed_RoundPWM(float value)
{
    value = WheelSpeed_Clamp(value, WHEEL_SPEED_PWM_LIMIT);
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int32_t WheelSpeed_CountDelta(int32_t current, int32_t previous)
{
    uint32_t delta = (uint32_t)current - (uint32_t)previous;
    if (delta <= 0x7FFFFFFFU) return (int32_t)delta;
    return -(int32_t)(0xFFFFFFFFU - delta) - 1;
}

static void WheelSpeed_ResetController(void)
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        PI_Reset(&wheel[i].pi);
        LowPassFilter_Reset(&wheel[i].filter);
        speed.requested_mm_s[i] = 0.0f;
        speed.ramped_mm_s[i] = 0.0f;
        speed.raw_mm_s[i] = 0.0f;
        speed.filtered_mm_s[i] = 0.0f;
        speed.pwm[i] = 0;
        speed.saturated[i] = false;
        speed.stall_ms[i] = 0U;
    }
}

void WheelSpeed_Init(void)
{
    memset(&speed, 0, sizeof(speed));
    PI_Init(&wheel[WHEEL_SPEED_LEFT].pi, WHEEL_SPEED_KP_A,
            WHEEL_SPEED_KI_A, -WHEEL_SPEED_PWM_LIMIT,
            WHEEL_SPEED_PWM_LIMIT);
    PI_Init(&wheel[WHEEL_SPEED_RIGHT].pi, WHEEL_SPEED_KP_B,
            WHEEL_SPEED_KI_B, -WHEEL_SPEED_PWM_LIMIT,
            WHEEL_SPEED_PWM_LIMIT);
    LowPassFilter_Init(&wheel[WHEEL_SPEED_LEFT].filter,
                       WHEEL_SPEED_FILTER_ALPHA);
    LowPassFilter_Init(&wheel[WHEEL_SPEED_RIGHT].filter,
                       WHEEL_SPEED_FILTER_ALPHA);
    have_encoder_baseline = false;
}

void WheelSpeed_Enable(void)
{
    WheelSpeed_ResetController();
    speed.fault_bits = WHEEL_SPEED_FAULT_NONE;
    speed.enabled = true;
}

void WheelSpeed_Disable(void)
{
    speed.enabled = false;
    WheelSpeed_ResetController();
}

void WheelSpeed_SetTarget(float left_mm_s, float right_mm_s)
{
    speed.requested_mm_s[WHEEL_SPEED_LEFT] =
        WheelSpeed_Clamp(left_mm_s, WHEEL_SPEED_MAX_TARGET_MM_S);
    speed.requested_mm_s[WHEEL_SPEED_RIGHT] =
        WheelSpeed_Clamp(right_mm_s, WHEEL_SPEED_MAX_TARGET_MM_S);
}

void WheelSpeed_SetGains(WheelSpeedWheel_t selected, float kp, float ki)
{
    if ((uint32_t)selected >= (uint32_t)WHEEL_SPEED_COUNT) return;
    PI_SetGains(&wheel[selected].pi, kp, ki);
}

void WheelSpeed_GetGains(WheelSpeedWheel_t selected, float *kp, float *ki)
{
    if ((uint32_t)selected >= (uint32_t)WHEEL_SPEED_COUNT) return;
    if (kp != NULL) *kp = wheel[selected].pi.kp;
    if (ki != NULL) *ki = wheel[selected].pi.ki;
}

void WheelSpeed_Update(int32_t encoder_a, int32_t encoder_b,
                       uint32_t now_ms)
{
    static const float counts_per_revolution[WHEEL_SPEED_COUNT] = {
        ENCODER_A_COUNTS_PER_WHEEL_REV,
        ENCODER_B_COUNTS_PER_WHEEL_REV
    };
    static const int8_t forward_sign[WHEEL_SPEED_COUNT] = {
        ENCODER_A_FORWARD_SIGN, ENCODER_B_FORWARD_SIGN
    };
    int32_t counts[WHEEL_SPEED_COUNT];
    uint32_t elapsed_ms;
    uint32_t i;
    float dt_s;
    float circumference_mm = WHEEL_DIAMETER_MM * PI_F;
    bool valid = true;

    counts[WHEEL_SPEED_LEFT] = encoder_a;
    counts[WHEEL_SPEED_RIGHT] = encoder_b;
    if (!have_encoder_baseline) {
        speed.encoder_count[WHEEL_SPEED_LEFT] = encoder_a;
        speed.encoder_count[WHEEL_SPEED_RIGHT] = encoder_b;
        speed.last_update_ms = now_ms;
        speed.sample_valid = false;
        have_encoder_baseline = true;
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - speed.last_update_ms);
    if (elapsed_ms == 0U) return;
    dt_s = (float)elapsed_ms * 0.001f;
    speed.last_update_ms = now_ms;
    speed.update_count++;

    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        float raw;
        speed.encoder_delta[i] = WheelSpeed_CountDelta(
            counts[i], speed.encoder_count[i]) * (int32_t)forward_sign[i];
        speed.encoder_count[i] = counts[i];
        raw = ((float)speed.encoder_delta[i] * circumference_mm) /
              (counts_per_revolution[i] * dt_s);
        speed.raw_mm_s[i] = raw;
        if ((WheelSpeed_Abs(raw) > WHEEL_SPEED_MAX_PHYSICAL_MM_S) ||
            (wheel[i].filter.initialized &&
             (WheelSpeed_Abs(raw - speed.filtered_mm_s[i]) >
              WHEEL_SPEED_MAX_JUMP_MM_S))) {
            valid = false;
        }
    }

    if (!valid) {
        speed.sample_valid = false;
        speed.invalid_sample_count++;
        if (speed.enabled)
            speed.fault_bits |= WHEEL_SPEED_FAULT_ENCODER_INVALID;
        speed.pwm[WHEEL_SPEED_LEFT] = 0;
        speed.pwm[WHEEL_SPEED_RIGHT] = 0;
        return;
    }
    speed.sample_valid = true;

    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        float ramp_rate;
        float error;
        float output;
        bool reversing = (speed.ramped_mm_s[i] *
                          speed.requested_mm_s[i]) < 0.0f;
        bool accelerating = !reversing &&
            (WheelSpeed_Abs(speed.requested_mm_s[i]) >
             WheelSpeed_Abs(speed.ramped_mm_s[i]));

        speed.filtered_mm_s[i] = LowPassFilter_Update(
            &wheel[i].filter, speed.raw_mm_s[i]);
        if (!speed.enabled) {
            speed.pwm[i] = 0;
            speed.stall_ms[i] = 0U;
            continue;
        }

        ramp_rate = accelerating ? WHEEL_SPEED_ACCEL_MM_S2 :
                                   WHEEL_SPEED_DECEL_MM_S2;
        speed.ramped_mm_s[i] = WheelSpeed_MoveTowards(
            speed.ramped_mm_s[i], speed.requested_mm_s[i],
            ramp_rate * dt_s);
        if ((speed.requested_mm_s[i] == 0.0f) &&
            (speed.ramped_mm_s[i] == 0.0f) &&
            (WheelSpeed_Abs(speed.filtered_mm_s[i]) < 10.0f)) {
            PI_Reset(&wheel[i].pi);
            speed.pwm[i] = 0;
            speed.saturated[i] = false;
        } else {
            error = speed.ramped_mm_s[i] - speed.filtered_mm_s[i];
            output = PI_Update(&wheel[i].pi, error, dt_s);
            speed.pwm[i] = WheelSpeed_RoundPWM(output);
            speed.saturated[i] =
                WheelSpeed_Abs(output) >= WHEEL_SPEED_PWM_LIMIT;
        }

        if ((WheelSpeed_Abs(speed.ramped_mm_s[i]) >=
             WHEEL_STALL_TARGET_MIN_MM_S) &&
            (WheelSpeed_Abs((float)speed.pwm[i]) >= WHEEL_STALL_PWM_MIN) &&
            (speed.encoder_delta[i] == 0)) {
            if (speed.stall_ms[i] < WHEEL_STALL_TIMEOUT_MS)
                speed.stall_ms[i] += elapsed_ms;
            if (speed.stall_ms[i] >= WHEEL_STALL_TIMEOUT_MS)
                speed.fault_bits |= WHEEL_SPEED_FAULT_STALL;
        } else {
            speed.stall_ms[i] = 0U;
        }
    }
}

void WheelSpeed_ClearFaults(void)
{
    if (speed.enabled) return;
    speed.fault_bits = WHEEL_SPEED_FAULT_NONE;
    speed.invalid_sample_count = 0U;
}

void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = speed;
}
