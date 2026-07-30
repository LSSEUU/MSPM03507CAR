#include "wheel_speed.h"

#include "control_config.h"
#include "filter.h"
#include "pid.h"

#include <stddef.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define WHEEL_SPEED_DEFAULT_JERK_MM_S3 20000.0f

typedef struct {
    PIController_t pi;
    LowPassFilter_t filter;
} WheelController_t;

static WheelSpeedSnapshot_t speed;
static WheelController_t wheel[WHEEL_SPEED_COUNT];
static WheelSpeedConfig_t active_config;
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

static bool WheelSpeed_IsFinite(float value)
{
    return (value == value) &&
           (value <= 3.402823466e+38F) &&
           (value >= -3.402823466e+38F);
}

static bool WheelSpeed_ConfigValid(const WheelSpeedConfig_t *config)
{
    uint32_t i;
    if (config == NULL) return false;
    if (!WheelSpeed_IsFinite(config->filter_alpha) ||
        (config->filter_alpha < 0.05f) ||
        (config->filter_alpha > 1.0f) ||
        !WheelSpeed_IsFinite(config->accel_mm_s2) ||
        (config->accel_mm_s2 < 100.0f) ||
        (config->accel_mm_s2 > 2000.0f) ||
        !WheelSpeed_IsFinite(config->decel_mm_s2) ||
        (config->decel_mm_s2 < 100.0f) ||
        (config->decel_mm_s2 > 2000.0f) ||
        !WheelSpeed_IsFinite(config->jerk_mm_s3) ||
        (config->jerk_mm_s3 < 1000.0f) ||
        (config->jerk_mm_s3 > 20000.0f)) return false;
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        if (!WheelSpeed_IsFinite(config->kp[i]) ||
            (config->kp[i] < 0.0f) || (config->kp[i] > 3.0f) ||
            !WheelSpeed_IsFinite(config->ki[i]) ||
            (config->ki[i] < 0.0f) || (config->ki[i] > 10.0f) ||
            !WheelSpeed_IsFinite(config->static_feedforward_pwm[i]) ||
            (config->static_feedforward_pwm[i] < 0.0f) ||
            (config->static_feedforward_pwm[i] >
             WHEEL_FEEDFORWARD_PARAM_MAX) ||
            !WheelSpeed_IsFinite(
                config->velocity_feedforward_pwm_per_mm_s[i]) ||
            (config->velocity_feedforward_pwm_per_mm_s[i] < 0.0f) ||
            (config->velocity_feedforward_pwm_per_mm_s[i] > 5.0f))
            return false;
    }
    return true;
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

static void WheelSpeed_ResetControlFields(uint32_t i)
{
    PI_Reset(&wheel[i].pi);
    speed.ramp_accel_mm_s2[i] = 0.0f;
    speed.error_mm_s[i] = 0.0f;
    speed.proportional_pwm[i] = 0.0f;
    speed.integral_pwm[i] = 0.0f;
    speed.feedforward_pwm[i] = 0.0f;
    speed.unsaturated_pwm[i] = 0.0f;
    speed.pwm[i] = 0;
    speed.saturated[i] = false;
    speed.saturation_ms[i] = 0U;
}

static void WheelSpeed_ResetController(void)
{
    uint32_t i;
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        LowPassFilter_Reset(&wheel[i].filter);
        speed.requested_mm_s[i] = 0.0f;
        speed.ramped_mm_s[i] = 0.0f;
        speed.raw_mm_s[i] = 0.0f;
        speed.filtered_mm_s[i] = 0.0f;
        speed.stall_ms[i] = 0U;
        WheelSpeed_ResetControlFields(i);
    }
    speed.actual_dt_us = 0U;
}

void WheelSpeed_GetDefaultConfig(WheelSpeedConfig_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->kp[WHEEL_SPEED_LEFT] = WHEEL_SPEED_KP_LEFT;
    config->ki[WHEEL_SPEED_LEFT] = WHEEL_SPEED_KI_LEFT;
    config->kp[WHEEL_SPEED_RIGHT] = WHEEL_SPEED_KP_RIGHT;
    config->ki[WHEEL_SPEED_RIGHT] = WHEEL_SPEED_KI_RIGHT;
    config->filter_alpha = WHEEL_SPEED_FILTER_ALPHA;
    config->accel_mm_s2 = WHEEL_SPEED_ACCEL_MM_S2;
    config->decel_mm_s2 = WHEEL_SPEED_DECEL_MM_S2;
    config->jerk_mm_s3 = WHEEL_SPEED_DEFAULT_JERK_MM_S3;
}

void WheelSpeed_GetConfig(WheelSpeedConfig_t *config)
{
    if (config != NULL) *config = active_config;
}

bool WheelSpeed_ApplyConfig(const WheelSpeedConfig_t *config,
                            bool reset_controllers)
{
    uint32_t i;
    if (!WheelSpeed_ConfigValid(config)) return false;
    active_config = *config;
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        PI_SetGains(&wheel[i].pi, active_config.kp[i], active_config.ki[i]);
        wheel[i].filter.alpha = active_config.filter_alpha;
        if (reset_controllers) WheelSpeed_ResetControlFields(i);
    }
    return true;
}

void WheelSpeed_Init(void)
{
    WheelSpeedConfig_t defaults;
    memset(&speed, 0, sizeof(speed));
    WheelSpeed_GetDefaultConfig(&defaults);
    active_config = defaults;
    PI_Init(&wheel[WHEEL_SPEED_LEFT].pi, defaults.kp[WHEEL_SPEED_LEFT],
            defaults.ki[WHEEL_SPEED_LEFT], -WHEEL_SPEED_PWM_LIMIT,
            WHEEL_SPEED_PWM_LIMIT);
    PI_Init(&wheel[WHEEL_SPEED_RIGHT].pi, defaults.kp[WHEEL_SPEED_RIGHT],
            defaults.ki[WHEEL_SPEED_RIGHT], -WHEEL_SPEED_PWM_LIMIT,
            WHEEL_SPEED_PWM_LIMIT);
    LowPassFilter_Init(&wheel[WHEEL_SPEED_LEFT].filter,
                       defaults.filter_alpha);
    LowPassFilter_Init(&wheel[WHEEL_SPEED_RIGHT].filter,
                       defaults.filter_alpha);
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
    if (!WheelSpeed_IsFinite(kp) || !WheelSpeed_IsFinite(ki) ||
        (kp < 0.0f) || (kp > 3.0f) ||
        (ki < 0.0f) || (ki > 10.0f)) return;
    active_config.kp[selected] = kp;
    active_config.ki[selected] = ki;
    PI_SetGains(&wheel[selected].pi, kp, ki);
    /* Live diagnostic gain changes are bumpless at the 5 ms boundary. */
    WheelSpeed_ResetControlFields((uint32_t)selected);
}

void WheelSpeed_GetGains(WheelSpeedWheel_t selected, float *kp, float *ki)
{
    if ((uint32_t)selected >= (uint32_t)WHEEL_SPEED_COUNT) return;
    if (kp != NULL) *kp = wheel[selected].pi.kp;
    if (ki != NULL) *ki = wheel[selected].pi.ki;
}

static void WheelSpeed_ShapeTarget(uint32_t i, float dt_s)
{
    float current = speed.ramped_mm_s[i];
    float requested = speed.requested_mm_s[i];
    float shaped_target = requested;
    float delta;
    float rate;
    float desired_accel;
    float next;
    bool reversing = (current * requested) < 0.0f;
    bool accelerating;

    /* A sign reversal is split into two ramps, guaranteeing a zero crossing. */
    if (reversing) shaped_target = 0.0f;
    delta = shaped_target - current;
    if (delta == 0.0f) {
        speed.ramp_accel_mm_s2[i] = WheelSpeed_MoveTowards(
            speed.ramp_accel_mm_s2[i], 0.0f,
            active_config.jerk_mm_s3 * dt_s);
        return;
    }
    accelerating = ((current == 0.0f) || ((current * shaped_target) > 0.0f)) &&
        (WheelSpeed_Abs(shaped_target) > WheelSpeed_Abs(current));
    rate = accelerating ? active_config.accel_mm_s2 :
                          active_config.decel_mm_s2;
    /* The maximum setting is the M5-compatible baseline: it preserves the
     * original acceleration ramp while retaining a safe zero-speed reversal.
     * Any lower configured value enables jerk-limited shaping. */
    if (active_config.jerk_mm_s3 >= WHEEL_SPEED_DEFAULT_JERK_MM_S3) {
        next = WheelSpeed_MoveTowards(current, shaped_target, rate * dt_s);
        speed.ramp_accel_mm_s2[i] = (next - current) / dt_s;
        speed.ramped_mm_s[i] = next;
        return;
    }
    desired_accel = WheelSpeed_Clamp(delta / dt_s, rate);
    speed.ramp_accel_mm_s2[i] = WheelSpeed_MoveTowards(
        speed.ramp_accel_mm_s2[i], desired_accel,
        active_config.jerk_mm_s3 * dt_s);
    next = current + (speed.ramp_accel_mm_s2[i] * dt_s);
    if (((delta > 0.0f) && (next >= shaped_target)) ||
        ((delta < 0.0f) && (next <= shaped_target))) {
        next = shaped_target;
        speed.ramp_accel_mm_s2[i] = 0.0f;
    }
    speed.ramped_mm_s[i] = next;
}

void WheelSpeed_UpdateLogical(int32_t encoder_left, int32_t encoder_right,
                              uint32_t now_ms)
{
    static const float counts_per_revolution[WHEEL_SPEED_COUNT] = {
        ENCODER_LEFT_COUNTS_PER_WHEEL_REV,
        ENCODER_RIGHT_COUNTS_PER_WHEEL_REV
    };
    static const int8_t forward_sign[WHEEL_SPEED_COUNT] = {
        ENCODER_LEFT_FORWARD_SIGN, ENCODER_RIGHT_FORWARD_SIGN
    };
    int32_t counts[WHEEL_SPEED_COUNT];
    uint32_t elapsed_ms;
    uint32_t i;
    float dt_s;
    float circumference_mm = WHEEL_DIAMETER_MM * PI_F;
    bool valid = true;

    counts[WHEEL_SPEED_LEFT] = encoder_left;
    counts[WHEEL_SPEED_RIGHT] = encoder_right;
    if (!have_encoder_baseline) {
        speed.encoder_count[WHEEL_SPEED_LEFT] = encoder_left;
        speed.encoder_count[WHEEL_SPEED_RIGHT] = encoder_right;
        speed.last_update_ms = now_ms;
        speed.sample_valid = false;
        have_encoder_baseline = true;
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - speed.last_update_ms);
    if (elapsed_ms == 0U) return;
    dt_s = (float)elapsed_ms * 0.001f;
    speed.last_update_ms = now_ms;
    speed.actual_dt_us = elapsed_ms * 1000U;
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
        float error;
        float feedforward = 0.0f;
        float output;

        speed.filtered_mm_s[i] = LowPassFilter_Update(
            &wheel[i].filter, speed.raw_mm_s[i]);
        if (!speed.enabled) {
            speed.pwm[i] = 0;
            speed.stall_ms[i] = 0U;
            continue;
        }

        WheelSpeed_ShapeTarget(i, dt_s);
        if ((speed.requested_mm_s[i] == 0.0f) &&
            (speed.ramped_mm_s[i] == 0.0f) &&
            (WheelSpeed_Abs(speed.filtered_mm_s[i]) < 10.0f)) {
            WheelSpeed_ResetControlFields(i);
        } else {
            error = speed.ramped_mm_s[i] - speed.filtered_mm_s[i];
            if (speed.ramped_mm_s[i] != 0.0f) {
                feedforward = active_config.static_feedforward_pwm[i] +
                    (active_config.velocity_feedforward_pwm_per_mm_s[i] *
                     WheelSpeed_Abs(speed.ramped_mm_s[i]));
                if (speed.ramped_mm_s[i] < 0.0f) feedforward = -feedforward;
            }
            output = PI_UpdateWithFeedforward(&wheel[i].pi, error,
                                              feedforward, dt_s);
            speed.error_mm_s[i] = error;
            speed.proportional_pwm[i] = wheel[i].pi.last_proportional;
            speed.integral_pwm[i] = wheel[i].pi.integral;
            speed.feedforward_pwm[i] = feedforward;
            speed.unsaturated_pwm[i] = wheel[i].pi.last_unsaturated;
            speed.pwm[i] = WheelSpeed_RoundPWM(output);
            speed.saturated[i] = wheel[i].pi.last_saturated ||
                (WheelSpeed_Abs(output) >= WHEEL_SPEED_PWM_LIMIT);
            if (speed.saturated[i]) {
                if (speed.saturation_ms[i] < (0xFFFFFFFFU - elapsed_ms))
                    speed.saturation_ms[i] += elapsed_ms;
                else speed.saturation_ms[i] = 0xFFFFFFFFU;
            } else {
                speed.saturation_ms[i] = 0U;
            }
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
