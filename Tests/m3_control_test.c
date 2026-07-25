#include "control_config.h"
#include "filter.h"
#include "pid.h"
#include "wheel_speed.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

static int close_to(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void test_low_pass_filter(void)
{
    LowPassFilter_t filter;
    LowPassFilter_Init(&filter, 0.25f);
    assert(close_to(LowPassFilter_Update(&filter, 100.0f), 100.0f, 0.01f));
    assert(close_to(LowPassFilter_Update(&filter, 0.0f), 75.0f, 0.01f));
    LowPassFilter_Reset(&filter);
    assert(close_to(LowPassFilter_Update(&filter, -20.0f), -20.0f, 0.01f));
}

static void test_pi_limits_and_anti_windup(void)
{
    PIController_t pi;
    PI_Init(&pi, 1.0f, 10.0f, -100.0f, 100.0f);
    assert(close_to(PI_Update(&pi, 200.0f, 1.0f), 100.0f, 0.01f));
    assert(close_to(pi.integral, 0.0f, 0.01f));
    assert(PI_Update(&pi, -20.0f, 0.1f) < 0.0f);
    PI_SetGains(&pi, 2.0f, 4.0f);
    assert(close_to(pi.kp, 2.0f, 0.01f));
    assert(close_to(pi.ki, 4.0f, 0.01f));
}

static void test_speed_conversion_direction_and_ramp(void)
{
    WheelSpeedSnapshot_t snapshot;
    WheelSpeed_Init();
    WheelSpeed_Update(0, 0, 0U);
    WheelSpeed_Update(8, -8, 5U);
    WheelSpeed_GetSnapshot(&snapshot);
    assert(snapshot.sample_valid);
    assert(snapshot.encoder_delta[WHEEL_SPEED_LEFT] == 8);
    assert(snapshot.encoder_delta[WHEEL_SPEED_RIGHT] == 8);
    assert(close_to(snapshot.raw_mm_s[WHEEL_SPEED_LEFT], 436.25f, 1.0f));
    assert(close_to(snapshot.raw_mm_s[WHEEL_SPEED_RIGHT], 435.10f, 1.0f));

    WheelSpeed_Enable();
    WheelSpeed_SetTarget(1000.0f, -1000.0f);
    WheelSpeed_Update(16, -16, 10U);
    WheelSpeed_GetSnapshot(&snapshot);
    assert(close_to(snapshot.requested_mm_s[WHEEL_SPEED_LEFT],
                    WHEEL_SPEED_MAX_TARGET_MM_S, 0.01f));
    assert(close_to(snapshot.requested_mm_s[WHEEL_SPEED_RIGHT],
                    -WHEEL_SPEED_MAX_TARGET_MM_S, 0.01f));
    assert(close_to(snapshot.ramped_mm_s[WHEEL_SPEED_LEFT], 3.0f, 0.01f));
    assert(close_to(snapshot.ramped_mm_s[WHEEL_SPEED_RIGHT], -3.0f, 0.01f));
    assert(close_to(snapshot.ramp_accel_mm_s2[WHEEL_SPEED_LEFT],
                    600.0f, 0.01f));
    assert(close_to(snapshot.ramp_accel_mm_s2[WHEEL_SPEED_RIGHT],
                    -600.0f, 0.01f));
    assert(snapshot.actual_dt_us == 5000U);
}

static void test_runtime_jerk_limit(void)
{
    WheelSpeedConfig_t config;
    WheelSpeedSnapshot_t snapshot;
    WheelSpeed_Init();
    WheelSpeed_GetDefaultConfig(&config);
    config.jerk_mm_s3 = 1000.0f;
    assert(WheelSpeed_ApplyConfig(&config, true));
    WheelSpeed_Update(0, 0, 0U);
    WheelSpeed_Enable();
    WheelSpeed_SetTarget(100.0f, 0.0f);
    WheelSpeed_Update(0, 0, 5U);
    WheelSpeed_GetSnapshot(&snapshot);
    assert(close_to(snapshot.ramp_accel_mm_s2[WHEEL_SPEED_LEFT],
                    5.0f, 0.01f));
    assert(close_to(snapshot.ramped_mm_s[WHEEL_SPEED_LEFT], 0.025f, 0.001f));
}

static void test_runtime_config_feedforward_and_atomic_reject(void)
{
    WheelSpeedConfig_t config;
    WheelSpeedConfig_t active;
    WheelSpeedSnapshot_t snapshot;
    WheelSpeed_Init();
    WheelSpeed_GetDefaultConfig(&config);
    config.kp[WHEEL_SPEED_LEFT] = 0.0f;
    config.ki[WHEEL_SPEED_LEFT] = 0.0f;
    config.static_feedforward_pwm[WHEEL_SPEED_LEFT] = 10.0f;
    config.velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_LEFT] = 0.1f;
    assert(WheelSpeed_ApplyConfig(&config, true));
    WheelSpeed_Update(0, 0, 0U);
    WheelSpeed_Enable();
    WheelSpeed_SetTarget(100.0f, 0.0f);
    WheelSpeed_Update(0, 0, 5U);
    WheelSpeed_GetSnapshot(&snapshot);
    assert(close_to(snapshot.feedforward_pwm[WHEEL_SPEED_LEFT],
                    10.3f, 0.01f));
    assert(close_to(snapshot.proportional_pwm[WHEEL_SPEED_LEFT],
                    0.0f, 0.01f));
    assert(snapshot.pwm[WHEEL_SPEED_LEFT] == 10);

    config.filter_alpha = 0.0f;
    assert(!WheelSpeed_ApplyConfig(&config, true));
    WheelSpeed_GetConfig(&active);
    assert(close_to(active.filter_alpha, WHEEL_SPEED_FILTER_ALPHA, 0.01f));
}

static void test_reverse_target_passes_through_zero(void)
{
    WheelSpeedSnapshot_t snapshot;
    uint32_t now;
    bool saw_zero = false;
    WheelSpeed_Init();
    WheelSpeed_Update(0, 0, 0U);
    WheelSpeed_Enable();
    WheelSpeed_SetTarget(100.0f, 100.0f);
    for (now = 5U; now <= 500U; now += 5U)
        WheelSpeed_Update(0, 0, now);
    WheelSpeed_SetTarget(-100.0f, -100.0f);
    for (; now <= 1500U; now += 5U) {
        WheelSpeed_Update(0, 0, now);
        WheelSpeed_GetSnapshot(&snapshot);
        if (close_to(snapshot.ramped_mm_s[WHEEL_SPEED_LEFT], 0.0f, 0.001f))
            saw_zero = true;
        if (snapshot.ramped_mm_s[WHEEL_SPEED_LEFT] < 0.0f) {
            assert(saw_zero);
            break;
        }
    }
    assert(saw_zero);
    assert(snapshot.ramped_mm_s[WHEEL_SPEED_LEFT] < 0.0f);
}

static void test_encoder_invalid_fault(void)
{
    WheelSpeedSnapshot_t snapshot;
    WheelSpeed_Init();
    WheelSpeed_Update(0, 0, 0U);
    WheelSpeed_Enable();
    WheelSpeed_Update(100, 0, 5U);
    WheelSpeed_GetSnapshot(&snapshot);
    assert(!snapshot.sample_valid);
    assert((snapshot.fault_bits &
            WHEEL_SPEED_FAULT_ENCODER_INVALID) != 0U);
    assert(snapshot.pwm[WHEEL_SPEED_LEFT] == 0);
    WheelSpeed_Disable();
    WheelSpeed_ClearFaults();
    WheelSpeed_GetSnapshot(&snapshot);
    assert(snapshot.fault_bits == WHEEL_SPEED_FAULT_NONE);
}

static void test_encoder_count_wrap(void)
{
    WheelSpeedSnapshot_t snapshot;
    WheelSpeed_Init();
    WheelSpeed_Update(INT32_MAX - 2, INT32_MIN + 2, 0U);
    WheelSpeed_Update(INT32_MIN + 2, INT32_MAX - 2, 5U);
    WheelSpeed_GetSnapshot(&snapshot);
    assert(snapshot.encoder_delta[WHEEL_SPEED_LEFT] == 5);
    assert(snapshot.encoder_delta[WHEEL_SPEED_RIGHT] == 5);
    assert(snapshot.sample_valid);
}

static void test_stall_fault(void)
{
    WheelSpeedSnapshot_t snapshot;
    uint32_t now;
    WheelSpeed_Init();
    WheelSpeed_Update(0, 0, 0U);
    WheelSpeed_Enable();
    WheelSpeed_SetTarget(700.0f, 0.0f);
    for (now = 5U; now <= 3000U; now += 5U)
        WheelSpeed_Update(0, 0, now);
    WheelSpeed_GetSnapshot(&snapshot);
    assert((snapshot.fault_bits & WHEEL_SPEED_FAULT_STALL) != 0U);
    assert(snapshot.stall_ms[WHEEL_SPEED_LEFT] >= WHEEL_STALL_TIMEOUT_MS);
}

int main(void)
{
    test_low_pass_filter();
    test_pi_limits_and_anti_windup();
    test_speed_conversion_direction_and_ramp();
    test_runtime_config_feedforward_and_atomic_reject();
    test_runtime_jerk_limit();
    test_reverse_target_passes_through_zero();
    test_encoder_invalid_fault();
    test_encoder_count_wrap();
    test_stall_fault();
    return 0;
}
