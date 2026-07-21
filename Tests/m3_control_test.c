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
    assert(close_to(snapshot.raw_mm_s[WHEEL_SPEED_LEFT], 314.16f, 1.0f));
    assert(close_to(snapshot.raw_mm_s[WHEEL_SPEED_RIGHT], 314.16f, 1.0f));

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
    test_encoder_invalid_fault();
    test_encoder_count_wrap();
    test_stall_fault();
    return 0;
}
