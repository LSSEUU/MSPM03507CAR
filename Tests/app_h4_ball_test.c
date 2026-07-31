#include "app_h4_ball.h"

#include "ball_vision.h"

#include <assert.h>
#include <string.h>

static BallVisionSnapshot_t vision;
static LeverActuatorSnapshot_t lever;
static LeverActuatorResult_t arm_result;
static LeverActuatorResult_t tilt_result;
static LeverActuatorResult_t return_result;
static uint32_t arm_calls;
static uint32_t tilt_calls;
static uint32_t return_calls;
static uint32_t stop_calls;
static bool stop_disabled;
static int32_t last_tilt_mdeg;

static void reset_fixture(void)
{
    memset(&vision, 0, sizeof(vision));
    memset(&lever, 0, sizeof(lever));
    vision.valid = true;
    vision.fresh = true;
    vision.valid_streak = 2U;
    vision.last_valid_ms = 1000U;
    lever.boot_homed = true;
    lever.boot_state = LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL;
    lever.current_counts = 4396;
    arm_result = LEVER_ACTUATOR_OK;
    tilt_result = LEVER_ACTUATOR_OK;
    return_result = LEVER_ACTUATOR_OK;
    arm_calls = 0U;
    tilt_calls = 0U;
    return_calls = 0U;
    stop_calls = 0U;
    stop_disabled = false;
    last_tilt_mdeg = 0;
    AppH4Ball_Init(1000U);
}

void BallVision_GetSnapshot(uint32_t now_ms, BallVisionSnapshot_t *snapshot)
{
    vision.time_ms = now_ms;
    vision.age_ms = now_ms - vision.last_valid_ms;
    *snapshot = vision;
}

void LeverActuator_GetSnapshot(LeverActuatorSnapshot_t *snapshot)
{
    *snapshot = lever;
}

LeverActuatorResult_t LeverActuator_BallArm(void)
{
    arm_calls++;
    if (arm_result == LEVER_ACTUATOR_OK) {
        lever.ball_active = true;
        lever.armed = true;
    }
    return arm_result;
}

LeverActuatorResult_t LeverActuator_BallSetTiltProfile(
    int32_t angle_mdeg, bool breakaway_fast,
    uint16_t tracking_rpm, uint16_t breakaway_rpm)
{
    (void)breakaway_fast;
    assert(tracking_rpm > 0U);
    assert(breakaway_rpm == tracking_rpm);
    tilt_calls++;
    last_tilt_mdeg = angle_mdeg;
    return tilt_result;
}

LeverActuatorResult_t LeverActuator_BallReturnHorizontalProfile(
    uint16_t return_rpm)
{
    assert(return_rpm > 0U);
    return_calls++;
    if (return_result == LEVER_ACTUATOR_OK) {
        lever.ball_returning = true;
        lever.moving = true;
    }
    return return_result;
}

void LeverActuator_BallStop(bool disable_after_stop)
{
    stop_calls++;
    stop_disabled = disable_after_stop;
    lever.ball_active = false;
    lever.ball_returning = false;
    lever.moving = false;
}

static AppH4BallSnapshot_t snapshot(void)
{
    AppH4BallSnapshot_t value;
    AppH4Ball_GetSnapshot(&value);
    return value;
}

static void test_preflight_and_cascade_tracking(void)
{
    AppH4BallSnapshot_t value;
    reset_fixture();
    assert(AppH4Ball_CheckReady(1000U));
    assert(AppH4Ball_Start(1000U));
    assert(arm_calls == 1U && AppH4Ball_IsActive());

    vision.x_mm_x10 = 70;
    vision.vx_mm_s = 0;
    vision.last_valid_ms = 1020U;
    AppH4Ball_Task20ms(1020U);
    value = snapshot();
    assert(value.target_velocity_mm_s == -168);
    assert(value.velocity_error_mm_s == -168);
    assert(value.raw_tilt_mdeg < -4900);
    assert(value.tilt_mdeg == -1000);
    assert(last_tilt_mdeg == value.tilt_mdeg && tilt_calls == 1U);

    vision.x_mm_x10 = -70;
    vision.last_valid_ms = 1040U;
    AppH4Ball_Task20ms(1040U);
    value = snapshot();
    assert(value.target_velocity_mm_s == 168);
    assert(value.raw_tilt_mdeg > 4900);
    assert(value.tilt_mdeg == 0);
}

static void test_vision_and_edge_fail_closed(void)
{
    reset_fixture();
    assert(AppH4Ball_Start(1000U));
    vision.valid = false;
    vision.fresh = false;
    AppH4Ball_Task20ms(1020U);
    assert(AppH4Ball_HasFault());
    assert(AppH4Ball_Fault() == APP_H4_BALL_FAULT_VISION);
    assert(stop_calls == 1U && stop_disabled);

    reset_fixture();
    assert(AppH4Ball_Start(1000U));
    vision.x_mm_x10 = 950;
    vision.vx_mm_s = 300;
    vision.last_valid_ms = 1020U;
    AppH4Ball_Task20ms(1020U);
    assert(AppH4Ball_Fault() == APP_H4_BALL_FAULT_EDGE);
    assert(stop_calls == 1U && tilt_calls == 0U);
}

static void test_actuator_fault_and_controlled_finish(void)
{
    reset_fixture();
    assert(AppH4Ball_Start(1000U));
    tilt_result = LEVER_ACTUATOR_IO;
    vision.last_valid_ms = 1020U;
    AppH4Ball_Task20ms(1020U);
    assert(AppH4Ball_Fault() == APP_H4_BALL_FAULT_ACTUATOR);
    assert(stop_calls == 1U);

    reset_fixture();
    assert(AppH4Ball_Start(1000U));
    assert(AppH4Ball_Finish());
    assert(return_calls == 1U);
    assert(snapshot().state == APP_H4_BALL_RETURNING);
    lever.ball_active = false;
    lever.ball_returning = false;
    lever.moving = false;
    AppH4Ball_Task20ms(1100U);
    assert(snapshot().state == APP_H4_BALL_COMPLETE);
    assert(AppH4Ball_IsComplete());
}

int main(void)
{
    test_preflight_and_cascade_tracking();
    test_vision_and_edge_fail_closed();
    test_actuator_fault_and_controlled_finish();
    return 0;
}
