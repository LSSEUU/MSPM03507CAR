#include "app_h4_ball.h"

#include "ball_vision.h"
#include "h4_ball_control_config.h"

#include <stddef.h>
#include <string.h>

static AppH4BallSnapshot_t s_h4_ball;
static BallCascadeState_t s_controller;

static uint32_t abs_i32(int32_t value)
{
    if (value >= 0) return (uint32_t)value;
    return (uint32_t)(-(value + 1)) + 1U;
}

static BallCascadeConfig_t controller_config(void)
{
    BallCascadeConfig_t config = {
        H4_BALL_POSITION_KP_X1000_PER_S,
        H4_BALL_POSITION_KI_X1000_PER_S2,
        H4_BALL_POSITION_INTEGRAL_LIMIT_MM_S,
        H4_BALL_POSITION_DEADBAND_X10,
        H4_BALL_VELOCITY_LIMIT_MM_S,
        H4_BALL_SPEED_KP_X100_MDEG_PER_MM_S,
        H4_BALL_SPEED_KI_X100_MDEG_PER_MM,
        H4_BALL_SPEED_INTEGRAL_LIMIT_MDEG,
        H4_BALL_POSITIVE_TILT_LIMIT_MDEG,
        H4_BALL_NEGATIVE_TILT_LIMIT_MDEG,
        H4_BALL_TILT_SLEW_MDEG_PER_20MS,
        H4_BALL_DIRECTION_SIGN
    };
    return config;
}

static bool ready_from_snapshots(const BallVisionSnapshot_t *vision,
                                 const LeverActuatorSnapshot_t *lever)
{
    return vision->valid && vision->fresh &&
        (vision->valid_streak >= 2U) &&
        (vision->age_ms <= BALL_VISION_FRESH_TIMEOUT_MS) &&
        (abs_i32(vision->x_mm_x10) <=
         (uint32_t)H4_BALL_START_POSITION_LIMIT_X10) &&
        lever->boot_homed &&
        (lever->boot_state == LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL) &&
        !lever->moving && !lever->ball_active &&
        (lever->fault_latched == 0U);
}

static void record_inputs(const BallVisionSnapshot_t *vision,
                          const LeverActuatorSnapshot_t *lever)
{
    s_h4_ball.vision_age_ms = vision->age_ms;
    s_h4_ball.position_mm_x10 = vision->x_mm_x10;
    s_h4_ball.velocity_mm_s = vision->vx_mm_s;
    s_h4_ball.vision_sequence = vision->seq;
    s_h4_ball.actuator_counts = lever->current_counts;
    s_h4_ball.actuator_fault = lever->fault_latched;
}

static void enter_fault(AppH4BallFault_t fault,
                        LeverActuatorResult_t actuator_result)
{
    if ((s_h4_ball.state == APP_H4_BALL_ACTIVE) ||
        (s_h4_ball.state == APP_H4_BALL_RETURNING))
        LeverActuator_BallStop(true);
    s_h4_ball.state = APP_H4_BALL_FAULT;
    s_h4_ball.fault = fault;
    s_h4_ball.actuator_result = actuator_result;
    s_h4_ball.target_velocity_mm_s = 0;
    s_h4_ball.velocity_error_mm_s = 0;
    s_h4_ball.raw_tilt_mdeg = 0;
    s_h4_ball.tilt_mdeg = 0;
}

void AppH4Ball_Init(uint32_t now_ms)
{
    memset(&s_h4_ball, 0, sizeof(s_h4_ball));
    s_h4_ball.time_ms = now_ms;
    s_h4_ball.target_mm_x10 = H4_BALL_TARGET_POSITION_X10;
    s_h4_ball.state = APP_H4_BALL_IDLE;
    s_h4_ball.actuator_result = LEVER_ACTUATOR_OK;
    BallCascade_Reset(&s_controller, now_ms);
}

bool AppH4Ball_CheckReady(uint32_t now_ms)
{
    BallVisionSnapshot_t vision;
    LeverActuatorSnapshot_t lever;
    if ((s_h4_ball.state == APP_H4_BALL_ACTIVE) ||
        (s_h4_ball.state == APP_H4_BALL_RETURNING) ||
        (s_h4_ball.state == APP_H4_BALL_FAULT)) return false;
    BallVision_GetSnapshot(now_ms, &vision);
    LeverActuator_GetSnapshot(&lever);
    return ready_from_snapshots(&vision, &lever);
}

bool AppH4Ball_Start(uint32_t now_ms)
{
    BallVisionSnapshot_t vision;
    LeverActuatorSnapshot_t lever;
    BallCascadeConfig_t config = controller_config();
    LeverActuatorResult_t result;

    if (!BallCascade_ConfigValid(&config)) {
        enter_fault(APP_H4_BALL_FAULT_CONFIG,
                    LEVER_ACTUATOR_CONFIG_CHANGED);
        return false;
    }
    if (!AppH4Ball_CheckReady(now_ms)) return false;
    BallVision_GetSnapshot(now_ms, &vision);
    LeverActuator_GetSnapshot(&lever);
    if (!ready_from_snapshots(&vision, &lever)) return false;
    result = LeverActuator_BallArm();
    if (result != LEVER_ACTUATOR_OK) {
        s_h4_ball.actuator_result = result;
        return false;
    }

    BallCascade_Reset(&s_controller, now_ms);
    s_h4_ball.time_ms = now_ms;
    s_h4_ball.target_mm_x10 = H4_BALL_TARGET_POSITION_X10;
    s_h4_ball.target_velocity_mm_s = 0;
    s_h4_ball.velocity_error_mm_s = 0;
    s_h4_ball.raw_tilt_mdeg = 0;
    s_h4_ball.tilt_mdeg = 0;
    s_h4_ball.position_saturated = false;
    s_h4_ball.tilt_saturated = false;
    s_h4_ball.fault = APP_H4_BALL_FAULT_NONE;
    s_h4_ball.actuator_result = LEVER_ACTUATOR_OK;
    s_h4_ball.state = APP_H4_BALL_ACTIVE;
    record_inputs(&vision, &lever);
    return true;
}

void AppH4Ball_Task20ms(uint32_t now_ms)
{
    BallVisionSnapshot_t vision;
    LeverActuatorSnapshot_t lever;
    BallCascadeConfig_t config;
    LeverActuatorResult_t result;
    int64_t predicted_x10;

    s_h4_ball.time_ms = now_ms;
    LeverActuator_GetSnapshot(&lever);
    if (s_h4_ball.state == APP_H4_BALL_RETURNING) {
        s_h4_ball.actuator_counts = lever.current_counts;
        s_h4_ball.actuator_fault = lever.fault_latched;
        if (lever.fault_latched != 0U) {
            enter_fault(APP_H4_BALL_FAULT_ACTUATOR,
                (LeverActuatorResult_t)lever.fault_latched);
        } else if (!lever.ball_active && !lever.moving) {
            s_h4_ball.state = APP_H4_BALL_COMPLETE;
        }
        return;
    }
    if (s_h4_ball.state != APP_H4_BALL_ACTIVE) return;

    BallVision_GetSnapshot(now_ms, &vision);
    record_inputs(&vision, &lever);
    if (!vision.valid || !vision.fresh ||
        (vision.valid_streak < 2U) ||
        (vision.age_ms > BALL_VISION_FRESH_TIMEOUT_MS)) {
        enter_fault(APP_H4_BALL_FAULT_VISION,
                    LEVER_ACTUATOR_SESSION_INVALID);
        return;
    }
    predicted_x10 = (int64_t)vision.x_mm_x10 +
        (int64_t)vision.vx_mm_s * H4_BALL_EDGE_PREDICT_MS / 100;
    if ((abs_i32(vision.x_mm_x10) >=
         (uint32_t)H4_BALL_EDGE_LIMIT_X10) ||
        (predicted_x10 >= H4_BALL_EDGE_LIMIT_X10) ||
        (predicted_x10 <= -H4_BALL_EDGE_LIMIT_X10)) {
        enter_fault(APP_H4_BALL_FAULT_EDGE,
                    LEVER_ACTUATOR_OUT_OF_RANGE);
        return;
    }
    if ((lever.fault_latched != 0U) || !lever.ball_active ||
        lever.ball_returning) {
        enter_fault(APP_H4_BALL_FAULT_ACTUATOR,
            lever.fault_latched != 0U ?
            (LeverActuatorResult_t)lever.fault_latched :
            LEVER_ACTUATOR_SESSION_INVALID);
        return;
    }

    config = controller_config();
    s_h4_ball.tilt_mdeg = BallCascade_Update(&s_controller,
        H4_BALL_TARGET_POSITION_X10, vision.x_mm_x10,
        vision.vx_mm_s, now_ms, &config);
    s_h4_ball.target_velocity_mm_s = s_controller.target_velocity_mm_s;
    s_h4_ball.velocity_error_mm_s = s_controller.velocity_error_mm_s;
    s_h4_ball.raw_tilt_mdeg = s_controller.raw_tilt_mdeg;
    s_h4_ball.position_saturated = s_controller.position_saturated != 0U;
    s_h4_ball.tilt_saturated = s_controller.tilt_saturated != 0U;
    result = LeverActuator_BallSetTiltProfile(s_h4_ball.tilt_mdeg,
        false, H4_BALL_TRACKING_RPM, H4_BALL_TRACKING_RPM);
    s_h4_ball.actuator_result = result;
    if (result != LEVER_ACTUATOR_OK)
        enter_fault(APP_H4_BALL_FAULT_ACTUATOR, result);
}

bool AppH4Ball_Finish(void)
{
    LeverActuatorResult_t result;
    if ((s_h4_ball.state == APP_H4_BALL_IDLE) ||
        (s_h4_ball.state == APP_H4_BALL_COMPLETE) ||
        (s_h4_ball.state == APP_H4_BALL_RETURNING)) return true;
    if (s_h4_ball.state != APP_H4_BALL_ACTIVE) return false;
    result = LeverActuator_BallReturnHorizontalProfile(H4_BALL_RETURN_RPM);
    s_h4_ball.actuator_result = result;
    if (result != LEVER_ACTUATOR_OK) {
        enter_fault(APP_H4_BALL_FAULT_ACTUATOR, result);
        return false;
    }
    s_h4_ball.target_velocity_mm_s = 0;
    s_h4_ball.velocity_error_mm_s = 0;
    s_h4_ball.raw_tilt_mdeg = 0;
    s_h4_ball.tilt_mdeg = 0;
    s_h4_ball.state = APP_H4_BALL_RETURNING;
    return true;
}

void AppH4Ball_Stop(bool emergency)
{
    if ((s_h4_ball.state != APP_H4_BALL_ACTIVE) &&
        (s_h4_ball.state != APP_H4_BALL_RETURNING)) return;
    if (emergency) {
        LeverActuator_BallStop(true);
        s_h4_ball.state = APP_H4_BALL_FAULT;
        s_h4_ball.fault = APP_H4_BALL_FAULT_EXTERNAL_STOP;
        s_h4_ball.actuator_result = LEVER_ACTUATOR_SESSION_INVALID;
    } else {
        (void)AppH4Ball_Finish();
    }
}

void AppH4Ball_EmergencyStop(void)
{
    if ((s_h4_ball.state != APP_H4_BALL_ACTIVE) &&
        (s_h4_ball.state != APP_H4_BALL_RETURNING)) return;
    LeverActuator_BallStop(true);
    s_h4_ball.state = APP_H4_BALL_FAULT;
    s_h4_ball.fault = APP_H4_BALL_FAULT_EMERGENCY;
    s_h4_ball.actuator_result = LEVER_ACTUATOR_SESSION_INVALID;
}

bool AppH4Ball_IsActive(void)
{
    return s_h4_ball.state == APP_H4_BALL_ACTIVE;
}

bool AppH4Ball_IsBusy(void)
{
    return (s_h4_ball.state == APP_H4_BALL_ACTIVE) ||
           (s_h4_ball.state == APP_H4_BALL_RETURNING);
}

bool AppH4Ball_IsComplete(void)
{
    return s_h4_ball.state == APP_H4_BALL_COMPLETE;
}

bool AppH4Ball_HasFault(void)
{
    return s_h4_ball.state == APP_H4_BALL_FAULT;
}

AppH4BallFault_t AppH4Ball_Fault(void)
{
    return s_h4_ball.fault;
}

void AppH4Ball_GetSnapshot(AppH4BallSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = s_h4_ball;
}
