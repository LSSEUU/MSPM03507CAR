#include "app_line.h"

#include "control_config.h"
#include "h_course_profile.h"
#include "heading.h"
#include "line_trace.h"
#include "line_yaw_assist.h"
#include "m6_wire.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "vehicle_config.h"
#include "wheel_speed.h"

#include <stddef.h>
#include <string.h>

#define APP_LINE_TIMED_SAFETY_GRACE_MS 250U

static AppLineSnapshot_t app_line;

static void AppLine_ResetYawAssist(void)
{
    LineYawAssist_Reset();
    app_line.yaw_assist_active = false;
    app_line.sensor_correction_mm_s = 0.0f;
    app_line.desired_yaw_rate_deg_s = 0.0f;
    app_line.measured_yaw_rate_deg_s = 0.0f;
    app_line.yaw_rate_error_deg_s = 0.0f;
    app_line.yaw_assist_correction_mm_s = 0.0f;
    app_line.final_correction_mm_s = 0.0f;
    app_line.final_left_target_mm_s = 0.0f;
    app_line.final_right_target_mm_s = 0.0f;
}

static int16_t AppLine_ScaleSigned(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void AppLine_ReturnIdle(void)
{
    app_line.state = APP_LINE_IDLE;
    app_line.deadline_ms = 0U;
}

static bool AppLine_StartRunning(uint32_t now_ms, uint32_t duration_ms)
{
    LineFollowSnapshot_t snapshot;
    LineFollow_GetSnapshot(&snapshot);
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
        !snapshot.calibration.valid || !snapshot.line_valid)
        return false;
    if (!SystemState_RequestRun()) return false;
    if (!LineFollow_Start(now_ms)) {
        (void)SystemState_RequestIdle();
        return false;
    }
    if (duration_ms == M6_LINE_ACTION_CONTINUOUS_DURATION_MS)
        Safety_BeginContinuousAction(now_ms,
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE));
    else
        Safety_BeginAction(now_ms, duration_ms,
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE));
    WheelSpeed_Enable();
    AppLine_ResetYawAssist();
    HCourse_Start(now_ms);
    app_line.state = APP_LINE_RUNNING;
    app_line.started_ms = now_ms;
    app_line.deadline_ms = now_ms + duration_ms;
    LineTrace_Start();
    return true;
}

void AppLine_Init(void)
{
    memset(&app_line, 0, sizeof(app_line));
    LineFollow_Init();
    LineYawAssist_Init();
    HCourse_Init();
    LineTrace_Init();
}

bool AppLine_Arm(uint32_t now_ms)
{
    LineFollowSnapshot_t snapshot;
    if ((app_line.state != APP_LINE_IDLE) ||
        (SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault())
        return false;
    LineFollow_GetSnapshot(&snapshot);
    if (!snapshot.calibration.valid || !snapshot.line_valid) return false;
    app_line.state = APP_LINE_ARMED;
    app_line.deadline_ms = now_ms + DIAGNOSTIC_ARM_TIMEOUT_MS;
    return true;
}

bool AppLine_ConfirmStart(uint32_t now_ms)
{
    if ((app_line.state != APP_LINE_ARMED) ||
        SystemTime_Reached(now_ms, app_line.deadline_ms)) {
        AppLine_ReturnIdle();
        return false;
    }
    AppLine_ReturnIdle();
    return AppLine_StartRunning(now_ms, LINE_FOLLOW_ACTION_TIMEOUT_MS);
}

bool AppLine_StartTimed(uint32_t now_ms, uint32_t duration_ms)
{
    bool started;
    if (app_line.state != APP_LINE_IDLE) return false;
    started = AppLine_StartRunning(now_ms, duration_ms);
    if (started && duration_ms != M6_LINE_ACTION_CONTINUOUS_DURATION_MS) {
        /* M6 owns the normal timed stop.  Keep Safety as a later fallback so
         * the 20 ms line task and M6 service cannot turn an on-time stop into
         * ACTION_TIMEOUT merely because Safety ran first at the deadline. */
        Safety_BeginAction(now_ms,
            duration_ms + APP_LINE_TIMED_SAFETY_GRACE_MS,
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE));
    }
    return started;
}

bool AppLine_Stop(uint32_t now_ms)
{
    LineFollowSnapshot_t control;
    bool was_active;
    (void)now_ms;
    LineFollow_GetSnapshot(&control);
    was_active = (app_line.state != APP_LINE_IDLE) ||
        (control.state != LINE_FOLLOW_IDLE);
    if (app_line.state == APP_LINE_RUNNING) {
        WheelSpeed_Disable();
        Safety_EndAction();
        (void)SystemState_RequestIdle();
    }
    LineFollow_Stop();
    HCourse_Stop();
    AppLine_ResetYawAssist();
    LineTrace_Freeze();
    AppLine_ReturnIdle();
    return was_active;
}

void AppLine_ProcessGrayscale(const GrayscaleSnapshot_t *gray)
{
    LineFollowSnapshot_t control;
    LineTraceFrame_t frame;
    uint32_t i;
    if ((gray == NULL) || !gray->last_scan_ok) return;
    LineFollow_ProcessDigitalMask(gray->conditioned_mask, gray->scan_count,
                                  gray->last_update_ms);
    if ((app_line.state != APP_LINE_RUNNING) ||
        ((gray->scan_count & 1U) != 0U)) return;
    LineFollow_GetSnapshot(&control);
    memset(&frame, 0, sizeof(frame));
    frame.time_ms = gray->last_update_ms;
    for (i = 0U; i < LINE_TRACE_CHANNEL_COUNT; i++)
        frame.adc[i] = gray->adc[i];
    frame.position_x1000 = AppLine_ScaleSigned(control.position, 1000.0f);
    frame.correction_mm_s = AppLine_ScaleSigned(
        app_line.final_correction_mm_s, 1.0f);
    frame.active_mask = control.active_mask;
    frame.active_count = control.active_count;
    frame.cluster_count = control.cluster_count;
    frame.state = (uint8_t)control.state;
    frame.search_direction = control.search_direction;
    frame.flags = control.branch_direction_latched ? 1U : 0U;
    if (app_line.yaw_assist_active) frame.flags |= 2U;
    LineTrace_Capture(&frame, control.state == LINE_FOLLOW_SEARCHING);
}

void AppLine_Task20ms(uint32_t now_ms)
{
    LineFollowSnapshot_t control;
    LineFollowSnapshot_t assisted_control;
    LineFollowConfig_t line_config;
    HeadingSnapshot_t heading;
    HeadingConfig_t heading_config;
    WheelSpeedSnapshot_t wheel;
    HCourseSnapshot_t course;
    LineYawAssistSnapshot_t yaw_assist;
    bool gyro_usable;
    if (app_line.state == APP_LINE_ARMED) {
        if (SystemTime_Reached(now_ms, app_line.deadline_ms))
            AppLine_ReturnIdle();
        return;
    }
    if (app_line.state != APP_LINE_RUNNING) return;
    if (Safety_HasFault()) {
        WheelSpeed_Disable();
        LineTrace_Freeze();
        LineFollow_Stop();
        HCourse_Stop();
        AppLine_ResetYawAssist();
        AppLine_ReturnIdle();
        return;
    }
    LineFollow_Update(now_ms);
    LineFollow_GetSnapshot(&control);
    if (control.state == LINE_FOLLOW_LOST_TIMEOUT) {
        WheelSpeed_Disable();
        LineTrace_Freeze();
        HCourse_Stop();
        AppLine_ResetYawAssist();
        Safety_RaiseFault(SAFETY_FAULT_LINE_LOST, now_ms);
        AppLine_ReturnIdle();
        return;
    }
    Heading_GetSnapshot(&heading);
    Heading_GetConfig(&heading_config);
    WheelSpeed_GetSnapshot(&wheel);
    LineFollow_GetConfig(&line_config);
    gyro_usable = heading.calibrated &&
        (heading.rate_source == HEADING_RATE_GYRO_FRAME) &&
        ((uint32_t)(now_ms - heading.last_rate_update_ms) <=
         LINE_YAW_ASSIST_GYRO_FRESH_MS);
    HCourse_Update(now_ms, wheel.sample_valid,
        wheel.filtered_mm_s[WHEEL_SPEED_LEFT],
        wheel.filtered_mm_s[WHEEL_SPEED_RIGHT],
        gyro_usable, heading.yaw_deg,
        gyro_usable ? heading.yaw_rate_deg_s : 0.0f,
        (control.state == LINE_FOLLOW_TRACKING) &&
        control.line_valid && !control.ambiguous,
        control.active_mask, control.position);
    HCourse_GetSnapshot(&course);
    assisted_control = control;
    assisted_control.dynamic_base_mm_s = HCourse_SelectBaseSpeed(
        control.dynamic_base_mm_s, line_config.speed_min_mm_s,
        line_config.speed_max_mm_s,
        (control.state == LINE_FOLLOW_TRACKING) &&
        control.line_valid && !control.ambiguous,
        control.position);
    assisted_control.left_target_mm_s =
        assisted_control.dynamic_base_mm_s + control.correction_mm_s;
    assisted_control.right_target_mm_s =
        assisted_control.dynamic_base_mm_s - control.correction_mm_s;
    LineYawAssist_SetMode(HCourse_IsCurvePhase() ?
        LINE_YAW_ASSIST_MODE_CURVE_POSITIVE :
        LINE_YAW_ASSIST_MODE_STRAIGHT);
    LineYawAssist_SetCurveFeedforwardScale(
        (course.phase == H_COURSE_BC_CURVE) ?
        H_COURSE_BC_CURVE_FEEDFORWARD_SCALE :
        (course.phase == H_COURSE_DA_CURVE) ?
        H_COURSE_DA_CURVE_FEEDFORWARD_SCALE : 1.0f);
    LineYawAssist_SetStraightReference(
        course.straight_heading_target_valid,
        course.straight_heading_target_deg);
    LineYawAssist_Update(&assisted_control,
        line_config.correction_limit_mm_s,
        heading.yaw_deg, heading.yaw_rate_deg_s,
        heading_config.straight_kp, heading_config.straight_kd,
        gyro_usable, now_ms);
    LineYawAssist_GetSnapshot(&yaw_assist);
    app_line.yaw_assist_active = yaw_assist.active;
    app_line.sensor_correction_mm_s = yaw_assist.sensor_correction_mm_s;
    app_line.desired_yaw_rate_deg_s = yaw_assist.desired_yaw_rate_deg_s;
    app_line.measured_yaw_rate_deg_s = yaw_assist.measured_yaw_rate_deg_s;
    app_line.yaw_rate_error_deg_s = yaw_assist.rate_error_deg_s;
    app_line.yaw_assist_correction_mm_s =
        yaw_assist.assist_correction_mm_s;
    app_line.final_correction_mm_s = yaw_assist.final_correction_mm_s;
    app_line.final_left_target_mm_s = yaw_assist.left_target_mm_s;
    app_line.final_right_target_mm_s = yaw_assist.right_target_mm_s;
    WheelSpeed_SetTarget(yaw_assist.left_target_mm_s,
                         yaw_assist.right_target_mm_s);
}

void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    *snapshot = app_line;
    LineFollow_GetSnapshot(&snapshot->control);
    if (snapshot->state == APP_LINE_RUNNING) {
        snapshot->control.correction_mm_s =
            snapshot->final_correction_mm_s;
        snapshot->control.dynamic_base_mm_s =
            (snapshot->final_left_target_mm_s +
             snapshot->final_right_target_mm_s) * 0.5f;
        snapshot->control.left_target_mm_s =
            snapshot->final_left_target_mm_s;
        snapshot->control.right_target_mm_s =
            snapshot->final_right_target_mm_s;
    }
}
