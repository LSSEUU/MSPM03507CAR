#include "app_line.h"

#include "control_config.h"
#include "h_course_profile.h"
#include "h2_control_profile.h"
#include "h_mission_control_profile.h"
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
#define APP_H4_AB_DISTANCE_MM          1500.0f
#define APP_H4_HEADING_HOLD_DISTANCE_MM 1400.0f
#define APP_H_MISSION_WHITE_LIMIT_MS      250U
#define APP_H_MISSION_OUTER_LIMIT_MS     1200U
#define APP_H_MISSION_SATURATION_MS      1200U
#define APP_H4_MISSION_STOP_TIMEOUT_MS   1500U
#define APP_H5_MISSION_STOP_TIMEOUT_MS   2000U
#define APP_H_MISSION_STOP_DISTANCE_MM    250.0f
#define APP_H_MISSION_STOPPED_SPEED_MM_S    5.0f
#define APP_H_MISSION_TASK_PERIOD_S         0.020f
#define APP_H4_STOP_CORRECTION_RATIO         0.20f
#define APP_H5_CURVE_CORRECTION_MARGIN        1.15f

static AppLineSnapshot_t app_line;
static uint16_t mission_white_ms;
static uint16_t mission_outer_ms;
static uint16_t mission_saturation_ms;
static uint32_t mission_stop_started_ms;
static float mission_stop_base_mm_s;
static float mission_stop_started_distance_mm;
static bool mission_stop_heading_valid;
static float mission_stop_heading_deg;
static bool mission_h4_heading_hold_active;

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
    bool restored = HMissionControlProfile_Restore();
    restored = H2ControlProfile_Restore() && restored;
    if (!restored) {
        Safety_EmergencyStop(SystemTime_Now());
    }
    app_line.state = APP_LINE_IDLE;
    app_line.deadline_ms = 0U;
}

static float AppLine_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint16_t AppLine_AddGuardTime(uint16_t value)
{
    return (value > (uint16_t)(65535U - 20U)) ? 65535U :
        (uint16_t)(value + 20U);
}

static void AppLine_ResetMissionGuards(void)
{
    mission_white_ms = 0U;
    mission_outer_ms = 0U;
    mission_saturation_ms = 0U;
    mission_stop_started_ms = 0U;
    mission_stop_base_mm_s = 0.0f;
    mission_stop_started_distance_mm = 0.0f;
    mission_stop_heading_valid = false;
    mission_stop_heading_deg = 0.0f;
    mission_h4_heading_hold_active = false;
}

static void AppLine_FaultAbort(uint32_t now_ms,
                               HMissionStopReason_t reason,
                               uint32_t fault_bits)
{
    WheelSpeed_Disable();
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    LineTrace_Freeze();
    LineFollow_Stop();
    HCourse_Stop();
    AppLine_ResetYawAssist();
    if (HMission_IsActive()) HMission_Abort(reason);
    if (fault_bits != 0U) Safety_RaiseFault(fault_bits, now_ms);
    AppLine_ReturnIdle();
}

static void AppLine_ResetFinish(bool enabled)
{
    app_line.h2_finish_enabled = enabled;
    app_line.finish_line_detected = false;
    app_line.finish_crawl_active = false;
    app_line.finish_complete = false;
    app_line.finish_line_distance_mm = 0.0f;
    app_line.finish_travel_mm = 0.0f;
}

static void AppLine_CompleteFinish(uint32_t now_ms)
{
    HMissionSnapshot_t mission;
    HMission_GetSnapshot(&mission);
    WheelSpeed_Disable();
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    LineFollow_Stop();
    HCourse_Stop();
    AppLine_ResetYawAssist();
    LineTrace_Freeze();
    app_line.finish_crawl_active = false;
    app_line.finish_complete = true;
    app_line.finished_ms = now_ms;
    app_line.elapsed_ms = (uint32_t)(now_ms - app_line.started_ms);
    if (HMission_IsActive() &&
        (mission.running_mode == H_MISSION_MODE_H2)) {
        HMission_Latch(now_ms, H_MISSION_STOP_H2_FINISH);
        HMission_BeginStopping();
        HMission_Complete();
    }
    AppLine_ReturnIdle();
}

static void AppLine_CompleteMissionStop(void)
{
    WheelSpeed_Disable();
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    LineFollow_Stop();
    HCourse_Stop();
    AppLine_ResetYawAssist();
    LineTrace_Freeze();
    HMission_Complete();
    AppLine_ReturnIdle();
}

static bool AppLine_StartRunning(uint32_t now_ms, uint32_t duration_ms,
                                 bool h_mission,
                                 HMissionMode_t mission_mode)
{
    LineFollowSnapshot_t snapshot;
    LineFollow_GetSnapshot(&snapshot);
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
        !snapshot.calibration.valid || !snapshot.line_valid)
        return false;
    if (!SystemState_RequestRun()) return false;
    if (h_mission && (mission_mode == H_MISSION_MODE_H2) &&
        !H2ControlProfile_Activate()) {
        (void)SystemState_RequestIdle();
        AppLine_ReturnIdle();
        return false;
    }
    if (h_mission &&
        ((mission_mode == H_MISSION_MODE_H4) ||
         (mission_mode == H_MISSION_MODE_H5)) &&
        !HMissionControlProfile_Activate(mission_mode)) {
        (void)SystemState_RequestIdle();
        AppLine_ReturnIdle();
        return false;
    }
    if (!LineFollow_Start(now_ms)) {
        (void)SystemState_RequestIdle();
        AppLine_ReturnIdle();
        return false;
    }
    if (duration_ms == M6_LINE_ACTION_CONTINUOUS_DURATION_MS)
        Safety_BeginContinuousAction(now_ms,
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE) |
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO));
    else
        Safety_BeginAction(now_ms, duration_ms,
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE));
    WheelSpeed_Enable();
    AppLine_ResetYawAssist();
    HCourse_Start(now_ms);
    AppLine_ResetFinish(h_mission &&
        (mission_mode == H_MISSION_MODE_H2));
    if (app_line.h2_finish_enabled) {
        HCourse_SetFinishEnabled(true);
    } else {
        HCourse_SetFinishMode(
            (h_mission && (mission_mode == H_MISSION_MODE_H5)) ?
            H_COURSE_FINISH_H5_LATCH : H_COURSE_FINISH_NONE);
    }
    AppLine_ResetMissionGuards();
    if (h_mission) HMission_Start(mission_mode, now_ms);
    app_line.state = APP_LINE_RUNNING;
    app_line.started_ms = now_ms;
    app_line.finished_ms = 0U;
    app_line.elapsed_ms = 0U;
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
    HMission_Init();
    H2ControlProfile_Init();
    HMissionControlProfile_Init();
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
    return AppLine_StartRunning(now_ms, LINE_FOLLOW_ACTION_TIMEOUT_MS,
                                false, H_MISSION_MODE_H2);
}

bool AppLine_StartTimed(uint32_t now_ms, uint32_t duration_ms)
{
    bool started;
    if (app_line.state != APP_LINE_IDLE) return false;
    started = AppLine_StartRunning(now_ms, duration_ms,
        duration_ms == M6_LINE_ACTION_CONTINUOUS_DURATION_MS,
        H_MISSION_MODE_H2);
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

bool AppLine_StartH2(uint32_t now_ms)
{
    return AppLine_StartTimed(now_ms,
        M6_LINE_ACTION_CONTINUOUS_DURATION_MS);
}

bool AppLine_StartH4(uint32_t now_ms)
{
    if (app_line.state != APP_LINE_IDLE) return false;
    return AppLine_StartRunning(now_ms,
        M6_LINE_ACTION_CONTINUOUS_DURATION_MS, true,
        H_MISSION_MODE_H4);
}

bool AppLine_StartH5(uint32_t now_ms)
{
    if (app_line.state != APP_LINE_IDLE) return false;
    return AppLine_StartRunning(now_ms,
        M6_LINE_ACTION_CONTINUOUS_DURATION_MS, true,
        H_MISSION_MODE_H5);
}

bool AppLine_StartMission(HMissionMode_t mode, uint32_t now_ms)
{
    if (mode == H_MISSION_MODE_H2) return AppLine_StartH2(now_ms);
    if (mode == H_MISSION_MODE_H4) return AppLine_StartH4(now_ms);
    if (mode == H_MISSION_MODE_H5) return AppLine_StartH5(now_ms);
    return false;
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
    if (HMission_IsActive())
        HMission_Abort(H_MISSION_STOP_USER_CANCEL);
    AppLine_ReturnIdle();
    return was_active;
}

bool AppLine_AbortHMission(uint32_t now_ms, HMissionStopReason_t reason)
{
    if (!HMission_IsActive()) return false;
    AppLine_FaultAbort(now_ms, reason, 0U);
    return true;
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
    WheelSpeedConfig_t wheel_config;
    HCourseSnapshot_t course;
    HMissionSnapshot_t mission;
    LineYawAssistSnapshot_t yaw_assist;
    bool gyro_usable;
    bool guarded_mission;
    bool mission_path_guard_active;
    bool h4_heading_hold;
    bool latch_development;
    float yaw_correction_limit_mm_s;
    if (app_line.state == APP_LINE_ARMED) {
        if (SystemTime_Reached(now_ms, app_line.deadline_ms))
            AppLine_ReturnIdle();
        return;
    }
    if (app_line.state != APP_LINE_RUNNING) return;
    if (Safety_HasFault()) {
        AppLine_FaultAbort(now_ms, H_MISSION_STOP_FAULT, 0U);
        return;
    }
    LineFollow_Update(now_ms);
    LineFollow_GetSnapshot(&control);
    if (control.state == LINE_FOLLOW_LOST_TIMEOUT) {
        AppLine_FaultAbort(now_ms, H_MISSION_STOP_LINE_WHITE,
                             SAFETY_FAULT_LINE_LOST);
        return;
    }
    Heading_GetSnapshot(&heading);
    Heading_GetConfig(&heading_config);
    WheelSpeed_GetSnapshot(&wheel);
    WheelSpeed_GetConfig(&wheel_config);
    LineFollow_GetConfig(&line_config);
    HMission_GetSnapshot(&mission);
    guarded_mission = HMission_IsActive() &&
        (mission.running_mode != H_MISSION_MODE_H2);
    mission_path_guard_active = guarded_mission &&
        (mission.state == H_MISSION_RUNNING);
    gyro_usable = heading.calibrated &&
        (heading.rate_source == HEADING_RATE_GYRO_FRAME) &&
        ((uint32_t)(now_ms - heading.last_rate_update_ms) <=
         LINE_YAW_ASSIST_GYRO_FRESH_MS);
    if (HMission_IsActive() && !gyro_usable) {
        AppLine_FaultAbort(now_ms, H_MISSION_STOP_GYRO,
                             SAFETY_FAULT_GYRO_TIMEOUT);
        return;
    }
    if (guarded_mission) {
        mission_white_ms = mission_path_guard_active &&
            (control.active_mask == 0U) ?
            AppLine_AddGuardTime(mission_white_ms) : 0U;
        mission_outer_ms = mission_path_guard_active &&
            ((control.active_mask == 0x01U) ||
             (control.active_mask == 0x80U)) ?
            AppLine_AddGuardTime(mission_outer_ms) : 0U;
        mission_saturation_ms = (wheel.saturated[WHEEL_SPEED_LEFT] ||
                                 wheel.saturated[WHEEL_SPEED_RIGHT]) ?
            AppLine_AddGuardTime(mission_saturation_ms) : 0U;
        if (mission_white_ms > APP_H_MISSION_WHITE_LIMIT_MS) {
            AppLine_FaultAbort(now_ms, H_MISSION_STOP_LINE_WHITE,
                                 SAFETY_FAULT_LINE_LOST);
            return;
        }
        if (mission_outer_ms > APP_H_MISSION_OUTER_LIMIT_MS) {
            AppLine_FaultAbort(now_ms, H_MISSION_STOP_OUTER_SENSOR,
                                 SAFETY_FAULT_LINE_LOST);
            return;
        }
        if (mission_saturation_ms > APP_H_MISSION_SATURATION_MS) {
            AppLine_FaultAbort(now_ms, H_MISSION_STOP_SATURATION,
                                 SAFETY_FAULT_ACTION_TIMEOUT);
            return;
        }
    }
    HCourse_Update(now_ms, wheel.sample_valid,
        wheel.filtered_mm_s[WHEEL_SPEED_LEFT],
        wheel.filtered_mm_s[WHEEL_SPEED_RIGHT],
        gyro_usable, heading.yaw_deg,
        gyro_usable ? heading.yaw_rate_deg_s : 0.0f,
        (control.state == LINE_FOLLOW_TRACKING) &&
        control.line_valid && !control.ambiguous,
        control.active_mask, control.position);
    HCourse_GetSnapshot(&course);
    if (guarded_mission &&
        (mission.state == H_MISSION_RUNNING) &&
        (mission.running_mode == H_MISSION_MODE_H4) &&
        !mission_h4_heading_hold_active &&
        (course.total_distance_mm >= APP_H4_HEADING_HOLD_DISTANCE_MM)) {
        mission_h4_heading_hold_active = true;
        mission_stop_heading_valid = gyro_usable;
        mission_stop_heading_deg = heading.yaw_deg;
        LineYawAssist_Reset();
    }
    if (app_line.h2_finish_enabled && course.finish_line_detected) {
        app_line.finish_line_detected = true;
        app_line.finish_crawl_active = true;
        app_line.finish_line_distance_mm =
            course.finish_line_distance_mm;
        app_line.finish_travel_mm = course.total_distance_mm -
            course.finish_line_distance_mm;
        if (app_line.finish_travel_mm >= H2_FINISH_STOP_OFFSET_MM) {
            AppLine_CompleteFinish(now_ms);
            return;
        }
    }
    HMission_GetSnapshot(&mission);
    latch_development = guarded_mission &&
        (mission.state == H_MISSION_RUNNING) &&
        (mission.active.development_end_distance_mm > 0.0f) &&
        (course.total_distance_mm >=
         mission.active.development_end_distance_mm);
    if (latch_development) {
        HMission_Latch(now_ms, H_MISSION_STOP_DEVELOPMENT_DISTANCE);
    } else if (guarded_mission &&
               (mission.state == H_MISSION_RUNNING) &&
               (mission.running_mode == H_MISSION_MODE_H4) &&
               (course.total_distance_mm >=
                APP_H4_AB_DISTANCE_MM + mission.active.h4_b_margin_mm)) {
        HMission_Latch(now_ms, H_MISSION_STOP_H4_DISTANCE);
    } else if (guarded_mission &&
               (mission.state == H_MISSION_RUNNING) &&
               (mission.running_mode == H_MISSION_MODE_H5) &&
               course.finish_line_detected) {
        app_line.finish_line_detected = true;
        app_line.finish_line_distance_mm = course.finish_line_distance_mm;
        HMission_Latch(now_ms, H_MISSION_STOP_H5_FINISH_LINE);
    }
    HMission_GetSnapshot(&mission);
    if (mission.state == H_MISSION_COMPLETION_LATCHED) {
        mission_stop_base_mm_s =
            (AppLine_Abs(wheel.ramped_mm_s[WHEEL_SPEED_LEFT]) +
             AppLine_Abs(wheel.ramped_mm_s[WHEEL_SPEED_RIGHT])) * 0.5f;
        if (mission_stop_base_mm_s < 10.0f) {
            mission_stop_base_mm_s =
                (mission.running_mode == H_MISSION_MODE_H4) ?
                mission.active.h4_straight_speed_mm_s :
                (HCourse_IsCurvePhase() ?
                 mission.active.h5_curve_speed_mm_s :
                 mission.active.h5_straight_speed_mm_s);
        }
        mission_stop_started_ms = now_ms;
        mission_stop_started_distance_mm = course.total_distance_mm;
        if ((mission.running_mode == H_MISSION_MODE_H4) &&
            !mission_h4_heading_hold_active) {
            mission_h4_heading_hold_active = true;
            mission_stop_heading_valid = gyro_usable;
            mission_stop_heading_deg = heading.yaw_deg;
            LineYawAssist_Reset();
        }
        app_line.finished_ms = now_ms;
        app_line.elapsed_ms = (uint32_t)(now_ms - app_line.started_ms);
        HMission_BeginStopping();
        HMission_GetSnapshot(&mission);
    }
    if (mission.state == H_MISSION_STOPPING) {
        uint32_t stop_timeout_ms =
            (mission.running_mode == H_MISSION_MODE_H4) ?
            APP_H4_MISSION_STOP_TIMEOUT_MS :
            APP_H5_MISSION_STOP_TIMEOUT_MS;
        if ((uint32_t)(now_ms - mission_stop_started_ms) >
            stop_timeout_ms) {
            AppLine_FaultAbort(now_ms, H_MISSION_STOP_TIMEOUT,
                                 SAFETY_FAULT_ACTION_TIMEOUT);
            return;
        }
        if ((course.total_distance_mm - mission_stop_started_distance_mm) >
            APP_H_MISSION_STOP_DISTANCE_MM) {
            AppLine_FaultAbort(now_ms, H_MISSION_STOP_DISTANCE,
                                 SAFETY_FAULT_ACTION_TIMEOUT);
            return;
        }
        mission_stop_base_mm_s -= wheel_config.decel_mm_s2 *
            APP_H_MISSION_TASK_PERIOD_S;
        if (mission_stop_base_mm_s < 0.0f)
            mission_stop_base_mm_s = 0.0f;
        if ((mission_stop_base_mm_s <= 0.5f) &&
            (AppLine_Abs(wheel.ramped_mm_s[WHEEL_SPEED_LEFT]) <=
             APP_H_MISSION_STOPPED_SPEED_MM_S) &&
            (AppLine_Abs(wheel.ramped_mm_s[WHEEL_SPEED_RIGHT]) <=
             APP_H_MISSION_STOPPED_SPEED_MM_S) &&
            (AppLine_Abs(wheel.filtered_mm_s[WHEEL_SPEED_LEFT]) <=
             APP_H_MISSION_STOPPED_SPEED_MM_S) &&
            (AppLine_Abs(wheel.filtered_mm_s[WHEEL_SPEED_RIGHT]) <=
             APP_H_MISSION_STOPPED_SPEED_MM_S)) {
            AppLine_CompleteMissionStop();
            return;
        }
    }
    h4_heading_hold = guarded_mission &&
        mission_h4_heading_hold_active &&
        (mission.running_mode == H_MISSION_MODE_H4);
    assisted_control = control;
    if (h4_heading_hold) {
        assisted_control.active_mask = 0x18U;
        assisted_control.active_count = 2U;
        assisted_control.cluster_count = 1U;
        assisted_control.line_valid = true;
        assisted_control.ambiguous = false;
        assisted_control.raw_position = 0.0f;
        assisted_control.position = 0.0f;
        assisted_control.raw_derivative = 0.0f;
        assisted_control.filtered_derivative = 0.0f;
        assisted_control.raw_correction_mm_s = 0.0f;
        assisted_control.correction_mm_s = 0.0f;
    }
    assisted_control.dynamic_base_mm_s = HCourse_SelectBaseSpeed(
        control.dynamic_base_mm_s, line_config.speed_min_mm_s,
        line_config.speed_max_mm_s,
        (control.state == LINE_FOLLOW_TRACKING) &&
        control.line_valid && !control.ambiguous,
        control.position);
    if (guarded_mission) {
        float mission_speed =
            (mission.running_mode == H_MISSION_MODE_H4) ?
            mission.active.h4_straight_speed_mm_s :
            (HCourse_IsCurvePhase() ?
             mission.active.h5_curve_speed_mm_s :
             mission.active.h5_straight_speed_mm_s);
        if (assisted_control.dynamic_base_mm_s > mission_speed)
            assisted_control.dynamic_base_mm_s = mission_speed;
        if ((mission.state == H_MISSION_STOPPING) &&
            (assisted_control.dynamic_base_mm_s > mission_stop_base_mm_s))
            assisted_control.dynamic_base_mm_s = mission_stop_base_mm_s;
    }
    assisted_control.left_target_mm_s =
        assisted_control.dynamic_base_mm_s +
        assisted_control.correction_mm_s;
    assisted_control.right_target_mm_s =
        assisted_control.dynamic_base_mm_s -
        assisted_control.correction_mm_s;
    LineYawAssist_SetMode(h4_heading_hold ?
        LINE_YAW_ASSIST_MODE_STRAIGHT : HCourse_IsCurvePhase() ?
        LINE_YAW_ASSIST_MODE_CURVE_POSITIVE :
        LINE_YAW_ASSIST_MODE_STRAIGHT);
    LineYawAssist_SetCurveFeedforwardScale(
        HCourse_SelectCurveFeedforwardScale());
    LineYawAssist_SetStraightReference(h4_heading_hold ?
        mission_stop_heading_valid : course.straight_heading_target_valid,
        h4_heading_hold ? mission_stop_heading_deg :
        course.straight_heading_target_deg);
    yaw_correction_limit_mm_s = line_config.correction_limit_mm_s;
    if (guarded_mission &&
        (mission.running_mode == H_MISSION_MODE_H5) &&
        HCourse_IsCurvePhase()) {
        float geometric_curve_correction =
            assisted_control.dynamic_base_mm_s * CHASSIS_DRIVE_TRACK_MM /
            (2.0f * LINE_CURVE_RADIUS_MM) *
            APP_H5_CURVE_CORRECTION_MARGIN;
        if (yaw_correction_limit_mm_s > geometric_curve_correction)
            yaw_correction_limit_mm_s = geometric_curve_correction;
    }
    LineYawAssist_Update(&assisted_control,
        yaw_correction_limit_mm_s,
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
    if ((guarded_mission && (mission.state == H_MISSION_STOPPING)) ||
        h4_heading_hold) {
        float correction = yaw_assist.final_correction_mm_s;
        float correction_limit = assisted_control.dynamic_base_mm_s;
        if (h4_heading_hold)
            correction_limit *= APP_H4_STOP_CORRECTION_RATIO;
        if (correction > correction_limit)
            correction = correction_limit;
        if (correction < -correction_limit)
            correction = -correction_limit;
        app_line.final_correction_mm_s = correction;
        app_line.final_left_target_mm_s =
            assisted_control.dynamic_base_mm_s + correction;
        app_line.final_right_target_mm_s =
            assisted_control.dynamic_base_mm_s - correction;
    } else {
        app_line.final_correction_mm_s =
            yaw_assist.final_correction_mm_s;
        app_line.final_left_target_mm_s = yaw_assist.left_target_mm_s;
        app_line.final_right_target_mm_s = yaw_assist.right_target_mm_s;
    }
    if (guarded_mission &&
        ((app_line.final_left_target_mm_s < 0.0f) ||
         (app_line.final_right_target_mm_s < 0.0f))) {
        AppLine_FaultAbort(now_ms, H_MISSION_STOP_NEGATIVE_TARGET,
                             SAFETY_FAULT_LINE_LOST);
        return;
    }
    WheelSpeed_SetTarget(app_line.final_left_target_mm_s,
                         app_line.final_right_target_mm_s);
    if (HMission_IsActive()) {
        HMission_Update(now_ms, course.total_distance_mm,
            app_line.final_left_target_mm_s,
            app_line.final_right_target_mm_s,
            mission_white_ms, mission_outer_ms, mission_saturation_ms);
    }
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
