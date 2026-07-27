#include "app_line.h"

#include "control_config.h"
#include "line_trace.h"
#include "m6_wire.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "vehicle_config.h"
#include "wheel_speed.h"

#include <stddef.h>
#include <string.h>

static AppLineSnapshot_t app_line;

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
    if (app_line.state != APP_LINE_IDLE) return false;
    return AppLine_StartRunning(now_ms, duration_ms);
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
    LineFollow_ProcessSample(gray->adc, gray->scan_count,
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
        control.correction_mm_s, 1.0f);
    frame.active_mask = control.active_mask;
    frame.active_count = control.active_count;
    frame.cluster_count = control.cluster_count;
    frame.state = (uint8_t)control.state;
    frame.search_direction = control.search_direction;
    frame.flags = control.branch_direction_latched ? 1U : 0U;
    LineTrace_Capture(&frame, control.state == LINE_FOLLOW_SEARCHING);
}

void AppLine_Task20ms(uint32_t now_ms)
{
    LineFollowSnapshot_t control;
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
        AppLine_ReturnIdle();
        return;
    }
    LineFollow_Update(now_ms);
    LineFollow_GetSnapshot(&control);
    if (control.state == LINE_FOLLOW_LOST_TIMEOUT) {
        WheelSpeed_Disable();
        LineTrace_Freeze();
        Safety_RaiseFault(SAFETY_FAULT_LINE_LOST, now_ms);
        AppLine_ReturnIdle();
        return;
    }
    WheelSpeed_SetTarget(control.left_target_mm_s,
                         control.right_target_mm_s);
}

void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    *snapshot = app_line;
    LineFollow_GetSnapshot(&snapshot->control);
}
