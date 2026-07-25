#include "app_line.h"

#include "control_config.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "vehicle_config.h"
#include "wheel_speed.h"

#include <stddef.h>
#include <string.h>

static AppLineSnapshot_t app_line;

static void AppLine_ReturnIdle(void)
{
    app_line.state = APP_LINE_IDLE;
    app_line.deadline_ms = 0U;
}

void AppLine_Init(void)
{
    memset(&app_line, 0, sizeof(app_line));
    LineFollow_Init();
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
    if (!SystemState_RequestRun()) {
        AppLine_ReturnIdle();
        return false;
    }
    if (!LineFollow_Start(now_ms)) {
        (void)SystemState_RequestIdle();
        AppLine_ReturnIdle();
        return false;
    }
    Safety_BeginAction(now_ms, LINE_FOLLOW_ACTION_TIMEOUT_MS,
        SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE));
    WheelSpeed_Enable();
    app_line.state = APP_LINE_RUNNING;
    app_line.started_ms = now_ms;
    app_line.deadline_ms = now_ms + LINE_FOLLOW_ACTION_TIMEOUT_MS;
    return true;
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
    AppLine_ReturnIdle();
    return was_active;
}

void AppLine_ProcessGrayscale(const GrayscaleSnapshot_t *gray)
{
    if ((gray == NULL) || !gray->last_scan_ok) return;
    LineFollow_ProcessSample(gray->adc, gray->scan_count,
                             gray->last_update_ms);
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
        LineFollow_Stop();
        AppLine_ReturnIdle();
        return;
    }
    LineFollow_Update(now_ms);
    LineFollow_GetSnapshot(&control);
    if (control.state == LINE_FOLLOW_LOST_TIMEOUT) {
        WheelSpeed_Disable();
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
