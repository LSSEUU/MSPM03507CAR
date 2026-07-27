#include "app_manual_drive.h"

#include "diagnostics.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "wheel_speed.h"

#include <string.h>

#define APP_MANUAL_MAX_SPEED_MM_S       400
#define APP_MANUAL_MIN_LEASE_MS         100U
#define APP_MANUAL_MAX_LEASE_MS         500U
#define APP_MANUAL_SAFETY_DEADLINE_MS   500U

static AppManualDriveSnapshot_t manual;

static bool valid_source(AppManualDriveSource_t source)
{
    return (source == APP_MANUAL_SOURCE_IR) ||
           (source == APP_MANUAL_SOURCE_VISION_UART) ||
           (source == APP_MANUAL_SOURCE_HOST_RPC);
}

static bool valid_target(int16_t value)
{
    return (value >= -APP_MANUAL_MAX_SPEED_MM_S) &&
           (value <= APP_MANUAL_MAX_SPEED_MM_S);
}

static void stop_active(AppManualDriveStopReason_t reason, uint32_t now_ms)
{
    if (!manual.active) return;
    WheelSpeed_Disable();
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    manual.active = false;
    manual.source = APP_MANUAL_SOURCE_NONE;
    manual.left_mm_s = 0;
    manual.right_mm_s = 0;
    manual.lease_deadline_ms = 0U;
    manual.last_update_ms = now_ms;
    manual.stop_reason = reason;
    manual.transition_id++;
    if (reason == APP_MANUAL_STOP_LEASE_EXPIRED) manual.lease_stops++;
}

void AppManualDrive_Init(void)
{
    memset(&manual, 0, sizeof(manual));
}

bool AppManualDrive_Submit(AppManualDriveSource_t source,
                           int16_t left_mm_s, int16_t right_mm_s,
                           uint16_t lease_ms, uint32_t required_sensor_mask,
                           uint32_t command_id, uint32_t now_ms)
{
    bool source_changed;
    if (!valid_source(source) || !valid_target(left_mm_s) ||
        !valid_target(right_mm_s) ||
        (lease_ms < APP_MANUAL_MIN_LEASE_MS) ||
        (lease_ms > APP_MANUAL_MAX_LEASE_MS) ||
        ((left_mm_s == 0) && (right_mm_s == 0))) {
        manual.rejected_commands++;
        return false;
    }
    if (Safety_HasFault()) {
        manual.rejected_commands++;
        return false;
    }
    if (manual.active && (source < manual.source)) {
        manual.rejected_commands++;
        return false;
    }

    source_changed = manual.active && (manual.source != source);
    if (source_changed) stop_active(APP_MANUAL_STOP_PREEMPTED, now_ms);
    if (!manual.active) {
        if ((SystemState_Get() != SYSTEM_STATE_IDLE) ||
            !Diagnostics_ActionIsIdle() || !SystemState_RequestRun()) {
            manual.rejected_commands++;
            return false;
        }
        WheelSpeed_Enable();
        manual.active = true;
        manual.source = source;
        manual.stop_reason = APP_MANUAL_STOP_NONE;
        manual.transition_id++;
    }

    WheelSpeed_SetTarget((float)left_mm_s, (float)right_mm_s);
    Safety_BeginAction(now_ms, APP_MANUAL_SAFETY_DEADLINE_MS,
                       required_sensor_mask);
    manual.left_mm_s = left_mm_s;
    manual.right_mm_s = right_mm_s;
    manual.command_id = command_id;
    manual.last_update_ms = now_ms;
    manual.lease_deadline_ms = now_ms + lease_ms;
    manual.accepted_commands++;
    return true;
}

bool AppManualDrive_Stop(AppManualDriveSource_t source,
                         AppManualDriveStopReason_t reason,
                         uint32_t now_ms)
{
    if (!manual.active) return false;
    if ((source != APP_MANUAL_SOURCE_NONE) && (source != manual.source))
        return false;
    stop_active(reason, now_ms);
    return true;
}

bool AppManualDrive_SubmitVision(int16_t left_mm_s, int16_t right_mm_s,
                                 uint16_t lease_ms, uint32_t command_id,
                                 uint32_t now_ms)
{
    return AppManualDrive_Submit(APP_MANUAL_SOURCE_VISION_UART,
        left_mm_s, right_mm_s, lease_ms, 0U, command_id, now_ms);
}

bool AppManualDrive_StopVision(uint32_t now_ms)
{
    return AppManualDrive_Stop(APP_MANUAL_SOURCE_VISION_UART,
        APP_MANUAL_STOP_RELEASE, now_ms);
}

void AppManualDrive_Task20ms(uint32_t now_ms)
{
    if (Safety_HasFault()) {
        if (manual.active) {
            manual.active = false;
            manual.source = APP_MANUAL_SOURCE_NONE;
            manual.left_mm_s = 0;
            manual.right_mm_s = 0;
            manual.lease_deadline_ms = 0U;
            manual.last_update_ms = now_ms;
            manual.stop_reason = APP_MANUAL_STOP_FAULT;
            manual.transition_id++;
        }
        return;
    }
    if (manual.active && SystemTime_Reached(now_ms,
                                             manual.lease_deadline_ms))
        stop_active(APP_MANUAL_STOP_LEASE_EXPIRED, now_ms);
}

bool AppManualDrive_IsIdle(void)
{
    return !manual.active;
}

void AppManualDrive_GetSnapshot(AppManualDriveSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = manual;
}
