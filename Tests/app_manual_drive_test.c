#include "app_manual_drive.h"
#include "diagnostics.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "wheel_speed.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

static bool faulted;
static bool diagnostics_idle;
static bool wheel_enabled;
static SystemState_t system_state;
static float left_target;
static float right_target;
static uint32_t safety_timeout;

bool Safety_HasFault(void) { return faulted; }
void Safety_BeginAction(uint32_t now_ms, uint32_t timeout_ms,
                        uint32_t required_sensor_mask)
{
    (void)now_ms;
    (void)required_sensor_mask;
    safety_timeout = timeout_ms;
}
void Safety_EndAction(void) { safety_timeout = 0U; }
bool Diagnostics_ActionIsIdle(void) { return diagnostics_idle; }
SystemState_t SystemState_Get(void) { return system_state; }
bool SystemState_RequestRun(void)
{
    if (system_state != SYSTEM_STATE_IDLE) return false;
    system_state = SYSTEM_STATE_RUN;
    return true;
}
bool SystemState_RequestIdle(void)
{
    if (system_state == SYSTEM_STATE_FAULT) return false;
    system_state = SYSTEM_STATE_IDLE;
    return true;
}
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}
void WheelSpeed_Enable(void) { wheel_enabled = true; }
void WheelSpeed_Disable(void)
{
    wheel_enabled = false;
    left_target = right_target = 0.0f;
}
void WheelSpeed_SetTarget(float left_mm_s, float right_mm_s)
{
    left_target = left_mm_s;
    right_target = right_mm_s;
}

static void reset_fixture(void)
{
    faulted = false;
    diagnostics_idle = true;
    wheel_enabled = false;
    system_state = SYSTEM_STATE_IDLE;
    left_target = right_target = 0.0f;
    safety_timeout = 0U;
    AppManualDrive_Init();
}

static void test_priority_and_renewal(void)
{
    AppManualDriveSnapshot_t before;
    AppManualDriveSnapshot_t after;
    reset_fixture();
    assert(AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
        160, 80, 300U, 0U, 1U, 10U));
    AppManualDrive_GetSnapshot(&before);
    assert(before.active && before.source == APP_MANUAL_SOURCE_HOST_RPC);
    assert(wheel_enabled && left_target == 160.0f && right_target == 80.0f);
    assert(safety_timeout == 500U);
    assert(!AppManualDrive_Submit(APP_MANUAL_SOURCE_IR,
        100, 100, 250U, 0U, 0U, 20U));
    assert(AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
        120, -60, 300U, 0U, 2U, 100U));
    AppManualDrive_GetSnapshot(&after);
    assert(after.transition_id == before.transition_id);
    assert(after.command_id == 2U && after.lease_deadline_ms == 400U);
    assert(AppManualDrive_Stop(APP_MANUAL_SOURCE_HOST_RPC,
        APP_MANUAL_STOP_RELEASE, 110U));
    assert(system_state == SYSTEM_STATE_IDLE && !wheel_enabled && !faulted);
}

static void test_source_preemption_and_lease(void)
{
    AppManualDriveSnapshot_t snapshot;
    reset_fixture();
    assert(AppManualDrive_Submit(APP_MANUAL_SOURCE_IR,
        100, 100, 250U, 0U, 0U, 0U));
    assert(AppManualDrive_SubmitVision(-120, 120, 200U, 7U, 20U));
    AppManualDrive_GetSnapshot(&snapshot);
    assert(snapshot.source == APP_MANUAL_SOURCE_VISION_UART);
    assert(AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
        200, 200, 300U, 0U, 8U, 30U));
    AppManualDrive_Task20ms(329U);
    assert(wheel_enabled);
    AppManualDrive_Task20ms(330U);
    AppManualDrive_GetSnapshot(&snapshot);
    assert(!snapshot.active && snapshot.stop_reason == APP_MANUAL_STOP_LEASE_EXPIRED);
    assert(snapshot.lease_stops == 1U && !faulted);
}

static void test_fault_and_rejections(void)
{
    AppManualDriveSnapshot_t snapshot;
    reset_fixture();
    diagnostics_idle = false;
    assert(!AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
        100, 100, 300U, 0U, 1U, 0U));
    diagnostics_idle = true;
    assert(!AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
        401, 0, 300U, 0U, 2U, 0U));
    assert(AppManualDrive_Submit(APP_MANUAL_SOURCE_HOST_RPC,
        100, 100, 300U, 0U, 3U, 0U));
    faulted = true;
    system_state = SYSTEM_STATE_FAULT;
    AppManualDrive_Task20ms(20U);
    AppManualDrive_GetSnapshot(&snapshot);
    assert(!snapshot.active && snapshot.stop_reason == APP_MANUAL_STOP_FAULT);
}

int main(void)
{
    test_priority_and_renewal();
    test_source_preemption_and_lease();
    test_fault_and_rejections();
    return 0;
}
