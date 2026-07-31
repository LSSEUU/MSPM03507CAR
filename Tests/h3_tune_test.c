#include "h3_tune.h"

#include "app_h3.h"
#include "lever_actuator.h"
#include "safety.h"
#include "system_state.h"

#include <assert.h>
#include <stdbool.h>
#include <string.h>

static bool app_active;
static bool app_start_allowed = true;
static bool safety_fault;
static SystemState_t system_state = SYSTEM_STATE_IDLE;
static LeverActuatorSnapshot_t lever;
static unsigned emergency_calls;
static unsigned center_start_calls;
static AppH3State_t app_state;
static AppH3Fault_t app_fault;

bool AppH3_IsActive(void) { return app_active; }
void AppH3_GetSnapshot(AppH3Snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->active = app_active;
    snapshot->state = app_state;
    snapshot->fault = app_fault;
}
bool AppH3_Start(uint32_t now_ms)
{
    (void)now_ms;
    if (!app_start_allowed || app_active) return false;
    app_active = true;
    return true;
}
bool AppH3_StartCenterHold(uint32_t now_ms)
{
    center_start_calls++;
    return AppH3_Start(now_ms);
}
void AppH3_RequestStop(uint32_t now_ms) { (void)now_ms; app_active = false; }
void AppH3_EmergencyStop(void) { app_active = false; emergency_calls++; }
void LeverActuator_GetSnapshot(LeverActuatorSnapshot_t *snapshot) { *snapshot = lever; }
bool Safety_HasFault(void) { return safety_fault; }
SystemState_t SystemState_Get(void) { return system_state; }

static void fixture(void)
{
    memset(&lever, 0, sizeof(lever));
    lever.boot_homed = true;
    lever.boot_state = LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL;
    app_active = false;
    app_state = APP_H3_WAIT_READY;
    app_fault = APP_H3_FAULT_NONE;
    app_start_allowed = true;
    safety_fault = false;
    system_state = SYSTEM_STATE_IDLE;
    emergency_calls = 0U;
    center_start_calls = 0U;
    H3Tune_Init(0U);
}

int main(void)
{
    H3TuneSnapshot_t snapshot;
    unsigned index;
    fixture();
    H3Tune_GetSnapshot(&snapshot);
    assert(H3Tune_ProfileValid(&snapshot.active));
    assert(snapshot.active.plus_max_mdeg == 15000U);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_MAX_MDEG, 22000U,
                             1U, 1U) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.stage_revision == 2U && snapshot.dirty);
    assert(H3Tune_Apply(1U, 1U) == H3_TUNE_STALE_STAGE);
    assert(H3Tune_Apply(1U, 2U) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.active_revision == 2U);
    assert(snapshot.active.plus_max_mdeg == 22000U);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_MAX_MDEG, 30001U,
                             2U, 2U) == H3_TUNE_OUT_OF_RANGE);

    assert(H3Tune_Authorize(123U, 9U, 1000U, 1000U, 10U) ==
           H3_TUNE_OUT_OF_RANGE);
    assert(H3Tune_Authorize(123U, 8U, 1000U, 1000U, 10U) == H3_TUNE_OK);
    for (index = 0U; index < 8U; index++) {
        assert(H3Tune_Start(123U, 20U + index) == H3_TUNE_OK);
        assert(app_active);
        app_active = false;
        H3Tune_Task20ms(21U + index);
    }
    assert(H3Tune_Start(123U, 50U) == H3_TUNE_AUTH_EXHAUSTED);
    assert(H3Tune_Revoke(123U, H3_TUNE_STOP_REVOKED) == H3_TUNE_OK);

    /* Authorization captures a RAM golden.  A heartbeat failure restores it
     * without needing to clear a fault or write flash. */
    H3Tune_GetSnapshot(&snapshot);
    assert(H3Tune_Authorize(456U, 2U, 1000U, 1000U, 100U) == H3_TUNE_OK);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_MAX_MDEG, 26000U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(H3Tune_Apply(snapshot.active_revision,
                        snapshot.stage_revision) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.active.plus_max_mdeg == 26000U);
    assert(H3Tune_Start(456U, 110U) == H3_TUNE_OK);
    H3Tune_Task20ms(411U);
    H3Tune_GetSnapshot(&snapshot);
    assert(!snapshot.authorized && !snapshot.host_run_active);
    assert(snapshot.active.plus_max_mdeg == 22000U);
    assert(snapshot.staged.plus_max_mdeg == 22000U);
    assert(snapshot.stop_reason == H3_TUNE_STOP_HEARTBEAT);
    assert(emergency_calls == 1U);

    /* An H3 fault that has already entered bounded RETURNING revokes the host
     * authorization without being overwritten by a second emergency stop. */
    fixture();
    assert(H3Tune_Authorize(654U, 1U, 1000U, 1000U, 200U) == H3_TUNE_OK);
    assert(H3Tune_Start(654U, 210U) == H3_TUNE_OK);
    app_state = APP_H3_RETURNING;
    app_fault = APP_H3_FAULT_TIMEOUT;
    H3Tune_Task20ms(220U);
    H3Tune_GetSnapshot(&snapshot);
    assert(!snapshot.authorized && !snapshot.host_run_active);
    assert(snapshot.stop_reason == H3_TUNE_STOP_FAULT);
    assert(app_active && emergency_calls == 0U);

    /* The PID scope consumes the same guarded quota but starts at target 0. */
    fixture();
    assert(H3Tune_Authorize(321U, 1U, 1000U, 1000U, 60U) == H3_TUNE_OK);
    assert(H3Tune_StartCenterHold(321U, 61U) == H3_TUNE_OK);
    assert(app_active && center_start_calls == 1U);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.host_run_active && snapshot.remaining_runs == 0U);
    assert(H3Tune_Stop(321U, 62U) == H3_TUNE_OK);
    assert(!app_active);
    H3Tune_Task20ms(63U);
    H3Tune_GetSnapshot(&snapshot);
    assert(!snapshot.host_run_active && snapshot.authorized);

    /* Running and unsafe states cannot mutate configuration. */
    fixture();
    app_active = true;
    assert(H3Tune_StageField(H3_TUNE_PARAM_KP, 110U, 1U, 1U) ==
           H3_TUNE_WRONG_STATE);
    app_active = false;
    safety_fault = true;
    assert(H3Tune_Revert(1U, 1U) == H3_TUNE_WRONG_STATE);

    fixture();
    assert(H3Tune_StageField(H3_TUNE_PARAM_KI, 20U, 1U, 1U) == H3_TUNE_OK);
    assert(H3Tune_Apply(1U, 2U) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.active.ki_mdeg_per_mm_s == 20U);
    assert(H3Tune_StageField(H3_TUNE_PARAM_KI, 101U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OUT_OF_RANGE);
    assert(H3Tune_StageField(H3_TUNE_PARAM_MINUS_BRAKE_TRIGGER_X10, 250U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.staged.minus_brake_trigger_x10 == 250U);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_BRAKE_MDEG, 12001U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OUT_OF_RANGE);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_BRAKE_TRIGGER_X10, 399U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OUT_OF_RANGE);
    assert(H3Tune_StageField(
               H3_TUNE_PARAM_PLUS_TOPUP_BRAKE_TRIGGER_X10, 260U,
               snapshot.active_revision,
               snapshot.stage_revision) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.staged.plus_topup_brake_trigger_x10 == 260U);
    assert(H3Tune_StageField(
               H3_TUNE_PARAM_MINUS_BRAKE_RETURN_PREDICT_MS, 251U,
               snapshot.active_revision,
               snapshot.stage_revision) == H3_TUNE_OUT_OF_RANGE);
    assert(H3Tune_StageField(H3_TUNE_PARAM_MINUS_RETURN_RPM, 8U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.staged.minus_return_rpm == 8U);
    assert(H3Tune_StageField(H3_TUNE_PARAM_MINUS_RETURN_RPM, 9U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OUT_OF_RANGE);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_TOPUP_MAX_SPEED_MM_S, 25U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OK);
    H3Tune_GetSnapshot(&snapshot);
    assert(snapshot.staged.plus_topup_max_speed_mm_s == 25U);
    assert(H3Tune_StageField(H3_TUNE_PARAM_PLUS_TOPUP_MAX_SPEED_MM_S, 31U,
                             snapshot.active_revision,
                             snapshot.stage_revision) == H3_TUNE_OUT_OF_RANGE);
    return 0;
}
