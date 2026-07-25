#include "control_protocol.h"

#include "app_remote.h"
#include "m6_telemetry.h"
#include "motor.h"
#include "parameter_store.h"
#include "safety.h"
#include "system_state.h"
#include "uart_debug.h"
#include "wheel_speed.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static char output[4096];
static size_t output_length;
static uint32_t emergency_epoch;
static DebugStats_t debug_stats;
static bool diagnostic_active;
static bool actions_idle = true;
static bool faulted;
static SystemState_t state = SYSTEM_STATE_IDLE;
static bool logging_enabled;
static uint16_t logging_period;
static uint32_t callback_count;
static ControlParamsCommit_t last_commit;

int debug_try_write(const char *data, size_t length)
{
    assert(output_length + length < sizeof(output));
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = '\0';
    return 1;
}

uint32_t debug_get_emergency_epoch(void) { return emergency_epoch; }
void debug_get_stats(DebugStats_t *stats) { *stats = debug_stats; }

bool Diagnostics_WheelSpeedDiagnosticActive(void) { return diagnostic_active; }
bool Diagnostics_ActionIsIdle(void) { return actions_idle; }

void M6Telemetry_SetLogging(bool enabled, uint16_t period_ms)
{
    logging_enabled = enabled;
    logging_period = period_ms;
}

void AppRemote_GetSnapshot(AppRemoteSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state = APP_REMOTE_IDLE;
}

void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
}

void Safety_GetSnapshot(SafetySnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
}

void Motor_GetStats(MotorStats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
}

bool Safety_HasFault(void) { return faulted; }
SystemState_t SystemState_Get(void) { return state; }

ParameterStoreResult_t ParameterStore_LoadStaged(void)
{
    return ControlParams_StageExternal(ControlParams_Defaults(),
        CONTROL_PARAMS_SOURCE_PERSISTED) == CONTROL_PARAMS_OK ?
        PARAMETER_STORE_OK : PARAMETER_STORE_ERR_PARAMS;
}

ParameterStoreResult_t ParameterStore_SaveActive(void)
{
    return PARAMETER_STORE_OK;
}

void ParameterStore_GetInfo(ParameterStoreInfo_t *info)
{
    memset(info, 0, sizeof(*info));
    info->selected_slot = PARAMETER_STORE_SLOT_A;
    info->health = PARAMETER_STORE_HEALTH_OK;
    info->generation = 9U;
}

const char *ParameterStore_SourceName(void) { return "flash-a"; }

static void committed(const ControlParamsCommit_t *commit)
{
    callback_count++;
    last_commit = *commit;
}

static void clear_output(void)
{
    output_length = 0U;
    output[0] = '\0';
}

static void feed(const char *text)
{
    while (*text != '\0')
        (void)ControlProtocol_ProcessByte(*text++, 100U);
    ControlProtocol_Service();
}

static void expect(const char *text)
{
    assert(strcmp(output, text) == 0);
    clear_output();
}

static void test_staging_and_apply(void)
{
    feed(":1,get,meta.schema\r\n");
    expect("ok,1,meta.schema,1\r\n");

    feed(":2,set,wheel.left.kp,0.9\n");
    expect("ok,2,staged,wheel.left.kp,0.9\r\n");
    assert(ControlParams_Active()->wheel.left_kp != 0.9f);
    feed(":3,get,wheel.left.kp\n");
    assert(strstr(output, "ok,3,wheel.left.kp,0.7,0.9\r\n") != NULL);
    clear_output();
    feed(":4,apply\n");
    expect("ok,4,applied,2\r\n");
    assert(fabsf(ControlParams_Active()->wheel.left_kp - 0.9f) < 0.0001f);
    assert(callback_count == 1U);
}

static void test_fragmentation_errors_and_emergency_epoch(void)
{
    feed(":0,get,meta.schema\n");
    expect("err,0,frame\r\n");
    feed(":5,set,wheel.left.ki,");
    assert(output_length == 0U);
    feed("nan\n");
    expect("err,5,value\r\n");
    feed(":6,set,wheel.left.ki,1e2\n");
    expect("err,6,value\r\n");
    feed(":7,GET,meta.schema\n");
    expect("err,7,frame\r\n");
    feed(":8,set,wheel.left.kp,4\n");
    expect("err,8,range\r\n");
    feed(":9,set,wheel.left.kp,");
    emergency_epoch++;
    ControlProtocol_Service();
    feed("1\n");
    assert(output_length == 0U);

    feed(":10,get,meta.revision\n:11,get,meta.dirty\n");
    expect("ok,10,meta.revision,2\r\nok,11,meta.dirty,0\r\n");
}

static void test_diagnostic_boundary_and_log(void)
{
    diagnostic_active = true;
    actions_idle = false;
    state = SYSTEM_STATE_RUN;
    feed(":12,set,wheel.right.kp,0.8\n");
    expect("ok,12,staged,wheel.right.kp,0.8\r\n");
    feed(":13,apply\n");
    assert(output_length == 0U);
    assert(ControlParams_HasPendingApply());
    ControlProtocol_WheelBoundary();
    /* The 5 ms boundary commits only; reply formatting is background work. */
    assert(output_length == 0U);
    ControlProtocol_Service();
    expect("ok,13,applied,3\r\n");
    assert(last_commit.diagnostic_boundary);

    diagnostic_active = false;
    actions_idle = true;
    state = SYSTEM_STATE_IDLE;
    feed(":14,log,m6,20\n");
    expect("ok,14,log,m6,20\r\n");
    assert(logging_enabled && logging_period == 20U);
    feed(":15,log,m6,10\n");
    expect("err,15,range\r\n");
    feed(":16,log,off,0\n");
    expect("ok,16,log,off,0\r\n");
    assert(!logging_enabled);
}

static void test_save_and_overflow(void)
{
    char too_long[96];
    feed(":17,save\n");
    expect("ok,17,saved,9\r\n");

    faulted = true;
    feed(":18,save\n");
    expect("err,18,state\r\n");
    faulted = false;

    memset(too_long, 'a', sizeof(too_long));
    memcpy(too_long, ":19,get,", 8U);
    too_long[sizeof(too_long) - 2U] = '\n';
    too_long[sizeof(too_long) - 1U] = '\0';
    feed(too_long);
    expect("err,19,too_long\r\n");
}

int main(void)
{
    ControlParams_Init();
    ControlProtocol_Init(committed);
    clear_output();
    test_staging_and_apply();
    test_fragmentation_errors_and_emergency_epoch();
    test_diagnostic_boundary_and_log();
    test_save_and_overflow();
    return 0;
}
