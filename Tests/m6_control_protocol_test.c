#include "control_protocol.h"

#include "app_remote.h"
#include "m6_telemetry.h"
#include "m6_link.h"
#include "m6_rpc.h"
#include "motor.h"
#include "parameter_store.h"
#include "reset_diag.h"
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
static M6LinkMode_t link_mode = M6_LINK_V1;

int debug_try_write(const char *data, size_t length)
{
    assert(output_length + length < sizeof(output));
    memcpy(output + output_length, data, length);
    output_length += length;
    output[output_length] = '\0';
    return 1;
}

int debug_try_write_control(const char *data, size_t length)
{
    return debug_try_write(data, length);
}

int debug_try_write_v2(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
    return 1;
}

int debug_tx_idle(void) { return 1; }
void debug_begin_v2_drain(void) {}
void debug_enter_v2_binary(void) {}
void debug_leave_v2(void) { link_mode = M6_LINK_V1; }

uint32_t debug_get_emergency_epoch(void) { return emergency_epoch; }
uint8_t debug_get_emergency_source(void) { return 0U; }
void debug_get_stats(DebugStats_t *stats) { *stats = debug_stats; }

uint32_t SystemTime_Now(void) { return 100U; }

void ResetDiag_GetSnapshot(ResetDiagSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->boot_id = 7U;
}

void M6Rpc_Init(ControlProtocolCommitCallback_t commit_callback)
{
    (void)commit_callback;
}

M6Result_t M6Rpc_Execute(uint16_t opcode, const uint8_t *body,
                         uint16_t body_length, uint8_t *success_body,
                         uint16_t *success_length, uint32_t *detail,
                         void *context)
{
    (void)opcode;
    (void)body;
    (void)body_length;
    (void)success_body;
    (void)success_length;
    (void)detail;
    (void)context;
    return M6_RESULT_UNSUPPORTED_OPCODE;
}

void M6Rpc_Service(uint32_t now_ms) { (void)now_ms; }
void M6Rpc_Telemetry20ms(uint32_t now_ms) { (void)now_ms; }
void M6Rpc_ParameterBoundaryCompleted(ControlParamsResult_t result,
                                      const ControlParamsCommit_t *commit)
{
    (void)result;
    (void)commit;
}
void M6Rpc_CancelPendingApply(M6Result_t result, uint32_t detail)
{
    (void)result;
    (void)detail;
}

void M6Link_Init(const M6LinkConfig_t *config, M6RpcExecutor_t executor,
                 M6LinkWrite_t write, M6LinkTxIdle_t tx_idle, void *context,
                 uint32_t initial_emergency_epoch,
                 uint32_t overflow_epoch)
{
    (void)config;
    (void)executor;
    (void)write;
    (void)tx_idle;
    (void)context;
    (void)initial_emergency_epoch;
    (void)overflow_epoch;
    link_mode = M6_LINK_V1;
}

M6LinkMode_t M6Link_Mode(void) { return link_mode; }
uint64_t M6Link_Capabilities(void) { return M6_DEVICE_CAPABILITIES; }
uint64_t M6Link_EnabledCapabilities(void) { return M6_DEVICE_CAPABILITIES; }
bool M6Link_BeginV2(uint64_t challenge, uint32_t now_ms)
{
    (void)challenge;
    (void)now_ms;
    link_mode = M6_LINK_V2_DRAIN;
    return true;
}
bool M6Link_DrainFailed(void) { return false; }
void M6Link_NotifyV1ReadyComplete(uint32_t now_ms)
{
    (void)now_ms;
    link_mode = M6_LINK_V2_ARMED;
}
void M6Link_ProcessByte(uint8_t byte, uint32_t now_ms,
                        uint32_t current_emergency_epoch,
                        uint32_t overflow_epoch)
{
    (void)byte;
    (void)now_ms;
    (void)current_emergency_epoch;
    (void)overflow_epoch;
}
void M6Link_Service(uint32_t now_ms, uint32_t current_emergency_epoch,
                    uint32_t overflow_epoch)
{
    (void)now_ms;
    (void)current_emergency_epoch;
    (void)overflow_epoch;
}

bool Diagnostics_WheelSpeedDiagnosticActive(void) { return diagnostic_active; }
bool Diagnostics_ActionIsIdle(void) { return actions_idle; }
void Diagnostics_SetLegacyTextEnabled(bool enabled) { (void)enabled; }

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
void AppRemote_SetLegacyTextEnabled(bool enabled) { (void)enabled; }

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
