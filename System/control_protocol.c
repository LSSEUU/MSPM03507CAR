#include "control_protocol.h"

#include "app_remote.h"
#include "diagnostics.h"
#include "m6_telemetry.h"
#include "motor.h"
#include "parameter_store.h"
#include "safety.h"
#include "system_state.h"
#include "uart_debug.h"
#include "wheel_speed.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CONTROL_PROTOCOL_REPLY_COUNT 8U
#define CONTROL_PROTOCOL_REPLY_SIZE  63U
#define CONTROL_PROTOCOL_MAX_FIELDS   5U

typedef struct {
    char text[CONTROL_PROTOCOL_REPLY_SIZE];
    uint8_t length;
} ControlProtocolReply_t;

static char s_frame[CONTROL_PROTOCOL_MAX_FRAME + 1U];
static uint8_t s_frame_length;
static bool s_frame_active;
static bool s_frame_overflow;
static uint32_t s_emergency_epoch;
static uint32_t s_rx_dropped;
static ControlProtocolReply_t s_replies[CONTROL_PROTOCOL_REPLY_COUNT];
static uint8_t s_reply_read;
static uint8_t s_reply_write;
static bool s_apply_waiting;
static uint16_t s_apply_id;
static bool s_apply_completion_pending;
static uint16_t s_apply_completion_id;
static ControlParamsResult_t s_apply_completion_result;
static uint32_t s_apply_completion_revision;
static ControlProtocolCommitCallback_t s_commit_callback;

static uint8_t next_reply(uint8_t index)
{
    return (uint8_t)((index + 1U) % CONTROL_PROTOCOL_REPLY_COUNT);
}

static void service_replies(void)
{
    while (s_reply_read != s_reply_write) {
        ControlProtocolReply_t *reply = &s_replies[s_reply_read];
        if (!debug_try_write(reply->text, reply->length))
            return;
        s_reply_read = next_reply(s_reply_read);
    }
}

static void queue_reply(const char *format, ...)
{
    char payload[CONTROL_PROTOCOL_MAX_FRAME + 1U];
    uint8_t next;
    int length;
    va_list args;

    service_replies();
    next = next_reply(s_reply_write);
    if (next == s_reply_read)
        return;
    va_start(args, format);
    length = vsnprintf(payload, sizeof(payload), format, args);
    va_end(args);
    if ((length < 0) || (length > (int)CONTROL_PROTOCOL_MAX_FRAME)) {
        length = snprintf(payload, sizeof(payload), "err,0,frame");
    }
    if ((length < 0) || (length > (int)CONTROL_PROTOCOL_MAX_FRAME))
        return;
    memcpy(s_replies[s_reply_write].text, payload, (size_t)length);
    s_replies[s_reply_write].text[length] = '\r';
    s_replies[s_reply_write].text[length + 1] = '\n';
    s_replies[s_reply_write].length = (uint8_t)(length + 2);
    s_reply_write = next;
    service_replies();
}

static void reset_frame(void)
{
    s_frame_length = 0U;
    s_frame_active = false;
    s_frame_overflow = false;
}

static bool parse_id(const char *text, uint16_t *id)
{
    uint32_t value = 0U;
    const char *cursor = text;

    if ((text == NULL) || (id == NULL) || (*text == '\0'))
        return false;
    while (*cursor != '\0') {
        if ((*cursor < '0') || (*cursor > '9'))
            return false;
        value = (value * 10U) + (uint32_t)(*cursor - '0');
        if (value > 65535U)
            return false;
        cursor++;
    }
    *id = (uint16_t)value;
    return value >= 1U;
}

static uint16_t best_effort_id(const char *frame)
{
    char id_text[6];
    uint8_t length = 0U;
    uint16_t id = 0U;
    const char *cursor = frame;

    if ((cursor != NULL) && (*cursor == ':'))
        cursor++;
    while ((cursor != NULL) && (*cursor >= '0') && (*cursor <= '9') &&
           (length < (sizeof(id_text) - 1U))) {
        id_text[length++] = *cursor++;
    }
    id_text[length] = '\0';
    return parse_id(id_text, &id) ? id : 0U;
}

static bool parse_decimal(const char *text, float *value)
{
    const char *cursor = text;
    float result = 0.0f;
    float fraction = 0.1f;
    bool digit = false;
    bool negative = false;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
        return false;
    if ((*cursor == '+') || (*cursor == '-')) {
        negative = (*cursor == '-');
        cursor++;
    }
    while ((*cursor >= '0') && (*cursor <= '9')) {
        digit = true;
        result = (result * 10.0f) + (float)(*cursor - '0');
        if (result > 1000000.0f)
            return false;
        cursor++;
    }
    if (*cursor == '.') {
        cursor++;
        while ((*cursor >= '0') && (*cursor <= '9')) {
            digit = true;
            result += (float)(*cursor - '0') * fraction;
            fraction *= 0.1f;
            cursor++;
        }
    }
    if (!digit || (*cursor != '\0'))
        return false;
    *value = negative ? -result : result;
    return true;
}

static uint8_t split_fields(char *text, char *fields[CONTROL_PROTOCOL_MAX_FIELDS])
{
    uint8_t count = 0U;
    char *cursor = text;

    while ((cursor != NULL) && (count < CONTROL_PROTOCOL_MAX_FIELDS)) {
        char *comma;
        if (*cursor == '\0')
            return 0U;
        fields[count++] = cursor;
        comma = strchr(cursor, ',');
        if (comma == NULL)
            return count;
        *comma = '\0';
        cursor = comma + 1;
    }
    return 0U;
}

static const char *params_error(ControlParamsResult_t result)
{
    switch (result) {
    case CONTROL_PARAMS_ERR_KEY:
        return "key";
    case CONTROL_PARAMS_ERR_READ_ONLY:
        return "readonly";
    case CONTROL_PARAMS_ERR_VALUE:
    case CONTROL_PARAMS_ERR_ARGUMENT:
        return "value";
    case CONTROL_PARAMS_ERR_RANGE:
    case CONTROL_PARAMS_ERR_RELATION:
        return "range";
    case CONTROL_PARAMS_ERR_STATE:
        return "state";
    case CONTROL_PARAMS_ERR_BUSY:
        return "busy";
    default:
        return "frame";
    }
}

static const char *active_source_name(void)
{
    if (ControlParams_Source() == CONTROL_PARAMS_SOURCE_PERSISTED)
        return ParameterStore_SourceName();
    if (ControlParams_Source() == CONTROL_PARAMS_SOURCE_RUNTIME)
        return "runtime";
    return "defaults";
}

static bool remote_is_idle(void)
{
    AppRemoteSnapshot_t remote;
    AppRemote_GetSnapshot(&remote);
    return remote.state == APP_REMOTE_IDLE;
}

static bool save_is_safe(void)
{
    WheelSpeedSnapshot_t wheel;
    SafetySnapshot_t safety;
    MotorStats_t motor;

    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
        !Diagnostics_ActionIsIdle() || !remote_is_idle()) return false;
    WheelSpeed_GetSnapshot(&wheel);
    Safety_GetSnapshot(&safety);
    Motor_GetStats(&motor);
    return !wheel.enabled && !safety.action_active &&
        (safety.requested_motor_a == 0) && (safety.requested_motor_b == 0) &&
        (safety.applied_motor_a == 0) && (safety.applied_motor_b == 0) &&
        (motor.last_a == 0) && (motor.last_b == 0);
}

static void reply_get(uint16_t id, const char *key)
{
    ControlParamReadback_t value;
    ControlParamsResult_t result;

    if (strcmp(key, "meta.firmware") == 0) {
        queue_reply("ok,%u,meta.firmware,%s", id,
                    CONTROL_PROTOCOL_FIRMWARE_ID);
        return;
    }
    if (strcmp(key, "meta.generation") == 0) {
        ParameterStoreInfo_t info;
        ParameterStore_GetInfo(&info);
        queue_reply("ok,%u,meta.generation,%lu", id,
                    (unsigned long)info.generation);
        return;
    }
    if (strcmp(key, "meta.source") == 0) {
        queue_reply("ok,%u,meta.source,%s", id, active_source_name());
        return;
    }
    result = ControlParams_Get(key, &value);
    if (result != CONTROL_PARAMS_OK) {
        queue_reply("err,%u,%s", id, params_error(result));
        return;
    }
    if (!value.writable) {
        if ((value.type == CONTROL_PARAM_VALUE_BOOL) ||
            (value.type == CONTROL_PARAM_VALUE_UINT32))
            queue_reply("ok,%u,%s,%lu", id, key,
                        (unsigned long)value.active.unsigned_value);
        else
            queue_reply("err,%u,key", id);
        return;
    }
    if (value.type == CONTROL_PARAM_VALUE_BOOL)
        queue_reply("ok,%u,%s,%lu,%lu", id, key,
            (unsigned long)value.active.unsigned_value,
            (unsigned long)value.staged.unsigned_value);
    else
        queue_reply("ok,%u,%s,%.6g,%.6g", id, key,
                    (double)value.active.real, (double)value.staged.real);
}

static void execute_frame(char *frame)
{
    char *fields[CONTROL_PROTOCOL_MAX_FIELDS];
    uint8_t count;
    uint16_t id = 0U;
    float numeric;
    ControlParamsResult_t param_result;

    if ((frame == NULL) || (*frame != ':')) {
        queue_reply("err,0,frame");
        return;
    }
    count = split_fields(frame + 1, fields);
    if ((count < 2U) || !parse_id(fields[0], &id)) {
        queue_reply("err,0,frame");
        return;
    }
    for (uint8_t index = 1U; index < count; index++) {
        const char *cursor = fields[index];
        while (*cursor != '\0') {
            if (((*cursor >= 'A') && (*cursor <= 'Z')) ||
                (*cursor == ' ') || (*cursor == '\t')) {
                queue_reply("err,%u,frame", id);
                return;
            }
            cursor++;
        }
    }

    if (strcmp(fields[1], "get") == 0) {
        if (count != 3U) queue_reply("err,%u,frame", id);
        else reply_get(id, fields[2]);
        return;
    }
    if (strcmp(fields[1], "set") == 0) {
        ControlParamReadback_t readback;
        if ((count != 4U) || !parse_decimal(fields[3], &numeric)) {
            queue_reply("err,%u,value", id);
            return;
        }
        param_result = ControlParams_Set(fields[2], numeric);
        if (param_result != CONTROL_PARAMS_OK) {
            queue_reply("err,%u,%s", id, params_error(param_result));
            return;
        }
        (void)ControlParams_Get(fields[2], &readback);
        if (readback.type == CONTROL_PARAM_VALUE_BOOL)
            queue_reply("ok,%u,staged,%s,%lu", id, fields[2],
                        (unsigned long)readback.staged.unsigned_value);
        else
            queue_reply("ok,%u,staged,%s,%.6g", id, fields[2],
                        (double)readback.staged.real);
        return;
    }
    if (strcmp(fields[1], "apply") == 0) {
        ControlParamsApplyContext_t context;
        ControlParamsCommit_t commit;
        if (count != 2U) {
            queue_reply("err,%u,frame", id);
            return;
        }
        context.wheel_diagnostic_armed =
            Diagnostics_WheelSpeedDiagnosticActive();
        context.fault_free_idle =
            (SystemState_Get() == SYSTEM_STATE_IDLE) && !Safety_HasFault() &&
            Diagnostics_ActionIsIdle() && remote_is_idle();
        param_result = ControlParams_RequestApply(context, &commit);
        if (param_result == CONTROL_PARAMS_PENDING) {
            s_apply_waiting = true;
            s_apply_id = id;
        } else if (param_result == CONTROL_PARAMS_OK) {
            if (s_commit_callback != NULL) s_commit_callback(&commit);
            queue_reply("ok,%u,applied,%lu", id,
                        (unsigned long)commit.revision);
        } else {
            queue_reply("err,%u,%s", id, params_error(param_result));
        }
        return;
    }
    if (strcmp(fields[1], "revert") == 0) {
        if (count != 2U) queue_reply("err,%u,frame", id);
        else if ((param_result = ControlParams_Revert()) != CONTROL_PARAMS_OK)
            queue_reply("err,%u,%s", id, params_error(param_result));
        else queue_reply("ok,%u,reverted,%lu", id,
                         (unsigned long)ControlParams_Revision());
        return;
    }
    if (strcmp(fields[1], "load") == 0) {
        ParameterStoreResult_t result;
        if (count != 2U) {
            queue_reply("err,%u,frame", id);
            return;
        }
        result = ParameterStore_LoadStaged();
        if (result == PARAMETER_STORE_OK)
            queue_reply("ok,%u,staged,%s", id, ParameterStore_SourceName());
        else
            queue_reply("err,%u,storage", id);
        return;
    }
    if (strcmp(fields[1], "defaults") == 0) {
        if (count != 2U) queue_reply("err,%u,frame", id);
        else if ((param_result = ControlParams_StageDefaults()) !=
                 CONTROL_PARAMS_OK)
            queue_reply("err,%u,%s", id, params_error(param_result));
        else queue_reply("ok,%u,staged,defaults", id);
        return;
    }
    if (strcmp(fields[1], "save") == 0) {
        ParameterStoreInfo_t info;
        ParameterStoreResult_t result;
        if (count != 2U) {
            queue_reply("err,%u,frame", id);
            return;
        }
        if (!save_is_safe()) {
            queue_reply("err,%u,state", id);
            return;
        }
        result = ParameterStore_SaveActive();
        if (result != PARAMETER_STORE_OK) {
            queue_reply("err,%u,storage", id);
            return;
        }
        ParameterStore_GetInfo(&info);
        queue_reply("ok,%u,saved,%lu", id,
                    (unsigned long)info.generation);
        return;
    }
    if (strcmp(fields[1], "log") == 0) {
        uint32_t period;
        if ((count != 4U) || !parse_decimal(fields[3], &numeric) ||
            (numeric < 0.0f) || (numeric > 1000.0f) ||
            (numeric != (float)(uint32_t)numeric)) {
            queue_reply("err,%u,value", id);
            return;
        }
        period = (uint32_t)numeric;
        if ((strcmp(fields[2], "off") == 0) && (period == 0U)) {
            M6Telemetry_SetLogging(false, 0U);
            queue_reply("ok,%u,log,off,0", id);
        } else if ((strcmp(fields[2], "m6") == 0) &&
                   (period >= 20U) && ((period % 20U) == 0U)) {
            M6Telemetry_SetLogging(true, (uint16_t)period);
            queue_reply("ok,%u,log,m6,%lu", id, (unsigned long)period);
        } else {
            queue_reply("err,%u,range", id);
        }
        return;
    }
    queue_reply("err,%u,verb", id);
}

void ControlProtocol_Init(ControlProtocolCommitCallback_t commit_callback)
{
    DebugStats_t stats;
    reset_frame();
    s_reply_read = s_reply_write = 0U;
    s_apply_waiting = false;
    s_apply_id = 0U;
    s_apply_completion_pending = false;
    s_apply_completion_id = 0U;
    s_apply_completion_result = CONTROL_PARAMS_OK;
    s_apply_completion_revision = 0U;
    s_commit_callback = commit_callback;
    s_emergency_epoch = debug_get_emergency_epoch();
    debug_get_stats(&stats);
    s_rx_dropped = stats.rx_dropped;
}

bool ControlProtocol_ProcessByte(char byte, uint32_t now_ms)
{
    uint32_t epoch;
    (void)now_ms;
    epoch = debug_get_emergency_epoch();
    if (epoch != s_emergency_epoch) {
        s_emergency_epoch = epoch;
        reset_frame();
    }

    if (!s_frame_active) {
        if (byte == ':') {
            s_frame_active = true;
            s_frame[0] = ':';
            s_frame_length = 1U;
            return true;
        }
        return (byte == '\r') || (byte == '\n');
    }
    if (byte == ':') {
        s_frame[0] = ':';
        s_frame_length = 1U;
        s_frame_overflow = false;
        return true;
    }
    if ((byte == '\r') || (byte == '\n')) {
        uint16_t id;
        if (s_frame_overflow) {
            s_frame[(s_frame_length <= CONTROL_PROTOCOL_MAX_FRAME) ?
                s_frame_length : CONTROL_PROTOCOL_MAX_FRAME] = '\0';
            id = best_effort_id(s_frame);
            queue_reply("err,%u,too_long", id);
        } else {
            s_frame[s_frame_length] = '\0';
            execute_frame(s_frame);
        }
        reset_frame();
        return true;
    }
    if (s_frame_length >= CONTROL_PROTOCOL_MAX_FRAME) {
        s_frame_overflow = true;
        return true;
    }
    s_frame[s_frame_length++] = byte;
    return true;
}

void ControlProtocol_WheelBoundary(void)
{
    ControlParamsCommit_t commit;
    ControlParamsResult_t result;

    if (!ControlParams_HasPendingApply())
        return;
    if (Safety_HasFault()) {
        ControlParams_CancelPendingApply();
        if (s_apply_waiting) {
            s_apply_completion_pending = true;
            s_apply_completion_id = s_apply_id;
            s_apply_completion_result = CONTROL_PARAMS_ERR_STATE;
            s_apply_completion_revision = 0U;
        }
        s_apply_waiting = false;
        return;
    }
    result = ControlParams_CommitPendingAtWheelBoundary(&commit);
    if (result == CONTROL_PARAMS_OK) {
        if (s_commit_callback != NULL) s_commit_callback(&commit);
        if (s_apply_waiting) {
            s_apply_completion_pending = true;
            s_apply_completion_id = s_apply_id;
            s_apply_completion_result = CONTROL_PARAMS_OK;
            s_apply_completion_revision = commit.revision;
        }
    } else {
        if (s_apply_waiting) {
            s_apply_completion_pending = true;
            s_apply_completion_id = s_apply_id;
            s_apply_completion_result = result;
            s_apply_completion_revision = 0U;
        }
    }
    s_apply_waiting = false;
}

bool ControlProtocol_ApplyLegacyStaged(void)
{
    ControlParamsApplyContext_t context;
    ControlParamsCommit_t commit;
    ControlParamsResult_t result;

    context.wheel_diagnostic_armed =
        Diagnostics_WheelSpeedDiagnosticActive();
    context.fault_free_idle =
        (SystemState_Get() == SYSTEM_STATE_IDLE) && !Safety_HasFault() &&
        Diagnostics_ActionIsIdle() && remote_is_idle();
    result = ControlParams_RequestApply(context, &commit);
    if (result == CONTROL_PARAMS_OK) {
        if (s_commit_callback != NULL) s_commit_callback(&commit);
        return true;
    }
    return result == CONTROL_PARAMS_PENDING;
}

void ControlProtocol_Service(void)
{
    DebugStats_t stats;
    uint32_t epoch = debug_get_emergency_epoch();

    if (epoch != s_emergency_epoch) {
        s_emergency_epoch = epoch;
        reset_frame();
        if (s_apply_waiting) {
            ControlParams_CancelPendingApply();
            s_apply_waiting = false;
        }
    }
    debug_get_stats(&stats);
    if (stats.rx_dropped != s_rx_dropped) {
        if (s_frame_active)
            queue_reply("err,0,frame");
        reset_frame();
        s_rx_dropped = stats.rx_dropped;
    }
    if (s_apply_completion_pending) {
        uint16_t id = s_apply_completion_id;
        ControlParamsResult_t result = s_apply_completion_result;
        uint32_t revision = s_apply_completion_revision;
        s_apply_completion_pending = false;
        if (result == CONTROL_PARAMS_OK)
            queue_reply("ok,%u,applied,%lu", id, (unsigned long)revision);
        else
            queue_reply("err,%u,%s", id, params_error(result));
    }
    service_replies();
}

bool ControlProtocol_IsFrameActive(void)
{
    return s_frame_active;
}
