#include "control_params.h"

#include "control_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef enum {
    PARAM_STORAGE_FLOAT = 0,
    PARAM_STORAGE_BOOL32
} ParamStorage_t;

typedef enum {
    PARAM_RULE_RANGE = 0,
    PARAM_RULE_DIAG_SPEED
} ParamRule_t;

typedef struct {
    ControlParamDescriptor_t public_descriptor;
    size_t offset;
    ParamStorage_t storage;
    ParamRule_t rule;
    uint32_t diagnostic_change_bit;
} ParamDescriptorInternal_t;

typedef struct {
    ControlParams_t params;
    uint32_t revision;
    ControlParamsSource_t source;
} ActiveSlot_t;

#define PARAM_FLOAT(key_, member_, min_, max_, diagnostic_bit_)               \
    {{key_, CONTROL_PARAM_VALUE_FLOAT, min_, max_},                            \
     offsetof(ControlParams_t, member_), PARAM_STORAGE_FLOAT, PARAM_RULE_RANGE,\
     diagnostic_bit_}
#define PARAM_BOOL(key_, member_)                                             \
    {{key_, CONTROL_PARAM_VALUE_BOOL, 0.0f, 1.0f},                             \
     offsetof(ControlParams_t, member_), PARAM_STORAGE_BOOL32,                 \
     PARAM_RULE_RANGE,                                                         \
     CONTROL_PARAMS_CHANGE_OTHER}

static const ControlParams_t k_defaults = {
    {
        WHEEL_SPEED_KP_A,
        WHEEL_SPEED_KI_A,
        WHEEL_SPEED_KP_B,
        WHEEL_SPEED_KI_B,
        WHEEL_SPEED_FILTER_ALPHA,
        WHEEL_SPEED_ACCEL_MM_S2,
        WHEEL_SPEED_DECEL_MM_S2,
        20000.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        SPEED_DIAGNOSTIC_DEFAULT_MM_S
    },
    {
        HEADING_HOLD_KP_MM_S_PER_DEG,
        HEADING_HOLD_KD_MM_S_PER_DEG_S,
        HEADING_TURN_KP_MM_S_PER_DEG,
        HEADING_TURN_KD_MM_S_PER_DEG_S,
        HEADING_YAW_RATE_FILTER_ALPHA,
        HEADING_HOLD_MAX_CORRECTION_MM_S,
        HEADING_TURN_MAX_SPEED_MM_S,
        HEADING_TURN_TOLERANCE_DEG
    },
    {
        LINE_FOLLOW_BASE_SPEED_MM_S,
        LINE_FOLLOW_BASE_SPEED_MM_S,
        1.0f,
        1.0f,
        LINE_FOLLOW_KP_MM_S,
        LINE_FOLLOW_KD_MM_S_PER_UNIT_S,
        LINE_FOLLOW_MAX_CORRECTION_MM_S,
        2000.0f,
        0.0f,
        LINE_SEARCH_TURN_MM_S
    },
    {
        100.0f,
        160.0f,
        220.0f,
        0U
    }
};

static const ParamDescriptorInternal_t k_descriptors[] = {
    PARAM_FLOAT("wheel.left.kp", wheel.left_kp, 0.0f, 3.0f,
                CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS),
    PARAM_FLOAT("wheel.left.ki", wheel.left_ki, 0.0f, 10.0f,
                CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS),
    PARAM_FLOAT("wheel.right.kp", wheel.right_kp, 0.0f, 3.0f,
                CONTROL_PARAMS_CHANGE_WHEEL_RIGHT_GAINS),
    PARAM_FLOAT("wheel.right.ki", wheel.right_ki, 0.0f, 10.0f,
                CONTROL_PARAMS_CHANGE_WHEEL_RIGHT_GAINS),
    PARAM_FLOAT("wheel.filter_alpha", wheel.filter_alpha, 0.05f, 1.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.accel", wheel.accel_mm_s2, 100.0f, 2000.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.decel", wheel.decel_mm_s2, 100.0f, 2000.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.jerk", wheel.jerk_mm_s3, 1000.0f, 20000.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.left.ks", wheel.left_ks, 0.0f, 850.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.left.kv", wheel.left_kv, 0.0f, 5.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.right.ks", wheel.right_ks, 0.0f, 850.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("wheel.right.kv", wheel.right_kv, 0.0f, 5.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    {{"wheel.diag_target", CONTROL_PARAM_VALUE_FLOAT, -400.0f, 400.0f},
     offsetof(ControlParams_t, wheel.diag_target_mm_s), PARAM_STORAGE_FLOAT,
     PARAM_RULE_DIAG_SPEED, CONTROL_PARAMS_CHANGE_WHEEL_DIAG_TARGET},
    PARAM_FLOAT("heading.straight.kp", heading.straight_kp, 0.0f, 10.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.straight.kd", heading.straight_kd, 0.0f, 5.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.turn.kp", heading.turn_kp, 0.0f, 10.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.turn.kd", heading.turn_kd, 0.0f, 5.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.rate_alpha", heading.rate_alpha, 0.05f, 1.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.correction_limit", heading.correction_limit_mm_s,
                0.0f, 250.0f, CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.turn_speed", heading.turn_speed_mm_s, 50.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("heading.smooth_zone", heading.smooth_zone_deg, 2.0f, 60.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.speed_min", line.speed_min_mm_s, 50.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.speed_max", line.speed_max_mm_s, 50.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.position_alpha", line.position_alpha, 0.05f, 1.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.derivative_alpha", line.derivative_alpha, 0.05f, 1.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.kp", line.kp, 0.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.kd", line.kd, 0.0f, 100.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.correction_limit", line.correction_limit_mm_s,
                0.0f, 250.0f, CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.correction_slew", line.correction_slew_mm_s2,
                100.0f, 2000.0f, CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.curve_gain", line.curve_gain, 0.0f, 2.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("line.lost_search_speed", line.lost_search_speed_mm_s,
                50.0f, 400.0f, CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("remote.speed_low", remote.speed_low_mm_s, 50.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("remote.speed_medium", remote.speed_medium_mm_s, 50.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_FLOAT("remote.speed_high", remote.speed_high_mm_s, 50.0f, 400.0f,
                CONTROL_PARAMS_CHANGE_OTHER),
    PARAM_BOOL("remote.heading_hold", remote.heading_hold)
};

static ActiveSlot_t s_active_slots[2];
static volatile uint8_t s_active_index;
static ControlParams_t s_staged;
static ControlParamsSource_t s_staged_source;
static bool s_dirty;
static bool s_pending_valid;
static ControlParams_t s_pending;
static ControlParamsSource_t s_pending_source;
static uint32_t s_pending_change_mask;
static uint32_t s_stage_revision;

static const ActiveSlot_t *active_slot(void)
{
    uint8_t index = s_active_index;
    return &s_active_slots[index];
}

static const ParamDescriptorInternal_t *find_descriptor(const char *key)
{
    size_t index;

    if (key == NULL)
        return NULL;

    for (index = 0U;
         index < (sizeof(k_descriptors) / sizeof(k_descriptors[0]));
         ++index) {
        if (strcmp(k_descriptors[index].public_descriptor.key, key) == 0)
            return &k_descriptors[index];
    }
    return NULL;
}

static float read_value(const ControlParams_t *params,
                        const ParamDescriptorInternal_t *descriptor)
{
    const uint8_t *address = (const uint8_t *)params + descriptor->offset;

    if (descriptor->storage == PARAM_STORAGE_BOOL32)
        return (float)(*(const uint32_t *)address);
    return *(const float *)address;
}

static void write_value(ControlParams_t *params,
                        const ParamDescriptorInternal_t *descriptor,
                        float value)
{
    uint8_t *address = (uint8_t *)params + descriptor->offset;

    if (descriptor->storage == PARAM_STORAGE_BOOL32)
        *(uint32_t *)address = (uint32_t)value;
    else
        *(float *)address = value;
}

static ControlParamsResult_t validate_descriptor_value(
    const ParamDescriptorInternal_t *descriptor,
    float value)
{
    if (!isfinite(value))
        return CONTROL_PARAMS_ERR_VALUE;
    if ((value < descriptor->public_descriptor.minimum) ||
        (value > descriptor->public_descriptor.maximum))
        return CONTROL_PARAMS_ERR_RANGE;
    if ((descriptor->storage == PARAM_STORAGE_BOOL32) &&
        (value != 0.0f) && (value != 1.0f))
        return CONTROL_PARAMS_ERR_RANGE;
    if ((descriptor->rule == PARAM_RULE_DIAG_SPEED) &&
        (value != 0.0f) && (fabsf(value) < 50.0f))
        return CONTROL_PARAMS_ERR_RANGE;
    return CONTROL_PARAMS_OK;
}

static bool params_equal(const ControlParams_t *left,
                         const ControlParams_t *right)
{
    size_t index;

    for (index = 0U;
         index < (sizeof(k_descriptors) / sizeof(k_descriptors[0]));
         ++index) {
        if (read_value(left, &k_descriptors[index]) !=
            read_value(right, &k_descriptors[index]))
            return false;
    }
    return true;
}

static uint32_t calculate_change_mask(const ControlParams_t *from,
                                      const ControlParams_t *to)
{
    size_t index;
    uint32_t mask = CONTROL_PARAMS_CHANGE_NONE;

    for (index = 0U;
         index < (sizeof(k_descriptors) / sizeof(k_descriptors[0]));
         ++index) {
        if (read_value(from, &k_descriptors[index]) !=
            read_value(to, &k_descriptors[index]))
            mask |= k_descriptors[index].diagnostic_change_bit;
    }
    return mask;
}

static void refresh_dirty(bool restore_active_source_when_clean)
{
    const ActiveSlot_t *slot = active_slot();
    s_dirty = !params_equal(&s_staged, &slot->params);
    if (!s_dirty && restore_active_source_when_clean)
        s_staged_source = slot->source;
}

static void fill_commit(ControlParamsCommit_t *commit,
                        const ActiveSlot_t *slot,
                        uint32_t change_mask,
                        bool diagnostic_boundary)
{
    if (commit == NULL)
        return;
    commit->revision = slot->revision;
    commit->change_mask = change_mask;
    commit->source = slot->source;
    commit->diagnostic_boundary = diagnostic_boundary;
}

static void commit_snapshot(const ControlParams_t *params,
                            ControlParamsSource_t source,
                            uint32_t change_mask,
                            bool diagnostic_boundary,
                            ControlParamsCommit_t *commit)
{
    const ActiveSlot_t *current = active_slot();
    uint8_t next_index = (uint8_t)(s_active_index ^ 1U);
    ActiveSlot_t *next = &s_active_slots[next_index];

    next->params = *params;
    next->revision = current->revision + 1U;
    next->source = source;
    s_active_index = next_index;
    s_staged = *params;
    s_staged_source = source;
    s_dirty = false;
    s_stage_revision++;
    fill_commit(commit, next, change_mask, diagnostic_boundary);
}

void ControlParams_Init(void)
{
    memset(s_active_slots, 0, sizeof(s_active_slots));
    s_active_slots[0].params = k_defaults;
    s_active_slots[0].revision = 1U;
    s_active_slots[0].source = CONTROL_PARAMS_SOURCE_DEFAULT;
    s_active_index = 0U;
    s_staged = k_defaults;
    s_staged_source = CONTROL_PARAMS_SOURCE_DEFAULT;
    s_dirty = false;
    s_pending_valid = false;
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_source = CONTROL_PARAMS_SOURCE_DEFAULT;
    s_pending_change_mask = CONTROL_PARAMS_CHANGE_NONE;
    s_stage_revision = 1U;
}

const ControlParams_t *ControlParams_Active(void)
{
    return &active_slot()->params;
}

const ControlParams_t *ControlParams_Staged(void)
{
    return &s_staged;
}

const ControlParams_t *ControlParams_Defaults(void)
{
    return &k_defaults;
}

uint32_t ControlParams_Revision(void)
{
    return active_slot()->revision;
}

uint32_t ControlParams_StageRevision(void)
{
    return s_stage_revision;
}

ControlParamsSource_t ControlParams_Source(void)
{
    return active_slot()->source;
}

ControlParamsSource_t ControlParams_StagedSource(void)
{
    return s_staged_source;
}

const char *ControlParams_SourceName(ControlParamsSource_t source)
{
    switch (source) {
    case CONTROL_PARAMS_SOURCE_DEFAULT:
        return "defaults";
    case CONTROL_PARAMS_SOURCE_PERSISTED:
        return "persisted";
    case CONTROL_PARAMS_SOURCE_RUNTIME:
        return "runtime";
    default:
        return "unknown";
    }
}

bool ControlParams_IsDirty(void)
{
    return s_dirty;
}

size_t ControlParams_DescriptorCount(void)
{
    return sizeof(k_descriptors) / sizeof(k_descriptors[0]);
}

const ControlParamDescriptor_t *ControlParams_DescriptorAt(size_t index)
{
    if (index >= ControlParams_DescriptorCount())
        return NULL;
    return &k_descriptors[index].public_descriptor;
}

bool ControlParams_ReadValueAt(size_t index, const ControlParams_t *params,
                               ControlParamScalar_t *value)
{
    float raw;
    if ((index >= ControlParams_DescriptorCount()) || (params == NULL) ||
        (value == NULL)) return false;
    raw = read_value(params, &k_descriptors[index]);
    if (k_descriptors[index].storage == PARAM_STORAGE_BOOL32)
        value->unsigned_value = (uint32_t)raw;
    else
        value->real = raw;
    return true;
}

ControlParamsResult_t ControlParams_Get(const char *key,
                                        ControlParamReadback_t *readback)
{
    const ParamDescriptorInternal_t *descriptor;
    float active_value;
    float staged_value;

    if ((key == NULL) || (readback == NULL))
        return CONTROL_PARAMS_ERR_ARGUMENT;

    memset(readback, 0, sizeof(*readback));
    if (strcmp(key, "meta.schema") == 0) {
        readback->type = CONTROL_PARAM_VALUE_UINT32;
        readback->active.unsigned_value = CONTROL_PARAMS_SCHEMA_VERSION;
        readback->staged.unsigned_value = CONTROL_PARAMS_SCHEMA_VERSION;
        return CONTROL_PARAMS_OK;
    }
    if (strcmp(key, "meta.revision") == 0) {
        readback->type = CONTROL_PARAM_VALUE_UINT32;
        readback->active.unsigned_value = ControlParams_Revision();
        readback->staged.unsigned_value = ControlParams_Revision();
        return CONTROL_PARAMS_OK;
    }
    if (strcmp(key, "meta.source") == 0) {
        readback->type = CONTROL_PARAM_VALUE_SOURCE;
        readback->active.source = ControlParams_Source();
        readback->staged.source = s_staged_source;
        return CONTROL_PARAMS_OK;
    }
    if (strcmp(key, "meta.dirty") == 0) {
        readback->type = CONTROL_PARAM_VALUE_BOOL;
        readback->active.unsigned_value = s_dirty ? 1U : 0U;
        readback->staged.unsigned_value = s_dirty ? 1U : 0U;
        return CONTROL_PARAMS_OK;
    }

    descriptor = find_descriptor(key);
    if (descriptor == NULL)
        return CONTROL_PARAMS_ERR_KEY;
    active_value = read_value(ControlParams_Active(), descriptor);
    staged_value = read_value(&s_staged, descriptor);
    readback->type = descriptor->public_descriptor.type;
    readback->writable = true;
    if (descriptor->storage == PARAM_STORAGE_BOOL32) {
        readback->active.unsigned_value = (uint32_t)active_value;
        readback->staged.unsigned_value = (uint32_t)staged_value;
    } else {
        readback->active.real = active_value;
        readback->staged.real = staged_value;
    }
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_Set(const char *key, float value)
{
    const ParamDescriptorInternal_t *descriptor;
    ControlParamsResult_t result;

    if (s_pending_valid)
        return CONTROL_PARAMS_ERR_BUSY;
    if (key == NULL)
        return CONTROL_PARAMS_ERR_ARGUMENT;
    if (strncmp(key, "meta.", 5U) == 0)
        return CONTROL_PARAMS_ERR_READ_ONLY;

    descriptor = find_descriptor(key);
    if (descriptor == NULL)
        return CONTROL_PARAMS_ERR_KEY;
    result = validate_descriptor_value(descriptor, value);
    if (result != CONTROL_PARAMS_OK)
        return result;

    write_value(&s_staged, descriptor, value);
    s_staged_source = CONTROL_PARAMS_SOURCE_RUNTIME;
    refresh_dirty(true);
    s_stage_revision++;
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_Validate(const ControlParams_t *params)
{
    size_t index;
    ControlParamsResult_t result;

    if (params == NULL)
        return CONTROL_PARAMS_ERR_ARGUMENT;

    for (index = 0U;
         index < (sizeof(k_descriptors) / sizeof(k_descriptors[0]));
         ++index) {
        result = validate_descriptor_value(&k_descriptors[index],
                                           read_value(params,
                                                      &k_descriptors[index]));
        if (result != CONTROL_PARAMS_OK)
            return result;
    }
    if (params->line.speed_min_mm_s > params->line.speed_max_mm_s)
        return CONTROL_PARAMS_ERR_RELATION;
    if ((params->wheel.left_ks + (params->wheel.left_kv * 400.0f)) > 680.0f)
        return CONTROL_PARAMS_ERR_RELATION;
    if ((params->wheel.right_ks + (params->wheel.right_kv * 400.0f)) > 680.0f)
        return CONTROL_PARAMS_ERR_RELATION;
    if ((params->remote.speed_low_mm_s > params->remote.speed_medium_mm_s) ||
        (params->remote.speed_medium_mm_s > params->remote.speed_high_mm_s))
        return CONTROL_PARAMS_ERR_RELATION;
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_StageDefaults(void)
{
    if (s_pending_valid)
        return CONTROL_PARAMS_ERR_BUSY;
    s_staged = k_defaults;
    s_staged_source = CONTROL_PARAMS_SOURCE_DEFAULT;
    refresh_dirty(false);
    s_stage_revision++;
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_Revert(void)
{
    const ActiveSlot_t *slot;

    if (s_pending_valid)
        return CONTROL_PARAMS_ERR_BUSY;
    slot = active_slot();
    s_staged = slot->params;
    s_staged_source = slot->source;
    s_dirty = false;
    s_stage_revision++;
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_StageExternal(
    const ControlParams_t *params,
    ControlParamsSource_t source)
{
    ControlParamsResult_t result;

    if (s_pending_valid)
        return CONTROL_PARAMS_ERR_BUSY;
    if ((source != CONTROL_PARAMS_SOURCE_DEFAULT) &&
        (source != CONTROL_PARAMS_SOURCE_PERSISTED) &&
        (source != CONTROL_PARAMS_SOURCE_RUNTIME))
        return CONTROL_PARAMS_ERR_ARGUMENT;
    result = ControlParams_Validate(params);
    if (result != CONTROL_PARAMS_OK)
        return result;
    s_staged = *params;
    s_staged_source = source;
    refresh_dirty(false);
    s_stage_revision++;
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_BootRestore(
    const ControlParams_t *params,
    uint32_t revision,
    ControlParamsSource_t source)
{
    ControlParamsResult_t result;

    if ((source != CONTROL_PARAMS_SOURCE_DEFAULT) &&
        (source != CONTROL_PARAMS_SOURCE_PERSISTED))
        return CONTROL_PARAMS_ERR_ARGUMENT;
    result = ControlParams_Validate(params);
    if (result != CONTROL_PARAMS_OK)
        return result;

    s_active_slots[0].params = *params;
    s_active_slots[0].revision = revision;
    s_active_slots[0].source = source;
    s_active_index = 0U;
    s_staged = s_active_slots[0].params;
    s_staged_source = source;
    s_dirty = false;
    s_pending_valid = false;
    s_stage_revision = 1U;
    return CONTROL_PARAMS_OK;
}

ControlParamsResult_t ControlParams_RequestApply(
    ControlParamsApplyContext_t context,
    ControlParamsCommit_t *commit)
{
    ControlParamsResult_t result;
    uint32_t change_mask;

    if (s_pending_valid)
        return CONTROL_PARAMS_ERR_BUSY;
    result = ControlParams_Validate(&s_staged);
    if (result != CONTROL_PARAMS_OK)
        return result;

    change_mask = calculate_change_mask(ControlParams_Active(), &s_staged);
    if (context.wheel_diagnostic_armed) {
        if ((change_mask & CONTROL_PARAMS_CHANGE_OTHER) != 0U)
            return CONTROL_PARAMS_ERR_STATE;
        s_pending = s_staged;
        s_pending_source = s_staged_source;
        s_pending_change_mask = change_mask;
        s_pending_valid = true;
        return CONTROL_PARAMS_PENDING;
    }
    if (!context.fault_free_idle)
        return CONTROL_PARAMS_ERR_STATE;

    commit_snapshot(&s_staged, s_staged_source, change_mask, false, commit);
    return CONTROL_PARAMS_OK;
}

bool ControlParams_HasPendingApply(void)
{
    return s_pending_valid;
}

ControlParamsResult_t ControlParams_CommitPendingAtWheelBoundary(
    ControlParamsCommit_t *commit)
{
    if (!s_pending_valid)
        return CONTROL_PARAMS_ERR_STATE;

    commit_snapshot(&s_pending, s_pending_source, s_pending_change_mask,
                    true, commit);
    s_pending_valid = false;
    s_pending_change_mask = CONTROL_PARAMS_CHANGE_NONE;
    return CONTROL_PARAMS_OK;
}

void ControlParams_CancelPendingApply(void)
{
    s_pending_valid = false;
    s_pending_change_mask = CONTROL_PARAMS_CHANGE_NONE;
}

float ControlParams_ClampWheelTarget(float target_mm_s)
{
    if (!isfinite(target_mm_s))
        return 0.0f;
    if (target_mm_s > CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S)
        return CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S;
    if (target_mm_s < -CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S)
        return -CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S;
    return target_mm_s;
}
