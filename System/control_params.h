#ifndef CONTROL_PARAMS_H
#define CONTROL_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CONTROL_PARAMS_SCHEMA_VERSION 1U
#define CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S 700.0f

typedef struct {
    float left_kp;
    float left_ki;
    float right_kp;
    float right_ki;
    float filter_alpha;
    float accel_mm_s2;
    float decel_mm_s2;
    float jerk_mm_s3;
    float left_ks;
    float left_kv;
    float right_ks;
    float right_kv;
    float diag_target_mm_s;
} ControlParamsWheel_t;

typedef struct {
    float straight_kp;
    float straight_kd;
    float turn_kp;
    float turn_kd;
    float rate_alpha;
    float correction_limit_mm_s;
    float turn_speed_mm_s;
    float smooth_zone_deg;
} ControlParamsHeading_t;

typedef struct {
    float speed_min_mm_s;
    float speed_max_mm_s;
    float position_alpha;
    float derivative_alpha;
    float kp;
    float kd;
    float correction_limit_mm_s;
    float correction_slew_mm_s2;
    float curve_gain;
    float lost_search_speed_mm_s;
} ControlParamsLine_t;

typedef struct {
    float speed_low_mm_s;
    float speed_medium_mm_s;
    float speed_high_mm_s;
    uint32_t heading_hold;
} ControlParamsRemote_t;

typedef struct {
    ControlParamsWheel_t wheel;
    ControlParamsHeading_t heading;
    ControlParamsLine_t line;
    ControlParamsRemote_t remote;
} ControlParams_t;

typedef enum {
    CONTROL_PARAMS_SOURCE_DEFAULT = 0,
    CONTROL_PARAMS_SOURCE_PERSISTED,
    CONTROL_PARAMS_SOURCE_RUNTIME
} ControlParamsSource_t;

typedef enum {
    CONTROL_PARAM_VALUE_FLOAT = 0,
    CONTROL_PARAM_VALUE_BOOL,
    CONTROL_PARAM_VALUE_UINT32,
    CONTROL_PARAM_VALUE_SOURCE
} ControlParamValueType_t;

typedef struct {
    const char *key;
    ControlParamValueType_t type;
    float minimum;
    float maximum;
} ControlParamDescriptor_t;

typedef union {
    float real;
    uint32_t unsigned_value;
    ControlParamsSource_t source;
} ControlParamScalar_t;

typedef struct {
    ControlParamValueType_t type;
    bool writable;
    ControlParamScalar_t active;
    ControlParamScalar_t staged;
} ControlParamReadback_t;

typedef enum {
    CONTROL_PARAMS_OK = 0,
    CONTROL_PARAMS_PENDING,
    CONTROL_PARAMS_ERR_ARGUMENT,
    CONTROL_PARAMS_ERR_KEY,
    CONTROL_PARAMS_ERR_READ_ONLY,
    CONTROL_PARAMS_ERR_VALUE,
    CONTROL_PARAMS_ERR_RANGE,
    CONTROL_PARAMS_ERR_RELATION,
    CONTROL_PARAMS_ERR_STATE,
    CONTROL_PARAMS_ERR_BUSY
} ControlParamsResult_t;

typedef struct {
    bool fault_free_idle;
    bool wheel_diagnostic_armed;
} ControlParamsApplyContext_t;

enum {
    CONTROL_PARAMS_CHANGE_NONE = 0U,
    CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS = (1UL << 0),
    CONTROL_PARAMS_CHANGE_WHEEL_RIGHT_GAINS = (1UL << 1),
    CONTROL_PARAMS_CHANGE_WHEEL_DIAG_TARGET = (1UL << 2),
    CONTROL_PARAMS_CHANGE_OTHER = (1UL << 3)
};

typedef struct {
    uint32_t revision;
    uint32_t change_mask;
    ControlParamsSource_t source;
    bool diagnostic_boundary;
} ControlParamsCommit_t;

/* Initialize active and staged values from the compiled compatibility defaults. */
void ControlParams_Init(void);

/* The returned pointer is read-only and must not be retained across two commits. */
const ControlParams_t *ControlParams_Active(void);
const ControlParams_t *ControlParams_Staged(void);
const ControlParams_t *ControlParams_Defaults(void);
uint32_t ControlParams_Revision(void);
ControlParamsSource_t ControlParams_Source(void);
const char *ControlParams_SourceName(ControlParamsSource_t source);
bool ControlParams_IsDirty(void);

size_t ControlParams_DescriptorCount(void);
const ControlParamDescriptor_t *ControlParams_DescriptorAt(size_t index);
ControlParamsResult_t ControlParams_Get(const char *key,
                                        ControlParamReadback_t *readback);
ControlParamsResult_t ControlParams_Set(const char *key, float value);
ControlParamsResult_t ControlParams_Validate(const ControlParams_t *params);

/* Staging operations never alter active control behavior. */
ControlParamsResult_t ControlParams_StageDefaults(void);
ControlParamsResult_t ControlParams_Revert(void);
ControlParamsResult_t ControlParams_StageExternal(
    const ControlParams_t *params,
    ControlParamsSource_t source);

/* Restore a validated record before scheduling starts; no runtime revision bump. */
ControlParamsResult_t ControlParams_BootRestore(
    const ControlParams_t *params,
    uint32_t revision,
    ControlParamsSource_t source);

/* IDLE commits immediately. Diagnostic commits wait for the next 5 ms boundary. */
ControlParamsResult_t ControlParams_RequestApply(
    ControlParamsApplyContext_t context,
    ControlParamsCommit_t *commit);
bool ControlParams_HasPendingApply(void);
ControlParamsResult_t ControlParams_CommitPendingAtWheelBoundary(
    ControlParamsCommit_t *commit);
void ControlParams_CancelPendingApply(void);

float ControlParams_ClampWheelTarget(float target_mm_s);

#endif
