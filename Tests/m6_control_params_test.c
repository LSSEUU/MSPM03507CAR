#include "control_config.h"
#include "control_params.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static int close_to(float actual, float expected)
{
    return fabsf(actual - expected) <= 0.0001f;
}

static ControlParamsApplyContext_t idle_context(void)
{
    ControlParamsApplyContext_t context = {true, false};
    return context;
}

static ControlParamsApplyContext_t diagnostic_context(void)
{
    ControlParamsApplyContext_t context = {false, true};
    return context;
}

static void test_defaults_and_descriptors(void)
{
    static const char *const expected_keys[] = {
        "wheel.left.kp", "wheel.left.ki", "wheel.right.kp",
        "wheel.right.ki", "wheel.filter_alpha", "wheel.accel",
        "wheel.decel", "wheel.jerk", "wheel.left.ks", "wheel.left.kv",
        "wheel.right.ks", "wheel.right.kv", "wheel.diag_target",
        "heading.straight.kp", "heading.straight.kd", "heading.turn.kp",
        "heading.turn.kd", "heading.rate_alpha",
        "heading.correction_limit", "heading.turn_speed",
        "heading.smooth_zone", "line.speed_min", "line.speed_max",
        "line.position_alpha", "line.derivative_alpha", "line.kp",
        "line.kd", "line.correction_limit", "line.correction_slew",
        "line.curve_gain", "line.lost_search_speed", "remote.speed_low",
        "remote.speed_medium", "remote.speed_high", "remote.heading_hold"
    };
    const ControlParams_t *active;
    size_t index;

    _Static_assert(sizeof(ControlParamsRemote_t) == 16U,
                   "persistent boolean must have a canonical 32-bit layout");
    _Static_assert(sizeof(ControlParams_t) == 140U,
                   "unexpected persistent parameter payload layout");

    ControlParams_Init();
    active = ControlParams_Active();
    assert(ControlParams_Validate(active) == CONTROL_PARAMS_OK);
    assert(close_to(active->wheel.left_kp, WHEEL_SPEED_KP_A));
    assert(close_to(active->wheel.left_ki, WHEEL_SPEED_KI_A));
    assert(close_to(active->wheel.filter_alpha, WHEEL_SPEED_FILTER_ALPHA));
    assert(close_to(active->wheel.accel_mm_s2, WHEEL_SPEED_ACCEL_MM_S2));
    assert(close_to(active->wheel.decel_mm_s2, WHEEL_SPEED_DECEL_MM_S2));
    assert(close_to(active->heading.straight_kp,
                    HEADING_HOLD_KP_MM_S_PER_DEG));
    assert(close_to(active->heading.smooth_zone_deg,
                    HEADING_TURN_TOLERANCE_DEG));
    assert(close_to(active->line.speed_min_mm_s,
                    LINE_FOLLOW_BASE_SPEED_MM_S));
    assert(close_to(active->line.speed_max_mm_s,
                    LINE_FOLLOW_BASE_SPEED_MM_S));
    assert(close_to(active->line.position_alpha, 1.0f));
    assert(close_to(active->line.derivative_alpha, 1.0f));
    assert(close_to(active->line.curve_gain, 0.0f));
    assert(close_to(active->remote.speed_low_mm_s, 100.0f));
    assert(close_to(active->remote.speed_medium_mm_s, 160.0f));
    assert(close_to(active->remote.speed_high_mm_s, 220.0f));
    assert(active->remote.heading_hold == 0U);
    assert(ControlParams_Revision() == 1U);
    assert(ControlParams_Source() == CONTROL_PARAMS_SOURCE_DEFAULT);
    assert(strcmp(ControlParams_SourceName(ControlParams_Source()),
                  "defaults") == 0);
    assert(!ControlParams_IsDirty());

    assert(ControlParams_DescriptorCount() ==
           (sizeof(expected_keys) / sizeof(expected_keys[0])));
    for (index = 0U; index < ControlParams_DescriptorCount(); ++index) {
        const ControlParamDescriptor_t *descriptor =
            ControlParams_DescriptorAt(index);
        assert(descriptor != NULL);
        assert(strcmp(descriptor->key, expected_keys[index]) == 0);
    }
    assert(ControlParams_DescriptorAt(ControlParams_DescriptorCount()) == NULL);
}

static void test_get_set_and_metadata(void)
{
    ControlParamReadback_t readback;

    ControlParams_Init();
    assert(ControlParams_Get("wheel.left.kp", &readback) == CONTROL_PARAMS_OK);
    assert(readback.type == CONTROL_PARAM_VALUE_FLOAT);
    assert(readback.writable);
    assert(close_to(readback.active.real, 0.70f));
    assert(close_to(readback.staged.real, 0.70f));

    assert(ControlParams_Set("wheel.left.kp", 1.25f) == CONTROL_PARAMS_OK);
    assert(ControlParams_Get("wheel.left.kp", &readback) == CONTROL_PARAMS_OK);
    assert(close_to(readback.active.real, 0.70f));
    assert(close_to(readback.staged.real, 1.25f));
    assert(ControlParams_IsDirty());

    assert(ControlParams_Get("meta.schema", &readback) == CONTROL_PARAMS_OK);
    assert(!readback.writable);
    assert(readback.active.unsigned_value == CONTROL_PARAMS_SCHEMA_VERSION);
    assert(ControlParams_Get("meta.revision", &readback) == CONTROL_PARAMS_OK);
    assert(readback.active.unsigned_value == 1U);
    assert(ControlParams_Get("meta.source", &readback) == CONTROL_PARAMS_OK);
    assert(readback.active.source == CONTROL_PARAMS_SOURCE_DEFAULT);
    assert(readback.staged.source == CONTROL_PARAMS_SOURCE_RUNTIME);
    assert(ControlParams_Get("meta.dirty", &readback) == CONTROL_PARAMS_OK);
    assert(readback.active.unsigned_value == 1U);

    assert(ControlParams_Set("meta.revision", 4.0f) ==
           CONTROL_PARAMS_ERR_READ_ONLY);
    assert(ControlParams_Set("safety.pwm_limit", 200.0f) ==
           CONTROL_PARAMS_ERR_KEY);
    assert(ControlParams_Get("unknown", &readback) == CONTROL_PARAMS_ERR_KEY);
}

static void test_value_validation_is_side_effect_free(void)
{
    ControlParamReadback_t readback;
    size_t index;

    ControlParams_Init();
    assert(ControlParams_Set("wheel.left.kp", NAN) == CONTROL_PARAMS_ERR_VALUE);
    assert(ControlParams_Set("wheel.left.kp", INFINITY) ==
           CONTROL_PARAMS_ERR_VALUE);
    assert(ControlParams_Set("wheel.left.kp", -0.01f) ==
           CONTROL_PARAMS_ERR_RANGE);
    assert(ControlParams_Set("wheel.left.ki", 10.01f) ==
           CONTROL_PARAMS_ERR_RANGE);
    assert(ControlParams_Set("wheel.filter_alpha", 0.049f) ==
           CONTROL_PARAMS_ERR_RANGE);
    assert(ControlParams_Set("wheel.diag_target", 49.0f) ==
           CONTROL_PARAMS_ERR_RANGE);
    assert(ControlParams_Set("wheel.diag_target", -49.0f) ==
           CONTROL_PARAMS_ERR_RANGE);
    assert(ControlParams_Set("remote.heading_hold", 0.5f) ==
           CONTROL_PARAMS_ERR_RANGE);
    assert(ControlParams_Get("wheel.left.kp", &readback) == CONTROL_PARAMS_OK);
    assert(close_to(readback.staged.real, WHEEL_SPEED_KP_A));
    assert(!ControlParams_IsDirty());

    for (index = 0U; index < ControlParams_DescriptorCount(); ++index) {
        const ControlParamDescriptor_t *descriptor =
            ControlParams_DescriptorAt(index);
        ControlParams_Init();
        assert(ControlParams_Set(descriptor->key, descriptor->minimum) ==
               CONTROL_PARAMS_OK);
        ControlParams_Init();
        assert(ControlParams_Set(descriptor->key, descriptor->maximum) ==
               CONTROL_PARAMS_OK);
    }
    ControlParams_Init();
    assert(ControlParams_Set("wheel.diag_target", 0.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_Set("wheel.diag_target", 50.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_Set("wheel.diag_target", -50.0f) == CONTROL_PARAMS_OK);
}

static void test_cross_field_validation_and_atomic_failure(void)
{
    const ControlParams_t *active;

    ControlParams_Init();
    assert(ControlParams_Set("line.speed_min", 300.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(idle_context(), NULL) ==
           CONTROL_PARAMS_ERR_RELATION);
    active = ControlParams_Active();
    assert(close_to(active->line.speed_min_mm_s, 160.0f));
    assert(ControlParams_Revision() == 1U);
    assert(ControlParams_IsDirty());

    ControlParams_Init();
    assert(ControlParams_Set("wheel.left.ks", 500.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_Set("wheel.left.kv", 0.5f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(idle_context(), NULL) ==
           CONTROL_PARAMS_ERR_RELATION);
    assert(close_to(ControlParams_Active()->wheel.left_ks, 0.0f));

    ControlParams_Init();
    assert(ControlParams_Set("remote.speed_low", 200.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(idle_context(), NULL) ==
           CONTROL_PARAMS_ERR_RELATION);
    assert(close_to(ControlParams_Active()->remote.speed_low_mm_s, 100.0f));
}

static void test_idle_apply_defaults_revert_and_external_staging(void)
{
    ControlParamsCommit_t commit;
    ControlParamReadback_t readback;
    ControlParams_t external;
    const ControlParams_t *old_active;

    ControlParams_Init();
    old_active = ControlParams_Active();
    assert(ControlParams_Set("wheel.left.kp", 1.20f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(idle_context(), &commit) ==
           CONTROL_PARAMS_OK);
    assert(commit.revision == 2U);
    assert(commit.change_mask == CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS);
    assert(commit.source == CONTROL_PARAMS_SOURCE_RUNTIME);
    assert(!commit.diagnostic_boundary);
    assert(close_to(ControlParams_Active()->wheel.left_kp, 1.20f));
    assert(close_to(old_active->wheel.left_kp, 0.70f));
    assert(!ControlParams_IsDirty());

    assert(ControlParams_StageDefaults() == CONTROL_PARAMS_OK);
    assert(close_to(ControlParams_Active()->wheel.left_kp, 1.20f));
    assert(close_to(ControlParams_Staged()->wheel.left_kp, 0.70f));
    assert(ControlParams_IsDirty());
    assert(ControlParams_Revert() == CONTROL_PARAMS_OK);
    assert(close_to(ControlParams_Staged()->wheel.left_kp, 1.20f));
    assert(!ControlParams_IsDirty());

    external = *ControlParams_Defaults();
    external.line.speed_min_mm_s = 200.0f;
    external.line.speed_max_mm_s = 200.0f;
    assert(ControlParams_StageExternal(&external,
                                       CONTROL_PARAMS_SOURCE_PERSISTED) ==
           CONTROL_PARAMS_OK);
    assert(ControlParams_Source() == CONTROL_PARAMS_SOURCE_RUNTIME);
    assert(ControlParams_Get("meta.source", &readback) == CONTROL_PARAMS_OK);
    assert(readback.staged.source == CONTROL_PARAMS_SOURCE_PERSISTED);
    assert(ControlParams_RequestApply(idle_context(), &commit) ==
           CONTROL_PARAMS_OK);
    assert(commit.revision == 3U);
    assert(ControlParams_Source() == CONTROL_PARAMS_SOURCE_PERSISTED);
    assert(close_to(ControlParams_Active()->line.speed_max_mm_s, 200.0f));
}

static void test_boot_restore_is_validated(void)
{
    ControlParams_t restored = *ControlParams_Defaults();
    ControlParams_t invalid;

    ControlParams_Init();
    restored.remote.heading_hold = 1U;
    assert(ControlParams_BootRestore(&restored, 77U,
                                     CONTROL_PARAMS_SOURCE_PERSISTED) ==
           CONTROL_PARAMS_OK);
    assert(ControlParams_Revision() == 77U);
    assert(ControlParams_Source() == CONTROL_PARAMS_SOURCE_PERSISTED);
    assert(ControlParams_Active()->remote.heading_hold == 1U);

    invalid = restored;
    invalid.heading.turn_kp = NAN;
    assert(ControlParams_BootRestore(&invalid, 99U,
                                     CONTROL_PARAMS_SOURCE_PERSISTED) ==
           CONTROL_PARAMS_ERR_VALUE);
    assert(ControlParams_Revision() == 77U);
    assert(ControlParams_Active()->remote.heading_hold == 1U);
    assert(ControlParams_BootRestore(&restored, 1U,
                                     CONTROL_PARAMS_SOURCE_RUNTIME) ==
           CONTROL_PARAMS_ERR_ARGUMENT);
}

static void test_state_gate_and_diagnostic_boundary(void)
{
    ControlParamsApplyContext_t blocked = {false, false};
    ControlParamsCommit_t commit;

    ControlParams_Init();
    assert(ControlParams_Set("wheel.left.kp", 1.10f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(blocked, NULL) == CONTROL_PARAMS_ERR_STATE);
    assert(close_to(ControlParams_Active()->wheel.left_kp, 0.70f));
    assert(ControlParams_Revision() == 1U);
    assert(ControlParams_IsDirty());

    assert(ControlParams_RequestApply(diagnostic_context(), NULL) ==
           CONTROL_PARAMS_PENDING);
    assert(ControlParams_HasPendingApply());
    assert(close_to(ControlParams_Active()->wheel.left_kp, 0.70f));
    assert(ControlParams_Set("wheel.right.kp", 1.2f) ==
           CONTROL_PARAMS_ERR_BUSY);
    assert(ControlParams_StageDefaults() == CONTROL_PARAMS_ERR_BUSY);
    assert(ControlParams_CommitPendingAtWheelBoundary(&commit) ==
           CONTROL_PARAMS_OK);
    assert(commit.revision == 2U);
    assert(commit.diagnostic_boundary);
    assert(commit.change_mask == CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS);
    assert(close_to(ControlParams_Active()->wheel.left_kp, 1.10f));
    assert(!ControlParams_HasPendingApply());
    assert(ControlParams_CommitPendingAtWheelBoundary(NULL) ==
           CONTROL_PARAMS_ERR_STATE);

    ControlParams_Init();
    assert(ControlParams_Set("line.kp", 180.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(diagnostic_context(), NULL) ==
           CONTROL_PARAMS_ERR_STATE);
    assert(!ControlParams_HasPendingApply());
    assert(close_to(ControlParams_Active()->line.kp, 150.0f));

    ControlParams_Init();
    assert(ControlParams_Set("wheel.diag_target", 220.0f) == CONTROL_PARAMS_OK);
    assert(ControlParams_RequestApply(diagnostic_context(), NULL) ==
           CONTROL_PARAMS_PENDING);
    ControlParams_CancelPendingApply();
    assert(!ControlParams_HasPendingApply());
    assert(close_to(ControlParams_Active()->wheel.diag_target_mm_s, 300.0f));
    assert(close_to(ControlParams_Staged()->wheel.diag_target_mm_s, 220.0f));
}

static void test_hard_wheel_target_clamp(void)
{
    assert(close_to(ControlParams_ClampWheelTarget(900.0f), 700.0f));
    assert(close_to(ControlParams_ClampWheelTarget(-900.0f), -700.0f));
    assert(close_to(ControlParams_ClampWheelTarget(220.0f), 220.0f));
    assert(close_to(ControlParams_ClampWheelTarget(NAN), 0.0f));
}

int main(void)
{
    test_defaults_and_descriptors();
    test_get_set_and_metadata();
    test_value_validation_is_side_effect_free();
    test_cross_field_validation_and_atomic_failure();
    test_idle_apply_defaults_revert_and_external_staging();
    test_boot_restore_is_validated();
    test_state_gate_and_diagnostic_boundary();
    test_hard_wheel_target_clamp();
    return 0;
}
