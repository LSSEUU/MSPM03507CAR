#include "line_yaw_assist.h"

#include "control_config.h"
#include "vehicle_config.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static LineFollowSnapshot_t make_line(float base, float correction)
{
    LineFollowSnapshot_t line;
    memset(&line, 0, sizeof(line));
    line.state = LINE_FOLLOW_TRACKING;
    line.line_valid = true;
    line.dynamic_base_mm_s = base;
    line.active_mask = 0x18U;
    line.correction_mm_s = correction;
    line.left_target_mm_s = base + correction;
    line.right_target_mm_s = base - correction;
    return line;
}

static LineYawAssistSnapshot_t update_twice(LineFollowSnapshot_t *line,
                                             float yaw_rate,
                                             bool gyro_usable)
{
    LineYawAssistSnapshot_t result;
    LineYawAssist_Init();
    LineYawAssist_Update(line, 30.0f, 0.0f, yaw_rate, 3.0f, 0.88f,
                         gyro_usable, 100U);
    LineYawAssist_Update(line, 30.0f, 0.0f, yaw_rate, 3.0f, 0.88f,
                         gyro_usable, 120U);
    LineYawAssist_GetSnapshot(&result);
    return result;
}

static void test_straight_uncommanded_yaw_is_damped(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 0.0f);
    LineYawAssistSnapshot_t result = update_twice(&line, 5.0f, true);
    assert(result.active);
    assert(fabsf(result.desired_yaw_rate_deg_s) < 0.001f);
    assert(result.assist_correction_mm_s > 0.0f);
    assert(result.left_target_mm_s > result.right_target_mm_s);
    assert(fabsf((result.left_target_mm_s + result.right_target_mm_s) *
                 0.5f - 160.0f) < 0.001f);
}

static void test_straight_heading_drift_is_corrected_at_zero_rate(void)
{
    LineFollowSnapshot_t line = make_line(220.0f, 0.0f);
    LineYawAssistSnapshot_t result;
    LineYawAssist_Init();
    LineYawAssist_Update(&line, 60.0f, 0.0f, 0.0f, 3.0f, 0.88f,
                         true, 100U);
    LineYawAssist_Update(&line, 60.0f, 0.5f, 0.0f, 3.0f, 0.88f,
                         true, 120U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(result.straight_heading_error_deg < -0.49f);
    assert(result.desired_yaw_rate_deg_s < -1.49f);
    assert(result.final_correction_mm_s > 0.0f);
}

static void test_commanded_curve_is_not_forced_straight(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 10.0f);
    float feedforward = 160.0f * CHASSIS_DRIVE_TRACK_MM /
                        (2.0f * LINE_CURVE_RADIUS_MM);
    float desired = feedforward +
                    LINE_CURVE_POSITION_TRIM_MM_S * (3.0f / 7.0f);
    float expected = (-2.0f * desired / CHASSIS_DRIVE_TRACK_MM) *
                     57.2957795f;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssistSnapshot_t result;
    uint32_t now_ms;
    LineYawAssist_Init();
    for (now_ms = 100U; now_ms <= 2000U; now_ms += 100U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, expected,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&result);
    assert(result.active);
    assert(result.curve_rate_active);
    assert(result.curve_direction == 1);
    assert(fabsf(result.curve_feedforward_correction_mm_s -
                 feedforward) < 0.01f);
    assert(fabsf(result.desired_yaw_rate_deg_s - expected) < 0.01f);
    assert(result.assist_correction_mm_s > 3.9f);
    assert(result.final_correction_mm_s > 13.9f);
}

static void test_overturn_reduces_and_underturn_increases_command(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 10.0f);
    LineYawAssistSnapshot_t overturn;
    LineYawAssistSnapshot_t underturn;
    uint32_t now_ms;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    for (now_ms = 100U; now_ms <= 1000U; now_ms += 100U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, -20.0f,
                             3.0f, 0.88f, true, now_ms);
    for (now_ms = 1020U; now_ms <= 1300U; now_ms += 20U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, -30.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&overturn);
    LineYawAssist_Init();
    for (now_ms = 100U; now_ms <= 1000U; now_ms += 100U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, -20.0f,
                             3.0f, 0.88f, true, now_ms);
    for (now_ms = 1020U; now_ms <= 1300U; now_ms += 20U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, 0.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&underturn);
    assert(overturn.curve_rate_active);
    assert(underturn.curve_rate_active);
    assert(overturn.rate_error_deg_s < 0.0f);
    assert(underturn.rate_error_deg_s > 0.0f);
    assert(overturn.final_correction_mm_s <
           underturn.final_correction_mm_s);
}

static void assert_disabled(LineFollowSnapshot_t *line, bool gyro_usable)
{
    LineYawAssistSnapshot_t result = update_twice(line, 20.0f, gyro_usable);
    assert(!result.active);
    assert(fabsf(result.assist_correction_mm_s) < 0.001f);
    assert(fabsf(result.final_correction_mm_s -
                 line->correction_mm_s) < 0.001f);
    assert(fabsf(result.left_target_mm_s -
                 line->left_target_mm_s) < 0.001f);
    assert(fabsf(result.right_target_mm_s -
                 line->right_target_mm_s) < 0.001f);
}

static void test_invalid_loss_ambiguous_and_low_speed_disable(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 5.0f);
    assert_disabled(&line, false);
    line.line_valid = false;
    assert_disabled(&line, true);
    line.line_valid = true;
    line.ambiguous = true;
    assert_disabled(&line, true);
    line.ambiguous = false;
    line.state = LINE_FOLLOW_SEARCHING;
    assert_disabled(&line, true);
    line.state = LINE_FOLLOW_TRACKING;
    line.dynamic_base_mm_s = 79.0f;
    line.left_target_mm_s = 84.0f;
    line.right_target_mm_s = 74.0f;
    assert_disabled(&line, true);
}

static void test_limits_slew_wrap_and_no_reverse(void)
{
    LineFollowSnapshot_t line = make_line(80.0f, 30.0f);
    LineYawAssistSnapshot_t result;
    LineYawAssist_Init();
    LineYawAssist_Update(&line, 150.0f, 0.0f, 100.0f, 3.0f, 0.88f, true,
                         UINT32_MAX - 10U);
    LineYawAssist_Update(&line, 150.0f, 0.0f, 100.0f, 3.0f, 0.88f, true, 9U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.active);
    assert(result.final_correction_mm_s <= 30.001f);
    assert(result.left_target_mm_s >= 50.0f);
    assert(result.right_target_mm_s >= 50.0f);
    assert(fabsf(result.assist_correction_mm_s) <= 4.001f);
}

static void test_deadband_and_immediate_invalid_reset(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 0.0f);
    LineYawAssistSnapshot_t result = update_twice(&line, 0.4f, true);
    assert(result.active);
    assert(fabsf(result.assist_correction_mm_s) < 0.001f);
    LineYawAssist_Update(&line, 30.0f, 0.0f, 10.0f, 3.0f, 0.88f, true, 140U);
    LineYawAssist_Update(&line, 30.0f, 0.0f, 10.0f, 3.0f, 0.88f, false, 160U);
    LineYawAssist_GetSnapshot(&result);
    assert(!result.active);
    assert(fabsf(result.assist_correction_mm_s) < 0.001f);
}

static void test_near_centre_uses_heading_and_countersteers_early(void)
{
    LineFollowSnapshot_t line = make_line(220.0f, 4.0f);
    LineYawAssistSnapshot_t result;
    line.active_mask = 0x10U;
    line.position = 0.5f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_Update(&line, 60.0f, 0.0f, 0.0f, 3.0f, 0.88f,
                         true, 100U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(result.straight_target_yaw_deg < -0.8f);
    assert(result.desired_yaw_rate_deg_s < -2.4f);

    LineYawAssist_Update(&line, 60.0f, -0.86f, 0.0f, 3.0f, 0.88f,
                         true, 120U);
    LineYawAssist_Update(&line, 60.0f, -0.86f, 0.0f, 3.0f, 0.88f,
                         true, 140U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(fabsf(result.desired_yaw_rate_deg_s) < 0.1f);
    assert(result.final_correction_mm_s < line.correction_mm_s);
}

static void test_confirmed_curve_feedforward_persists_at_centre(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 36.0f);
    LineYawAssistSnapshot_t result;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    {
        uint32_t now_ms;
        for (now_ms = 100U; now_ms <= 1100U; now_ms += 100U)
            LineYawAssist_Update(&line, 60.0f, 0.0f, -18.0f,
                                 3.0f, 0.88f, true, now_ms);
    }
    LineYawAssist_GetSnapshot(&result);
    assert(!result.straight_heading_active);
    assert(result.curve_rate_active);

    line = make_line(220.0f, 0.0f);
    LineYawAssist_Update(&line, 60.0f, -20.0f, -18.0f, 3.0f, 0.88f,
                         true, 1200U);
    LineYawAssist_GetSnapshot(&result);
    assert(!result.straight_heading_active);
    assert(result.curve_rate_active);
    assert(result.desired_yaw_rate_deg_s < -17.0f);
    LineYawAssist_Update(&line, 60.0f, -20.0f, 0.0f, 3.0f, 0.88f,
                         true, 1300U);
    LineYawAssist_Update(&line, 60.0f, -20.0f, 0.0f, 3.0f, 0.88f,
                         true, 1620U);
    LineYawAssist_GetSnapshot(&result);
    assert(!result.straight_heading_active);
    assert(result.curve_rate_active);
    assert(result.desired_yaw_rate_deg_s < -17.0f);
}

static void test_outer_transient_crossing_back_is_not_a_curve(void)
{
    LineFollowSnapshot_t line = make_line(200.0f, 20.0f);
    LineYawAssistSnapshot_t result;
    uint32_t now_ms;
    line.active_mask = 0x30U;
    line.position = 0.25f;
    LineYawAssist_Init();
    for (now_ms = 100U; now_ms <= 500U; now_ms += 100U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, -5.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&result);
    assert(result.curve_candidate_active);
    assert(!result.curve_rate_active);
    assert(result.curve_progress_deg < LINE_CURVE_CONFIRM_TURN_DEG);

    line.active_mask = 0x04U;
    line.position = -0.3f;
    LineYawAssist_Update(&line, 60.0f, -3.0f, -5.0f,
                         3.0f, 0.88f, true, 600U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(!result.curve_candidate_active);
    assert(!result.curve_rate_active);
}

static void test_semicircle_progress_arms_exit_and_guards_straight(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 36.0f);
    LineYawAssistSnapshot_t result;
    uint32_t now_ms;
    float curve_rate = -(160.0f / LINE_CURVE_RADIUS_MM) * 57.2957795f;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_Update(&line, 60.0f, 0.0f, curve_rate,
                         3.0f, 0.88f, true, 100U);
    line = make_line(160.0f, 0.0f);
    for (now_ms = 200U; now_ms <= 11000U; now_ms += 100U) {
        LineYawAssist_Update(&line, 60.0f, -170.0f, curve_rate,
                             3.0f, 0.88f, true, now_ms);
        LineYawAssist_GetSnapshot(&result);
        if (result.straight_heading_active) break;
    }
    assert(result.straight_heading_active);
    assert(!result.curve_rate_active);
    assert(result.curve_progress_deg >= LINE_CURVE_EXIT_ARM_DEG);

    line.active_mask = 0x04U;
    line.position = -3.0f / 7.0f;
    LineYawAssist_Update(&line, 60.0f, -176.0f, 5.0f,
                         3.0f, 0.88f, true, now_ms + 20U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(!result.curve_rate_active);
}

static void test_short_tracking_gap_preserves_confirmed_curve(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 36.0f);
    LineYawAssistSnapshot_t result;
    uint32_t now_ms;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    for (now_ms = 100U; now_ms <= 1100U; now_ms += 100U)
        LineYawAssist_Update(&line, 60.0f, 0.0f, -18.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&result);
    assert(result.curve_rate_active);
    assert(result.curve_direction == 1);

    line.line_valid = false;
    line.active_mask = 0x00U;
    line.dynamic_base_mm_s = 105.0f;
    LineYawAssist_Update(&line, 60.0f, -20.0f, -18.0f,
                         3.0f, 0.88f, true, 1120U);
    LineYawAssist_GetSnapshot(&result);
    assert(!result.active);
    assert(result.curve_direction == 1);

    line = make_line(211.0f, -8.0f);
    line.active_mask = 0x08U;
    line.position = -0.5f / 7.0f;
    LineYawAssist_Update(&line, 60.0f, -20.5f, -18.0f,
                         3.0f, 0.88f, true, 1140U);
    LineYawAssist_Update(&line, 60.0f, -21.0f, -18.0f,
                         3.0f, 0.88f, true, 1160U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.active);
    assert(result.curve_rate_active);
    assert(result.curve_direction == 1);
    assert(result.desired_yaw_rate_deg_s < -20.0f);
    assert(result.final_correction_mm_s > line.correction_mm_s);
}

static void test_fixed_straight_mode_ignores_early_outer_mask(void)
{
    LineFollowSnapshot_t line = make_line(220.0f, 36.0f);
    LineYawAssistSnapshot_t result;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_STRAIGHT);
    LineYawAssist_Update(&line, 60.0f, 0.0f, -5.0f,
                         3.0f, 0.88f, true, 100U);
    LineYawAssist_Update(&line, 60.0f, -1.0f, -5.0f,
                         3.0f, 0.88f, true, 120U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(!result.curve_candidate_active);
    assert(!result.curve_rate_active);
    assert(result.curve_direction == 0);
}

static void test_fixed_curve_mode_starts_feedforward_at_centre(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 0.0f);
    LineYawAssistSnapshot_t result;
    float feedforward = 160.0f * CHASSIS_DRIVE_TRACK_MM /
                        (2.0f * LINE_CURVE_RADIUS_MM);
    float expected = (-2.0f *
        (feedforward - LINE_CURVE_POSITION_TRIM_MM_S *
         LINE_CURVE_TARGET_POSITION) / CHASSIS_DRIVE_TRACK_MM) *
        57.2957795f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_CURVE_POSITIVE);
    LineYawAssist_Update(&line, 60.0f, 0.0f, expected,
                         3.0f, 0.88f, true, 100U);
    LineYawAssist_Update(&line, 60.0f, -0.4f, expected,
                         3.0f, 0.88f, true, 120U);
    LineYawAssist_GetSnapshot(&result);
    assert(!result.straight_heading_active);
    assert(result.curve_rate_active);
    assert(result.curve_direction == 1);
    assert(result.curve_feedforward_correction_mm_s > 34.0f);
    assert(fabsf(result.desired_yaw_rate_deg_s - expected) < 0.1f);
}

static void test_fixed_curve_rejects_straight_pd_derivative_spike(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 50.0f);
    LineYawAssistSnapshot_t before;
    LineYawAssistSnapshot_t after;
    uint32_t now_ms;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_CURVE_POSITIVE);
    for (now_ms = 100U; now_ms <= 500U; now_ms += 20U)
        LineYawAssist_Update(&line, 60.0f, -10.0f, -18.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&before);

    line.correction_mm_s = -60.0f;
    line.left_target_mm_s = 100.0f;
    line.right_target_mm_s = 220.0f;
    line.active_mask = 0x04U;
    line.position = -3.0f / 7.0f;
    LineYawAssist_Update(&line, 60.0f, -10.4f, -18.0f,
                         3.0f, 0.88f, true, 520U);
    LineYawAssist_GetSnapshot(&after);
    assert(before.final_correction_mm_s > 25.0f);
    assert(after.final_correction_mm_s > 25.0f);
    assert(fabsf(after.final_correction_mm_s -
                 before.final_correction_mm_s) <= 4.01f);
    assert(after.assist_correction_mm_s > 80.0f);
}

static void test_fixed_curve_uses_low_gain_derivative_damping(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 0.0f);
    LineYawAssistSnapshot_t moving_outward;
    LineYawAssistSnapshot_t moving_inward;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    line.filtered_derivative = 1.0f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_CURVE_POSITIVE);
    LineYawAssist_Update(&line, 60.0f, -20.0f, -18.0f,
                         3.0f, 0.88f, true, 100U);
    LineYawAssist_GetSnapshot(&moving_outward);

    line.filtered_derivative = -1.0f;
    LineYawAssist_Update(&line, 60.0f, -20.4f, -18.0f,
                         3.0f, 0.88f, true, 120U);
    LineYawAssist_GetSnapshot(&moving_inward);
    assert(moving_outward.desired_yaw_rate_deg_s <
           moving_inward.desired_yaw_rate_deg_s - 5.0f);
}

static void test_fixed_curve_holds_command_through_tracking_gap(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 40.0f);
    LineYawAssistSnapshot_t before;
    LineYawAssistSnapshot_t gap;
    uint32_t now_ms;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_CURVE_POSITIVE);
    for (now_ms = 100U; now_ms <= 500U; now_ms += 20U)
        LineYawAssist_Update(&line, 60.0f, -20.0f, -18.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&before);

    line.line_valid = false;
    line.active_mask = 0x00U;
    line.dynamic_base_mm_s = 105.0f;
    line.correction_mm_s = -48.0f;
    line.left_target_mm_s = 57.0f;
    line.right_target_mm_s = 153.0f;
    LineYawAssist_Update(&line, 60.0f, -20.4f, -18.0f,
                         3.0f, 0.88f, true, 520U);
    LineYawAssist_GetSnapshot(&gap);
    assert(before.final_correction_mm_s > 25.0f);
    assert(gap.active);
    assert(gap.curve_rate_active);
    assert(gap.final_correction_mm_s > 20.0f);
    assert(gap.left_target_mm_s > gap.right_target_mm_s);
}

static void test_fixed_curve_to_straight_transition_captures_heading(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 0.0f);
    LineYawAssistSnapshot_t result;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_CURVE_POSITIVE);
    LineYawAssist_Update(&line, 60.0f, -170.0f, -18.0f,
                         3.0f, 0.88f, true, 100U);
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_STRAIGHT);
    line.active_mask = 0x04U;
    line.position = -3.0f / 7.0f;
    LineYawAssist_Update(&line, 60.0f, -175.0f, -5.0f,
                         3.0f, 0.88f, true, 120U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(!result.curve_rate_active);
    assert(result.curve_direction == 0);
    assert(fabsf(result.straight_reference_yaw_deg + 175.0f) < 0.1f);
}

static void test_fixed_straight_can_hold_planned_exit_heading(void)
{
    LineFollowSnapshot_t line = make_line(220.0f, 0.0f);
    LineYawAssistSnapshot_t result;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_STRAIGHT);
    LineYawAssist_SetStraightReference(true, -179.0f);
    LineYawAssist_Update(&line, 60.0f, -174.0f, 0.0f,
                         3.0f, 0.88f, true, 100U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(fabsf(result.straight_reference_yaw_deg + 179.0f) < 0.1f);
    assert(result.straight_heading_error_deg < -4.9f);
    assert(result.desired_yaw_rate_deg_s < -5.9f);
}

static void test_planned_exit_uses_lateral_capture_angle(void)
{
    LineFollowSnapshot_t line = make_line(220.0f, 0.0f);
    LineYawAssistSnapshot_t result;
    line.active_mask = 0x04U;
    line.position = -3.0f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_STRAIGHT);
    LineYawAssist_SetStraightReference(true, -177.7f);
    LineYawAssist_Update(&line, 60.0f, -176.4f, 0.0f,
                         3.0f, 0.88f, true, 100U);
    LineYawAssist_GetSnapshot(&result);
    assert(result.straight_heading_active);
    assert(fabsf(result.straight_target_yaw_deg + 175.1286f) < 0.1f);
    assert(result.desired_yaw_rate_deg_s > 3.5f);
}

static void test_planned_exit_keeps_curve_command_continuous(void)
{
    LineFollowSnapshot_t line = make_line(160.0f, 50.0f);
    LineYawAssistSnapshot_t curve;
    LineYawAssistSnapshot_t exit;
    uint32_t now_ms;
    line.active_mask = 0x20U;
    line.position = 3.0f / 7.0f;
    LineYawAssist_Init();
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_CURVE_POSITIVE);
    for (now_ms = 100U; now_ms <= 500U; now_ms += 20U)
        LineYawAssist_Update(&line, 60.0f, -170.0f, -18.0f,
                             3.0f, 0.88f, true, now_ms);
    LineYawAssist_GetSnapshot(&curve);

    line = make_line(220.0f, -20.0f);
    line.active_mask = 0x02U;
    line.position = -5.0f / 7.0f;
    LineYawAssist_SetMode(LINE_YAW_ASSIST_MODE_STRAIGHT);
    LineYawAssist_SetStraightReference(true, -179.0f);
    LineYawAssist_Update(&line, 60.0f, -174.0f, -4.0f,
                         3.0f, 0.88f, true, 520U);
    LineYawAssist_GetSnapshot(&exit);
    assert(curve.final_correction_mm_s > 25.0f);
    assert(exit.final_correction_mm_s > 20.0f);
    assert(exit.final_correction_mm_s <=
           curve.final_correction_mm_s + 0.01f);
    assert(exit.straight_heading_active);
    assert(exit.desired_yaw_rate_deg_s < -2.0f);
}

int main(void)
{
    test_straight_uncommanded_yaw_is_damped();
    test_straight_heading_drift_is_corrected_at_zero_rate();
    test_commanded_curve_is_not_forced_straight();
    test_overturn_reduces_and_underturn_increases_command();
    test_invalid_loss_ambiguous_and_low_speed_disable();
    test_limits_slew_wrap_and_no_reverse();
    test_deadband_and_immediate_invalid_reset();
    test_near_centre_uses_heading_and_countersteers_early();
    test_confirmed_curve_feedforward_persists_at_centre();
    test_outer_transient_crossing_back_is_not_a_curve();
    test_semicircle_progress_arms_exit_and_guards_straight();
    test_short_tracking_gap_preserves_confirmed_curve();
    test_fixed_straight_mode_ignores_early_outer_mask();
    test_fixed_curve_mode_starts_feedforward_at_centre();
    test_fixed_curve_rejects_straight_pd_derivative_spike();
    test_fixed_curve_uses_low_gain_derivative_damping();
    test_fixed_curve_holds_command_through_tracking_gap();
    test_fixed_curve_to_straight_transition_captures_heading();
    test_fixed_straight_can_hold_planned_exit_heading();
    test_planned_exit_uses_lateral_capture_angle();
    test_planned_exit_keeps_curve_command_continuous();
    return 0;
}
