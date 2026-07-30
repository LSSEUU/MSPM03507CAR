#include "h_course_profile.h"

#include "control_config.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static uint32_t now_ms;

static void advance(float speed, float yaw_rate, bool line_usable,
                    uint8_t mask, float position, uint32_t steps)
{
    uint32_t i;
    for (i = 0U; i < steps; i++) {
        now_ms += 20U;
        HCourse_Update(now_ms, true, speed, speed, true,
                       yaw_rate * ((float)(now_ms - 100U) * 0.001f), yaw_rate,
                       line_usable, mask, position);
    }
}

static HCourseSnapshot_t snapshot(void)
{
    HCourseSnapshot_t result;
    HCourse_GetSnapshot(&result);
    return result;
}

static void test_early_outer_transient_cannot_start_curve(void)
{
    HCourseSnapshot_t result;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(220.0f, 0.0f, true, 0x30U, 0.3f, 100U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.phase_distance_mm < H_COURSE_STRAIGHT_ARM_MM);
}

static void test_distance_window_and_line_evidence_enter_curve(void)
{
    HCourseSnapshot_t result;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(200.0f, 0.0f, true, 0x18U, 0.0f, 340U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.phase_distance_mm >= H_COURSE_STRAIGHT_ARM_MM);
    advance(200.0f, -2.0f, true, 0x20U, 0.3f,
            H_COURSE_CURVE_EVIDENCE_SAMPLES);
    result = snapshot();
    assert(result.phase == H_COURSE_BC_CURVE);
    assert(fabsf(result.curve_progress_deg) < 0.01f);
    assert(fabsf(HCourse_SelectBaseSpeed(220.0f, 160.0f, 300.0f,
                                        true, 0.0f) - 160.0f) <
           0.01f);
}

static void test_fixed_course_forces_high_speed_transition_before_tangent(void)
{
    HCourseSnapshot_t result;
    uint32_t steps_before_force;
    now_ms = 100U;
    HCourse_Start(now_ms);

    assert(H_COURSE_STRAIGHT_FORCE_MM < 1500.0f);
    steps_before_force =
        (uint32_t)(H_COURSE_STRAIGHT_FORCE_MM / (300.0f * 0.02f));
    advance(300.0f, 0.0f, true, 0x18U, 0.0f,
            steps_before_force - 1U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);

    advance(300.0f, 0.0f, true, 0x18U, 0.0f, 2U);
    result = snapshot();
    assert(result.phase == H_COURSE_BC_CURVE);
    assert(result.total_distance_mm < 1500.0f);
}

static void test_fast_profile_decelerates_before_steering(void)
{
    HCourseSnapshot_t result;
    float selected;
    now_ms = 100U;
    HCourse_Start(now_ms);

    assert(H_COURSE_FAST_STRAIGHT_SPEED_MM_S > 300.0f);
    assert(H_COURSE_FAST_STRAIGHT_FORCE_MM >
           H_COURSE_STRAIGHT_FORCE_MM);
    assert(H_COURSE_FAST_CURVE_SPEED_START_MM <
           H_COURSE_FAST_STRAIGHT_FORCE_MM);

    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 169U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.fast_straight_latched);
    assert(result.phase_distance_mm >=
           H_COURSE_FAST_CURVE_SPEED_START_MM);
    assert(result.phase_distance_mm < H_COURSE_FAST_STRAIGHT_FORCE_MM);
    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       true, 0.0f);
    assert(fabsf(selected - 280.0f) < 0.01f);

    /* The fast latch survives the commanded slowdown, so the ordinary
     * 1400 mm fallback cannot start yaw feed-forward during deceleration. */
    advance(280.0f, 0.0f, true, 0x18U, 0.0f, 18U);
    result = snapshot();
    assert(result.phase == H_COURSE_BC_CURVE);
    assert(result.total_distance_mm >= H_COURSE_FAST_STRAIGHT_FORCE_MM);
}

static void test_fast_profile_ignores_early_curve_evidence_during_deceleration(void)
{
    HCourseSnapshot_t result;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 169U);
    result = snapshot();
    assert(result.fast_straight_latched);
    assert(result.phase_distance_mm >=
           H_COURSE_FAST_CURVE_SPEED_START_MM);
    assert(result.phase_distance_mm <
           H_COURSE_FAST_CURVE_EVIDENCE_ARM_MM);

    advance(280.0f, 0.0f, true, 0x30U, 0.3f,
            H_COURSE_CURVE_EVIDENCE_SAMPLES + 3U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.curve_evidence_samples == 0U);

    advance(280.0f, 0.0f, true, 0x30U, 0.3f, 12U);
    result = snapshot();
    assert(result.phase == H_COURSE_BC_CURVE);
}

static void test_semicircle_enters_second_straight_immediately(void)
{
    HCourseSnapshot_t result;
    uint32_t steps;
    now_ms = 100U;
    HCourse_Start(now_ms);
    assert(H_COURSE_CURVE_EXIT_DEG < 175.0f);
    advance(200.0f, 0.0f, true, 0x18U, 0.0f, 375U);
    result = snapshot();
    assert(result.phase == H_COURSE_BC_CURVE);
    steps = (uint32_t)(H_COURSE_CURVE_EXIT_DEG /
                      (18.0f * 0.02f)) + 1U;
    advance(160.0f, -18.0f, false, 0x00U, 0.0f, steps);
    result = snapshot();
    assert(result.phase == H_COURSE_CD_STRAIGHT);
    assert(fabsf(result.phase_distance_mm) < 0.01f);
    assert(result.straight_heading_target_valid);
    assert(fabsf(fabsf(result.straight_heading_target_deg) - 180.0f) <
           1.0f);
    assert(fabsf(HCourse_SelectBaseSpeed(220.0f, 160.0f, 220.0f,
                                        true, 0.0f) - 220.0f) <
           0.01f);
}

static void test_fixed_straight_has_safe_high_speed_envelope(void)
{
    float selected;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 1U);

    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       true, 0.286f);
    assert(fabsf(selected - 400.0f) < 0.01f);
    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       true, 0.45f);
    assert(fabsf(selected - 340.0f) < 0.01f);
    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       true, 0.60f);
    assert(fabsf(selected - 280.0f) < 0.01f);
    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       false, 0.0f);
    assert(fabsf(selected - 280.0f) < 0.01f);
}

static void test_exit_heading_remains_owned_after_line_capture(void)
{
    HCourseSnapshot_t result;
    uint32_t steps;
    uint32_t i;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(200.0f, 0.0f, true, 0x18U, 0.0f, 375U);
    steps = (uint32_t)(H_COURSE_CURVE_EXIT_DEG /
                      (18.0f * 0.02f)) + 1U;
    advance(160.0f, -18.0f, true, 0x20U, 0.4f, steps);

    for (i = 0U; i < 50U; i++) {
        now_ms += 20U;
        HCourse_Update(now_ms, true, 220.0f, 220.0f, true,
                       -170.0f, 0.0f, true, 0x18U, 0.0f);
    }
    result = snapshot();
    assert(result.phase_distance_mm > H_COURSE_EXIT_CAPTURE_MIN_MM);
    assert(result.straight_heading_target_valid);

    for (i = 0U; i < H_COURSE_EXIT_CAPTURE_SAMPLES; i++) {
        now_ms += 20U;
        HCourse_Update(now_ms, true, 220.0f, 220.0f, true,
                       179.0f, 0.0f, true, 0x18U, 0.0f);
    }
    result = snapshot();
    assert(result.straight_heading_target_valid);
    assert(result.straight_capture_samples >=
           H_COURSE_EXIT_CAPTURE_SAMPLES);
}

static void test_exit_heading_remains_owned_for_whole_straight(void)
{
    HCourseSnapshot_t result;
    uint32_t steps;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(200.0f, 0.0f, true, 0x18U, 0.0f, 375U);
    steps = (uint32_t)(H_COURSE_CURVE_EXIT_DEG /
                      (18.0f * 0.02f)) + 1U;
    advance(160.0f, -18.0f, true, 0x20U, 0.4f, steps);

    advance(220.0f, 0.0f, true, 0x04U, -0.4f, 205U);
    result = snapshot();
    assert(result.phase_distance_mm > 800.0f);
    assert(result.phase_distance_mm < H_COURSE_EXIT_CAPTURE_MAX_MM);
    assert(result.straight_heading_target_valid);

    advance(220.0f, 0.0f, true, 0x04U, -0.4f, 30U);
    result = snapshot();
    assert(result.phase_distance_mm >= H_COURSE_EXIT_CAPTURE_MAX_MM);
    assert(result.straight_heading_target_valid);
}

static void test_initial_ab_heading_is_owned_until_curve(void)
{
    HCourseSnapshot_t result;
    now_ms = 100U;
    HCourse_Start(now_ms);

    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 1U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.straight_heading_target_valid);
    assert(fabsf(result.straight_heading_target_deg) < 0.01f);

    advance(400.0f, 0.0f, true, 0x04U, -0.4f, 150U);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.straight_heading_target_valid);

    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 32U);
    result = snapshot();
    assert(result.phase == H_COURSE_BC_CURVE);
    assert(!result.straight_heading_target_valid);
}

static void test_fixed_straight_learns_bounded_track_tangent(void)
{
    HCourseSnapshot_t result;
    uint32_t i;
    now_ms = 100U;
    HCourse_Start(now_ms);

    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 1U);
    for (i = 0U; i < 100U; i++) {
        now_ms += 20U;
        HCourse_Update(now_ms, true, 200.0f, 200.0f, true,
                       0.0f, 0.0f, true, 0x04U, -0.4f);
    }
    result = snapshot();
    assert(result.straight_heading_target_deg > 1.5f);
    assert(result.straight_heading_target_deg <=
           H_COURSE_STRAIGHT_TANGENT_TRIM_LIMIT_DEG);

    for (i = 0U; i < 500U; i++) {
        now_ms += 20U;
        HCourse_Update(now_ms, true, 0.0f, 0.0f, true,
                       0.0f, 0.0f, true, 0x04U, -0.4f);
    }
    result = snapshot();
    assert(fabsf(result.straight_heading_target_deg -
                 H_COURSE_STRAIGHT_TANGENT_TRIM_LIMIT_DEG) < 0.01f);
}

static void test_second_curve_completes_lap(void)
{
    HCourseSnapshot_t result;
    uint32_t steps;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(200.0f, 0.0f, true, 0x18U, 0.0f, 375U);
    steps = (uint32_t)(H_COURSE_CURVE_EXIT_DEG /
                      (18.0f * 0.02f)) + 1U;
    advance(160.0f, -18.0f, true, 0x20U, 0.4f, steps);
    advance(200.0f, 0.0f, true, 0x18U, 0.0f, 375U);
    result = snapshot();
    assert(result.phase == H_COURSE_DA_CURVE);
    advance(160.0f, -18.0f, true, 0x20U, 0.4f, steps);
    result = snapshot();
    assert(result.phase == H_COURSE_AB_STRAIGHT);
    assert(result.lap_count == 1U);
}

static void test_h2_finish_line_uses_window_and_qualified_crossbar(void)
{
    HCourseSnapshot_t result;
    float selected;
    now_ms = 100U;
    HCourse_Start(now_ms);
    HCourse_SetFinishEnabled(true);

    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 650U);
    advance(400.0f, 0.0f, false, 0x3CU, 0.0f,
            H2_FINISH_QUALIFY_SAMPLES);
    result = snapshot();
    assert(!result.finish_line_detected);

    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 20U);
    assert(snapshot().total_distance_mm >= H2_FINISH_WINDOW_MIN_MM);
    advance(180.0f, 0.0f, false, 0x3CU, 0.0f, 1U);
    result = snapshot();
    assert(!result.finish_line_detected);
    advance(180.0f, 0.0f, true, 0x18U, 0.0f, 1U);
    advance(180.0f, 0.0f, false, 0x3CU, 0.0f,
            H2_FINISH_QUALIFY_SAMPLES);
    result = snapshot();
    assert(result.finish_line_detected);
    assert(result.finish_line_distance_mm >= H2_FINISH_WINDOW_MIN_MM);

    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       false, 0.0f);
    assert(fabsf(selected - H2_FINISH_CRAWL_SPEED_MM_S) < 0.01f);
}

static void test_h2_finish_approach_does_not_affect_generic_line_action(void)
{
    float selected;
    now_ms = 100U;
    HCourse_Start(now_ms);
    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 690U);
    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       true, 0.0f);
    assert(fabsf(selected - 280.0f) < 0.01f);

    HCourse_SetFinishEnabled(true);
    advance(400.0f, 0.0f, true, 0x18U, 0.0f, 1U);
    selected = HCourse_SelectBaseSpeed(280.0f, 280.0f, 400.0f,
                                       true, 0.0f);
    assert(fabsf(selected - H2_FINISH_APPROACH_SPEED_MM_S) < 0.01f);
}

int main(void)
{
    test_early_outer_transient_cannot_start_curve();
    test_distance_window_and_line_evidence_enter_curve();
    test_fixed_course_forces_high_speed_transition_before_tangent();
    test_fast_profile_decelerates_before_steering();
    test_fast_profile_ignores_early_curve_evidence_during_deceleration();
    test_semicircle_enters_second_straight_immediately();
    test_fixed_straight_has_safe_high_speed_envelope();
    test_exit_heading_remains_owned_after_line_capture();
    test_exit_heading_remains_owned_for_whole_straight();
    test_initial_ab_heading_is_owned_until_curve();
    test_fixed_straight_learns_bounded_track_tangent();
    test_second_curve_completes_lap();
    test_h2_finish_line_uses_window_and_qualified_crossbar();
    test_h2_finish_approach_does_not_affect_generic_line_action();
    return 0;
}
