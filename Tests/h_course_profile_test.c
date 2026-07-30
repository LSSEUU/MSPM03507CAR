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
    assert(fabsf(HCourse_SelectBaseSpeed(220.0f, 160.0f) - 160.0f) <
           0.01f);
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
    assert(fabsf(HCourse_SelectBaseSpeed(220.0f, 160.0f) - 220.0f) <
           0.01f);
}

static void test_exit_heading_waits_for_line_and_heading_capture(void)
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
    assert(!result.straight_heading_target_valid);
}

static void test_exit_heading_fallback_keeps_control_past_800_mm(void)
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
    assert(!result.straight_heading_target_valid);
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

int main(void)
{
    test_early_outer_transient_cannot_start_curve();
    test_distance_window_and_line_evidence_enter_curve();
    test_semicircle_enters_second_straight_immediately();
    test_exit_heading_waits_for_line_and_heading_capture();
    test_exit_heading_fallback_keeps_control_past_800_mm();
    test_second_curve_completes_lap();
    return 0;
}
