#include "h_course_profile.h"

#include "control_config.h"

#include <stddef.h>
#include <string.h>

static HCourseSnapshot_t course;

static float HCourse_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float HCourse_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees <= -180.0f) degrees += 360.0f;
    return degrees;
}

static bool HCourse_IsPositiveCurveEvidence(uint8_t mask, float position)
{
    return (position >= H_COURSE_CURVE_EVIDENCE_POSITION) ||
           ((mask & 0xE0U) != 0U);
}

static void HCourse_EnterCurve(HCoursePhase_t phase,
                               bool heading_sample_valid, float yaw_deg)
{
    course.phase = phase;
    course.phase_distance_mm = 0.0f;
    course.curve_progress_deg = 0.0f;
    course.curve_entry_heading_valid = heading_sample_valid;
    course.curve_entry_yaw_deg = yaw_deg;
    course.straight_heading_target_valid = false;
    course.straight_capture_samples = 0U;
    course.curve_evidence_samples = 0U;
}

static void HCourse_EnterStraight(HCoursePhase_t phase)
{
    course.phase = phase;
    course.phase_distance_mm = 0.0f;
    course.curve_progress_deg = 0.0f;
    course.straight_heading_target_valid =
        course.curve_entry_heading_valid;
    course.straight_heading_target_deg = HCourse_NormalizeDegrees(
        course.curve_entry_yaw_deg - 180.0f);
    course.straight_capture_samples = 0U;
    course.curve_evidence_samples = 0U;
}

void HCourse_Init(void)
{
    memset(&course, 0, sizeof(course));
    course.phase = H_COURSE_AB_STRAIGHT;
}

void HCourse_Start(uint32_t now_ms)
{
    HCourse_Init();
    course.running = true;
    course.last_update_ms = now_ms;
}

void HCourse_Stop(void)
{
    course.running = false;
}

bool HCourse_IsCurvePhase(void)
{
    return (course.phase == H_COURSE_BC_CURVE) ||
           (course.phase == H_COURSE_DA_CURVE);
}

float HCourse_SelectBaseSpeed(float tracking_base_mm_s,
                              float curve_base_mm_s)
{
    if (HCourse_IsCurvePhase() &&
        (tracking_base_mm_s > curve_base_mm_s)) {
        return curve_base_mm_s;
    }
    return tracking_base_mm_s;
}

void HCourse_Update(uint32_t now_ms, bool wheel_sample_valid,
                    float left_speed_mm_s, float right_speed_mm_s,
                    bool heading_sample_valid, float yaw_deg,
                    float yaw_rate_deg_s, bool line_usable,
                    uint8_t active_mask, float line_position)
{
    uint32_t dt_ms;
    float dt_s;
    float forward_speed;
    bool curve_evidence;
    if (!course.running) return;
    dt_ms = (uint32_t)(now_ms - course.last_update_ms);
    course.last_update_ms = now_ms;
    if (dt_ms == 0U) return;
    if (dt_ms > H_COURSE_MAX_DT_MS) dt_ms = H_COURSE_MAX_DT_MS;
    dt_s = (float)dt_ms * 0.001f;

    if (wheel_sample_valid) {
        forward_speed = (left_speed_mm_s + right_speed_mm_s) * 0.5f;
        if (forward_speed > 0.0f) {
            course.total_distance_mm += forward_speed * dt_s;
            course.phase_distance_mm += forward_speed * dt_s;
        }
    }

    if (HCourse_IsCurvePhase()) {
        course.curve_progress_deg += -yaw_rate_deg_s * dt_s;
        if (course.curve_progress_deg < 0.0f)
            course.curve_progress_deg = 0.0f;
        if (course.curve_progress_deg >= H_COURSE_CURVE_EXIT_DEG) {
            if (course.phase == H_COURSE_BC_CURVE) {
                HCourse_EnterStraight(H_COURSE_CD_STRAIGHT);
            } else {
                course.lap_count++;
                HCourse_EnterStraight(H_COURSE_AB_STRAIGHT);
            }
        }
        return;
    }

    if (course.straight_heading_target_valid) {
        float heading_error = HCourse_NormalizeDegrees(
            course.straight_heading_target_deg - yaw_deg);
        bool central = (active_mask == 0x08U) ||
                       (active_mask == 0x18U) ||
                       (active_mask == 0x10U);
        bool captured =
            (course.phase_distance_mm >=
             H_COURSE_EXIT_CAPTURE_MIN_MM) &&
            heading_sample_valid && line_usable && central &&
            (HCourse_Abs(line_position) <=
             H_COURSE_EXIT_CAPTURE_POSITION_MAX) &&
            (HCourse_Abs(heading_error) <=
             H_COURSE_EXIT_CAPTURE_HEADING_MAX_DEG) &&
            (HCourse_Abs(yaw_rate_deg_s) <=
             H_COURSE_EXIT_CAPTURE_RATE_MAX_DEG_S);
        if (captured) {
            if (course.straight_capture_samples < 0xFFU)
                course.straight_capture_samples++;
        } else {
            course.straight_capture_samples = 0U;
        }
        if ((course.straight_capture_samples >=
             H_COURSE_EXIT_CAPTURE_SAMPLES) ||
            (course.phase_distance_mm >=
             H_COURSE_EXIT_CAPTURE_MAX_MM)) {
            course.straight_heading_target_valid = false;
            course.straight_capture_samples = 0U;
        }
    }

    curve_evidence = line_usable &&
        HCourse_IsPositiveCurveEvidence(active_mask, line_position);
    if ((course.phase_distance_mm >= H_COURSE_STRAIGHT_ARM_MM) &&
        curve_evidence) {
        if (course.curve_evidence_samples < 0xFFU)
            course.curve_evidence_samples++;
    } else {
        course.curve_evidence_samples = 0U;
    }
    if ((course.curve_evidence_samples >=
         H_COURSE_CURVE_EVIDENCE_SAMPLES) ||
        (course.phase_distance_mm >= H_COURSE_STRAIGHT_FORCE_MM)) {
        HCourse_EnterCurve(
            (course.phase == H_COURSE_AB_STRAIGHT) ?
            H_COURSE_BC_CURVE : H_COURSE_DA_CURVE,
            heading_sample_valid, yaw_deg);
    }
}

void HCourse_GetSnapshot(HCourseSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = course;
}
