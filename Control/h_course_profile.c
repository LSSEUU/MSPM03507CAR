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

static uint8_t HCourse_ActiveCount(uint8_t mask)
{
    uint8_t count = 0U;
    while (mask != 0U) {
        count = (uint8_t)(count + (mask & 1U));
        mask >>= 1U;
    }
    return count;
}

static uint8_t HCourse_MaxContiguousCount(uint8_t mask)
{
    uint8_t maximum = 0U;
    uint8_t current = 0U;
    uint8_t i;
    for (i = 0U; i < 8U; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            current++;
            if (current > maximum) maximum = current;
        } else {
            current = 0U;
        }
    }
    return maximum;
}

static void HCourse_UpdateFinishDetector(uint8_t active_mask)
{
    uint8_t active_count;
    bool ordinary_line;
    bool crossbar_candidate;
    if (!course.finish_enabled || course.finish_line_detected) return;
    active_count = HCourse_ActiveCount(active_mask);
    ordinary_line = (active_count >= 1U) && (active_count <= 2U);
    crossbar_candidate =
        (active_count >= H2_FINISH_MIN_ACTIVE_CHANNELS) &&
        (HCourse_MaxContiguousCount(active_mask) >=
         H2_FINISH_MIN_CONTIGUOUS_CHANNELS);
    /* H5 can pass short, wide painted features while it is still in the
     * second semicircle.  Distance alone cannot distinguish those features
     * from A, especially when the loaded chassis cuts the inside of DA.
     * Arm the H5 finish only after gyro-owned course progress has completed
     * both semicircles.  H2 keeps its verified distance-only behaviour. */
    if ((course.finish_mode == H_COURSE_FINISH_H5_LATCH) &&
        (course.lap_count == 0U)) {
        course.finish_candidate_samples = 0U;
        if (ordinary_line) {
            if (course.finish_preline_samples < 0xFFU)
                course.finish_preline_samples++;
        } else {
            course.finish_preline_samples = 0U;
        }
        return;
    }
    if ((course.total_distance_mm < H2_FINISH_WINDOW_MIN_MM) ||
        (course.total_distance_mm > H2_FINISH_WINDOW_MAX_MM)) {
        course.finish_candidate_samples = 0U;
        if (ordinary_line) {
            if (course.finish_preline_samples < 0xFFU)
                course.finish_preline_samples++;
        } else {
            course.finish_preline_samples = 0U;
        }
        return;
    }
    if (crossbar_candidate &&
        (course.finish_preline_samples >= H2_FINISH_PRELINE_SAMPLES)) {
        if (course.finish_candidate_samples < 0xFFU)
            course.finish_candidate_samples++;
        if (course.finish_candidate_samples >=
            H2_FINISH_QUALIFY_SAMPLES) {
            course.finish_line_detected = true;
            course.finish_line_distance_mm = course.total_distance_mm;
        }
        return;
    }
    course.finish_candidate_samples = 0U;
    if (ordinary_line) {
        if (course.finish_preline_samples < 0xFFU)
            course.finish_preline_samples++;
    } else {
        course.finish_preline_samples = 0U;
    }
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
    course.fast_straight_latched = false;
}

static void HCourse_EnterStraight(HCoursePhase_t phase)
{
    course.phase = phase;
    course.phase_distance_mm = 0.0f;
    course.curve_progress_deg = 0.0f;
    course.straight_heading_target_valid =
        course.curve_entry_heading_valid;
    course.straight_heading_anchor_deg = HCourse_NormalizeDegrees(
        course.curve_entry_yaw_deg - 180.0f);
    course.straight_heading_target_deg =
        course.straight_heading_anchor_deg;
    course.straight_capture_samples = 0U;
    course.curve_evidence_samples = 0U;
    course.fast_straight_latched = false;
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

void HCourse_SetFinishEnabled(bool enabled)
{
    HCourse_SetFinishMode(enabled ? H_COURSE_FINISH_H2_STOP :
                          H_COURSE_FINISH_NONE);
}

void HCourse_SetH2OfficialStart(void)
{
    if (!course.initial_ab_offset_applied &&
        (course.phase == H_COURSE_AB_STRAIGHT) &&
        (course.total_distance_mm <= 0.01f)) {
        course.phase_distance_mm +=
            H2_COURSE_INITIAL_AB_PHASE_OFFSET_MM;
        course.initial_ab_offset_applied = true;
    }
}

void HCourse_SetFinishMode(HCourseFinishMode_t mode)
{
    course.finish_mode = mode;
    course.finish_enabled = mode != H_COURSE_FINISH_NONE;
    if (mode == H_COURSE_FINISH_H2_STOP) HCourse_SetH2OfficialStart();
    course.finish_preline_samples = 0U;
    course.finish_candidate_samples = 0U;
    course.finish_line_detected = false;
    course.finish_line_distance_mm = 0.0f;
}

bool HCourse_IsCurvePhase(void)
{
    return (course.phase == H_COURSE_BC_CURVE) ||
           (course.phase == H_COURSE_DA_CURVE);
}

float HCourse_SelectCurveFeedforwardScale(void)
{
    float blend;
    if (course.phase == H_COURSE_BC_CURVE)
        return H_COURSE_BC_CURVE_FEEDFORWARD_SCALE;
    if (course.phase != H_COURSE_DA_CURVE) return 1.0f;
    blend = course.curve_progress_deg /
        H_COURSE_DA_CURVE_FEEDFORWARD_RAMP_DEG;
    if (blend < 0.0f) blend = 0.0f;
    if (blend > 1.0f) blend = 1.0f;
    return H_COURSE_DA_CURVE_FEEDFORWARD_START_SCALE +
        (H_COURSE_DA_CURVE_FEEDFORWARD_SCALE -
         H_COURSE_DA_CURVE_FEEDFORWARD_START_SCALE) * blend;
}

float HCourse_SelectBaseSpeed(float tracking_base_mm_s,
                              float curve_base_mm_s,
                              float straight_base_mm_s,
                              bool line_usable,
                              float line_position)
{
    float position_abs;
    float blend;
    if (course.finish_mode == H_COURSE_FINISH_H2_STOP) {
        if (course.finish_line_detected) {
            return (tracking_base_mm_s > H2_FINISH_CRAWL_SPEED_MM_S) ?
                H2_FINISH_CRAWL_SPEED_MM_S : tracking_base_mm_s;
        }
        if ((course.total_distance_mm >=
             H2_FINISH_APPROACH_START_MM) &&
            (course.total_distance_mm <= H2_FINISH_WINDOW_MAX_MM)) {
            return (tracking_base_mm_s >
                    H2_FINISH_APPROACH_SPEED_MM_S) ?
                H2_FINISH_APPROACH_SPEED_MM_S : tracking_base_mm_s;
        }
    }
    if (HCourse_IsCurvePhase() &&
        (tracking_base_mm_s > curve_base_mm_s)) {
        return curve_base_mm_s;
    }
    if (!HCourse_IsCurvePhase() && course.fast_straight_latched &&
        (course.phase_distance_mm >=
         H_COURSE_FAST_CURVE_SPEED_START_MM)) {
        return (tracking_base_mm_s > curve_base_mm_s) ?
            curve_base_mm_s : tracking_base_mm_s;
    }
    if (!HCourse_IsCurvePhase() &&
        course.straight_heading_target_valid && line_usable &&
        (straight_base_mm_s > tracking_base_mm_s)) {
        position_abs = HCourse_Abs(line_position);
        if (position_abs <=
            H_COURSE_STRAIGHT_FULL_SPEED_POSITION_MAX) {
            return straight_base_mm_s;
        }
        if (position_abs < H_COURSE_STRAIGHT_FALLBACK_POSITION) {
            blend = (position_abs -
                     H_COURSE_STRAIGHT_FULL_SPEED_POSITION_MAX) /
                    (H_COURSE_STRAIGHT_FALLBACK_POSITION -
                     H_COURSE_STRAIGHT_FULL_SPEED_POSITION_MAX);
            return straight_base_mm_s +
                (tracking_base_mm_s - straight_base_mm_s) * blend;
        }
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
    float forward_speed = 0.0f;
    float force_distance_mm;
    float curve_evidence_arm_mm;
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
        if (!HCourse_IsCurvePhase() &&
            (forward_speed >= H_COURSE_FAST_STRAIGHT_SPEED_MM_S)) {
            course.fast_straight_latched = true;
        }
    }
    HCourse_UpdateFinishDetector(active_mask);

    /* The fixed course starts tangent to AB.  Capture that initial heading
     * from the first valid gyro sample so the high-speed first straight has
     * the same planned-heading ownership as a straight entered from a curve. */
    if ((course.phase == H_COURSE_AB_STRAIGHT) &&
        (course.lap_count == 0U) &&
        !course.straight_heading_target_valid &&
        heading_sample_valid &&
        (course.phase_distance_mm <= 100.0f)) {
        course.straight_heading_target_valid = true;
        course.straight_heading_anchor_deg = yaw_deg;
        course.straight_heading_target_deg = yaw_deg;
        course.straight_capture_samples = 0U;
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
        float tangent_trim_deg;
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
        if (line_usable && heading_sample_valid &&
            (HCourse_Abs(line_position) <=
             H_COURSE_STRAIGHT_TANGENT_POSITION_MAX) &&
            (HCourse_Abs(yaw_rate_deg_s) <=
             H_COURSE_STRAIGHT_TANGENT_RATE_MAX_DEG_S)) {
            course.straight_heading_target_deg = HCourse_NormalizeDegrees(
                course.straight_heading_target_deg -
                H_COURSE_STRAIGHT_TANGENT_ADAPT_DEG_S * line_position *
                dt_s);
            tangent_trim_deg = HCourse_NormalizeDegrees(
                course.straight_heading_target_deg -
                course.straight_heading_anchor_deg);
            if (tangent_trim_deg >
                H_COURSE_STRAIGHT_TANGENT_TRIM_LIMIT_DEG) {
                tangent_trim_deg =
                    H_COURSE_STRAIGHT_TANGENT_TRIM_LIMIT_DEG;
            } else if (tangent_trim_deg <
                       -H_COURSE_STRAIGHT_TANGENT_TRIM_LIMIT_DEG) {
                tangent_trim_deg =
                    -H_COURSE_STRAIGHT_TANGENT_TRIM_LIMIT_DEG;
            }
            course.straight_heading_target_deg = HCourse_NormalizeDegrees(
                course.straight_heading_anchor_deg + tangent_trim_deg);
        }
        /* Keep the planned heading for the whole fixed-course straight.
         * The capture count remains useful diagnostic evidence, but handing
         * ownership back to digital-mask PD here caused a 400 mm/s limit
         * cycle.  HCourse_EnterCurve() releases the reference at the known
         * next tangent instead. */
    }

    curve_evidence = line_usable &&
        HCourse_IsPositiveCurveEvidence(active_mask, line_position);
    curve_evidence_arm_mm = course.fast_straight_latched ?
        H_COURSE_FAST_CURVE_EVIDENCE_ARM_MM :
        H_COURSE_STRAIGHT_ARM_MM;
    if ((course.phase_distance_mm >= curve_evidence_arm_mm) &&
        curve_evidence) {
        if (course.curve_evidence_samples < 0xFFU)
            course.curve_evidence_samples++;
    } else {
        course.curve_evidence_samples = 0U;
    }
    force_distance_mm = H_COURSE_STRAIGHT_FORCE_MM;
    if (course.fast_straight_latched) {
        force_distance_mm = H_COURSE_FAST_STRAIGHT_FORCE_MM;
    }
    if ((course.curve_evidence_samples >=
         H_COURSE_CURVE_EVIDENCE_SAMPLES) ||
        (course.phase_distance_mm >= force_distance_mm)) {
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
