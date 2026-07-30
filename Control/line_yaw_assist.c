#include "line_yaw_assist.h"

#include "control_config.h"
#include "vehicle_config.h"

#include <stddef.h>
#include <string.h>

#define LINE_RAD_TO_DEG 57.2957795f

static LineYawAssistSnapshot_t assist;
static bool slew_initialized;
static bool straight_reference_initialized;
static bool straight_heading_mode;
static uint32_t straight_reacquire_since_ms;
static float straight_reference_yaw_deg;
static int8_t curve_direction;
static bool curve_confirmed;
static float curve_progress_deg;
static float curve_applied_correction_mm_s;
static uint32_t curve_exit_guard_started_ms;
static LineYawAssistMode_t requested_mode;
static LineYawAssistMode_t applied_mode;
static float requested_curve_feedforward_scale;
static bool requested_straight_reference_valid;
static float requested_straight_reference_yaw_deg;

static float LineYawAssist_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float LineYawAssist_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float LineYawAssist_MoveTowards(float current, float target,
                                       float maximum_delta)
{
    float delta = target - current;
    if (delta > maximum_delta) return current + maximum_delta;
    if (delta < -maximum_delta) return current - maximum_delta;
    return target;
}

static bool LineYawAssist_IsFinite(float value)
{
    return (value == value) &&
           (value <= 3.402823466e+38F) &&
           (value >= -3.402823466e+38F);
}

static float LineYawAssist_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees <= -180.0f) degrees += 360.0f;
    return degrees;
}

static bool LineYawAssist_IsCentralMask(uint8_t mask)
{
    return (mask == 0x08U) || (mask == 0x18U) || (mask == 0x10U);
}

static int8_t LineYawAssist_LineDirection(const LineFollowSnapshot_t *line)
{
    if (line->position > 0.01f) return 1;
    if (line->position < -0.01f) return -1;
    if ((line->active_mask & 0xE0U) != 0U) return 1;
    if ((line->active_mask & 0x07U) != 0U) return -1;
    return 0;
}

void LineYawAssist_Init(void)
{
    memset(&assist, 0, sizeof(assist));
    slew_initialized = false;
    straight_reference_initialized = false;
    straight_heading_mode = true;
    straight_reacquire_since_ms = 0U;
    straight_reference_yaw_deg = 0.0f;
    curve_direction = 0;
    curve_confirmed = false;
    curve_progress_deg = 0.0f;
    curve_applied_correction_mm_s = 0.0f;
    curve_exit_guard_started_ms = 0U;
    requested_mode = LINE_YAW_ASSIST_MODE_AUTO;
    applied_mode = LINE_YAW_ASSIST_MODE_AUTO;
    requested_curve_feedforward_scale = 1.0f;
    requested_straight_reference_valid = false;
    requested_straight_reference_yaw_deg = 0.0f;
}

void LineYawAssist_Reset(void)
{
    LineYawAssist_Init();
}

void LineYawAssist_SetMode(LineYawAssistMode_t mode)
{
    if ((mode < LINE_YAW_ASSIST_MODE_AUTO) ||
        (mode > LINE_YAW_ASSIST_MODE_CURVE_NEGATIVE)) {
        mode = LINE_YAW_ASSIST_MODE_AUTO;
    }
    requested_mode = mode;
}

void LineYawAssist_SetCurveFeedforwardScale(float scale)
{
    if (!LineYawAssist_IsFinite(scale)) scale = 1.0f;
    requested_curve_feedforward_scale = LineYawAssist_Clamp(
        scale, 0.5f, 1.2f);
}

void LineYawAssist_SetStraightReference(bool valid, float yaw_deg)
{
    requested_straight_reference_valid = valid &&
        LineYawAssist_IsFinite(yaw_deg);
    if (requested_straight_reference_valid) {
        requested_straight_reference_yaw_deg =
            LineYawAssist_NormalizeDegrees(yaw_deg);
    }
}

void LineYawAssist_Update(const LineFollowSnapshot_t *line,
                          float correction_limit_mm_s,
                          float measured_yaw_deg,
                          float measured_yaw_rate_deg_s,
                          float heading_gain_per_s,
                          float damping_gain_mm_s_per_deg_s,
                          bool gyro_usable, uint32_t now_ms)
{
    float target_assist;
    float desired_correction;
    float maximum_correction;
    float assist_limit;
    float owned_damping_gain;
    float combined;
    float curve_position_target;
    bool exit_guard_active;
    bool keep_owned_correction;
    bool fixed_curve_owned;
    bool fixed_exit_owned;
    bool line_measurement_usable;
    float curve_measured_position;
    float curve_measured_derivative;
    uint32_t dt_ms;
    uint32_t previous_update_ms = assist.last_update_ms;

    if (line == NULL) {
        LineYawAssist_Reset();
        return;
    }

    assist.active = false;
    assist.straight_heading_active = false;
    assist.curve_candidate_active = false;
    assist.curve_rate_active = false;
    assist.curve_direction = curve_direction;
    assist.last_update_ms = now_ms;
    assist.sensor_correction_mm_s = line->correction_mm_s;
    assist.straight_reference_yaw_deg = straight_reference_initialized ?
        straight_reference_yaw_deg : measured_yaw_deg;
    assist.straight_target_yaw_deg = measured_yaw_deg;
    assist.straight_heading_error_deg = 0.0f;
    assist.curve_progress_deg = curve_progress_deg;
    assist.curve_feedforward_correction_mm_s = 0.0f;
    assist.desired_yaw_rate_deg_s =
        (-2.0f * line->correction_mm_s / CHASSIS_DRIVE_TRACK_MM) *
        LINE_RAD_TO_DEG;
    assist.measured_yaw_rate_deg_s = measured_yaw_rate_deg_s;
    assist.rate_error_deg_s = 0.0f;
    assist.final_correction_mm_s = line->correction_mm_s;
    assist.left_target_mm_s = line->left_target_mm_s;
    assist.right_target_mm_s = line->right_target_mm_s;

    if (!gyro_usable || !LineYawAssist_IsFinite(measured_yaw_deg) ||
        !LineYawAssist_IsFinite(measured_yaw_rate_deg_s) ||
        !LineYawAssist_IsFinite(heading_gain_per_s) ||
        !LineYawAssist_IsFinite(damping_gain_mm_s_per_deg_s) ||
        (heading_gain_per_s < 0.0f) ||
        (damping_gain_mm_s_per_deg_s < 0.0f) ||
        (line->state != LINE_FOLLOW_TRACKING) ||
        (line->dynamic_base_mm_s < LINE_YAW_ASSIST_MIN_BASE_MM_S)) {
        assist.assist_correction_mm_s = 0.0f;
        slew_initialized = false;
        straight_reference_initialized = false;
        straight_heading_mode = true;
        straight_reacquire_since_ms = 0U;
        curve_direction = 0;
        curve_confirmed = false;
        curve_progress_deg = 0.0f;
        curve_exit_guard_started_ms = 0U;
        applied_mode = LINE_YAW_ASSIST_MODE_AUTO;
        return;
    }

    line_measurement_usable = line->line_valid && !line->ambiguous;
    fixed_curve_owned =
        (requested_mode == LINE_YAW_ASSIST_MODE_CURVE_POSITIVE) ||
        (requested_mode == LINE_YAW_ASSIST_MODE_CURVE_NEGATIVE);
    if (!line_measurement_usable && !fixed_curve_owned) {
        /* The digital conditioner deliberately bridges very short gaps and
         * ambiguous scans while remaining in TRACKING.  Pause gyro output
         * for that frame, but retain the confirmed curve/straight observer;
         * otherwise one untelemetered scan turns the middle of a semicircle
         * into a new straight and reverses the steering command. */
        assist.assist_correction_mm_s = 0.0f;
        slew_initialized = false;
        return;
    }

    assist.active = true;
    dt_ms = (previous_update_ms == 0U) ? 0U :
            (uint32_t)(now_ms - previous_update_ms);
    if (dt_ms > LINE_YAW_ASSIST_MAX_DT_MS)
        dt_ms = LINE_YAW_ASSIST_MAX_DT_MS;

    if (requested_mode != applied_mode) {
        keep_owned_correction =
            ((applied_mode == LINE_YAW_ASSIST_MODE_CURVE_POSITIVE) ||
             (applied_mode == LINE_YAW_ASSIST_MODE_CURVE_NEGATIVE)) &&
            (requested_mode == LINE_YAW_ASSIST_MODE_STRAIGHT) &&
            requested_straight_reference_valid;
        if (!keep_owned_correction) slew_initialized = false;
        straight_reference_initialized = false;
        straight_reacquire_since_ms = 0U;
        curve_exit_guard_started_ms = 0U;
        curve_progress_deg = 0.0f;
        if (requested_mode == LINE_YAW_ASSIST_MODE_STRAIGHT) {
            straight_heading_mode = true;
            curve_direction = 0;
            curve_confirmed = false;
        } else if ((requested_mode ==
                    LINE_YAW_ASSIST_MODE_CURVE_POSITIVE) ||
                   (requested_mode ==
                    LINE_YAW_ASSIST_MODE_CURVE_NEGATIVE)) {
            straight_heading_mode = false;
            curve_direction =
                (requested_mode == LINE_YAW_ASSIST_MODE_CURVE_POSITIVE) ?
                1 : -1;
            curve_confirmed = true;
        } else {
            straight_heading_mode = true;
            curve_direction = 0;
            curve_confirmed = false;
        }
        applied_mode = requested_mode;
    }

    exit_guard_active = (curve_exit_guard_started_ms != 0U) &&
        ((uint32_t)(now_ms - curve_exit_guard_started_ms) <
         LINE_CURVE_EXIT_STRAIGHT_GUARD_MS);
    if (!exit_guard_active) curve_exit_guard_started_ms = 0U;

    if ((requested_mode == LINE_YAW_ASSIST_MODE_AUTO) &&
        straight_heading_mode && !exit_guard_active &&
        (!LineYawAssist_IsCentralMask(line->active_mask) ||
         (LineYawAssist_Abs(line->position) >
          LINE_STRAIGHT_HEADING_POSITION_MAX))) {
        straight_heading_mode = false;
        straight_reference_initialized = false;
        straight_reacquire_since_ms = 0U;
        curve_direction = LineYawAssist_LineDirection(line);
        curve_confirmed = false;
        curve_progress_deg = 0.0f;
    }
    if (!straight_heading_mode) {
        float progress_delta = 0.0f;
        if ((curve_direction != 0) && (dt_ms > 0U)) {
            progress_delta = -(float)curve_direction *
                measured_yaw_rate_deg_s * (float)dt_ms * 0.001f;
            curve_progress_deg += progress_delta;
            if (curve_progress_deg < 0.0f) curve_progress_deg = 0.0f;
        }
        if ((requested_mode == LINE_YAW_ASSIST_MODE_AUTO) &&
            (curve_direction != 0) && !curve_confirmed &&
            (((float)curve_direction * line->position) <=
             -LINE_CURVE_EXIT_OPPOSITE_POSITION)) {
            straight_heading_mode = true;
            straight_reference_initialized = false;
            straight_reacquire_since_ms = 0U;
            curve_direction = 0;
            curve_progress_deg = 0.0f;
            curve_exit_guard_started_ms = now_ms;
        } else if ((requested_mode == LINE_YAW_ASSIST_MODE_AUTO) &&
                   (curve_direction != 0) && !curve_confirmed &&
                   (curve_progress_deg >=
                    LINE_CURVE_CONFIRM_TURN_DEG)) {
            curve_confirmed = true;
        }
        if ((requested_mode == LINE_YAW_ASSIST_MODE_AUTO) &&
            (curve_direction != 0) && curve_confirmed &&
            (((curve_progress_deg >= LINE_CURVE_EXIT_ARM_DEG) &&
              (LineYawAssist_IsCentralMask(line->active_mask) ||
               (((float)curve_direction * line->position) <=
                -LINE_CURVE_EXIT_OPPOSITE_POSITION))) ||
             (curve_progress_deg >= LINE_CURVE_FORCE_EXIT_DEG))) {
            straight_heading_mode = true;
            straight_reference_initialized = false;
            straight_reacquire_since_ms = 0U;
            curve_direction = 0;
            curve_confirmed = false;
            curve_exit_guard_started_ms = now_ms;
        }
    }
    if ((requested_mode == LINE_YAW_ASSIST_MODE_AUTO) &&
        !straight_heading_mode && !curve_confirmed) {
        if (LineYawAssist_IsCentralMask(line->active_mask) &&
            (LineYawAssist_Abs(line->position) <=
             LINE_STRAIGHT_HEADING_POSITION_MAX) &&
            (LineYawAssist_Abs(measured_yaw_rate_deg_s) <=
             LINE_STRAIGHT_HEADING_REACQUIRE_RATE_DEG_S)) {
            if (straight_reacquire_since_ms == 0U)
                straight_reacquire_since_ms = now_ms;
            else if ((uint32_t)(now_ms - straight_reacquire_since_ms) >=
                     LINE_STRAIGHT_HEADING_REACQUIRE_MS) {
                straight_heading_mode = true;
                straight_reference_initialized = false;
                straight_reacquire_since_ms = 0U;
                curve_direction = 0;
                curve_confirmed = false;
                curve_progress_deg = 0.0f;
            }
        } else {
            straight_reacquire_since_ms = 0U;
        }
    }

    desired_correction = line->correction_mm_s;
    if (straight_heading_mode) {
        if (requested_straight_reference_valid) {
            straight_reference_yaw_deg =
                requested_straight_reference_yaw_deg;
            straight_reference_initialized = true;
        } else if (!straight_reference_initialized) {
            straight_reference_yaw_deg = measured_yaw_deg;
            straight_reference_initialized = true;
        } else if (dt_ms > 0U) {
            straight_reference_yaw_deg = LineYawAssist_NormalizeDegrees(
                straight_reference_yaw_deg -
                LINE_STRAIGHT_HEADING_REFERENCE_ADAPT_DEG_S *
                line->position * (float)dt_ms * 0.001f);
        }
        assist.straight_reference_yaw_deg = straight_reference_yaw_deg;
        assist.straight_target_yaw_deg = LineYawAssist_NormalizeDegrees(
            straight_reference_yaw_deg -
            (requested_straight_reference_valid ?
             LINE_COURSE_EXIT_OFFSET_DEG_PER_POSITION :
             LINE_STRAIGHT_HEADING_OFFSET_DEG_PER_POSITION) *
            line->position);
        assist.straight_heading_error_deg = LineYawAssist_NormalizeDegrees(
            assist.straight_target_yaw_deg - measured_yaw_deg);
        assist.desired_yaw_rate_deg_s = LineYawAssist_Clamp(
            heading_gain_per_s * assist.straight_heading_error_deg,
            -LINE_STRAIGHT_HEADING_RATE_LIMIT_DEG_S,
             LINE_STRAIGHT_HEADING_RATE_LIMIT_DEG_S);
        desired_correction =
            (-assist.desired_yaw_rate_deg_s * CHASSIS_DRIVE_TRACK_MM) /
            (2.0f * LINE_RAD_TO_DEG);
        assist.straight_heading_active = true;
    } else if ((curve_direction != 0) && curve_confirmed) {
        curve_position_target =
            ((requested_mode == LINE_YAW_ASSIST_MODE_CURVE_POSITIVE) ||
             (requested_mode == LINE_YAW_ASSIST_MODE_CURVE_NEGATIVE)) ?
            ((float)curve_direction * LINE_CURVE_TARGET_POSITION) : 0.0f;
        curve_measured_position = line_measurement_usable ?
            line->position : curve_position_target;
        curve_measured_derivative = line_measurement_usable ?
            line->filtered_derivative : 0.0f;
        assist.curve_feedforward_correction_mm_s =
            (float)curve_direction * line->dynamic_base_mm_s *
            CHASSIS_DRIVE_TRACK_MM / (2.0f * LINE_CURVE_RADIUS_MM) *
            requested_curve_feedforward_scale;
        desired_correction = LineYawAssist_Clamp(
            assist.curve_feedforward_correction_mm_s +
            LINE_CURVE_POSITION_TRIM_MM_S *
            (curve_measured_position - curve_position_target) +
            LINE_CURVE_DERIVATIVE_TRIM_MM_S_PER_UNIT_S *
            curve_measured_derivative,
            -LINE_CURVE_DESIRED_CORRECTION_LIMIT_MM_S,
             LINE_CURVE_DESIRED_CORRECTION_LIMIT_MM_S);
        assist.desired_yaw_rate_deg_s =
            (-2.0f * desired_correction / CHASSIS_DRIVE_TRACK_MM) *
            LINE_RAD_TO_DEG;
        assist.curve_rate_active = true;
    }

    assist.curve_direction = curve_direction;
    assist.curve_progress_deg = curve_progress_deg;
    assist.curve_candidate_active =
        !straight_heading_mode && (curve_direction != 0) && !curve_confirmed;

    assist.rate_error_deg_s = measured_yaw_rate_deg_s -
                              assist.desired_yaw_rate_deg_s;
    fixed_exit_owned =
        (requested_mode == LINE_YAW_ASSIST_MODE_STRAIGHT) &&
        requested_straight_reference_valid;
    if (fixed_curve_owned || fixed_exit_owned) {
        /* On the fixed course the gyro owns the semicircle.  Do not add a
         * bounded correction to the straight-line PD output: a digital-mask
         * derivative step can otherwise move the final command by tens of
         * mm/s before the assist catches up.  The same continuous command is
         * retained briefly across curve exit so the planned outgoing heading
         * cannot be overwritten by a large line-edge derivative. */
        target_assist = desired_correction;
        owned_damping_gain = damping_gain_mm_s_per_deg_s;
        if (fixed_curve_owned) {
            owned_damping_gain *=
                LINE_FIXED_CURVE_DAMPING_MULTIPLIER;
        }
        if (LineYawAssist_Abs(assist.rate_error_deg_s) >
            LINE_YAW_ASSIST_RATE_DEADBAND_DEG_S) {
            target_assist += owned_damping_gain *
                             assist.rate_error_deg_s;
        }
        maximum_correction = correction_limit_mm_s;
        if (maximum_correction < 0.0f) maximum_correction = 0.0f;
        if ((line->dynamic_base_mm_s - LINE_YAW_ASSIST_MIN_INNER_MM_S) <
            maximum_correction) {
            maximum_correction = line->dynamic_base_mm_s -
                                 LINE_YAW_ASSIST_MIN_INNER_MM_S;
        }
        if (maximum_correction < 0.0f) maximum_correction = 0.0f;
        target_assist = LineYawAssist_Clamp(target_assist,
            -maximum_correction, maximum_correction);
        if (!slew_initialized) {
            curve_applied_correction_mm_s = LineYawAssist_Clamp(
                line->correction_mm_s, -maximum_correction,
                maximum_correction);
            slew_initialized = true;
        } else {
            curve_applied_correction_mm_s = LineYawAssist_MoveTowards(
                curve_applied_correction_mm_s, target_assist,
                LINE_YAW_ASSIST_SLEW_MM_S2 * (float)dt_ms * 0.001f);
        }
        combined = LineYawAssist_Clamp(curve_applied_correction_mm_s,
            -maximum_correction, maximum_correction);
        curve_applied_correction_mm_s = combined;
        assist.assist_correction_mm_s = combined -
                                        line->correction_mm_s;
        assist.final_correction_mm_s = combined;
        assist.left_target_mm_s = line->dynamic_base_mm_s + combined;
        assist.right_target_mm_s = line->dynamic_base_mm_s - combined;
        return;
    }
    target_assist = desired_correction - line->correction_mm_s;
    if (LineYawAssist_Abs(assist.rate_error_deg_s) <=
        LINE_YAW_ASSIST_RATE_DEADBAND_DEG_S) {
        /* Keep the heading-to-wheel conversion even inside the rate
         * deadband; only the damping term is suppressed. */
    } else {
        target_assist += damping_gain_mm_s_per_deg_s *
                         assist.rate_error_deg_s;
    }
    assist_limit = assist.curve_rate_active ?
        LINE_CURVE_YAW_ASSIST_LIMIT_MM_S : LINE_YAW_ASSIST_LIMIT_MM_S;
    target_assist = LineYawAssist_Clamp(target_assist,
        -assist_limit, assist_limit);

    if (!slew_initialized) {
        assist.assist_correction_mm_s = 0.0f;
        slew_initialized = true;
    } else {
        assist.assist_correction_mm_s = LineYawAssist_MoveTowards(
            assist.assist_correction_mm_s, target_assist,
            LINE_YAW_ASSIST_SLEW_MM_S2 * (float)dt_ms * 0.001f);
    }

    maximum_correction = correction_limit_mm_s;
    if (maximum_correction < 0.0f) maximum_correction = 0.0f;
    if ((line->dynamic_base_mm_s - LINE_YAW_ASSIST_MIN_INNER_MM_S) <
        maximum_correction) {
        maximum_correction = line->dynamic_base_mm_s -
                             LINE_YAW_ASSIST_MIN_INNER_MM_S;
    }
    if (maximum_correction < 0.0f) maximum_correction = 0.0f;
    combined = LineYawAssist_Clamp(
        line->correction_mm_s + assist.assist_correction_mm_s,
        -maximum_correction, maximum_correction);
    assist.assist_correction_mm_s = combined - line->correction_mm_s;
    assist.final_correction_mm_s = combined;
    assist.left_target_mm_s = line->dynamic_base_mm_s + combined;
    assist.right_target_mm_s = line->dynamic_base_mm_s - combined;
}

void LineYawAssist_GetSnapshot(LineYawAssistSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = assist;
}
