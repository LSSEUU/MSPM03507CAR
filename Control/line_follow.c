#include "line_follow.h"

#include "control_config.h"
#include "grayscale_config.h"

#include <stddef.h>
#include <string.h>

static const float line_channel_position[LINE_SENSOR_CHANNEL_COUNT] = {
    -1.0f, -0.7142857f, -0.4285714f, -0.1428571f,
     0.1428571f, 0.4285714f,  0.7142857f,  1.0f
};

static const uint16_t configured_white_adc[LINE_SENSOR_CHANNEL_COUNT] =
    GRAYSCALE_WHITE_ADC_VALUES;
static const uint16_t configured_black_adc[LINE_SENSOR_CHANNEL_COUNT] =
    GRAYSCALE_BLACK_ADC_VALUES;

static LineFollowSnapshot_t line;
static LineFollowConfig_t active_config;
static uint8_t valid_samples;
static uint8_t invalid_samples;
static float previous_position;
static uint32_t previous_control_ms;
static float last_valid_position;
static bool position_initialized;
static bool derivative_initialized;
static bool derivative_reset_pending;
static bool previous_raw_valid;
static uint8_t branch_release_samples;
static int8_t ambiguity_direction_candidate;
static uint8_t ambiguity_direction_samples;
static int8_t position_direction_candidate;
static uint8_t position_direction_samples;
static bool digital_input;
static int8_t digital_direction_candidate;
static uint8_t digital_direction_samples;
static float last_tracking_base_mm_s;
static bool loss_timing_active;

static void LineFollow_ResetLockState(void)
{
    memset(line.level, 0, sizeof(line.level));
    memset(line.strength, 0, sizeof(line.strength));
    line.line_valid = false;
    line.ambiguous = false;
    line.branch_direction_latched = false;
    line.search_direction = 0;
    line.active_mask = 0U;
    line.active_count = 0U;
    line.cluster_count = 0U;
    line.raw_position = 0.0f;
    line.position = 0.0f;
    line.raw_derivative = 0.0f;
    line.filtered_derivative = 0.0f;
    line.confidence = 0.0f;
    valid_samples = 0U;
    invalid_samples = 0U;
    previous_position = 0.0f;
    previous_control_ms = 0U;
    last_valid_position = 0.0f;
    position_initialized = false;
    derivative_initialized = false;
    derivative_reset_pending = true;
    previous_raw_valid = false;
    branch_release_samples = 0U;
    ambiguity_direction_candidate = 0;
    ambiguity_direction_samples = 0U;
    position_direction_candidate = 0;
    position_direction_samples = 0U;
    digital_input = false;
    digital_direction_candidate = 0;
    digital_direction_samples = 0U;
    last_tracking_base_mm_s = 0.0f;
    loss_timing_active = false;
}

static float LineFollow_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float LineFollow_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float LineFollow_NormalizedStrength(uint16_t adc, uint16_t white,
                                           uint16_t black)
{
    float span = (float)((uint32_t)white - (uint32_t)black);
    float blackness = ((float)white - (float)adc) / span;
    blackness = LineFollow_Clamp(blackness, 0.0f, 1.0f);
    if (blackness <= LINE_ANALOG_STRENGTH_FLOOR) return 0.0f;
    return (blackness - LINE_ANALOG_STRENGTH_FLOOR) /
           (1.0f - LINE_ANALOG_STRENGTH_FLOOR);
}

static float LineFollow_MoveTowards(float current, float target,
                                    float maximum_delta)
{
    float delta = target - current;
    if (delta > maximum_delta) return current + maximum_delta;
    if (delta < -maximum_delta) return current - maximum_delta;
    return target;
}

static bool LineFollow_IsFinite(float value)
{
    return (value == value) &&
           (value <= 3.402823466e+38F) &&
           (value >= -3.402823466e+38F);
}

static float LineFollow_ProportionalCorrection(float position)
{
    float magnitude = LineFollow_Abs(position);
    float gain = active_config.kp;
    float blend;

    if (magnitude <= LINE_CURVE_KP_BOOST_START_POSITION)
        return gain * position;

    blend = LineFollow_Clamp(
        (magnitude - LINE_CURVE_KP_BOOST_START_POSITION) /
        (LINE_CURVE_KP_BOOST_FULL_POSITION -
         LINE_CURVE_KP_BOOST_START_POSITION),
        0.0f, 1.0f);
    blend = blend * blend * (3.0f - (2.0f * blend));
    gain *= 1.0f + (LINE_CURVE_KP_BOOST_RATIO * blend);
    return gain * position;
}

void LineFollow_GetDefaultConfig(LineFollowConfig_t *config)
{
    if (config == NULL) return;
    config->speed_min_mm_s = LINE_FOLLOW_BASE_SPEED_MM_S;
    config->speed_max_mm_s = LINE_FOLLOW_BASE_SPEED_MM_S;
    config->position_alpha = 1.0f;
    config->derivative_alpha = 1.0f;
    config->kp = LINE_FOLLOW_KP_MM_S;
    config->kd = LINE_FOLLOW_KD_MM_S_PER_UNIT_S;
    config->correction_limit_mm_s = LINE_FOLLOW_MAX_CORRECTION_MM_S;
    config->correction_slew_mm_s2 = 2000.0f;
    config->curve_gain = 0.0f;
    config->lost_search_speed_mm_s = LINE_SEARCH_TURN_MM_S;
}

void LineFollow_GetConfig(LineFollowConfig_t *config)
{
    if (config != NULL) *config = active_config;
}

bool LineFollow_ApplyConfig(const LineFollowConfig_t *config)
{
    if ((config == NULL) || (line.state != LINE_FOLLOW_IDLE) ||
        !LineFollow_IsFinite(config->speed_min_mm_s) ||
        !LineFollow_IsFinite(config->speed_max_mm_s) ||
        !LineFollow_IsFinite(config->position_alpha) ||
        !LineFollow_IsFinite(config->derivative_alpha) ||
        !LineFollow_IsFinite(config->kp) ||
        !LineFollow_IsFinite(config->kd) ||
        !LineFollow_IsFinite(config->correction_limit_mm_s) ||
        !LineFollow_IsFinite(config->correction_slew_mm_s2) ||
        !LineFollow_IsFinite(config->curve_gain) ||
        !LineFollow_IsFinite(config->lost_search_speed_mm_s) ||
        (config->speed_min_mm_s < 50.0f) ||
        (config->speed_max_mm_s > 400.0f) ||
        (config->speed_min_mm_s > config->speed_max_mm_s) ||
        (config->position_alpha < 0.05f) ||
        (config->position_alpha > 1.0f) ||
        (config->derivative_alpha < 0.05f) ||
        (config->derivative_alpha > 1.0f) ||
        (config->kp < 0.0f) || (config->kp > 500.0f) ||
        (config->kd < 0.0f) || (config->kd > 100.0f) ||
        (config->correction_limit_mm_s < 0.0f) ||
        (config->correction_limit_mm_s > 250.0f) ||
        (config->correction_slew_mm_s2 < 10.0f) ||
        (config->correction_slew_mm_s2 > 2000.0f) ||
        (config->curve_gain < 0.0f) || (config->curve_gain > 2.0f) ||
        (config->lost_search_speed_mm_s < 50.0f) ||
        (config->lost_search_speed_mm_s > 400.0f)) return false;
    active_config = *config;
    return true;
}

static void LineFollow_InitCalibration(void)
{
    uint32_t i;
    uint8_t valid_mask = 0U;

    line.calibration.compiled_verified =
        (GRAYSCALE_CALIBRATION_VERIFIED != 0U);
    line.calibration.source = LINE_CALIBRATION_SOURCE_COMPILED;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint16_t white = configured_white_adc[i];
        uint16_t black = configured_black_adc[i];
        line.calibration.white_adc[i] = white;
        line.calibration.black_adc[i] = black;
        line.calibration.enter_black_adc[i] = (uint16_t)(
            ((2U * (uint32_t)black) + (uint32_t)white) / 3U);
        line.calibration.exit_black_adc[i] = (uint16_t)(
            ((uint32_t)black + (2U * (uint32_t)white)) / 3U);
        if ((white > black) &&
            ((uint16_t)(white - black) >=
             GRAYSCALE_CALIBRATION_MIN_SPAN_ADC)) {
            valid_mask |= (uint8_t)(1U << i);
        }
    }
    line.calibration.valid_mask = valid_mask;
    line.calibration.valid = line.calibration.compiled_verified &&
        (valid_mask == 0xFFU);
}

bool LineFollow_ValidateCalibration(const LineCalibrationValues_t *values,
                                    uint8_t *valid_mask)
{
    uint8_t mask = 0U;
    uint32_t i;
    if (values == NULL) return false;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint16_t white = values->white_adc[i];
        uint16_t black = values->black_adc[i];
        if ((white > black) &&
            ((uint16_t)(white - black) >=
             GRAYSCALE_CALIBRATION_MIN_SPAN_ADC))
            mask |= (uint8_t)(1U << i);
    }
    if (valid_mask != NULL) *valid_mask = mask;
    return mask == 0xFFU;
}

bool LineFollow_ApplyCalibration(const LineCalibrationValues_t *values,
                                 LineCalibrationSource_t source)
{
    uint8_t mask;
    uint32_t i;
    if ((line.state != LINE_FOLLOW_IDLE) ||
        !LineFollow_ValidateCalibration(values, &mask)) return false;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint16_t white = values->white_adc[i];
        uint16_t black = values->black_adc[i];
        line.calibration.white_adc[i] = white;
        line.calibration.black_adc[i] = black;
        line.calibration.enter_black_adc[i] = (uint16_t)(
            ((2U * (uint32_t)black) + (uint32_t)white) / 3U);
        line.calibration.exit_black_adc[i] = (uint16_t)(
            ((uint32_t)black + (2U * (uint32_t)white)) / 3U);
    }
    line.calibration.valid_mask = mask;
    line.calibration.source = source;
    line.calibration.valid = true;
    LineFollow_ResetLockState();
    return true;
}

void LineFollow_GetCalibrationValues(LineCalibrationValues_t *values,
                                     LineCalibrationSource_t *source)
{
    if (values != NULL) {
        memcpy(values->white_adc, line.calibration.white_adc,
               sizeof(values->white_adc));
        memcpy(values->black_adc, line.calibration.black_adc,
               sizeof(values->black_adc));
    }
    if (source != NULL) *source = line.calibration.source;
}

void LineFollow_Init(void)
{
    memset(&line, 0, sizeof(line));
    LineFollow_GetDefaultConfig(&active_config);
    line.polarity = LINE_POLARITY_ACTIVE_HIGH;
    line.state = LINE_FOLLOW_IDLE;
    LineFollow_InitCalibration();
    LineFollow_ResetLockState();
}

static uint8_t LineFollow_CountClusters(uint8_t active_mask)
{
    uint8_t clusters = 0U;
    uint8_t previous = 0U;
    uint8_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint8_t active = (uint8_t)((active_mask >> i) & 1U);
        if ((active != 0U) && (previous == 0U)) clusters++;
        previous = active;
    }
    return clusters;
}

static uint8_t LineFollow_CountActive(uint8_t active_mask)
{
    uint8_t count = 0U;
    uint8_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (((active_mask >> i) & 1U) != 0U) count++;
    }
    return count;
}

static float LineFollow_DigitalPosition(uint8_t active_mask,
                                        uint8_t active_count)
{
    float sum = 0.0f;
    uint8_t i;
    if (active_count == 0U) return 0.0f;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (((active_mask >> i) & 1U) != 0U)
            sum += line_channel_position[i];
    }
    sum /= (float)active_count;
    return sum;
}

static void LineFollow_ClearInvalidObservation(void)
{
    memset(line.level, 0, sizeof(line.level));
    memset(line.strength, 0, sizeof(line.strength));
    line.active_mask = 0U;
    line.active_count = 0U;
    line.cluster_count = 0U;
    line.ambiguous = false;
    line.line_valid = false;
    line.confidence = 0.0f;
    valid_samples = 0U;
    invalid_samples = 0U;
}

static int8_t LineFollow_AmbiguousDirection(uint8_t active_mask)
{
    float left_extent = 0.0f;
    float right_extent = 0.0f;
    uint8_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        float position;
        if (((active_mask >> i) & 1U) == 0U) continue;
        position = line_channel_position[i];
        if ((position < 0.0f) && (-position > left_extent))
            left_extent = -position;
        else if ((position > 0.0f) && (position > right_extent))
            right_extent = position;
    }
    if (left_extent > right_extent) return -1;
    if (right_extent > left_extent) return 1;
    return 0;
}

void LineFollow_ProcessSample(const uint16_t adc[LINE_SENSOR_CHANNEL_COUNT],
                              uint32_t scan_id, uint32_t now_ms)
{
    uint32_t i;
    float weighted_sum = 0.0f;
    float position_weight_sum = 0.0f;
    float raw_confidence;
    bool raw_valid;
    uint8_t active_mask = 0U;
    uint8_t position_mask;
    if ((adc == NULL) || (scan_id == line.scan_id)) return;
    digital_input = false;
    line.scan_id = scan_id;
    line.last_update_ms = now_ms;
    line.update_count++;

    if (!line.calibration.valid) {
        LineFollow_ClearInvalidObservation();
        return;
    }

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (line.level[i] == 0U) {
            if (adc[i] <= line.calibration.enter_black_adc[i])
                line.level[i] = 1U;
        } else if (adc[i] >= line.calibration.exit_black_adc[i]) {
            line.level[i] = 0U;
        }
        if (line.level[i] != 0U) active_mask |= (uint8_t)(1U << i);
    }
    line.active_mask = active_mask;
    line.active_count = 0U;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (((active_mask >> i) & 1U) != 0U) line.active_count++;
    }

    /* Preserve the hysteretic binary mask for validity/topology, while the
     * steering centroid uses calibrated analog blackness. Only the active
     * cluster and its immediate neighbours may influence position, so a
     * distant weak reflection cannot pull the vehicle toward track exterior. */
    position_mask = (uint8_t)(active_mask |
        (uint8_t)(active_mask << 1U) | (active_mask >> 1U));
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        float strength = LineFollow_NormalizedStrength(
            adc[i], line.calibration.white_adc[i],
            line.calibration.black_adc[i]);
        line.strength[i] = strength;
        if (((position_mask >> i) & 1U) != 0U) {
            position_weight_sum += strength;
            weighted_sum += strength * line_channel_position[i];
        }
    }
    line.cluster_count = LineFollow_CountClusters(active_mask);
    line.ambiguous = (line.active_count == LINE_SENSOR_CHANNEL_COUNT) ||
                     (line.active_count > 4U) ||
                     (line.cluster_count > 1U);
    if (line.ambiguous && !line.branch_direction_latched) {
        int8_t direction = LineFollow_AmbiguousDirection(active_mask);
        line.ambiguity_count++;
        if (direction == 0) {
            ambiguity_direction_candidate = 0;
            ambiguity_direction_samples = 0U;
        } else if (direction == ambiguity_direction_candidate) {
            if (ambiguity_direction_samples < LINE_BRANCH_HINT_SAMPLES)
                ambiguity_direction_samples++;
        } else {
            ambiguity_direction_candidate = direction;
            ambiguity_direction_samples = 1U;
        }
        if (ambiguity_direction_samples >= LINE_BRANCH_HINT_SAMPLES) {
            line.search_direction = direction;
            line.branch_direction_latched = true;
            branch_release_samples = 0U;
        }
    } else {
        ambiguity_direction_candidate = 0;
        ambiguity_direction_samples = 0U;
    }
    raw_confidence = LineFollow_Clamp(
        (float)line.active_count / LINE_EXPECTED_ACTIVE_SUM, 0.0f, 1.0f);
    raw_valid = (raw_confidence >= LINE_VALID_CONFIDENCE) &&
                !line.ambiguous && (line.active_count != 0U) &&
                (position_weight_sum > 0.0f);
    if (raw_valid) {
        float raw_position = weighted_sum / position_weight_sum;
        int8_t direction = 0;
        if (!previous_raw_valid || !position_initialized) {
            line.position = raw_position;
            position_initialized = true;
            derivative_reset_pending = true;
        } else {
            line.position += active_config.position_alpha *
                (raw_position - line.position);
        }
        line.raw_position = raw_position;
        if (raw_position < -0.05f) direction = -1;
        else if (raw_position > 0.05f) direction = 1;
        if (line.branch_direction_latched) {
            position_direction_candidate = 0;
            position_direction_samples = 0U;
            if ((line.active_count >= LINE_BRANCH_RELEASE_MIN_ACTIVE) &&
                (LineFollow_Abs(raw_position) <=
                 LINE_BRANCH_RELEASE_POSITION)) {
                if (branch_release_samples < LINE_BRANCH_RELEASE_SAMPLES)
                    branch_release_samples++;
                if (branch_release_samples >= LINE_BRANCH_RELEASE_SAMPLES) {
                    line.branch_direction_latched = false;
                    branch_release_samples = 0U;
                    line.search_direction = 0;
                }
            } else {
                branch_release_samples = 0U;
            }
        } else {
            if (direction == 0) {
                position_direction_candidate = 0;
                position_direction_samples = 0U;
            } else if (direction == position_direction_candidate) {
                if (position_direction_samples < LINE_BRANCH_HINT_SAMPLES)
                    position_direction_samples++;
            } else {
                position_direction_candidate = direction;
                position_direction_samples = 1U;
            }
            if (position_direction_samples >= LINE_BRANCH_HINT_SAMPLES)
                line.search_direction = direction;
        }
        if (valid_samples < LINE_VALID_ACQUIRE_SAMPLES) valid_samples++;
        invalid_samples = 0U;
        line.confidence = raw_confidence;
        if (line.line_valid ||
            (valid_samples >= LINE_VALID_ACQUIRE_SAMPLES)) {
            line.line_valid = true;
            last_valid_position = line.position;
        }
    } else {
        branch_release_samples = 0U;
        position_direction_candidate = 0;
        position_direction_samples = 0U;
        if (invalid_samples < LINE_VALID_RELEASE_SAMPLES) invalid_samples++;
        valid_samples = 0U;
        line.confidence = raw_confidence;
        if (!line.line_valid ||
            (invalid_samples >= LINE_VALID_RELEASE_SAMPLES))
            line.line_valid = false;
    }
    previous_raw_valid = raw_valid;
}

void LineFollow_ProcessDigitalMask(uint8_t active_mask,
                                   uint32_t scan_id, uint32_t now_ms)
{
    uint8_t active_count;
    bool raw_valid;
    bool was_raw_valid;
    uint8_t i;

    if (scan_id == line.scan_id) return;
    digital_input = true;
    line.scan_id = scan_id;
    line.last_update_ms = now_ms;
    line.update_count++;

    if (!line.calibration.valid) {
        LineFollow_ClearInvalidObservation();
        return;
    }

    active_count = LineFollow_CountActive(active_mask);
    line.active_mask = active_mask;
    line.active_count = active_count;
    line.cluster_count = LineFollow_CountClusters(active_mask);
    line.ambiguous = (active_count == LINE_SENSOR_CHANNEL_COUNT) ||
                     (active_count > 4U) ||
                     (line.cluster_count > 1U);
    line.branch_direction_latched = false;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint8_t active = (uint8_t)((active_mask >> i) & 1U);
        line.level[i] = active;
        line.strength[i] = (float)active;
    }
    line.confidence = LineFollow_Clamp(
        (float)active_count / LINE_EXPECTED_ACTIVE_SUM, 0.0f, 1.0f);
    raw_valid = (active_count != 0U) && !line.ambiguous;
    was_raw_valid = previous_raw_valid;

    if (raw_valid) {
        float geometric_position = LineFollow_DigitalPosition(active_mask,
                                                              active_count);
        float raw_position;
        int8_t direction = 0;
        bool single_center = false;
        uint8_t direction_samples_required =
            LINE_DIGITAL_DIRECTION_SAMPLES;

        if (geometric_position < -LINE_DIGITAL_CENTER_DEADBAND)
            direction = -1;
        else if (geometric_position > LINE_DIGITAL_CENTER_DEADBAND)
            direction = 1;
        else if (active_mask == 0x08U) {
            direction = -1;
            single_center = true;
            direction_samples_required = LINE_DIGITAL_CENTER_HINT_SAMPLES;
        } else if (active_mask == 0x10U) {
            direction = 1;
            single_center = true;
            direction_samples_required = LINE_DIGITAL_CENTER_HINT_SAMPLES;
        }
        if (direction == 0) {
            digital_direction_candidate = 0;
            digital_direction_samples = 0U;
        } else if (direction == digital_direction_candidate) {
            if (digital_direction_samples < LINE_DIGITAL_CENTER_HINT_SAMPLES)
                digital_direction_samples++;
        } else {
            digital_direction_candidate = direction;
            digital_direction_samples = 1U;
        }
        /* Ignore brief 0x08/0x10 chatter, but do not leave a persistent
         * one-channel centre error uncorrected.  The former prevents sensor
         * quantisation jitter; the latter starts a small correction before
         * the line reaches 0x04/0x20 and avoids late, alternating saturation. */
        raw_position = geometric_position;
        if (single_center &&
            (digital_direction_samples <
             LINE_DIGITAL_CENTER_CONTROL_SAMPLES)) {
            raw_position = 0.0f;
        } else if (single_center) {
            /* One centre bit represents only the quantisation boundary of
             * the 18 mm line between two probes, not a full 1/7 lateral
             * displacement.  Keep its geometric sign for recovery above,
             * but use a gentler steering hint to avoid a slow 0x10/0x08
             * limit cycle on otherwise straight track. */
            raw_position *= LINE_DIGITAL_SINGLE_CENTER_POSITION_SCALE;
        }
        line.raw_position = raw_position;
        if (!position_initialized) {
            line.position = raw_position;
            position_initialized = true;
        } else {
            line.position += active_config.position_alpha *
                             (raw_position - line.position);
        }
        if (!was_raw_valid) derivative_reset_pending = true;

        /* Five stable centre samples are still required before that small
         * positional hint may select the forward-arc recovery direction. */
        if (digital_direction_samples >= direction_samples_required)
            line.search_direction = direction;

        if (valid_samples < LINE_DIGITAL_REACQUIRE_SAMPLES)
            valid_samples++;
        invalid_samples = 0U;
        if (valid_samples >= LINE_DIGITAL_REACQUIRE_SAMPLES) {
            line.line_valid = true;
            last_valid_position = line.position;
        } else {
            line.line_valid = false;
        }
    } else {
        line.line_valid = false;
        valid_samples = 0U;
        if (invalid_samples < 0xFFU) invalid_samples++;
        digital_direction_candidate = 0;
        digital_direction_samples = 0U;
        derivative_reset_pending = true;
        if (line.ambiguous) line.ambiguity_count++;
    }
    previous_raw_valid = raw_valid;
}

bool LineFollow_Start(uint32_t now_ms)
{
    if ((line.state != LINE_FOLLOW_IDLE) || !line.calibration.valid ||
        !line.line_valid ||
        line.ambiguous) return false;
    line.state = LINE_FOLLOW_TRACKING;
    line.lost_since_ms = 0U;
    loss_timing_active = false;
    line.correction_mm_s = 0.0f;
    line.raw_correction_mm_s = 0.0f;
    line.raw_derivative = 0.0f;
    line.filtered_derivative = 0.0f;
    line.dynamic_base_mm_s = active_config.speed_max_mm_s;
    previous_position = line.position;
    previous_control_ms = now_ms;
    last_valid_position = line.position;
    derivative_initialized = false;
    derivative_reset_pending = true;
    line.branch_direction_latched = false;
    branch_release_samples = 0U;
    digital_direction_candidate = 0;
    digital_direction_samples = 0U;
    last_tracking_base_mm_s = active_config.speed_max_mm_s;
    return true;
}

static void LineFollow_ApplyDriveTargets(float base_speed, float correction)
{
    float minimum_inner = 50.0f;
    float maximum_correction;
    base_speed = LineFollow_Clamp(base_speed, 0.0f, 400.0f);
    if (minimum_inner > base_speed) minimum_inner = base_speed;
    maximum_correction = base_speed - minimum_inner;
    correction = LineFollow_Clamp(correction, -maximum_correction,
                                  maximum_correction);
    line.correction_mm_s = correction;
    line.left_target_mm_s = base_speed + correction;
    line.right_target_mm_s = base_speed - correction;
}

static float LineFollow_CorrectionSlew(void)
{
    float slew = active_config.correction_slew_mm_s2;
    if (digital_input && (slew > LINE_DIGITAL_MAX_CORRECTION_SLEW))
        slew = LINE_DIGITAL_MAX_CORRECTION_SLEW;
    return slew;
}

static void LineFollow_ApproachCorrection(float target, float dt_s)
{
    target = LineFollow_Clamp(target,
        -active_config.correction_limit_mm_s,
         active_config.correction_limit_mm_s);
    line.raw_correction_mm_s = target;
    if (dt_s > 0.0f) {
        line.correction_mm_s = LineFollow_MoveTowards(
            line.correction_mm_s, target,
            LineFollow_CorrectionSlew() * dt_s);
    }
}

static void LineFollow_EnterLostTimeout(void)
{
    line.state = LINE_FOLLOW_LOST_TIMEOUT;
    line.dynamic_base_mm_s = 0.0f;
    line.raw_correction_mm_s = 0.0f;
    line.correction_mm_s = 0.0f;
    line.left_target_mm_s = 0.0f;
    line.right_target_mm_s = 0.0f;
}

void LineFollow_Update(uint32_t now_ms)
{
    float dt_s = 0.0f;
    float severity;
    float reduction;
    float correction;
    if ((line.state == LINE_FOLLOW_IDLE) ||
        (line.state == LINE_FOLLOW_LOST_TIMEOUT)) return;
    if (previous_control_ms != now_ms)
        dt_s = (float)(uint32_t)(now_ms - previous_control_ms) * 0.001f;

    if (line.line_valid) {
        if (line.ambiguous) {
            /* One ambiguous scan keeps the prior steering sign at low speed. */
            line.state = LINE_FOLLOW_TRACKING;
            line.dynamic_base_mm_s = active_config.speed_min_mm_s;
            correction = line.correction_mm_s;
            if ((correction == 0.0f) && (last_valid_position != 0.0f))
                correction = (last_valid_position < 0.0f) ? -1.0f : 1.0f;
            LineFollow_ApplyDriveTargets(line.dynamic_base_mm_s, correction);
            previous_control_ms = now_ms;
            return;
        }
        if ((line.state != LINE_FOLLOW_TRACKING) ||
            derivative_reset_pending || !derivative_initialized ||
            (dt_s <= 0.0f)) {
            line.raw_derivative = 0.0f;
            line.filtered_derivative = 0.0f;
            derivative_initialized = true;
            derivative_reset_pending = false;
        } else {
            line.raw_derivative = (line.position - previous_position) / dt_s;
            if (digital_input) {
                line.raw_derivative = LineFollow_Clamp(
                    line.raw_derivative,
                    -LINE_DIGITAL_DERIVATIVE_LIMIT,
                     LINE_DIGITAL_DERIVATIVE_LIMIT);
            }
            line.filtered_derivative += active_config.derivative_alpha *
                (line.raw_derivative - line.filtered_derivative);
        }
        correction =
            LineFollow_ProportionalCorrection(line.position) +
            (active_config.kd * line.filtered_derivative);
        LineFollow_ApproachCorrection(correction, dt_s);
        severity = LineFollow_Abs(line.position);
        if ((LineFollow_Abs(line.filtered_derivative) * 0.05f) > severity)
            severity = LineFollow_Abs(line.filtered_derivative) * 0.05f;
        if ((active_config.correction_limit_mm_s > 0.0f) &&
            ((LineFollow_Abs(line.raw_correction_mm_s) /
              active_config.correction_limit_mm_s) > severity)) {
            severity = LineFollow_Abs(line.raw_correction_mm_s) /
                       active_config.correction_limit_mm_s;
        }
        reduction = LineFollow_Clamp(active_config.curve_gain * severity,
                                     0.0f, 1.0f);
        line.dynamic_base_mm_s = active_config.speed_max_mm_s -
            ((active_config.speed_max_mm_s - active_config.speed_min_mm_s) *
             reduction);
        line.state = LINE_FOLLOW_TRACKING;
        line.lost_since_ms = 0U;
        loss_timing_active = false;
        LineFollow_ApplyDriveTargets(line.dynamic_base_mm_s,
                                     line.correction_mm_s);
        last_tracking_base_mm_s = line.dynamic_base_mm_s;
        previous_position = line.position;
        previous_control_ms = now_ms;
        last_valid_position = line.position;
        return;
    }

    if (!loss_timing_active) {
        line.lost_since_ms = now_ms;
        loss_timing_active = true;
        derivative_reset_pending = true;
    }
    if ((uint32_t)(now_ms - line.lost_since_ms) >= LINE_LOST_TIMEOUT_MS) {
        LineFollow_EnterLostTimeout();
        return;
    }

    line.dynamic_base_mm_s = LineFollow_Clamp(
        active_config.lost_search_speed_mm_s, 50.0f, 400.0f);
    if ((last_tracking_base_mm_s > 0.0f) &&
        (((uint32_t)(now_ms - line.lost_since_ms) <
           LINE_DIGITAL_LOSS_GRACE_MS) ||
         (digital_input && line.ambiguous &&
          ((uint32_t)(now_ms - line.lost_since_ms) <
           LINE_DIGITAL_AMBIGUOUS_BRIDGE_MS)))) {
        if (last_tracking_base_mm_s < line.dynamic_base_mm_s)
            line.dynamic_base_mm_s = last_tracking_base_mm_s;
    }

    if (digital_input && line.ambiguous &&
        ((uint32_t)(now_ms - line.lost_since_ms) <
         LINE_DIGITAL_AMBIGUOUS_BRIDGE_MS)) {
        line.state = LINE_FOLLOW_TRACKING;
        correction = 0.0f;
    } else if (digital_input &&
               ((uint32_t)(now_ms - line.lost_since_ms) <
                LINE_DIGITAL_LOSS_GRACE_MS)) {
        line.state = LINE_FOLLOW_TRACKING;
        correction = 0.0f;
    } else {
        int8_t search_direction = line.search_direction;
        line.state = LINE_FOLLOW_SEARCHING;
        /* Retain the legacy ADC fallback only for the analog API. The digital
         * module must never invent a turn direction without stable evidence. */
        if (!digital_input && (search_direction == 0))
            search_direction = (last_valid_position < 0.0f) ? -1 : 1;
        correction = line.dynamic_base_mm_s *
                     LINE_DIGITAL_SEARCH_CORRECTION_RATIO *
                     (float)search_direction;
    }
    LineFollow_ApproachCorrection(correction, dt_s);
    LineFollow_ApplyDriveTargets(line.dynamic_base_mm_s,
                                 line.correction_mm_s);
    previous_control_ms = now_ms;
}

void LineFollow_Stop(void)
{
    line.state = LINE_FOLLOW_IDLE;
    line.lost_since_ms = 0U;
    loss_timing_active = false;
    line.dynamic_base_mm_s = 0.0f;
    line.raw_correction_mm_s = 0.0f;
    line.correction_mm_s = 0.0f;
    line.left_target_mm_s = 0.0f;
    line.right_target_mm_s = 0.0f;
    derivative_initialized = false;
    derivative_reset_pending = true;
    line.search_direction = 0;
    line.branch_direction_latched = false;
    branch_release_samples = 0U;
    ambiguity_direction_candidate = 0;
    ambiguity_direction_samples = 0U;
    position_direction_candidate = 0;
    position_direction_samples = 0U;
}

void LineFollow_GetSnapshot(LineFollowSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = line;
}
