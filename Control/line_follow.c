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

void LineFollow_Init(void)
{
    memset(&line, 0, sizeof(line));
    LineFollow_GetDefaultConfig(&active_config);
    line.polarity = LINE_POLARITY_ACTIVE_HIGH;
    line.state = LINE_FOLLOW_IDLE;
    LineFollow_InitCalibration();
    valid_samples = 0U;
    invalid_samples = 0U;
    previous_position = 0.0f;
    previous_control_ms = 0U;
    last_valid_position = 0.0f;
    position_initialized = false;
    derivative_initialized = false;
    derivative_reset_pending = true;
    previous_raw_valid = false;
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

void LineFollow_ProcessSample(const uint16_t adc[LINE_SENSOR_CHANNEL_COUNT],
                              uint32_t scan_id, uint32_t now_ms)
{
    uint32_t i;
    float weighted_sum = 0.0f;
    float active_sum = 0.0f;
    float raw_confidence;
    bool raw_valid;
    uint8_t active_mask = 0U;
    if ((adc == NULL) || (scan_id == line.scan_id)) return;
    line.scan_id = scan_id;
    line.last_update_ms = now_ms;
    line.update_count++;

    if (!line.calibration.valid) {
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
        return;
    }

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        float strength;
        if (line.level[i] == 0U) {
            if (adc[i] <= line.calibration.enter_black_adc[i])
                line.level[i] = 1U;
        } else if (adc[i] >= line.calibration.exit_black_adc[i]) {
            line.level[i] = 0U;
        }
        strength = (line.level[i] != 0U) ? 1.0f : 0.0f;
        line.strength[i] = strength;
        if (strength > 0.5f) active_mask |= (uint8_t)(1U << i);
        active_sum += strength;
        weighted_sum += strength * line_channel_position[i];
    }
    line.active_mask = active_mask;
    line.active_count = (uint8_t)active_sum;
    line.cluster_count = LineFollow_CountClusters(active_mask);
    line.ambiguous = (line.active_count == LINE_SENSOR_CHANNEL_COUNT) ||
                     (line.active_count > 4U) ||
                     (line.cluster_count > 1U);
    if (line.ambiguous) line.ambiguity_count++;
    raw_confidence = LineFollow_Clamp(
        active_sum / LINE_EXPECTED_ACTIVE_SUM, 0.0f, 1.0f);
    raw_valid = (raw_confidence >= LINE_VALID_CONFIDENCE) &&
                !line.ambiguous && (line.active_count != 0U);
    if (raw_valid) {
        float raw_position = weighted_sum / active_sum;
        if (!previous_raw_valid || !position_initialized) {
            line.position = raw_position;
            position_initialized = true;
            derivative_reset_pending = true;
        } else {
            line.position += active_config.position_alpha *
                (raw_position - line.position);
        }
        line.raw_position = raw_position;
        if (valid_samples < LINE_VALID_ACQUIRE_SAMPLES) valid_samples++;
        invalid_samples = 0U;
        line.confidence = raw_confidence;
        if (line.line_valid ||
            (valid_samples >= LINE_VALID_ACQUIRE_SAMPLES)) {
            line.line_valid = true;
            last_valid_position = line.position;
        }
    } else {
        if (invalid_samples < LINE_VALID_RELEASE_SAMPLES) invalid_samples++;
        valid_samples = 0U;
        line.confidence = raw_confidence;
        if (!line.line_valid ||
            (invalid_samples >= LINE_VALID_RELEASE_SAMPLES))
            line.line_valid = false;
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
    return true;
}

static void LineFollow_ApplyDriveTargets(float base_speed, float correction)
{
    float minimum_inner = 50.0f;
    float maximum_correction;
    bool compatibility_mode =
        (active_config.speed_min_mm_s == active_config.speed_max_mm_s) &&
        (active_config.position_alpha == 1.0f) &&
        (active_config.derivative_alpha == 1.0f) &&
        (active_config.curve_gain == 0.0f);
    if (compatibility_mode) {
        line.correction_mm_s = correction;
        line.left_target_mm_s = base_speed + correction;
        line.right_target_mm_s = base_speed - correction;
        return;
    }
    if (minimum_inner > base_speed) minimum_inner = base_speed;
    maximum_correction = base_speed - minimum_inner;
    correction = LineFollow_Clamp(correction, -maximum_correction,
                                  maximum_correction);
    line.correction_mm_s = correction;
    line.left_target_mm_s = base_speed + correction;
    line.right_target_mm_s = base_speed - correction;
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
            line.filtered_derivative += active_config.derivative_alpha *
                (line.raw_derivative - line.filtered_derivative);
        }
        line.raw_correction_mm_s =
            (active_config.kp * line.position) +
            (active_config.kd * line.filtered_derivative);
        line.raw_correction_mm_s = LineFollow_Clamp(
            line.raw_correction_mm_s,
            -active_config.correction_limit_mm_s,
             active_config.correction_limit_mm_s);
        if ((active_config.speed_min_mm_s == active_config.speed_max_mm_s) &&
            (active_config.position_alpha == 1.0f) &&
            (active_config.derivative_alpha == 1.0f) &&
            (active_config.curve_gain == 0.0f)) {
            line.correction_mm_s = line.raw_correction_mm_s;
        } else if (dt_s > 0.0f) {
            line.correction_mm_s = LineFollow_MoveTowards(
                line.correction_mm_s, line.raw_correction_mm_s,
                active_config.correction_slew_mm_s2 * dt_s);
        }
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
        LineFollow_ApplyDriveTargets(line.dynamic_base_mm_s,
                                     line.correction_mm_s);
        previous_position = line.position;
        previous_control_ms = now_ms;
        last_valid_position = line.position;
        return;
    }

    if (line.state != LINE_FOLLOW_SEARCHING) {
        line.state = LINE_FOLLOW_SEARCHING;
        line.lost_since_ms = now_ms;
        derivative_reset_pending = true;
    }
    if ((uint32_t)(now_ms - line.lost_since_ms) >= LINE_LOST_TIMEOUT_MS) {
        line.state = LINE_FOLLOW_LOST_TIMEOUT;
        line.dynamic_base_mm_s = 0.0f;
        line.raw_correction_mm_s = 0.0f;
        line.correction_mm_s = 0.0f;
        line.left_target_mm_s = 0.0f;
        line.right_target_mm_s = 0.0f;
        return;
    }
    correction = (last_valid_position < 0.0f) ?
        -active_config.lost_search_speed_mm_s :
         active_config.lost_search_speed_mm_s;
    line.dynamic_base_mm_s = LineFollow_Clamp(
        active_config.lost_search_speed_mm_s * (3.0f / 7.0f),
        0.0f, active_config.speed_min_mm_s);
    line.raw_correction_mm_s = correction;
    line.correction_mm_s = correction;
    line.left_target_mm_s = line.dynamic_base_mm_s + correction;
    line.right_target_mm_s = line.dynamic_base_mm_s - correction;
    previous_control_ms = now_ms;
}

void LineFollow_Stop(void)
{
    line.state = LINE_FOLLOW_IDLE;
    line.lost_since_ms = 0U;
    line.dynamic_base_mm_s = 0.0f;
    line.raw_correction_mm_s = 0.0f;
    line.correction_mm_s = 0.0f;
    line.left_target_mm_s = 0.0f;
    line.right_target_mm_s = 0.0f;
    derivative_initialized = false;
    derivative_reset_pending = true;
}

void LineFollow_GetSnapshot(LineFollowSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = line;
}
