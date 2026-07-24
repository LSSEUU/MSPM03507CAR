#include "line_follow.h"

#include "control_config.h"

#include <stddef.h>
#include <string.h>

static const float line_channel_position[LINE_SENSOR_CHANNEL_COUNT] = {
    -1.0f, -0.7142857f, -0.4285714f, -0.1428571f,
     0.1428571f, 0.4285714f,  0.7142857f,  1.0f
};

static LineFollowSnapshot_t line;
static uint8_t valid_samples;
static uint8_t invalid_samples;
static float previous_position;
static uint32_t previous_control_ms;
static float last_valid_position;

static float LineFollow_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void LineFollow_Init(LinePolarity_t polarity)
{
    memset(&line, 0, sizeof(line));
    line.polarity = polarity;
    line.state = LINE_FOLLOW_IDLE;
    valid_samples = 0U;
    invalid_samples = 0U;
    previous_position = 0.0f;
    previous_control_ms = 0U;
    last_valid_position = 0.0f;
}

void LineFollow_SetPolarity(LinePolarity_t polarity)
{
    if (line.state != LINE_FOLLOW_IDLE || line.calibration.active) return;
    line.polarity = polarity;
    line.line_valid = false;
    line.confidence = 0.0f;
    valid_samples = 0U;
    invalid_samples = 0U;
}

void LineFollow_StartCalibration(void)
{
    uint32_t i;
    if (line.state != LINE_FOLLOW_IDLE) return;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        line.calibration.minimum[i] = UINT16_MAX;
        line.calibration.maximum[i] = 0U;
    }
    line.calibration.valid_mask = 0U;
    line.calibration.sample_count = 0U;
    line.calibration.valid = false;
    line.calibration.active = true;
    line.line_valid = false;
    line.confidence = 0.0f;
    valid_samples = 0U;
    invalid_samples = 0U;
}

bool LineFollow_FinishCalibration(void)
{
    uint32_t i;
    uint8_t valid_mask = 0U;
    if (!line.calibration.active) return line.calibration.valid;
    line.calibration.active = false;
    if (line.calibration.sample_count < LINE_CALIBRATION_MIN_SAMPLES) {
        line.calibration.valid = false;
        return false;
    }
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint16_t span = (uint16_t)(line.calibration.maximum[i] -
                                   line.calibration.minimum[i]);
        if (span >= LINE_CALIBRATION_MIN_SPAN)
            valid_mask |= (uint8_t)(1U << i);
    }
    line.calibration.valid_mask = valid_mask;
    line.calibration.valid = (valid_mask == 0xFFU);
    return line.calibration.valid;
}

static void LineFollow_UpdateCalibration(
    const uint8_t level[LINE_SENSOR_CHANNEL_COUNT])
{
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (level[i] < line.calibration.minimum[i])
            line.calibration.minimum[i] = level[i];
        if (level[i] > line.calibration.maximum[i])
            line.calibration.maximum[i] = level[i];
    }
    line.calibration.sample_count++;
}

void LineFollow_ProcessSample(const uint8_t level[LINE_SENSOR_CHANNEL_COUNT],
                              uint32_t scan_id, uint32_t now_ms)
{
    uint32_t i;
    float weighted_sum = 0.0f;
    float active_sum = 0.0f;
    float raw_confidence;
    bool raw_valid;
    if ((level == NULL) || (scan_id == line.scan_id)) return;
    line.scan_id = scan_id;
    line.last_update_ms = now_ms;
    line.update_count++;
    if (line.calibration.active) {
        LineFollow_UpdateCalibration(level);
        return;
    }
    if (!line.calibration.valid) return;

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        float span = (float)(line.calibration.maximum[i] -
                             line.calibration.minimum[i]);
        float normalized = ((float)level[i] -
                            (float)line.calibration.minimum[i]) / span;
        float strength;
        normalized = LineFollow_Clamp(normalized, 0.0f, 1.0f);
        strength = (line.polarity == LINE_POLARITY_ACTIVE_HIGH) ?
            normalized : (1.0f - normalized);
        strength = LineFollow_Clamp(
            (strength - LINE_SIGNAL_FLOOR) / (1.0f - LINE_SIGNAL_FLOOR),
            0.0f, 1.0f);
        line.strength[i] = strength;
        active_sum += strength;
        weighted_sum += strength * line_channel_position[i];
    }
    raw_confidence = LineFollow_Clamp(
        active_sum / LINE_EXPECTED_ACTIVE_SUM, 0.0f, 1.0f);
    raw_valid = raw_confidence >= LINE_VALID_CONFIDENCE;
    if (raw_valid) {
        if (valid_samples < LINE_VALID_ACQUIRE_SAMPLES) valid_samples++;
        invalid_samples = 0U;
        line.position = weighted_sum / active_sum;
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
}

bool LineFollow_Start(uint32_t now_ms)
{
    if ((line.state != LINE_FOLLOW_IDLE) || !line.calibration.valid ||
        !line.line_valid || line.calibration.active)
        return false;
    line.state = LINE_FOLLOW_TRACKING;
    line.lost_since_ms = 0U;
    line.correction_mm_s = 0.0f;
    previous_position = line.position;
    previous_control_ms = now_ms;
    last_valid_position = line.position;
    return true;
}

void LineFollow_Update(uint32_t now_ms)
{
    float derivative = 0.0f;
    float correction;
    if (line.state == LINE_FOLLOW_IDLE ||
        line.state == LINE_FOLLOW_LOST_TIMEOUT)
        return;

    if (line.line_valid) {
        if (previous_control_ms != now_ms) {
            derivative = (line.position - previous_position) * 1000.0f /
                (float)(uint32_t)(now_ms - previous_control_ms);
        }
        correction = (LINE_FOLLOW_KP_MM_S * line.position) +
                     (LINE_FOLLOW_KD_MM_S_PER_UNIT_S * derivative);
        correction = LineFollow_Clamp(correction,
            -LINE_FOLLOW_MAX_CORRECTION_MM_S,
             LINE_FOLLOW_MAX_CORRECTION_MM_S);
        line.state = LINE_FOLLOW_TRACKING;
        line.lost_since_ms = 0U;
        line.correction_mm_s = correction;
        line.left_target_mm_s = LINE_FOLLOW_BASE_SPEED_MM_S + correction;
        line.right_target_mm_s = LINE_FOLLOW_BASE_SPEED_MM_S - correction;
        previous_position = line.position;
        previous_control_ms = now_ms;
        last_valid_position = line.position;
        return;
    }

    if (line.state != LINE_FOLLOW_SEARCHING) {
        line.state = LINE_FOLLOW_SEARCHING;
        line.lost_since_ms = now_ms;
    }
    if ((uint32_t)(now_ms - line.lost_since_ms) >= LINE_LOST_TIMEOUT_MS) {
        line.state = LINE_FOLLOW_LOST_TIMEOUT;
        line.correction_mm_s = 0.0f;
        line.left_target_mm_s = 0.0f;
        line.right_target_mm_s = 0.0f;
        return;
    }
    correction = (last_valid_position < 0.0f) ?
        -LINE_SEARCH_TURN_MM_S : LINE_SEARCH_TURN_MM_S;
    line.correction_mm_s = correction;
    line.left_target_mm_s = LINE_SEARCH_FORWARD_MM_S + correction;
    line.right_target_mm_s = LINE_SEARCH_FORWARD_MM_S - correction;
    previous_control_ms = now_ms;
}

void LineFollow_Stop(void)
{
    line.calibration.active = false;
    line.state = LINE_FOLLOW_IDLE;
    line.lost_since_ms = 0U;
    line.correction_mm_s = 0.0f;
    line.left_target_mm_s = 0.0f;
    line.right_target_mm_s = 0.0f;
}

void LineFollow_GetSnapshot(LineFollowSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = line;
}
