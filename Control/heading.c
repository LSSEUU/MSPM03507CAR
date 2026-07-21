#include "heading.h"

#include "control_config.h"

#include <stddef.h>
#include <string.h>

static HeadingSnapshot_t heading;
static float calibration_reference_deg;
static float calibration_sum_deg;
static float hold_base_speed_mm_s;
static uint8_t turn_settle_samples;
static bool yaw_rate_initialized;
static float previous_yaw_deg;
static uint32_t previous_yaw_ms;

static float Heading_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Heading_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

float Heading_NormalizeDegrees(float degrees)
{
    while (degrees >= 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

void Heading_Init(void)
{
    memset(&heading, 0, sizeof(heading));
    heading.mode = HEADING_MODE_IDLE;
    Heading_StartCalibration();
}

void Heading_StartCalibration(void)
{
    heading.calibrated = false;
    heading.calibrating = true;
    heading.calibration_count = 0U;
    calibration_reference_deg = 0.0f;
    calibration_sum_deg = 0.0f;
    heading.yaw_rate_deg_s = 0.0f;
    yaw_rate_initialized = false;
    Heading_Stop();
}

static void Heading_UpdateControl(void)
{
    float correction;
    float turn_speed;
    heading.error_deg = Heading_NormalizeDegrees(
        heading.target_deg - heading.yaw_deg);
    if (heading.mode == HEADING_MODE_HOLD) {
        correction = Heading_Clamp(
            HEADING_HOLD_KP_MM_S_PER_DEG * heading.error_deg,
            HEADING_HOLD_MAX_CORRECTION_MM_S);
        heading.left_target_mm_s = hold_base_speed_mm_s - correction;
        heading.right_target_mm_s = hold_base_speed_mm_s + correction;
    } else if (heading.mode == HEADING_MODE_TURN) {
        if (Heading_Abs(heading.error_deg) <= HEADING_TURN_TOLERANCE_DEG) {
            if (turn_settle_samples < HEADING_TURN_SETTLE_SAMPLES)
                turn_settle_samples++;
            heading.left_target_mm_s = 0.0f;
            heading.right_target_mm_s = 0.0f;
            if (turn_settle_samples >= HEADING_TURN_SETTLE_SAMPLES)
                heading.complete = true;
        } else {
            turn_settle_samples = 0U;
            turn_speed = Heading_Clamp(
                (HEADING_TURN_KP_MM_S_PER_DEG * heading.error_deg) -
                (HEADING_TURN_KD_MM_S_PER_DEG_S *
                 heading.yaw_rate_deg_s),
                HEADING_TURN_MAX_SPEED_MM_S);
            if ((turn_speed > 0.0f) &&
                (turn_speed < HEADING_TURN_MIN_SPEED_MM_S))
                turn_speed = HEADING_TURN_MIN_SPEED_MM_S;
            if ((turn_speed < 0.0f) &&
                (turn_speed > -HEADING_TURN_MIN_SPEED_MM_S))
                turn_speed = -HEADING_TURN_MIN_SPEED_MM_S;
            heading.left_target_mm_s = -turn_speed;
            heading.right_target_mm_s = turn_speed;
        }
    }
}

void Heading_Update(float raw_yaw_deg, uint32_t frame_id, uint32_t now_ms)
{
    bool calibration_completed = false;
    float unwrapped;
    if ((frame_id == 0U) || (frame_id == heading.last_frame_id)) return;
    heading.last_frame_id = frame_id;
    heading.last_update_ms = now_ms;
    heading.sample_count++;
    heading.valid = true;

    if (heading.calibrating) {
        if (heading.calibration_count == 0U) {
            calibration_reference_deg = raw_yaw_deg;
            calibration_sum_deg = raw_yaw_deg;
        } else {
            unwrapped = calibration_reference_deg +
                Heading_NormalizeDegrees(raw_yaw_deg -
                                         calibration_reference_deg);
            calibration_sum_deg += unwrapped;
        }
        heading.calibration_count++;
        if (heading.calibration_count >= HEADING_CALIBRATION_SAMPLES) {
            heading.zero_offset_deg = Heading_NormalizeDegrees(
                calibration_sum_deg / (float)heading.calibration_count);
            heading.calibrating = false;
            heading.calibrated = true;
            calibration_completed = true;
        }
    }
    heading.yaw_deg = Heading_NormalizeDegrees(
        raw_yaw_deg - heading.zero_offset_deg);
    if (!heading.calibrated || calibration_completed ||
        !yaw_rate_initialized) {
        heading.yaw_rate_deg_s = 0.0f;
        previous_yaw_deg = heading.yaw_deg;
        previous_yaw_ms = now_ms;
        yaw_rate_initialized = heading.calibrated;
    } else if (now_ms != previous_yaw_ms) {
        float raw_rate = Heading_NormalizeDegrees(
            heading.yaw_deg - previous_yaw_deg) * 1000.0f /
            (float)(uint32_t)(now_ms - previous_yaw_ms);
        heading.yaw_rate_deg_s += HEADING_YAW_RATE_FILTER_ALPHA *
            (raw_rate - heading.yaw_rate_deg_s);
        previous_yaw_deg = heading.yaw_deg;
        previous_yaw_ms = now_ms;
    }
    if (heading.calibrated) Heading_UpdateControl();
}

bool Heading_StartHold(float base_speed_mm_s)
{
    if (!heading.calibrated || !heading.valid) return false;
    heading.mode = HEADING_MODE_HOLD;
    heading.complete = false;
    heading.reference_deg = heading.yaw_deg;
    heading.target_deg = heading.reference_deg;
    hold_base_speed_mm_s = base_speed_mm_s;
    Heading_UpdateControl();
    return true;
}

bool Heading_StartTurn(float relative_degrees)
{
    if (!heading.calibrated || !heading.valid) return false;
    heading.mode = HEADING_MODE_TURN;
    heading.complete = false;
    turn_settle_samples = 0U;
    heading.reference_deg = heading.yaw_deg;
    heading.target_deg = Heading_NormalizeDegrees(
        heading.reference_deg + relative_degrees);
    heading.yaw_rate_deg_s = 0.0f;
    previous_yaw_deg = heading.yaw_deg;
    previous_yaw_ms = heading.last_update_ms;
    Heading_UpdateControl();
    return true;
}

void Heading_Stop(void)
{
    heading.mode = HEADING_MODE_IDLE;
    heading.complete = false;
    heading.error_deg = 0.0f;
    heading.left_target_mm_s = 0.0f;
    heading.right_target_mm_s = 0.0f;
    turn_settle_samples = 0U;
}

void Heading_GetSnapshot(HeadingSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = heading;
}
