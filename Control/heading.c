#include "heading.h"

#include "control_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HEADING_RATE_FRESH_MS 100U
#define HEADING_CALIBRATION_MAX_RATE_DEG_S 3.0f
#define HEADING_CALIBRATION_MAX_SPAN_DEG 1.0f
#define HEADING_CALIBRATION_MAX_STDDEV_DEG 0.35f

static HeadingSnapshot_t heading;
static HeadingConfig_t active_config;
static float calibration_reference_deg;
static float calibration_sum_deg;
static float calibration_sum_delta_sq;
static float calibration_min_deg;
static float calibration_max_deg;
static float hold_base_speed_mm_s;
static uint8_t turn_settle_samples;
static float continuous_yaw_deg;
static float previous_normalized_yaw_deg;
static float turn_target_continuous_deg;
static bool continuous_yaw_valid;
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

static float Heading_ClampRange(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static bool Heading_IsFinite(float value)
{
    return (value == value) &&
           (value <= 3.402823466e+38F) &&
           (value >= -3.402823466e+38F);
}

float Heading_NormalizeDegrees(float degrees)
{
    while (degrees >= 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

void Heading_GetDefaultConfig(HeadingConfig_t *config)
{
    if (config == NULL) return;
    config->straight_kp = HEADING_HOLD_KP_MM_S_PER_DEG;
    config->straight_kd = HEADING_HOLD_KD_MM_S_PER_DEG_S;
    config->turn_kp = HEADING_TURN_KP_MM_S_PER_DEG;
    config->turn_kd = HEADING_TURN_KD_MM_S_PER_DEG_S;
    config->rate_alpha = HEADING_YAW_RATE_FILTER_ALPHA;
    config->correction_limit_mm_s = HEADING_HOLD_MAX_CORRECTION_MM_S;
    config->turn_speed_mm_s = HEADING_TURN_MAX_SPEED_MM_S;
    /* Equal to tolerance is the compatibility setting (legacy minimum speed). */
    config->smooth_zone_deg = HEADING_TURN_TOLERANCE_DEG;
}

void Heading_GetConfig(HeadingConfig_t *config)
{
    if (config != NULL) *config = active_config;
}

bool Heading_ApplyConfig(const HeadingConfig_t *config)
{
    if ((config == NULL) ||
        !Heading_IsFinite(config->straight_kp) ||
        !Heading_IsFinite(config->straight_kd) ||
        !Heading_IsFinite(config->turn_kp) ||
        !Heading_IsFinite(config->turn_kd) ||
        !Heading_IsFinite(config->rate_alpha) ||
        !Heading_IsFinite(config->correction_limit_mm_s) ||
        !Heading_IsFinite(config->turn_speed_mm_s) ||
        !Heading_IsFinite(config->smooth_zone_deg) ||
        (config->straight_kp < 0.0f) || (config->straight_kp > 50.0f) ||
        (config->straight_kd < 0.0f) || (config->straight_kd > 10.0f) ||
        (config->turn_kp < 0.0f) || (config->turn_kp > 50.0f) ||
        (config->turn_kd < 0.0f) || (config->turn_kd > 10.0f) ||
        (config->rate_alpha < 0.05f) || (config->rate_alpha > 1.0f) ||
        (config->correction_limit_mm_s < 0.0f) ||
        (config->correction_limit_mm_s > 250.0f) ||
        (config->turn_speed_mm_s < 50.0f) ||
        (config->turn_speed_mm_s > 400.0f) ||
        (config->smooth_zone_deg < HEADING_TURN_TOLERANCE_DEG) ||
        (config->smooth_zone_deg > 90.0f)) return false;
    active_config = *config;
    return true;
}

void Heading_Init(void)
{
    memset(&heading, 0, sizeof(heading));
    Heading_GetDefaultConfig(&active_config);
    heading.mode = HEADING_MODE_IDLE;
    Heading_StartCalibration();
}

void Heading_StartCalibration(void)
{
    heading.calibrated = false;
    heading.calibrating = true;
    heading.calibration_stationary = true;
    heading.calibration_count = 0U;
    heading.calibration_span_deg = 0.0f;
    heading.calibration_stddev_deg = 0.0f;
    calibration_reference_deg = 0.0f;
    calibration_sum_deg = 0.0f;
    calibration_sum_delta_sq = 0.0f;
    calibration_min_deg = 0.0f;
    calibration_max_deg = 0.0f;
    heading.raw_yaw_rate_deg_s = 0.0f;
    heading.yaw_rate_deg_s = 0.0f;
    heading.rate_source = HEADING_RATE_NONE;
    yaw_rate_initialized = false;
    continuous_yaw_valid = false;
    continuous_yaw_deg = 0.0f;
    previous_normalized_yaw_deg = 0.0f;
    turn_target_continuous_deg = 0.0f;
    Heading_Stop();
}

static void Heading_UpdateControl(void)
{
    float correction;
    float turn_speed;
    float speed_limit;
    float zone_fraction;
    heading.error_deg = heading.mode == HEADING_MODE_TURN ?
        (turn_target_continuous_deg - continuous_yaw_deg) :
        Heading_NormalizeDegrees(heading.target_deg - heading.yaw_deg);
    if (heading.mode == HEADING_MODE_HOLD) {
        correction = Heading_Clamp(
            (active_config.straight_kp * heading.error_deg) -
            (active_config.straight_kd * heading.yaw_rate_deg_s),
            active_config.correction_limit_mm_s);
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
            turn_speed = (active_config.turn_kp * heading.error_deg) -
                         (active_config.turn_kd * heading.yaw_rate_deg_s);
            if (active_config.smooth_zone_deg <=
                HEADING_TURN_TOLERANCE_DEG) {
                turn_speed = Heading_Clamp(turn_speed,
                                            active_config.turn_speed_mm_s);
                if ((turn_speed > 0.0f) &&
                    (turn_speed < HEADING_TURN_MIN_SPEED_MM_S))
                    turn_speed = HEADING_TURN_MIN_SPEED_MM_S;
                if ((turn_speed < 0.0f) &&
                    (turn_speed > -HEADING_TURN_MIN_SPEED_MM_S))
                    turn_speed = -HEADING_TURN_MIN_SPEED_MM_S;
            } else {
                zone_fraction = (Heading_Abs(heading.error_deg) -
                    HEADING_TURN_TOLERANCE_DEG) /
                    (active_config.smooth_zone_deg -
                     HEADING_TURN_TOLERANCE_DEG);
                zone_fraction = Heading_ClampRange(zone_fraction,
                                                    0.0f, 1.0f);
                zone_fraction = zone_fraction * zone_fraction *
                    (3.0f - (2.0f * zone_fraction));
                speed_limit = active_config.turn_speed_mm_s * zone_fraction;
                turn_speed = Heading_Clamp(turn_speed, speed_limit);
            }
            heading.left_target_mm_s = -turn_speed;
            heading.right_target_mm_s = turn_speed;
        }
    }
}

static void Heading_ResetCalibrationWindow(void)
{
    heading.calibration_count = 0U;
    heading.calibration_span_deg = 0.0f;
    heading.calibration_stddev_deg = 0.0f;
    calibration_reference_deg = 0.0f;
    calibration_sum_deg = 0.0f;
    calibration_sum_delta_sq = 0.0f;
    calibration_min_deg = 0.0f;
    calibration_max_deg = 0.0f;
}

static bool Heading_AddCalibrationSample(float raw_yaw_deg,
                                         bool direct_rate_fresh,
                                         float raw_rate_deg_s)
{
    float mean_delta;
    float variance;
    float unwrapped;
    if (direct_rate_fresh &&
        (Heading_Abs(raw_rate_deg_s) >
         HEADING_CALIBRATION_MAX_RATE_DEG_S)) {
        heading.calibration_stationary = false;
        heading.calibration_rejects++;
        Heading_ResetCalibrationWindow();
        return false;
    }
    heading.calibration_stationary = true;
    if (heading.calibration_count == 0U) {
        calibration_reference_deg = raw_yaw_deg;
        calibration_sum_deg = raw_yaw_deg;
        calibration_sum_delta_sq = 0.0f;
        calibration_min_deg = raw_yaw_deg;
        calibration_max_deg = raw_yaw_deg;
    } else {
        unwrapped = calibration_reference_deg +
            Heading_NormalizeDegrees(raw_yaw_deg -
                                     calibration_reference_deg);
        calibration_sum_deg += unwrapped;
        calibration_sum_delta_sq +=
            (unwrapped - calibration_reference_deg) *
            (unwrapped - calibration_reference_deg);
        if (unwrapped < calibration_min_deg) calibration_min_deg = unwrapped;
        if (unwrapped > calibration_max_deg) calibration_max_deg = unwrapped;
    }
    heading.calibration_count++;
    heading.calibration_span_deg = calibration_max_deg - calibration_min_deg;
    mean_delta = (calibration_sum_deg /
                  (float)heading.calibration_count) -
                 calibration_reference_deg;
    variance = (calibration_sum_delta_sq /
                (float)heading.calibration_count) -
               (mean_delta * mean_delta);
    if (variance < 0.0f) variance = 0.0f;
    heading.calibration_stddev_deg = sqrtf(variance);
    if (heading.calibration_count < HEADING_CALIBRATION_SAMPLES) return false;
    if ((heading.calibration_span_deg > HEADING_CALIBRATION_MAX_SPAN_DEG) ||
        (heading.calibration_stddev_deg >
         HEADING_CALIBRATION_MAX_STDDEV_DEG)) {
        heading.calibration_stationary = false;
        heading.calibration_rejects++;
        Heading_ResetCalibrationWindow();
        return false;
    }
    heading.zero_offset_deg = Heading_NormalizeDegrees(
        calibration_sum_deg / (float)heading.calibration_count);
    heading.calibrating = false;
    heading.calibrated = true;
    return true;
}

void Heading_UpdateWithRate(float raw_yaw_deg, uint32_t frame_id,
                            float raw_yaw_rate_deg_s,
                            uint32_t rate_frame_id,
                            uint32_t rate_update_ms, uint32_t now_ms)
{
    bool new_angle = (frame_id != 0U) &&
                     (frame_id != heading.last_frame_id);
    bool new_rate = (rate_frame_id != 0U) &&
                    (rate_frame_id != heading.last_rate_frame_id);
    bool direct_rate_fresh = (rate_frame_id != 0U) &&
        ((uint32_t)(now_ms - rate_update_ms) <= HEADING_RATE_FRESH_MS);
    bool calibration_completed = false;
    float raw_rate = 0.0f;

    if (!new_angle && !new_rate && !direct_rate_fresh) return;
    if (new_rate) {
        heading.last_rate_frame_id = rate_frame_id;
        heading.last_rate_update_ms = rate_update_ms;
    }
    if (new_angle) {
        heading.last_frame_id = frame_id;
        heading.last_update_ms = now_ms;
        heading.sample_count++;
        heading.valid = true;
        if (heading.calibrating) {
            calibration_completed = Heading_AddCalibrationSample(
                raw_yaw_deg, direct_rate_fresh, raw_yaw_rate_deg_s);
        }
        heading.yaw_deg = Heading_NormalizeDegrees(
            raw_yaw_deg - heading.zero_offset_deg);
        if (!continuous_yaw_valid || calibration_completed) {
            continuous_yaw_deg = heading.yaw_deg;
            continuous_yaw_valid = true;
        } else {
            continuous_yaw_deg += Heading_NormalizeDegrees(
                heading.yaw_deg - previous_normalized_yaw_deg);
        }
        previous_normalized_yaw_deg = heading.yaw_deg;
    }

    if (direct_rate_fresh) {
        raw_rate = raw_yaw_rate_deg_s;
        heading.rate_source = HEADING_RATE_GYRO_FRAME;
    } else if (new_angle && heading.calibrated && yaw_rate_initialized &&
               (now_ms != previous_yaw_ms)) {
        raw_rate = Heading_NormalizeDegrees(
            heading.yaw_deg - previous_yaw_deg) * 1000.0f /
            (float)(uint32_t)(now_ms - previous_yaw_ms);
        heading.rate_source = HEADING_RATE_ANGLE_DIFFERENCE;
    } else {
        heading.rate_source = HEADING_RATE_NONE;
    }

    if (!heading.calibrated || calibration_completed ||
        !yaw_rate_initialized) {
        heading.raw_yaw_rate_deg_s = 0.0f;
        heading.yaw_rate_deg_s = 0.0f;
        if (new_angle) {
            previous_yaw_deg = heading.yaw_deg;
            previous_yaw_ms = now_ms;
        }
        yaw_rate_initialized = heading.calibrated;
    } else {
        if (heading.rate_source != HEADING_RATE_NONE) {
            heading.raw_yaw_rate_deg_s = raw_rate;
            if ((heading.rate_source == HEADING_RATE_ANGLE_DIFFERENCE) ||
                new_rate) {
                heading.yaw_rate_deg_s += active_config.rate_alpha *
                    (raw_rate - heading.yaw_rate_deg_s);
            }
        }
        if (new_angle) {
            previous_yaw_deg = heading.yaw_deg;
            previous_yaw_ms = now_ms;
        }
    }
    if (heading.calibrated) Heading_UpdateControl();
}

void Heading_Update(float raw_yaw_deg, uint32_t frame_id, uint32_t now_ms)
{
    Heading_UpdateWithRate(raw_yaw_deg, frame_id, 0.0f, 0U, 0U, now_ms);
}

bool Heading_StartHold(float base_speed_mm_s)
{
    if (!heading.calibrated || !heading.valid) return false;
    heading.mode = HEADING_MODE_HOLD;
    heading.complete = false;
    heading.reference_deg = heading.yaw_deg;
    heading.target_deg = heading.reference_deg;
    hold_base_speed_mm_s = Heading_Clamp(base_speed_mm_s,
                                         WHEEL_SPEED_MAX_TARGET_MM_S);
    Heading_UpdateControl();
    return true;
}

bool Heading_SetHoldSpeed(float base_speed_mm_s)
{
    if (heading.mode != HEADING_MODE_HOLD) return false;
    hold_base_speed_mm_s = Heading_Clamp(base_speed_mm_s,
                                         WHEEL_SPEED_MAX_TARGET_MM_S);
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
    turn_target_continuous_deg = continuous_yaw_deg + relative_degrees;
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
