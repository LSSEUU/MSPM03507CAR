#ifndef HEADING_H
#define HEADING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HEADING_MODE_IDLE = 0,
    HEADING_MODE_HOLD,
    HEADING_MODE_TURN,
    HEADING_MODE_RATE
} HeadingMode_t;

typedef enum {
    HEADING_RATE_NONE = 0,
    HEADING_RATE_ANGLE_DIFFERENCE,
    HEADING_RATE_GYRO_FRAME
} HeadingRateSource_t;

typedef struct {
    float straight_kp;
    float straight_kd;
    float turn_kp;
    float turn_kd;
    float rate_alpha;
    float correction_limit_mm_s;
    float turn_speed_mm_s;
    float smooth_zone_deg;
} HeadingConfig_t;

typedef struct {
    bool valid;
    bool calibrated;
    bool calibrating;
    bool calibration_stationary;
    bool complete;
    HeadingMode_t mode;
    HeadingRateSource_t rate_source;
    uint32_t calibration_count;
    uint32_t calibration_rejects;
    uint32_t sample_count;
    uint32_t last_frame_id;
    uint32_t last_update_ms;
    uint32_t last_rate_frame_id;
    uint32_t last_rate_update_ms;
    float zero_offset_deg;
    float yaw_deg;
    float raw_yaw_rate_deg_s;
    float yaw_rate_deg_s;
    float calibration_span_deg;
    float calibration_stddev_deg;
    float reference_deg;
    float target_deg;
    float error_deg;
    float target_rate_deg_s;
    float rate_error_deg_s;
    float left_target_mm_s;
    float right_target_mm_s;
} HeadingSnapshot_t;

float Heading_NormalizeDegrees(float degrees);
void Heading_Init(void);
void Heading_StartCalibration(void);
void Heading_Update(float raw_yaw_deg, uint32_t frame_id, uint32_t now_ms);
void Heading_UpdateWithRate(float raw_yaw_deg, uint32_t frame_id,
                            float raw_yaw_rate_deg_s,
                            uint32_t rate_frame_id,
                            uint32_t rate_update_ms, uint32_t now_ms);
bool Heading_StartHold(float base_speed_mm_s);
bool Heading_SetHoldSpeed(float base_speed_mm_s);
bool Heading_StartTurn(float relative_degrees);
bool Heading_StartRate(float target_rate_deg_s);
void Heading_Stop(void);
void Heading_GetDefaultConfig(HeadingConfig_t *config);
void Heading_GetConfig(HeadingConfig_t *config);
bool Heading_ApplyConfig(const HeadingConfig_t *config);
void Heading_GetSnapshot(HeadingSnapshot_t *snapshot);

#endif
