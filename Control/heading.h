#ifndef HEADING_H
#define HEADING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HEADING_MODE_IDLE = 0,
    HEADING_MODE_HOLD,
    HEADING_MODE_TURN
} HeadingMode_t;

typedef struct {
    bool valid;
    bool calibrated;
    bool calibrating;
    bool complete;
    HeadingMode_t mode;
    uint32_t calibration_count;
    uint32_t sample_count;
    uint32_t last_frame_id;
    uint32_t last_update_ms;
    float zero_offset_deg;
    float yaw_deg;
    float yaw_rate_deg_s;
    float reference_deg;
    float target_deg;
    float error_deg;
    float left_target_mm_s;
    float right_target_mm_s;
} HeadingSnapshot_t;

float Heading_NormalizeDegrees(float degrees);
void Heading_Init(void);
void Heading_StartCalibration(void);
void Heading_Update(float raw_yaw_deg, uint32_t frame_id, uint32_t now_ms);
bool Heading_StartHold(float base_speed_mm_s);
bool Heading_StartTurn(float relative_degrees);
void Heading_Stop(void);
void Heading_GetSnapshot(HeadingSnapshot_t *snapshot);

#endif
