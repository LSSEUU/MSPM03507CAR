#ifndef LINE_YAW_ASSIST_H
#define LINE_YAW_ASSIST_H

#include "line_follow.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_YAW_ASSIST_MODE_AUTO = 0,
    LINE_YAW_ASSIST_MODE_STRAIGHT,
    LINE_YAW_ASSIST_MODE_CURVE_POSITIVE,
    LINE_YAW_ASSIST_MODE_CURVE_NEGATIVE
} LineYawAssistMode_t;

typedef struct {
    bool active;
    bool straight_heading_active;
    bool curve_candidate_active;
    bool curve_rate_active;
    int8_t curve_direction;
    uint32_t last_update_ms;
    float sensor_correction_mm_s;
    float straight_reference_yaw_deg;
    float straight_target_yaw_deg;
    float straight_heading_error_deg;
    float curve_progress_deg;
    float curve_feedforward_correction_mm_s;
    float desired_yaw_rate_deg_s;
    float measured_yaw_rate_deg_s;
    float rate_error_deg_s;
    float assist_correction_mm_s;
    float final_correction_mm_s;
    float left_target_mm_s;
    float right_target_mm_s;
} LineYawAssistSnapshot_t;

void LineYawAssist_Init(void);
void LineYawAssist_Reset(void);
void LineYawAssist_SetMode(LineYawAssistMode_t mode);
void LineYawAssist_SetCurveFeedforwardScale(float scale);
void LineYawAssist_SetStraightReference(bool valid, float yaw_deg);
void LineYawAssist_Update(const LineFollowSnapshot_t *line,
                          float correction_limit_mm_s,
                          float measured_yaw_deg,
                          float measured_yaw_rate_deg_s,
                          float heading_gain_per_s,
                          float damping_gain_mm_s_per_deg_s,
                          bool gyro_usable, uint32_t now_ms);
void LineYawAssist_GetSnapshot(LineYawAssistSnapshot_t *snapshot);

#endif
