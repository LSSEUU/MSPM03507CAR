#ifndef H_COURSE_PROFILE_H
#define H_COURSE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H_COURSE_AB_STRAIGHT = 0,
    H_COURSE_BC_CURVE,
    H_COURSE_CD_STRAIGHT,
    H_COURSE_DA_CURVE
} HCoursePhase_t;

typedef struct {
    bool running;
    HCoursePhase_t phase;
    uint32_t lap_count;
    uint32_t last_update_ms;
    float total_distance_mm;
    float phase_distance_mm;
    float curve_progress_deg;
    bool curve_entry_heading_valid;
    float curve_entry_yaw_deg;
    bool straight_heading_target_valid;
    float straight_heading_anchor_deg;
    float straight_heading_target_deg;
    uint8_t straight_capture_samples;
    uint8_t curve_evidence_samples;
    bool fast_straight_latched;
    bool initial_ab_offset_applied;
    bool finish_enabled;
    uint8_t finish_preline_samples;
    uint8_t finish_candidate_samples;
    bool finish_line_detected;
    float finish_line_distance_mm;
} HCourseSnapshot_t;

void HCourse_Init(void);
void HCourse_Start(uint32_t now_ms);
void HCourse_Stop(void);
void HCourse_SetFinishEnabled(bool enabled);
void HCourse_Update(uint32_t now_ms, bool wheel_sample_valid,
                    float left_speed_mm_s, float right_speed_mm_s,
                    bool heading_sample_valid, float yaw_deg,
                    float yaw_rate_deg_s, bool line_usable,
                    uint8_t active_mask, float line_position);
void HCourse_GetSnapshot(HCourseSnapshot_t *snapshot);
bool HCourse_IsCurvePhase(void);
float HCourse_SelectBaseSpeed(float tracking_base_mm_s,
                              float curve_base_mm_s,
                              float straight_base_mm_s,
                              bool line_usable,
                              float line_position);

#endif
