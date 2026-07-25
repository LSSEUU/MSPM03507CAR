#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_CHANNEL_COUNT 8U

typedef enum {
    LINE_POLARITY_ACTIVE_LOW = 0,
    LINE_POLARITY_ACTIVE_HIGH
} LinePolarity_t;

typedef enum {
    LINE_FOLLOW_IDLE = 0,
    LINE_FOLLOW_TRACKING,
    LINE_FOLLOW_SEARCHING,
    LINE_FOLLOW_LOST_TIMEOUT
} LineFollowState_t;

typedef struct {
    float speed_min_mm_s;
    float speed_max_mm_s;
    float position_alpha;
    float derivative_alpha;
    float kp;
    float kd;
    float correction_limit_mm_s;
    float correction_slew_mm_s2;
    float curve_gain;
    float lost_search_speed_mm_s;
} LineFollowConfig_t;

typedef struct {
    LinePolarity_t polarity;
    LineFollowState_t state;
    bool line_valid;
    bool ambiguous;
    uint8_t active_mask;
    uint8_t active_count;
    uint8_t cluster_count;
    uint32_t scan_id;
    uint32_t last_update_ms;
    uint32_t lost_since_ms;
    uint32_t update_count;
    uint32_t ambiguity_count;
    float strength[LINE_SENSOR_CHANNEL_COUNT];
    float raw_position;
    float position;
    float raw_derivative;
    float filtered_derivative;
    float confidence;
    float dynamic_base_mm_s;
    float raw_correction_mm_s;
    float correction_mm_s;
    float left_target_mm_s;
    float right_target_mm_s;
} LineFollowSnapshot_t;

void LineFollow_Init(LinePolarity_t polarity);
void LineFollow_GetDefaultConfig(LineFollowConfig_t *config);
void LineFollow_GetConfig(LineFollowConfig_t *config);
bool LineFollow_ApplyConfig(const LineFollowConfig_t *config);
void LineFollow_SetPolarity(LinePolarity_t polarity);
void LineFollow_ProcessSample(const uint8_t level[LINE_SENSOR_CHANNEL_COUNT],
                              uint32_t scan_id, uint32_t now_ms);
bool LineFollow_Start(uint32_t now_ms);
void LineFollow_Update(uint32_t now_ms);
void LineFollow_Stop(void);
void LineFollow_GetSnapshot(LineFollowSnapshot_t *snapshot);

#endif
