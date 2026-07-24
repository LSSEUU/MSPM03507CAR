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
    uint16_t minimum[LINE_SENSOR_CHANNEL_COUNT];
    uint16_t maximum[LINE_SENSOR_CHANNEL_COUNT];
    uint8_t valid_mask;
    uint32_t sample_count;
    bool active;
    bool valid;
} LineCalibration_t;

typedef struct {
    LineCalibration_t calibration;
    LinePolarity_t polarity;
    LineFollowState_t state;
    bool line_valid;
    uint32_t scan_id;
    uint32_t last_update_ms;
    uint32_t lost_since_ms;
    uint32_t update_count;
    float strength[LINE_SENSOR_CHANNEL_COUNT];
    float position;
    float confidence;
    float correction_mm_s;
    float left_target_mm_s;
    float right_target_mm_s;
} LineFollowSnapshot_t;

void LineFollow_Init(LinePolarity_t polarity);
void LineFollow_SetPolarity(LinePolarity_t polarity);
void LineFollow_StartCalibration(void);
bool LineFollow_FinishCalibration(void);
void LineFollow_ProcessSample(const uint8_t level[LINE_SENSOR_CHANNEL_COUNT],
                              uint32_t scan_id, uint32_t now_ms);
bool LineFollow_Start(uint32_t now_ms);
void LineFollow_Update(uint32_t now_ms);
void LineFollow_Stop(void);
void LineFollow_GetSnapshot(LineFollowSnapshot_t *snapshot);

#endif
