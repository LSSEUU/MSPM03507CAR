#ifndef H_MISSION_H
#define H_MISSION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H_MISSION_MODE_H2 = 2,
    H_MISSION_MODE_H4 = 4,
    H_MISSION_MODE_H5 = 5
} HMissionMode_t;

typedef enum {
    H_MISSION_IDLE = 0,
    H_MISSION_RUNNING,
    H_MISSION_COMPLETION_LATCHED,
    H_MISSION_STOPPING,
    H_MISSION_COMPLETE,
    H_MISSION_ABORTED
} HMissionState_t;

typedef enum {
    H_MISSION_STOP_NONE = 0,
    H_MISSION_STOP_H2_FINISH,
    H_MISSION_STOP_H4_DISTANCE,
    H_MISSION_STOP_H5_FINISH_LINE,
    H_MISSION_STOP_DEVELOPMENT_DISTANCE,
    H_MISSION_STOP_USER_CANCEL,
    H_MISSION_STOP_FAULT,
    H_MISSION_STOP_GYRO,
    H_MISSION_STOP_LINE_WHITE,
    H_MISSION_STOP_OUTER_SENSOR,
    H_MISSION_STOP_NEGATIVE_TARGET,
    H_MISSION_STOP_SATURATION,
    H_MISSION_STOP_TIMEOUT,
    H_MISSION_STOP_TELEMETRY_GAP
} HMissionStopReason_t;

typedef struct {
    float h4_straight_speed_mm_s;
    float h4_b_margin_mm;
    float h5_straight_speed_mm_s;
    float h5_curve_speed_mm_s;
    float development_end_distance_mm;
} HMissionConfig_t;

typedef struct {
    HMissionMode_t selected_mode;
    HMissionMode_t running_mode;
    HMissionState_t state;
    HMissionStopReason_t stop_reason;
    HMissionConfig_t active;
    HMissionConfig_t staged;
    uint32_t active_revision;
    uint32_t stage_revision;
    uint32_t started_ms;
    uint32_t latched_ms;
    uint32_t elapsed_ms;
    float distance_mm;
    float final_left_target_mm_s;
    float final_right_target_mm_s;
    uint16_t white_ms;
    uint16_t outer_ms;
    uint16_t saturation_ms;
    bool dirty;
} HMissionSnapshot_t;

void HMission_Init(void);
bool HMission_Select(HMissionMode_t mode);
HMissionMode_t HMission_SelectedMode(void);
void HMission_Start(HMissionMode_t mode, uint32_t now_ms);
void HMission_Update(uint32_t now_ms, float distance_mm,
                     float left_target_mm_s, float right_target_mm_s,
                     uint16_t white_ms, uint16_t outer_ms,
                     uint16_t saturation_ms);
void HMission_Latch(uint32_t now_ms, HMissionStopReason_t reason);
void HMission_BeginStopping(void);
void HMission_Complete(void);
void HMission_Abort(HMissionStopReason_t reason);
bool HMission_IsActive(void);
bool HMission_IsConfigMutable(void);
bool HMission_Stage(const HMissionConfig_t *config);
bool HMission_Apply(void);
bool HMission_Revert(void);
void HMission_GetSnapshot(HMissionSnapshot_t *snapshot);

#endif
