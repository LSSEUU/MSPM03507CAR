#include "h_mission.h"

#include <stddef.h>
#include <string.h>

#define H_MISSION_H4_SPEED_MIN_MM_S          100.0f
#define H_MISSION_H4_SPEED_MAX_MM_S          300.0f
#define H_MISSION_H4_MARGIN_MIN_MM             0.0f
#define H_MISSION_H4_MARGIN_MAX_MM           150.0f
#define H_MISSION_H5_SPEED_MIN_MM_S          100.0f
#define H_MISSION_H5_SPEED_MAX_MM_S          350.0f
#define H_MISSION_DEV_END_MAX_MM            7000.0f

static HMissionSnapshot_t mission;

static HMissionConfig_t HMission_DefaultConfig(void)
{
    HMissionConfig_t config;
    config.h4_straight_speed_mm_s = 220.0f;
    config.h4_b_margin_mm = 0.0f;
    config.h5_straight_speed_mm_s = 230.0f;
    config.h5_curve_speed_mm_s = 210.0f;
    config.development_end_distance_mm = 0.0f;
    return config;
}

static bool HMission_ConfigValid(const HMissionConfig_t *config)
{
    if (config == NULL) return false;
    return (config->h4_straight_speed_mm_s >=
            H_MISSION_H4_SPEED_MIN_MM_S) &&
        (config->h4_straight_speed_mm_s <=
         H_MISSION_H4_SPEED_MAX_MM_S) &&
        (config->h4_b_margin_mm >= H_MISSION_H4_MARGIN_MIN_MM) &&
        (config->h4_b_margin_mm <= H_MISSION_H4_MARGIN_MAX_MM) &&
        (config->h5_straight_speed_mm_s >=
         H_MISSION_H5_SPEED_MIN_MM_S) &&
        (config->h5_straight_speed_mm_s <=
         H_MISSION_H5_SPEED_MAX_MM_S) &&
        (config->h5_curve_speed_mm_s >= H_MISSION_H5_SPEED_MIN_MM_S) &&
        (config->h5_curve_speed_mm_s <=
         config->h5_straight_speed_mm_s) &&
        (config->development_end_distance_mm >= 0.0f) &&
        (config->development_end_distance_mm <=
         H_MISSION_DEV_END_MAX_MM);
}

void HMission_Init(void)
{
    memset(&mission, 0, sizeof(mission));
    mission.selected_mode = H_MISSION_MODE_H2;
    mission.running_mode = H_MISSION_MODE_H2;
    mission.state = H_MISSION_IDLE;
    mission.active = HMission_DefaultConfig();
    mission.staged = mission.active;
    mission.active_revision = 1U;
    mission.stage_revision = 1U;
}

bool HMission_Select(HMissionMode_t mode)
{
    if ((mission.state == H_MISSION_RUNNING) ||
        (mission.state == H_MISSION_COMPLETION_LATCHED) ||
        (mission.state == H_MISSION_STOPPING)) return false;
    if ((mode != H_MISSION_MODE_H2) && (mode != H_MISSION_MODE_H4) &&
        (mode != H_MISSION_MODE_H5)) return false;
    mission.selected_mode = mode;
    if ((mission.state == H_MISSION_COMPLETE) ||
        (mission.state == H_MISSION_ABORTED)) {
        mission.state = H_MISSION_IDLE;
        mission.stop_reason = H_MISSION_STOP_NONE;
    }
    return true;
}

HMissionMode_t HMission_SelectedMode(void)
{
    return mission.selected_mode;
}

void HMission_Start(HMissionMode_t mode, uint32_t now_ms)
{
    mission.selected_mode = mode;
    mission.running_mode = mode;
    mission.state = H_MISSION_RUNNING;
    mission.stop_reason = H_MISSION_STOP_NONE;
    mission.started_ms = now_ms;
    mission.latched_ms = 0U;
    mission.elapsed_ms = 0U;
    mission.distance_mm = 0.0f;
    mission.final_left_target_mm_s = 0.0f;
    mission.final_right_target_mm_s = 0.0f;
    mission.white_ms = 0U;
    mission.outer_ms = 0U;
    mission.saturation_ms = 0U;
}

void HMission_Update(uint32_t now_ms, float distance_mm,
                     float left_target_mm_s, float right_target_mm_s,
                     uint16_t white_ms, uint16_t outer_ms,
                     uint16_t saturation_ms)
{
    mission.distance_mm = distance_mm;
    mission.final_left_target_mm_s = left_target_mm_s;
    mission.final_right_target_mm_s = right_target_mm_s;
    mission.white_ms = white_ms;
    mission.outer_ms = outer_ms;
    mission.saturation_ms = saturation_ms;
    if (mission.latched_ms == 0U)
        mission.elapsed_ms = (uint32_t)(now_ms - mission.started_ms);
}

void HMission_Latch(uint32_t now_ms, HMissionStopReason_t reason)
{
    if (mission.state != H_MISSION_RUNNING) return;
    mission.latched_ms = now_ms;
    mission.elapsed_ms = (uint32_t)(now_ms - mission.started_ms);
    mission.stop_reason = reason;
    mission.state = H_MISSION_COMPLETION_LATCHED;
}

void HMission_BeginStopping(void)
{
    if (mission.state == H_MISSION_COMPLETION_LATCHED)
        mission.state = H_MISSION_STOPPING;
}

void HMission_Complete(void)
{
    if ((mission.state == H_MISSION_STOPPING) ||
        (mission.state == H_MISSION_COMPLETION_LATCHED))
        mission.state = H_MISSION_COMPLETE;
}

void HMission_Abort(HMissionStopReason_t reason)
{
    mission.stop_reason = reason;
    mission.state = H_MISSION_ABORTED;
}

bool HMission_IsActive(void)
{
    return (mission.state == H_MISSION_RUNNING) ||
        (mission.state == H_MISSION_COMPLETION_LATCHED) ||
        (mission.state == H_MISSION_STOPPING);
}

bool HMission_IsConfigMutable(void)
{
    return !HMission_IsActive();
}

bool HMission_Stage(const HMissionConfig_t *config)
{
    if (!HMission_IsConfigMutable() || !HMission_ConfigValid(config))
        return false;
    mission.staged = *config;
    mission.stage_revision++;
    mission.dirty = memcmp(&mission.active, &mission.staged,
                           sizeof(mission.active)) != 0;
    return true;
}

bool HMission_Apply(void)
{
    if (!HMission_IsConfigMutable() ||
        !HMission_ConfigValid(&mission.staged)) return false;
    mission.active = mission.staged;
    mission.active_revision++;
    mission.dirty = false;
    return true;
}

bool HMission_Revert(void)
{
    if (!HMission_IsConfigMutable()) return false;
    mission.staged = mission.active;
    mission.stage_revision++;
    mission.dirty = false;
    return true;
}

void HMission_GetSnapshot(HMissionSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = mission;
}
