#include "h_mission.h"

#include <assert.h>

int main(void)
{
    HMissionSnapshot_t snapshot;
    HMissionConfig_t config;

    HMission_Init();
    HMission_GetSnapshot(&snapshot);
    assert(snapshot.selected_mode == H_MISSION_MODE_H2);
    assert(snapshot.active.h4_straight_speed_mm_s == 220.0f);
    assert(snapshot.active.h4_b_margin_mm == 0.0f);
    assert(snapshot.active.h5_straight_speed_mm_s == 230.0f);
    assert(snapshot.active.h5_curve_speed_mm_s == 210.0f);
    assert(snapshot.active_revision == 1U);
    assert(!snapshot.dirty);

    assert(HMission_Select(H_MISSION_MODE_H4));
    config = snapshot.active;
    config.h4_straight_speed_mm_s = 200.0f;
    config.development_end_distance_mm = 500.0f;
    assert(HMission_Stage(&config));
    HMission_GetSnapshot(&snapshot);
    assert(snapshot.dirty);
    assert(snapshot.stage_revision == 2U);
    assert(snapshot.active.h4_straight_speed_mm_s == 220.0f);
    assert(HMission_Apply());
    HMission_GetSnapshot(&snapshot);
    assert(!snapshot.dirty);
    assert(snapshot.active_revision == 2U);
    assert(snapshot.active.h4_straight_speed_mm_s == 200.0f);

    HMission_Start(H_MISSION_MODE_H4, 1000U);
    assert(!HMission_Select(H_MISSION_MODE_H5));
    assert(!HMission_Stage(&config));
    HMission_Update(1200U, 40.5f, 200.0f, 190.0f, 0U, 0U, 20U);
    HMission_Latch(1300U, H_MISSION_STOP_H4_DISTANCE);
    HMission_BeginStopping();
    HMission_Complete();
    HMission_GetSnapshot(&snapshot);
    assert(snapshot.state == H_MISSION_COMPLETE);
    assert(snapshot.elapsed_ms == 300U);
    assert(snapshot.distance_mm == 40.5f);
    assert(snapshot.stop_reason == H_MISSION_STOP_H4_DISTANCE);

    assert(HMission_Select(H_MISSION_MODE_H5));
    assert(HMission_Revert());
    config = snapshot.active;
    config.h5_curve_speed_mm_s = 351.0f;
    assert(!HMission_Stage(&config));
    return 0;
}
