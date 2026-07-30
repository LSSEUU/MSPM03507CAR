#ifndef H_MISSION_CONTROL_PROFILE_H
#define H_MISSION_CONTROL_PROFILE_H

#include "h_mission.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    HMissionMode_t active_mode;
    uint32_t activation_count;
    uint32_t restore_count;
} HMissionControlProfileSnapshot_t;

/* H4/H5 own their verified controller profile only while the selected
 * mission runs.  The previous module configuration is restored on every
 * exit.  This path never changes ControlParams and cannot write Flash. */
void HMissionControlProfile_Init(void);
bool HMissionControlProfile_Activate(HMissionMode_t mode);
bool HMissionControlProfile_Restore(void);
void HMissionControlProfile_GetSnapshot(
    HMissionControlProfileSnapshot_t *snapshot);

#endif
