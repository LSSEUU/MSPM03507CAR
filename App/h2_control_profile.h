#ifndef H2_CONTROL_PROFILE_H
#define H2_CONTROL_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    uint32_t activation_count;
    uint32_t restore_count;
} H2ControlProfileSnapshot_t;

/* H2 owns a proven, firmware-local controller profile only while H2 runs.
 * The caller's previous module configuration is restored on every exit, so
 * H4/H5 and host-side RAM tuning remain independent. */
void H2ControlProfile_Init(void);
bool H2ControlProfile_Activate(void);
bool H2ControlProfile_Restore(void);
void H2ControlProfile_GetSnapshot(H2ControlProfileSnapshot_t *snapshot);

#endif
