#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdbool.h>

typedef enum {
    SYSTEM_STATE_BOOT = 0,
    SYSTEM_STATE_SELF_TEST,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUN,
    SYSTEM_STATE_FAULT,
    SYSTEM_STATE_CALIBRATION = 5
} SystemState_t;

void SystemState_Init(void);
SystemState_t SystemState_Get(void);
const char *SystemState_Name(SystemState_t state);
bool SystemState_BeginSelfTest(void);
bool SystemState_CompleteSelfTest(bool passed);
bool SystemState_RequestRun(void);
bool SystemState_RequestIdle(void);
bool SystemState_BeginCalibration(void);
bool SystemState_EndCalibration(void);
void SystemState_EnterFault(void);
bool SystemState_ClearFault(void);

#endif
