#include "system_state.h"

#include <stdint.h>

#ifndef SYSTEM_STATE_HOST_TEST
#include "ti_msp_dl_config.h"
#endif

static volatile SystemState_t system_state;

static uint32_t SystemState_EnterCritical(void)
{
#ifdef SYSTEM_STATE_HOST_TEST
    return 0U;
#else
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#endif
}

static void SystemState_ExitCritical(uint32_t key)
{
#ifdef SYSTEM_STATE_HOST_TEST
    (void)key;
#else
    if (key == 0U) __enable_irq();
#endif
}

void SystemState_Init(void)
{
    system_state = SYSTEM_STATE_BOOT;
}

SystemState_t SystemState_Get(void)
{
    return system_state;
}

const char *SystemState_Name(SystemState_t state)
{
    static const char *const names[] = {
        "BOOT", "SELF_TEST", "IDLE", "RUN", "FAULT"
    };
    if ((unsigned int)state >= (unsigned int)(sizeof(names) / sizeof(names[0])))
        return "UNKNOWN";
    return names[state];
}

bool SystemState_BeginSelfTest(void)
{
    uint32_t key = SystemState_EnterCritical();
    if (system_state != SYSTEM_STATE_BOOT) {
        SystemState_ExitCritical(key);
        return false;
    }
    system_state = SYSTEM_STATE_SELF_TEST;
    SystemState_ExitCritical(key);
    return true;
}

bool SystemState_CompleteSelfTest(bool passed)
{
    uint32_t key = SystemState_EnterCritical();
    if (system_state != SYSTEM_STATE_SELF_TEST) {
        SystemState_ExitCritical(key);
        return false;
    }
    system_state = passed ? SYSTEM_STATE_IDLE : SYSTEM_STATE_FAULT;
    SystemState_ExitCritical(key);
    return passed;
}

bool SystemState_RequestRun(void)
{
    uint32_t key = SystemState_EnterCritical();
    if (system_state != SYSTEM_STATE_IDLE) {
        SystemState_ExitCritical(key);
        return false;
    }
    system_state = SYSTEM_STATE_RUN;
    SystemState_ExitCritical(key);
    return true;
}

bool SystemState_RequestIdle(void)
{
    uint32_t key = SystemState_EnterCritical();
    if (system_state != SYSTEM_STATE_RUN) {
        SystemState_ExitCritical(key);
        return false;
    }
    system_state = SYSTEM_STATE_IDLE;
    SystemState_ExitCritical(key);
    return true;
}

void SystemState_EnterFault(void)
{
    uint32_t key = SystemState_EnterCritical();
    system_state = SYSTEM_STATE_FAULT;
    SystemState_ExitCritical(key);
}

bool SystemState_ClearFault(void)
{
    uint32_t key = SystemState_EnterCritical();
    if (system_state != SYSTEM_STATE_FAULT) {
        SystemState_ExitCritical(key);
        return false;
    }
    system_state = SYSTEM_STATE_IDLE;
    SystemState_ExitCritical(key);
    return true;
}
