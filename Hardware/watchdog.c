#include "watchdog.h"

#include "board.h"

void Watchdog_Init(void)
{
    DL_WWDT_enablePower(WWDT0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_WWDT_initWatchdogMode(WWDT0,
        DL_WWDT_CLOCK_DIVIDE_1,
        DL_WWDT_TIMER_PERIOD_10_BITS,
        DL_WWDT_RUN_IN_SLEEP,
        DL_WWDT_WINDOW_PERIOD_0,
        DL_WWDT_WINDOW_PERIOD_0);
    DL_WWDT_setCoreHaltBehavior(WWDT0, DL_WWDT_CORE_HALT_STOP);
}

void Watchdog_Service(void)
{
    DL_WWDT_restart(WWDT0);
}

bool Watchdog_WasResetCause(uint32_t reset_cause)
{
    return (reset_cause ==
        (uint32_t)DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION);
}
