#include "system_time.h"

#include "scheduler.h"

static volatile uint32_t system_time_ms;

void SystemTime_Init(void)
{
    system_time_ms = 0U;
}

uint32_t SystemTime_Now(void)
{
    return system_time_ms;
}

bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0);
}

void SystemTime_TickISR(void)
{
    system_time_ms++;
    Scheduler_TickISR(system_time_ms);
}

void SysTick_Handler(void)
{
    SystemTime_TickISR();
}
