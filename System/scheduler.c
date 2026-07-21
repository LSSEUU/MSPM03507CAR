#include "scheduler.h"

#include "vehicle_config.h"

#ifndef SCHEDULER_HOST_TEST
#include "ti_msp_dl_config.h"
#endif

#include <stddef.h>
#include <string.h>

static volatile uint32_t pending_mask;
static volatile uint32_t missed_mask;
static volatile uint32_t health_window_mask;
static volatile uint32_t health_window_missed_mask;
static volatile uint32_t release_count[SCHEDULER_TASK_COUNT];
static volatile uint32_t completion_count[SCHEDULER_TASK_COUNT];
static volatile uint32_t miss_count[SCHEDULER_TASK_COUNT];

static uint32_t Scheduler_Period(SchedulerTask_t task)
{
    static const uint16_t periods[SCHEDULER_TASK_COUNT] = {
        TASK_5MS_PERIOD_MS,
        TASK_10MS_PERIOD_MS,
        TASK_20MS_PERIOD_MS,
        TASK_100MS_PERIOD_MS
    };
    return periods[task];
}

static uint32_t Scheduler_EnterCritical(void)
{
#ifdef SCHEDULER_HOST_TEST
    return 0U;
#else
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#endif
}

static void Scheduler_ExitCritical(uint32_t key)
{
#ifdef SCHEDULER_HOST_TEST
    (void)key;
#else
    if (key == 0U) __enable_irq();
#endif
}

void Scheduler_Init(void)
{
    pending_mask = 0U;
    missed_mask = 0U;
    health_window_mask = 0U;
    health_window_missed_mask = 0U;
    for (uint32_t i = 0U; i < (uint32_t)SCHEDULER_TASK_COUNT; i++) {
        release_count[i] = 0U;
        completion_count[i] = 0U;
        miss_count[i] = 0U;
    }
}

void Scheduler_TickISR(uint32_t now_ms)
{
    for (uint32_t i = 0U; i < (uint32_t)SCHEDULER_TASK_COUNT; i++) {
        SchedulerTask_t task = (SchedulerTask_t)i;
        uint32_t bit = SCHEDULER_TASK_BIT(task);
        if ((now_ms % Scheduler_Period(task)) != 0U) continue;
        release_count[i]++;
        if ((pending_mask & bit) != 0U) {
            missed_mask |= bit;
            health_window_missed_mask |= bit;
            miss_count[i]++;
        }
        pending_mask |= bit;
    }
}

bool Scheduler_Take(SchedulerTask_t task)
{
    uint32_t bit = SCHEDULER_TASK_BIT(task);
    uint32_t key = Scheduler_EnterCritical();
    bool ready = ((pending_mask & bit) != 0U);
    pending_mask &= ~bit;
    Scheduler_ExitCritical(key);
    return ready;
}

void Scheduler_Complete(SchedulerTask_t task)
{
    uint32_t key = Scheduler_EnterCritical();
    completion_count[task]++;
    health_window_mask |= SCHEDULER_TASK_BIT(task);
    Scheduler_ExitCritical(key);
}

uint32_t Scheduler_TakeMissedMask(void)
{
    uint32_t key = Scheduler_EnterCritical();
    uint32_t result = missed_mask;
    missed_mask = 0U;
    Scheduler_ExitCritical(key);
    return result;
}

bool Scheduler_ConsumeHealthyWindow(void)
{
    uint32_t key = Scheduler_EnterCritical();
    bool healthy =
        ((health_window_mask & SCHEDULER_CRITICAL_TASKS_MASK) ==
            SCHEDULER_CRITICAL_TASKS_MASK) &&
        ((health_window_missed_mask & SCHEDULER_CRITICAL_TASKS_MASK) == 0U);
    health_window_mask &= ~SCHEDULER_CRITICAL_TASKS_MASK;
    health_window_missed_mask &= ~SCHEDULER_CRITICAL_TASKS_MASK;
    Scheduler_ExitCritical(key);
    return healthy;
}

void Scheduler_GetStats(SchedulerStats_t *stats)
{
    uint32_t key;
    if (stats == NULL) return;
    key = Scheduler_EnterCritical();
    memset(stats, 0, sizeof(*stats));
    for (uint32_t i = 0U; i < (uint32_t)SCHEDULER_TASK_COUNT; i++) {
        stats->release_count[i] = release_count[i];
        stats->completion_count[i] = completion_count[i];
        stats->miss_count[i] = miss_count[i];
    }
    stats->pending_mask = pending_mask;
    stats->missed_mask = missed_mask;
    stats->health_window_mask = health_window_mask;
    Scheduler_ExitCritical(key);
}
