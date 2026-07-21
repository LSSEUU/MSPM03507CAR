#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SCHEDULER_TASK_5MS = 0,
    SCHEDULER_TASK_10MS,
    SCHEDULER_TASK_20MS,
    SCHEDULER_TASK_100MS,
    SCHEDULER_TASK_COUNT
} SchedulerTask_t;

#define SCHEDULER_TASK_BIT(task) (1UL << (uint32_t)(task))
#define SCHEDULER_ALL_TASKS_MASK ((1UL << (uint32_t)SCHEDULER_TASK_COUNT) - 1UL)
#define SCHEDULER_CRITICAL_TASKS_MASK \
    (SCHEDULER_TASK_BIT(SCHEDULER_TASK_5MS) | \
     SCHEDULER_TASK_BIT(SCHEDULER_TASK_10MS) | \
     SCHEDULER_TASK_BIT(SCHEDULER_TASK_20MS))

typedef struct {
    uint32_t release_count[SCHEDULER_TASK_COUNT];
    uint32_t completion_count[SCHEDULER_TASK_COUNT];
    uint32_t miss_count[SCHEDULER_TASK_COUNT];
    uint32_t pending_mask;
    uint32_t missed_mask;
    uint32_t health_window_mask;
} SchedulerStats_t;

void Scheduler_Init(void);
void Scheduler_TickISR(uint32_t now_ms);
bool Scheduler_Take(SchedulerTask_t task);
void Scheduler_Complete(SchedulerTask_t task);
uint32_t Scheduler_TakeMissedMask(void);
bool Scheduler_ConsumeHealthyWindow(void);
void Scheduler_GetStats(SchedulerStats_t *stats);

#endif
