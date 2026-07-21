#include "board.h"

#include "diagnostics.h"
#include "encoder.h"
#include "grayscale.h"
#include "gyro.h"
#include "led.h"
#include "motor.h"
#include "oled.h"
#include "safety.h"
#include "scheduler.h"
#include "system_state.h"
#include "system_time.h"
#include "uart_debug.h"
#include "vehicle_config.h"
#include "watchdog.h"

static void Main_EmergencyStopCallback(void)
{
    Safety_EmergencyStop(SystemTime_Now());
}

static void Main_CompleteTask(SchedulerTask_t task, uint32_t started_ms,
                              uint32_t deadline_ms)
{
    if ((uint32_t)(SystemTime_Now() - started_ms) >= deadline_ms) {
        Safety_RaiseFault(SAFETY_FAULT_SCHEDULER_DEADLINE, SystemTime_Now());
    }
    Scheduler_Complete(task);
}

int main(void)
{
    uint32_t reset_cause = (uint32_t)DL_SYSCTL_getResetCause();
    uint32_t last_gyro_frame = 0U;
    uint32_t last_gray_scan = 0U;

    SYSCFG_DL_init();
    Motor_Init();

    Scheduler_Init();
    SystemTime_Init();
    SystemState_Init();
    Safety_Init(0U);
    (void)SystemState_BeginSelfTest();

    debug_init();
    debug_set_emergency_callback(Main_EmergencyStopCallback);
    Encoder_Init();
    Grayscale_Init();
    Gyro_Init();
    LED_Init();
    OLED_Init();
    Watchdog_Init();

    DL_SYSTICK_init(SYSTICK_PERIOD);
    DL_SYSTICK_enableInterrupt();
    DL_SYSTICK_enable();

    if (Watchdog_WasResetCause(reset_cause)) {
        Safety_RaiseFault(SAFETY_FAULT_WATCHDOG_RESET, SystemTime_Now());
    }

    Diagnostics_Init();
    Diagnostics_RunBootSelfTest();

    while (1) {
        uint32_t missed = Scheduler_TakeMissedMask();
        if (missed != 0U) {
            Safety_RaiseFault(SAFETY_FAULT_SCHEDULER_DEADLINE,
                              SystemTime_Now());
        }

        Diagnostics_Poll();

        if (Scheduler_Take(SCHEDULER_TASK_5MS)) {
            uint32_t started = SystemTime_Now();
            Safety_Evaluate(started);
            Main_CompleteTask(SCHEDULER_TASK_5MS, started,
                              TASK_5MS_PERIOD_MS);
        }

        if (Scheduler_Take(SCHEDULER_TASK_10MS)) {
            uint32_t started = SystemTime_Now();
            GyroSnapshot_t gyro;
            GrayscaleSnapshot_t gray;

            Gyro_Update();
            Gyro_GetSnapshot(&gyro);
            if (gyro.valid_frames != last_gyro_frame) {
                last_gyro_frame = gyro.valid_frames;
                Safety_NotifySensor(SAFETY_SENSOR_GYRO,
                                    gyro.last_update_ms);
            }

            if (!Grayscale_IsScanPending()) (void)Grayscale_RequestScan();
            Grayscale_Task(started);
            Grayscale_GetSnapshot(&gray);
            if ((gray.scan_count != last_gray_scan) && gray.last_scan_ok) {
                last_gray_scan = gray.scan_count;
                Safety_NotifySensor(SAFETY_SENSOR_GRAYSCALE,
                                    gray.last_update_ms);
            }

            Main_CompleteTask(SCHEDULER_TASK_10MS, started,
                              TASK_10MS_PERIOD_MS);
        }

        if (Scheduler_Take(SCHEDULER_TASK_20MS)) {
            uint32_t started = SystemTime_Now();
            /* Reserved for M3/M4 motion and heading state machines. */
            Main_CompleteTask(SCHEDULER_TASK_20MS, started,
                              TASK_20MS_PERIOD_MS);
            if (Scheduler_ConsumeHealthyWindow()) Watchdog_Service();
        }

        if (Scheduler_Take(SCHEDULER_TASK_100MS)) {
            uint32_t started = SystemTime_Now();
            Diagnostics_Task100ms();
            Main_CompleteTask(SCHEDULER_TASK_100MS, started,
                              TASK_100MS_PERIOD_MS);
        }

        __WFI();
    }
}
