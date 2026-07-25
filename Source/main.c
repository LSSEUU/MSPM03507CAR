#include "board.h"

#include "app_line.h"
#include "app_remote.h"
#include "control_params.h"
#include "control_protocol.h"
#include "control_runtime.h"
#include "diagnostics.h"
#include "encoder.h"
#include "grayscale.h"
#include "gyro.h"
#include "heading.h"
#include "led.h"
#include "motor.h"
#include "m6_telemetry.h"
#include "oled.h"
#include "parameter_store.h"
#include "safety.h"
#include "scheduler.h"
#include "system_state.h"
#include "system_time.h"
#include "uart_debug.h"
#include "vehicle_config.h"
#include "watchdog.h"
#include "wheel_speed.h"

static void Main_EmergencyStopCallback(void)
{
    M6Telemetry_StopFromISR();
    Safety_EmergencyStop(SystemTime_Now());
}

static void Main_ControlParamsCommitted(const ControlParamsCommit_t *commit)
{
    if (!ControlRuntime_ApplyActive(commit))
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
    ControlParams_Init();
    ParameterStore_Init();

    debug_init();
    debug_set_emergency_callback(Main_EmergencyStopCallback);
    Encoder_Init();
    WheelSpeed_Init();
    Heading_Init();
    AppLine_Init();
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
    AppRemote_Init();
    if (!ControlRuntime_ApplyActive(NULL))
        Safety_EmergencyStop(SystemTime_Now());
    M6Telemetry_Init();
    ControlProtocol_Init(Main_ControlParamsCommitted);
    Diagnostics_RunBootSelfTest();

    while (1) {
        uint32_t missed = Scheduler_TakeMissedMask();
        if (missed != 0U) {
            Safety_RaiseFault(SAFETY_FAULT_SCHEDULER_DEADLINE,
                              SystemTime_Now());
        }

        AppRemote_Poll();
        Diagnostics_Poll();

        if (Scheduler_Take(SCHEDULER_TASK_5MS)) {
            uint32_t started = SystemTime_Now();
            EncoderSnapshot_t encoder;
            WheelSpeedSnapshot_t wheel;

            ControlProtocol_WheelBoundary();
            Encoder_GetSnapshot(&encoder);
            WheelSpeed_Update(encoder.count_a, encoder.count_b, started);
            WheelSpeed_GetSnapshot(&wheel);
            Safety_SetControlContext(
                (int16_t)wheel.ramped_mm_s[WHEEL_SPEED_LEFT],
                (int16_t)wheel.ramped_mm_s[WHEEL_SPEED_RIGHT],
                (int16_t)wheel.filtered_mm_s[WHEEL_SPEED_LEFT],
                (int16_t)wheel.filtered_mm_s[WHEEL_SPEED_RIGHT]);

            if (wheel.enabled && !Safety_HasFault() &&
                ((wheel.fault_bits &
                  WHEEL_SPEED_FAULT_ENCODER_INVALID) != 0U)) {
                WheelSpeed_Disable();
                Safety_RaiseFault(SAFETY_FAULT_ENCODER_INVALID, started);
            } else if (wheel.enabled && !Safety_HasFault() &&
                       ((wheel.fault_bits &
                         WHEEL_SPEED_FAULT_STALL) != 0U)) {
                WheelSpeed_Disable();
                Safety_RaiseFault(SAFETY_FAULT_WHEEL_STALL, started);
            }
            Safety_Evaluate(started);
            if (wheel.enabled && !Safety_HasFault() &&
                !Safety_RequestMotor(wheel.pwm[WHEEL_SPEED_LEFT],
                                     wheel.pwm[WHEEL_SPEED_RIGHT])) {
                WheelSpeed_Disable();
            }
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
                AppLine_ProcessGrayscale(&gray);
                Safety_NotifySensor(SAFETY_SENSOR_GRAYSCALE,
                                    gray.last_update_ms);
            }

            Main_CompleteTask(SCHEDULER_TASK_10MS, started,
                              TASK_10MS_PERIOD_MS);
        }

        if (Scheduler_Take(SCHEDULER_TASK_20MS)) {
            uint32_t started = SystemTime_Now();
            GyroSnapshot_t gyro;
            HeadingSnapshot_t heading;

            AppRemote_Task20ms(started);
            Gyro_GetSnapshot(&gyro);
            Heading_UpdateWithRate(
                gyro.sample.yaw, gyro.valid_frames,
                gyro.sample.yaw_rate_deg_s, gyro.rate_frames,
                gyro.last_rate_update_ms, started);
            Heading_GetSnapshot(&heading);
            if (heading.mode != HEADING_MODE_IDLE) {
                WheelSpeed_SetTarget(heading.left_target_mm_s,
                                     heading.right_target_mm_s);
            }
            AppLine_Task20ms(started);
            M6Telemetry_Capture20ms(started);
            Main_CompleteTask(SCHEDULER_TASK_20MS, started,
                              TASK_20MS_PERIOD_MS);
            if (Scheduler_ConsumeHealthyWindow()) {
#ifdef M2_FAULT_INJECTION_TEST
                if (!Diagnostics_FaultInjectionStopWatchdog())
#endif
                    Watchdog_Service();
            }
        }

        if (Scheduler_Take(SCHEDULER_TASK_100MS)) {
            uint32_t started = SystemTime_Now();
            Diagnostics_Task100ms();
            Main_CompleteTask(SCHEDULER_TASK_100MS, started,
                              TASK_100MS_PERIOD_MS);
        }

        M6Telemetry_Service();
        Grayscale_Task(SystemTime_Now());
        OLED_Service();
        __WFI();
    }
}
