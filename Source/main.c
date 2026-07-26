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
#include "m6_rpc.h"
#include "oled.h"
#include "parameter_store.h"
#include "reset_diag.h"
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
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_EMERGENCY);
}

static void Main_ControlParamsCommitted(const ControlParamsCommit_t *commit)
{
    if (!ControlRuntime_ApplyActive(commit)) {
        Safety_EmergencyStop(SystemTime_Now());
    }
}

static void Main_CompleteTask(SchedulerTask_t task, uint32_t started_ms,
                              uint32_t deadline_ms)
{
    if ((uint32_t)(SystemTime_Now() - started_ms) >= deadline_ms) {
        Safety_RaiseFault(SAFETY_FAULT_SCHEDULER_DEADLINE, SystemTime_Now());
    }
    Scheduler_Complete(task);
}

static void Main_RecordHealthyWindow(uint32_t now_ms,
                                     uint32_t last_watchdog_service_ms)
{
    SchedulerStats_t scheduler;
    SafetySnapshot_t safety;
    DebugStats_t debug;
    M6TelemetryStats_t telemetry;
    ResetDiagHealth_t health = {0};
    uint32_t index;

    Scheduler_GetStats(&scheduler);
    Safety_GetSnapshot(&safety);
    debug_get_stats(&debug);
    M6Telemetry_GetStats(&telemetry);
    health.now_ms = now_ms;
    health.last_watchdog_service_ms = last_watchdog_service_ms;
    for (index = 0U; index < (uint32_t)SCHEDULER_TASK_COUNT; index++)
        health.scheduler_missed_total += scheduler.miss_count[index];
    health.safety_fault_bits = safety.fault_bits;
    health.uart_tx_dropped = debug.tx_dropped;
    health.uart_rx_dropped = debug.rx_dropped;
    health.telemetry_dropped = telemetry.dropped;
    health.scheduler_pending_mask = scheduler.pending_mask;
    health.scheduler_missed_mask = scheduler.missed_mask;
    health.checkpoint = RESET_DIAG_CHECKPOINT_TASK_20MS;
    health.system_state = (uint8_t)SystemState_Get();
    health.telemetry_queued = telemetry.queued;
    health.scheduler_health_mask = (uint8_t)SCHEDULER_CRITICAL_TASKS_MASK;
    ResetDiag_RecordHealthyWindow(&health);
}

int main(void)
{
    uint32_t reset_cause = (uint32_t)DL_SYSCTL_getResetCause();
    uint32_t last_gyro_frame = 0U;
    uint32_t last_gray_scan = 0U;
    uint32_t last_watchdog_service_ms = 0U;

    ResetDiag_EarlyInit(reset_cause);
    SYSCFG_DL_init();
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_INIT);
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
    if (!ControlRuntime_ApplyActive(NULL)) {
        Safety_EmergencyStop(SystemTime_Now());
    }
    M6Telemetry_Init();
    ControlProtocol_Init(Main_ControlParamsCommitted);
    Diagnostics_RunBootSelfTest();

    while (1) {
        uint32_t missed;
        M6TelemetryStats_t background_telemetry;
        missed = Scheduler_TakeMissedMask();
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

            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TASK_5MS);
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

            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TASK_10MS);
            Gyro_Update();
            Gyro_GetSnapshot(&gyro);
            if (gyro.valid_frames != last_gyro_frame) {
                last_gyro_frame = gyro.valid_frames;
                Safety_NotifySensor(SAFETY_SENSOR_GYRO,
                                    gyro.last_update_ms);
            }

            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_GRAYSCALE);
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

            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TASK_20MS);
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
            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TELEMETRY);
            M6Telemetry_Capture20ms(started);
            M6Rpc_Telemetry20ms(started);
            Main_CompleteTask(SCHEDULER_TASK_20MS, started,
                              TASK_20MS_PERIOD_MS);
            if (Scheduler_ConsumeHealthyWindow()) {
                bool service_watchdog = true;
#ifdef M2_FAULT_INJECTION_TEST
                service_watchdog = !Diagnostics_FaultInjectionStopWatchdog();
#endif
                if (service_watchdog) {
                    Watchdog_Service();
                    last_watchdog_service_ms = SystemTime_Now();
                    ResetDiag_Checkpoint(
                        RESET_DIAG_CHECKPOINT_WWDT_SERVICE);
                }
                Main_RecordHealthyWindow(SystemTime_Now(),
                                         last_watchdog_service_ms);
            }
        }

        if (Scheduler_Take(SCHEDULER_TASK_100MS)) {
            uint32_t started = SystemTime_Now();
            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TASK_100MS);
            Diagnostics_Task100ms();
            Main_CompleteTask(SCHEDULER_TASK_100MS, started,
                              TASK_100MS_PERIOD_MS);
        }

        Diagnostics_ServiceBackground();
        M6Telemetry_GetStats(&background_telemetry);
        if (background_telemetry.queued != 0U)
            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TELEMETRY);
        M6Telemetry_Service();
        if (Grayscale_IsScanPending())
            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_GRAYSCALE);
        Grayscale_Task(SystemTime_Now());
        if (OLED_ServicePending())
            ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_OLED);
        OLED_Service();
        ResetDiag_Checkpoint(Safety_HasFault() ?
            RESET_DIAG_CHECKPOINT_SAFETY_FAULT :
            RESET_DIAG_CHECKPOINT_IDLE);
        if (!debug_kbhit()) __WFI();
    }
}
