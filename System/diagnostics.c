#include "diagnostics.h"

#include "control_config.h"
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
#include "wheel_speed.h"

#include <stdbool.h>
#include <string.h>

#define GRAY_ADC_RESPONSE_DELTA 128U

typedef enum {
    MOTOR_DIAG_IDLE = 0,
    MOTOR_DIAG_ARMED,
    MOTOR_DIAG_RUNNING
} MotorDiagState_t;

typedef enum {
    SPEED_DIAG_IDLE = 0,
    SPEED_DIAG_ARMED,
    SPEED_DIAG_RUNNING,
    SPEED_DIAG_RAMP_DOWN
} SpeedDiagState_t;

#define SPEED_DIAG_LEFT_MASK  (1U << WHEEL_SPEED_LEFT)
#define SPEED_DIAG_RIGHT_MASK (1U << WHEEL_SPEED_RIGHT)

static DiagReport_t reports[DIAG_PERIPHERAL_COUNT];
static bool motor_test_passed;
static bool oled_visual_passed;
static bool gray_contrast_observed;
static bool boot_reported;
static uint32_t boot_started_ms;
static MotorDiagState_t motor_diag_state;
static uint32_t motor_diag_deadline_ms;
static uint8_t motor_diag_phase;
static bool motor_diag_driving;
static EncoderSnapshot_t motor_diag_before;
static bool gray_print_pending;
static uint32_t gray_requested_scan_count;
static bool report_pending;
static uint8_t report_line;
static SpeedDiagState_t speed_diag_state;
static uint8_t speed_diag_wheel_mask;
static uint32_t speed_diag_started_ms;
static uint32_t speed_diag_deadline_ms;
static float speed_diag_target_mm_s;
static float speed_diag_error_sum[WHEEL_SPEED_COUNT];
static float speed_diag_previous_error[WHEEL_SPEED_COUNT];
static uint32_t speed_diag_samples[WHEEL_SPEED_COUNT];
static uint16_t speed_diag_sign_changes[WHEEL_SPEED_COUNT];

#ifdef M2_FAULT_INJECTION_TEST
static bool fault_injection_stop_watchdog;
#endif

static void Diagnostics_Set(DiagPeripheral_t peripheral, DiagStatus_t status,
    uint32_t samples, uint32_t errors, const char *detail)
{
    reports[peripheral].status = status;
    reports[peripheral].sample_count = samples;
    reports[peripheral].error_count = errors;
    reports[peripheral].detail = detail;
}

static void Diagnostics_RefreshReports(void)
{
    MotorStats_t motor;
    EncoderSnapshot_t encoder;
    GyroSnapshot_t gyro;
    GrayscaleSnapshot_t gray;
    OLED_Stats_t oled;
    DebugStats_t debug;

    Motor_GetStats(&motor);
    Encoder_GetSnapshot(&encoder);
    Gyro_GetSnapshot(&gyro);
    Grayscale_GetSnapshot(&gray);
    OLED_GetStats(&oled);
    debug_get_stats(&debug);

    Diagnostics_Set(DIAG_MOTOR,
        motor_test_passed ? DIAG_STATUS_OK : DIAG_STATUS_DEGRADED,
        motor.command_count, motor.clamp_count,
        motor_test_passed ? "encoder response observed" : "manual test not run");
    Diagnostics_Set(DIAG_ENCODER,
        (encoder.edge_count_a > 0U && encoder.edge_count_b > 0U) ?
            DIAG_STATUS_OK : DIAG_STATUS_DEGRADED,
        encoder.edge_count_a + encoder.edge_count_b, 0U,
        (encoder.edge_count_a > 0U && encoder.edge_count_b > 0U) ?
            "both channels active" : "rotate both wheels or run motor test");
    Diagnostics_Set(DIAG_GYRO,
        (gyro.valid_frames > 0U) ? DIAG_STATUS_OK : DIAG_STATUS_FAULT,
        gyro.valid_frames, gyro.checksum_errors + gyro.frame_errors,
        (gyro.valid_frames > 0U) ? "valid JY61P angle frames" :
                                  "no valid 0x55 0x53 frame");
    Diagnostics_Set(DIAG_GRAYSCALE,
        (gray.error_count > 0U) ? DIAG_STATUS_FAULT :
            (gray_contrast_observed ? DIAG_STATUS_OK : DIAG_STATUS_DEGRADED),
        gray.scan_count, gray.error_count,
        (gray.error_count > 0U) ? "address/ADC scan timeout" :
            (gray_contrast_observed ? "ADC optical contrast observed" :
                ((gray.scan_count > 0U) ? "ADC contrast below threshold" :
                                          "waiting for scheduled ADC scan")));
    Diagnostics_Set(DIAG_OLED,
        !oled.present ? DIAG_STATUS_FAULT :
            ((oled.ack_errors > 0U || !oled_visual_passed) ?
                DIAG_STATUS_DEGRADED : DIAG_STATUS_OK),
        oled.refresh_count, oled.ack_errors,
        !oled.present ? "no ACK at 0x3C/0x3D" :
            (oled_visual_passed ? "pattern confirmed" : "visual check pending"));
    Diagnostics_Set(DIAG_LED, DIAG_STATUS_DEGRADED, 0U, LED_GetErrorCount(),
        "no MCU-controlled LED populated");
    Diagnostics_Set(DIAG_COMMUNICATION,
        (debug.rx_bytes > 0U) ? DIAG_STATUS_OK : DIAG_STATUS_DEGRADED,
        debug.rx_bytes + debug.tx_bytes,
        debug.format_errors + debug.rx_dropped + debug.tx_dropped,
        (debug.rx_bytes > 0U) ? "interrupt RX/TX active" : "waiting for RX command");
}

static void Diagnostics_PrintEncoder(void)
{
    EncoderSnapshot_t s;
    Encoder_GetSnapshot(&s);
    debug_printf("ENC countA=%ld countB=%ld edgeA=%lu edgeB=%lu\r\n",
        (long)s.count_a, (long)s.count_b, (unsigned long)s.edge_count_a,
        (unsigned long)s.edge_count_b);
}

static float Diagnostics_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t Diagnostics_Round(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint32_t Diagnostics_ScalePositive(float value, float scale)
{
    if (value <= 0.0f) return 0U;
    return (uint32_t)((value * scale) + 0.5f);
}

static const char *Diagnostics_SpeedSelectionName(void)
{
    if (speed_diag_wheel_mask == SPEED_DIAG_LEFT_MASK) return "LEFT";
    if (speed_diag_wheel_mask == SPEED_DIAG_RIGHT_MASK) return "RIGHT";
    return "BOTH";
}

static void Diagnostics_PrintWheelSpeed(void)
{
    WheelSpeedSnapshot_t s;
    float kp_a;
    float ki_a;
    float kp_b;
    float ki_b;
    WheelSpeed_GetSnapshot(&s);
    WheelSpeed_GetGains(WHEEL_SPEED_LEFT, &kp_a, &ki_a);
    WheelSpeed_GetGains(WHEEL_SPEED_RIGHT, &kp_b, &ki_b);
    debug_printf("WHEEL enabled=%u valid=%u target=%ld,%ld ramp=%ld,%ld speed=%ld,%ld raw=%ld,%ld pwm=%d,%d faults=0x%02lX invalid=%lu\r\n",
        s.enabled, s.sample_valid,
        (long)Diagnostics_Round(s.requested_mm_s[WHEEL_SPEED_LEFT]),
        (long)Diagnostics_Round(s.requested_mm_s[WHEEL_SPEED_RIGHT]),
        (long)Diagnostics_Round(s.ramped_mm_s[WHEEL_SPEED_LEFT]),
        (long)Diagnostics_Round(s.ramped_mm_s[WHEEL_SPEED_RIGHT]),
        (long)Diagnostics_Round(s.filtered_mm_s[WHEEL_SPEED_LEFT]),
        (long)Diagnostics_Round(s.filtered_mm_s[WHEEL_SPEED_RIGHT]),
        (long)Diagnostics_Round(s.raw_mm_s[WHEEL_SPEED_LEFT]),
        (long)Diagnostics_Round(s.raw_mm_s[WHEEL_SPEED_RIGHT]),
        s.pwm[WHEEL_SPEED_LEFT], s.pwm[WHEEL_SPEED_RIGHT],
        (unsigned long)s.fault_bits,
        (unsigned long)s.invalid_sample_count);
    debug_printf("TUNE selection=%s test=%ldmm/s A(kp_x100=%lu ki_x100=%lu) B(kp_x100=%lu ki_x100=%lu) filter_x100=%lu cpr=%lu/%lu wheel_x10=%lu\r\n",
        Diagnostics_SpeedSelectionName(),
        (long)Diagnostics_Round(speed_diag_target_mm_s),
        (unsigned long)Diagnostics_ScalePositive(kp_a, 100.0f),
        (unsigned long)Diagnostics_ScalePositive(ki_a, 100.0f),
        (unsigned long)Diagnostics_ScalePositive(kp_b, 100.0f),
        (unsigned long)Diagnostics_ScalePositive(ki_b, 100.0f),
        (unsigned long)Diagnostics_ScalePositive(
            WHEEL_SPEED_FILTER_ALPHA, 100.0f),
        (unsigned long)Diagnostics_Round(
            ENCODER_A_COUNTS_PER_WHEEL_REV),
        (unsigned long)Diagnostics_Round(
            ENCODER_B_COUNTS_PER_WHEEL_REV),
        (unsigned long)Diagnostics_ScalePositive(WHEEL_DIAMETER_MM, 10.0f));
}

static void Diagnostics_PrintGyro(void)
{
    static const char hex[] = "0123456789ABCDEF";
    char raw[(sizeof(((GyroSnapshot_t *)0)->recent_bytes) * 2U) + 1U];
    GyroSnapshot_t s;
    Gyro_GetSnapshot(&s);
    debug_printf("GYRO yaw=%.2f pitch=%.2f roll=%.2f valid=%lu age=%lu checksum=%lu frame=%lu\r\n",
        (double)s.sample.yaw, (double)s.sample.pitch, (double)s.sample.roll,
        (unsigned long)s.valid_frames,
        (unsigned long)(SystemTime_Now() - s.last_update_ms),
        (unsigned long)s.checksum_errors, (unsigned long)s.frame_errors);
    for (uint8_t i = 0U; i < s.recent_count; i++) {
        raw[i * 2U] = hex[s.recent_bytes[i] >> 4U];
        raw[i * 2U + 1U] = hex[s.recent_bytes[i] & 0x0FU];
    }
    raw[s.recent_count * 2U] = '\0';
    debug_printf("GYRO raw=%s\r\n", raw);
}

static void Diagnostics_PrintGraySnapshot(void)
{
    GrayscaleSnapshot_t s;
    uint16_t min_value;
    uint16_t max_value;
    Grayscale_GetSnapshot(&s);
    min_value = max_value = s.adc[0];
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if (s.adc[i] < min_value) min_value = s.adc[i];
        if (s.adc[i] > max_value) max_value = s.adc[i];
    }
    if ((uint16_t)(max_value - min_value) >= GRAY_ADC_RESPONSE_DELTA)
        gray_contrast_observed = true;
    debug_printf("GRAY adc=[%u,%u,%u,%u,%u,%u,%u,%u] span=%u scans=%lu errors=%lu pending=%u\r\n",
        s.adc[0], s.adc[1], s.adc[2], s.adc[3],
        s.adc[4], s.adc[5], s.adc[6], s.adc[7],
        (uint16_t)(max_value - min_value), (unsigned long)s.scan_count,
        (unsigned long)s.error_count, Grayscale_IsScanPending());
    Diagnostics_RefreshReports();
}

static void Diagnostics_PrintCommunication(void)
{
    DebugStats_t s;
    debug_get_stats(&s);
    debug_printf("COMM tx=%lu rx=%lu tx_drop=%lu rx_drop=%lu format=%lu baud=115200 8N1\r\n",
        (unsigned long)s.tx_bytes, (unsigned long)s.rx_bytes,
        (unsigned long)s.tx_dropped, (unsigned long)s.rx_dropped,
        (unsigned long)s.format_errors);
}

static void Diagnostics_PrintSafety(void)
{
    SafetySnapshot_t safety;
    SchedulerStats_t scheduler;
    Safety_GetSnapshot(&safety);
    Scheduler_GetStats(&scheduler);
    debug_printf("SAFETY state=%s faults=0x%08lX first=%lu last=%lu action=%u deadline=%lu pwm=%d,%d\r\n",
        SystemState_Name(SystemState_Get()), (unsigned long)safety.fault_bits,
        (unsigned long)safety.first_fault_ms, (unsigned long)safety.last_fault_ms,
        safety.action_active, (unsigned long)safety.action_deadline_ms,
        safety.applied_motor_a, safety.applied_motor_b);
    debug_printf("CONTEXT target=%d,%d speed=%d,%d requested_pwm=%d,%d\r\n",
        safety.target_speed_left_mm_s, safety.target_speed_right_mm_s,
        safety.measured_speed_left_mm_s, safety.measured_speed_right_mm_s,
        safety.requested_motor_a, safety.requested_motor_b);
    debug_printf("SCHED release=%lu/%lu/%lu/%lu miss=%lu/%lu/%lu/%lu pending=0x%02lX\r\n",
        (unsigned long)scheduler.release_count[0],
        (unsigned long)scheduler.release_count[1],
        (unsigned long)scheduler.release_count[2],
        (unsigned long)scheduler.release_count[3],
        (unsigned long)scheduler.miss_count[0],
        (unsigned long)scheduler.miss_count[1],
        (unsigned long)scheduler.miss_count[2],
        (unsigned long)scheduler.miss_count[3],
        (unsigned long)scheduler.pending_mask);
}

static void Diagnostics_SetMotorPhase(uint32_t now_ms)
{
    static const int16_t pwm_a[4] = {
        DIAGNOSTIC_MOTOR_PWM, -DIAGNOSTIC_MOTOR_PWM, 0, 0
    };
    static const int16_t pwm_b[4] = {
        0, 0, DIAGNOSTIC_MOTOR_PWM, -DIAGNOSTIC_MOTOR_PWM
    };
    if (!Safety_RequestMotor(pwm_a[motor_diag_phase], pwm_b[motor_diag_phase])) {
        motor_diag_state = MOTOR_DIAG_IDLE;
        return;
    }
    motor_diag_driving = true;
    motor_diag_deadline_ms = now_ms + DIAGNOSTIC_MOTOR_PHASE_MS;
}

static void Diagnostics_StartMotorTest(uint32_t now_ms)
{
    if ((motor_diag_state != MOTOR_DIAG_ARMED) ||
        SystemTime_Reached(now_ms, motor_diag_deadline_ms)) {
        debug_println("Motor arm expired or not requested; type m first.");
        motor_diag_state = MOTOR_DIAG_IDLE;
        return;
    }
    if (!SystemState_RequestRun()) {
        debug_println("Motor test rejected: system is not IDLE.");
        motor_diag_state = MOTOR_DIAG_IDLE;
        return;
    }
    Encoder_GetSnapshot(&motor_diag_before);
    Safety_BeginAction(now_ms, DIAGNOSTIC_ACTION_TIMEOUT_MS, 0U);
    motor_diag_state = MOTOR_DIAG_RUNNING;
    motor_diag_phase = 0U;
    debug_println("Motor test running; S/space is immediate emergency stop.");
    Diagnostics_SetMotorPhase(now_ms);
    if (motor_diag_state == MOTOR_DIAG_IDLE) {
        Safety_EndAction();
        (void)SystemState_RequestIdle();
    }
}

static void Diagnostics_FinishMotorTest(void)
{
    EncoderSnapshot_t after;
    Encoder_GetSnapshot(&after);
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    motor_test_passed =
        (after.edge_count_a > motor_diag_before.edge_count_a) &&
        (after.edge_count_b > motor_diag_before.edge_count_b);
    debug_printf("Motor test %s: dEdgeA=%lu dEdgeB=%lu\r\n",
        motor_test_passed ? "OK" : "DEGRADED",
        (unsigned long)(after.edge_count_a - motor_diag_before.edge_count_a),
        (unsigned long)(after.edge_count_b - motor_diag_before.edge_count_b));
    motor_diag_state = MOTOR_DIAG_IDLE;
    Diagnostics_RefreshReports();
}

static void Diagnostics_ServiceMotorTest(uint32_t now_ms)
{
    if (motor_diag_state == MOTOR_DIAG_ARMED) {
        if (SystemTime_Reached(now_ms, motor_diag_deadline_ms)) {
            motor_diag_state = MOTOR_DIAG_IDLE;
            debug_println("Motor test arm timed out.");
        }
        return;
    }
    if (motor_diag_state != MOTOR_DIAG_RUNNING) return;
    if (Safety_HasFault()) {
        Safety_StopMotor();
        motor_diag_state = MOTOR_DIAG_IDLE;
        debug_println("Motor test aborted by safety fault.");
        return;
    }
    if (!SystemTime_Reached(now_ms, motor_diag_deadline_ms)) return;
    if (motor_diag_driving) {
        Safety_StopMotor();
        motor_diag_driving = false;
        motor_diag_deadline_ms = now_ms + DIAGNOSTIC_MOTOR_GAP_MS;
        return;
    }
    motor_diag_phase++;
    if (motor_diag_phase >= 4U) {
        Diagnostics_FinishMotorTest();
    } else {
        Diagnostics_SetMotorPhase(now_ms);
    }
}

static void Diagnostics_ResetSpeedMetrics(void)
{
    memset(speed_diag_error_sum, 0, sizeof(speed_diag_error_sum));
    memset(speed_diag_previous_error, 0,
           sizeof(speed_diag_previous_error));
    memset(speed_diag_samples, 0, sizeof(speed_diag_samples));
    memset(speed_diag_sign_changes, 0,
           sizeof(speed_diag_sign_changes));
}

static void Diagnostics_ArmSpeedTest(uint8_t wheel_mask, uint32_t now_ms)
{
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
        (motor_diag_state != MOTOR_DIAG_IDLE)) {
        debug_println("Speed test rejected: clear faults and return IDLE.");
        return;
    }
    speed_diag_wheel_mask = wheel_mask;
    speed_diag_state = SPEED_DIAG_ARMED;
    speed_diag_deadline_ms = now_ms + DIAGNOSTIC_ARM_TIMEOUT_MS;
    debug_printf("Lift wheels: %s closed-loop %ld mm/s; type uppercase A within 10 s.\r\n",
        Diagnostics_SpeedSelectionName(),
        (long)Diagnostics_Round(speed_diag_target_mm_s));
}

static void Diagnostics_StartSpeedTest(uint32_t now_ms)
{
    float left_target;
    float right_target;
    if ((speed_diag_state != SPEED_DIAG_ARMED) ||
        SystemTime_Reached(now_ms, speed_diag_deadline_ms)) {
        speed_diag_state = SPEED_DIAG_IDLE;
        debug_println("Speed arm expired or not requested.");
        return;
    }
    if (!SystemState_RequestRun()) {
        speed_diag_state = SPEED_DIAG_IDLE;
        debug_println("Speed test rejected: system is not IDLE.");
        return;
    }

    left_target = ((speed_diag_wheel_mask & SPEED_DIAG_LEFT_MASK) != 0U) ?
        speed_diag_target_mm_s : 0.0f;
    right_target = ((speed_diag_wheel_mask & SPEED_DIAG_RIGHT_MASK) != 0U) ?
        speed_diag_target_mm_s : 0.0f;
    Safety_BeginAction(now_ms, SPEED_DIAGNOSTIC_ACTION_TIMEOUT_MS, 0U);
    WheelSpeed_Enable();
    WheelSpeed_SetTarget(left_target, right_target);
    Diagnostics_ResetSpeedMetrics();
    speed_diag_started_ms = now_ms;
    speed_diag_deadline_ms = now_ms + SPEED_DIAGNOSTIC_HOLD_MS;
    speed_diag_state = SPEED_DIAG_RUNNING;
    debug_println("CSV: M3,time_ms,mode,targetL,targetR,rampedL,rampedR,speedL,speedR,pwmL,pwmR,saturated,faults");
    debug_println("Closed-loop test running; S/space is immediate emergency stop.");
}

static void Diagnostics_FinishSpeedTest(void)
{
    uint32_t i;
    WheelSpeed_Disable();
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        if ((speed_diag_wheel_mask & (1U << i)) == 0U) continue;
        if (speed_diag_samples[i] == 0U) {
            debug_printf("M3 RESULT wheel=%c no steady samples\r\n",
                (i == (uint32_t)WHEEL_SPEED_LEFT) ? 'A' : 'B');
        } else {
            float percent = (speed_diag_error_sum[i] * 100.0f) /
                ((float)speed_diag_samples[i] * speed_diag_target_mm_s);
            uint32_t percent_x100 = Diagnostics_ScalePositive(
                percent, 100.0f);
            debug_printf("M3 RESULT wheel=%c mean_abs_error=%lu.%02lu%% samples=%lu sign_changes=%u target=%ldmm/s\r\n",
                (i == (uint32_t)WHEEL_SPEED_LEFT) ? 'A' : 'B',
                (unsigned long)(percent_x100 / 100U),
                (unsigned long)(percent_x100 % 100U),
                (unsigned long)speed_diag_samples[i],
                speed_diag_sign_changes[i],
                (long)Diagnostics_Round(speed_diag_target_mm_s));
        }
    }
    speed_diag_state = SPEED_DIAG_IDLE;
}

static void Diagnostics_ServiceSpeedTest(uint32_t now_ms)
{
    WheelSpeedSnapshot_t speed_snapshot;
    if (speed_diag_state == SPEED_DIAG_ARMED) {
        if (SystemTime_Reached(now_ms, speed_diag_deadline_ms)) {
            speed_diag_state = SPEED_DIAG_IDLE;
            debug_println("Speed test arm timed out.");
        }
        return;
    }
    if ((speed_diag_state != SPEED_DIAG_RUNNING) &&
        (speed_diag_state != SPEED_DIAG_RAMP_DOWN)) return;
    if (Safety_HasFault()) {
        WheelSpeed_Disable();
        speed_diag_state = SPEED_DIAG_IDLE;
        debug_println("Speed test aborted by safety fault.");
        return;
    }
    if ((speed_diag_state == SPEED_DIAG_RUNNING) &&
        SystemTime_Reached(now_ms, speed_diag_deadline_ms)) {
        WheelSpeed_SetTarget(0.0f, 0.0f);
        speed_diag_state = SPEED_DIAG_RAMP_DOWN;
        speed_diag_deadline_ms = now_ms +
            SPEED_DIAGNOSTIC_STOP_TIMEOUT_MS;
        return;
    }
    if (speed_diag_state != SPEED_DIAG_RAMP_DOWN) return;
    WheelSpeed_GetSnapshot(&speed_snapshot);
    if (((Diagnostics_Abs(speed_snapshot.ramped_mm_s[WHEEL_SPEED_LEFT]) <
          0.5f) &&
         (Diagnostics_Abs(speed_snapshot.ramped_mm_s[WHEEL_SPEED_RIGHT]) <
          0.5f) &&
         (Diagnostics_Abs(speed_snapshot.filtered_mm_s[WHEEL_SPEED_LEFT]) <
          20.0f) &&
         (Diagnostics_Abs(speed_snapshot.filtered_mm_s[WHEEL_SPEED_RIGHT]) <
          20.0f)) ||
        SystemTime_Reached(now_ms, speed_diag_deadline_ms)) {
        Diagnostics_FinishSpeedTest();
    }
}

static void Diagnostics_RestartSpeedMeasurement(uint32_t now_ms)
{
    Diagnostics_ResetSpeedMetrics();
    speed_diag_started_ms = now_ms;
    speed_diag_deadline_ms = now_ms + SPEED_DIAGNOSTIC_HOLD_MS;
    Safety_BeginAction(now_ms, SPEED_DIAGNOSTIC_ACTION_TIMEOUT_MS, 0U);
}

static void Diagnostics_AdjustSpeedTarget(float delta, uint32_t now_ms)
{
    float left_target;
    float right_target;
    if (speed_diag_state == SPEED_DIAG_RAMP_DOWN) {
        debug_println("M3 tuning rejected while stopping.");
        return;
    }
    speed_diag_target_mm_s += delta;
    if (speed_diag_target_mm_s < 50.0f) speed_diag_target_mm_s = 50.0f;
    if (speed_diag_target_mm_s > WHEEL_SPEED_MAX_TARGET_MM_S)
        speed_diag_target_mm_s = WHEEL_SPEED_MAX_TARGET_MM_S;
    if (speed_diag_state == SPEED_DIAG_RUNNING) {
        left_target = ((speed_diag_wheel_mask & SPEED_DIAG_LEFT_MASK) != 0U) ?
            speed_diag_target_mm_s : 0.0f;
        right_target = ((speed_diag_wheel_mask & SPEED_DIAG_RIGHT_MASK) != 0U) ?
            speed_diag_target_mm_s : 0.0f;
        WheelSpeed_SetTarget(left_target, right_target);
        Diagnostics_RestartSpeedMeasurement(now_ms);
    }
    debug_printf("M3 test target=%ld mm/s\r\n",
        (long)Diagnostics_Round(speed_diag_target_mm_s));
}

static void Diagnostics_AdjustSpeedGain(bool proportional, float delta,
                                        uint32_t now_ms)
{
    uint32_t i;
    if (speed_diag_state == SPEED_DIAG_RAMP_DOWN) {
        debug_println("M3 tuning rejected while stopping.");
        return;
    }
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        float kp;
        float ki;
        if ((speed_diag_wheel_mask & (1U << i)) == 0U) continue;
        WheelSpeed_GetGains((WheelSpeedWheel_t)i, &kp, &ki);
        if (proportional) kp += delta;
        else ki += delta;
        if (kp < 0.0f) kp = 0.0f;
        if (ki < 0.0f) ki = 0.0f;
        WheelSpeed_SetGains((WheelSpeedWheel_t)i, kp, ki);
    }
    if (speed_diag_state == SPEED_DIAG_RUNNING)
        Diagnostics_RestartSpeedMeasurement(now_ms);
    Diagnostics_PrintWheelSpeed();
}

void Diagnostics_Init(void)
{
    memset(reports, 0, sizeof(reports));
    motor_test_passed = false;
    oled_visual_passed = false;
    gray_contrast_observed = false;
    boot_reported = false;
    boot_started_ms = SystemTime_Now();
    motor_diag_state = MOTOR_DIAG_IDLE;
    gray_print_pending = false;
    report_pending = false;
    report_line = 0U;
    speed_diag_state = SPEED_DIAG_IDLE;
    speed_diag_wheel_mask = SPEED_DIAG_LEFT_MASK |
                            SPEED_DIAG_RIGHT_MASK;
    speed_diag_target_mm_s = SPEED_DIAGNOSTIC_DEFAULT_MM_S;
    Diagnostics_ResetSpeedMetrics();
#ifdef M2_FAULT_INJECTION_TEST
    fault_injection_stop_watchdog = false;
#endif
    Safety_StopMotor();
}

void Diagnostics_RunBootSelfTest(void)
{
    debug_println("\r\n=== MSPM0 CAR M3 DUAL WHEEL SPEED CONTROL ===");
#ifdef M2_FAULT_INJECTION_TEST
    debug_println("*** FAULT INJECTION BUILD - ZERO PWM TESTS ONLY ***");
#endif
    debug_println("Boot self-test settling; motor output is safety-locked.");
    Diagnostics_PrintHelp();
}

void Diagnostics_PrintHelp(void)
{
    debug_println("Commands: h=help s=status d=all e=encoder y=gyro q=grayscale");
    debug_println("          o=OLED pattern p/f=confirm l=LED c=comm m=motor A=arm");
    debug_println("          j=left k=right b=both speed test; n=wheel snapshot");
    debug_println("          +/- target, [/]=Kp, ,/.=Ki for selected wheel(s)");
    debug_println("          v=safety x=clear fault; S or space=immediate e-stop");
#ifdef M2_FAULT_INJECTION_TEST
    debug_println("TEST ONLY: T=action timeout G=gyro disconnect W=stop watchdog");
#endif
}

#ifdef M2_FAULT_INJECTION_TEST
static void Diagnostics_StartZeroPwmFaultTest(uint32_t now_ms,
    uint32_t timeout_ms, uint32_t required_sensor_mask, const char *message)
{
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault()) {
        debug_println("Fault test rejected: clear faults and return IDLE.");
        return;
    }
    if (!SystemState_RequestRun()) {
        debug_println("Fault test rejected: cannot enter RUN.");
        return;
    }
    Safety_BeginAction(now_ms, timeout_ms, required_sensor_mask);
    if (!Safety_RequestMotor(0, 0)) {
        Safety_EndAction();
        (void)SystemState_RequestIdle();
        debug_println("Fault test rejected by safety layer.");
        return;
    }
    debug_println(message);
}
#endif

void Diagnostics_PrintReport(void)
{
    if (report_pending) {
        debug_println("Status report already queued.");
        return;
    }
    Diagnostics_RefreshReports();
    report_line = 0U;
    report_pending = true;
}

static void Diagnostics_ServiceReport(void)
{
    uint8_t peripheral;
    if (!report_pending) return;

    if (report_line == 0U) {
        debug_println("--- SELF TEST ---");
    } else if (report_line <= (uint8_t)DIAG_PERIPHERAL_COUNT) {
        peripheral = (uint8_t)(report_line - 1U);
        debug_printf("%-10s %-8s samples=%lu errors=%lu %s\r\n",
            Diag_PeripheralName((DiagPeripheral_t)peripheral),
            Diag_StatusName(reports[peripheral].status),
            (unsigned long)reports[peripheral].sample_count,
            (unsigned long)reports[peripheral].error_count,
            reports[peripheral].detail);
    } else if (report_line ==
               ((uint8_t)DIAG_PERIPHERAL_COUNT + 1U)) {
        debug_printf("OVERALL    %s\r\n", Diag_StatusName(
            Diag_WorstStatus(reports, (uint8_t)DIAG_PERIPHERAL_COUNT)));
    } else {
        Diagnostics_PrintSafety();
        report_pending = false;
        return;
    }
    report_line++;
}

void Diagnostics_Poll(void)
{
    uint32_t now_ms = SystemTime_Now();
    GrayscaleSnapshot_t gray;
    Diagnostics_ServiceMotorTest(now_ms);
    Diagnostics_ServiceSpeedTest(now_ms);

    if (gray_print_pending) {
        Grayscale_GetSnapshot(&gray);
        if (gray.scan_count != gray_requested_scan_count) {
            gray_print_pending = false;
            Diagnostics_PrintGraySnapshot();
        }
    }

    if (!debug_kbhit()) return;
    switch (debug_getc()) {
        case 'h': case 'H': Diagnostics_PrintHelp(); break;
        case 's': Diagnostics_PrintReport(); break;
        case 'd': case 'D':
            Diagnostics_PrintEncoder(); Diagnostics_PrintGyro();
            Diagnostics_PrintGraySnapshot(); Diagnostics_PrintCommunication();
            Diagnostics_PrintWheelSpeed(); Diagnostics_PrintSafety(); break;
        case 'e': case 'E': Diagnostics_PrintEncoder(); break;
        case 'y': case 'Y': Diagnostics_PrintGyro(); break;
        case 'q': case 'Q':
            Grayscale_GetSnapshot(&gray);
            gray_requested_scan_count = gray.scan_count;
            gray_print_pending = true;
            (void)Grayscale_RequestScan();
            debug_println("GRAY scan requested (non-blocking)."); break;
        case 'o': case 'O':
            if (OLED_IsPresent()) {
                OLED_ShowTestPattern();
                debug_println("OLED refresh queued; wait about 2 s, then type p=pass or f=fail.");
            } else debug_println("OLED FAULT: no ACK.");
            break;
        case 'p': oled_visual_passed = true; break;
        case 'f': case 'F': oled_visual_passed = false; break;
        case 'l': case 'L':
            debug_printf("LED configured=%u errors=%lu\r\n",
                LED_GetConfiguredCount(), (unsigned long)LED_GetErrorCount()); break;
        case 'c': case 'C': Diagnostics_PrintCommunication(); break;
        case 'v': case 'V': Diagnostics_PrintSafety(); break;
        case 'n': case 'N': Diagnostics_PrintWheelSpeed(); break;
        case 'j': case 'J':
            Diagnostics_ArmSpeedTest(SPEED_DIAG_LEFT_MASK, now_ms); break;
        case 'k': case 'K':
            Diagnostics_ArmSpeedTest(SPEED_DIAG_RIGHT_MASK, now_ms); break;
        case 'b': case 'B':
            Diagnostics_ArmSpeedTest(SPEED_DIAG_LEFT_MASK |
                                     SPEED_DIAG_RIGHT_MASK, now_ms); break;
        case '+': Diagnostics_AdjustSpeedTarget(
                      SPEED_DIAGNOSTIC_STEP_MM_S, now_ms); break;
        case '-': Diagnostics_AdjustSpeedTarget(
                      -SPEED_DIAGNOSTIC_STEP_MM_S, now_ms); break;
        case '[': Diagnostics_AdjustSpeedGain(true, -0.05f, now_ms); break;
        case ']': Diagnostics_AdjustSpeedGain(true, 0.05f, now_ms); break;
        case ',': Diagnostics_AdjustSpeedGain(false, -0.25f, now_ms); break;
        case '.': Diagnostics_AdjustSpeedGain(false, 0.25f, now_ms); break;
        case 'm': case 'M':
            if ((SystemState_Get() != SYSTEM_STATE_IDLE) ||
                Safety_HasFault() ||
                (speed_diag_state != SPEED_DIAG_IDLE)) {
                debug_println("Motor test rejected: clear faults and return IDLE.");
            } else {
                motor_diag_state = MOTOR_DIAG_ARMED;
                motor_diag_deadline_ms = now_ms + DIAGNOSTIC_ARM_TIMEOUT_MS;
                debug_println("Lift wheels, then type uppercase A within 10 s.");
            }
            break;
        case 'A':
            if (speed_diag_state == SPEED_DIAG_ARMED)
                Diagnostics_StartSpeedTest(now_ms);
            else
                Diagnostics_StartMotorTest(now_ms);
            break;
#ifdef M2_FAULT_INJECTION_TEST
        case 'T':
            Diagnostics_StartZeroPwmFaultTest(now_ms, 200U, 0U,
                "TEST action timeout armed: zero PWM, expect fault 0x08.");
            break;
        case 'G':
            Diagnostics_StartZeroPwmFaultTest(now_ms, 5000U,
                SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO),
                "TEST gyro dependency armed: zero PWM, expect fault 0x02 after disconnect.");
            break;
        case 'W':
            Safety_StopMotor();
            fault_injection_stop_watchdog = true;
            debug_println("TEST watchdog service stopped: zero PWM, expect reset 0x20.");
            break;
#endif
        case 'x': case 'X':
            if (Safety_HasFault()) WheelSpeed_Disable();
            if (Safety_ClearFaults(now_ms)) {
                WheelSpeed_ClearFaults();
                speed_diag_state = SPEED_DIAG_IDLE;
                motor_diag_state = MOTOR_DIAG_IDLE;
                debug_println("Faults cleared; state=IDLE.");
            } else {
                debug_println("Fault clear rejected.");
            }
            break;
        case '\r': case '\n': break;
        default: debug_println("Unknown command; h=help."); break;
    }
}

void Diagnostics_Task100ms(void)
{
    uint32_t now_ms = SystemTime_Now();
    if (!boot_reported &&
        ((uint32_t)(now_ms - boot_started_ms) >= BOOT_SELF_TEST_SETTLE_MS)) {
        boot_reported = true;
        Diagnostics_PrintReport();
        if (!Safety_HasFault()) (void)SystemState_CompleteSelfTest(true);
    }
    Diagnostics_ServiceReport();
    if ((speed_diag_state == SPEED_DIAG_RUNNING) ||
        (speed_diag_state == SPEED_DIAG_RAMP_DOWN)) {
        WheelSpeedSnapshot_t wheel_snapshot;
        SafetySnapshot_t safety_snapshot;
        uint32_t i;
        uint32_t saturated;
        WheelSpeed_GetSnapshot(&wheel_snapshot);
        Safety_GetSnapshot(&safety_snapshot);
        saturated = (wheel_snapshot.saturated[WHEEL_SPEED_LEFT] ? 1U : 0U) |
            (wheel_snapshot.saturated[WHEEL_SPEED_RIGHT] ? 2U : 0U);
        debug_printf("M3,%lu,%s,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%lu,0x%08lX\r\n",
            (unsigned long)now_ms,
            (speed_diag_state == SPEED_DIAG_RUNNING) ? "RUN" : "STOP",
            (long)Diagnostics_Round(
                wheel_snapshot.requested_mm_s[WHEEL_SPEED_LEFT]),
            (long)Diagnostics_Round(
                wheel_snapshot.requested_mm_s[WHEEL_SPEED_RIGHT]),
            (long)Diagnostics_Round(
                wheel_snapshot.ramped_mm_s[WHEEL_SPEED_LEFT]),
            (long)Diagnostics_Round(
                wheel_snapshot.ramped_mm_s[WHEEL_SPEED_RIGHT]),
            (long)Diagnostics_Round(
                wheel_snapshot.filtered_mm_s[WHEEL_SPEED_LEFT]),
            (long)Diagnostics_Round(
                wheel_snapshot.filtered_mm_s[WHEEL_SPEED_RIGHT]),
            wheel_snapshot.pwm[WHEEL_SPEED_LEFT],
            wheel_snapshot.pwm[WHEEL_SPEED_RIGHT],
            (unsigned long)saturated,
            (unsigned long)safety_snapshot.fault_bits);

        if ((speed_diag_state == SPEED_DIAG_RUNNING) &&
            ((uint32_t)(now_ms - speed_diag_started_ms) >=
             SPEED_DIAGNOSTIC_SETTLE_MS)) {
            for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
                float error;
                float deadband;
                if ((speed_diag_wheel_mask & (1U << i)) == 0U) continue;
                error = speed_diag_target_mm_s -
                        wheel_snapshot.filtered_mm_s[i];
                deadband = speed_diag_target_mm_s * 0.05f;
                speed_diag_error_sum[i] += Diagnostics_Abs(error);
                speed_diag_samples[i]++;
                if ((Diagnostics_Abs(error) > deadband) &&
                    (speed_diag_previous_error[i] != 0.0f) &&
                    ((error * speed_diag_previous_error[i]) < 0.0f)) {
                    speed_diag_sign_changes[i]++;
                }
                if (Diagnostics_Abs(error) > deadband)
                    speed_diag_previous_error[i] = error;
            }
        }
    }
}

const DiagReport_t *Diagnostics_GetReports(void)
{
    return reports;
}

#ifdef M2_FAULT_INJECTION_TEST
bool Diagnostics_FaultInjectionStopWatchdog(void)
{
    return fault_injection_stop_watchdog;
}
#endif
