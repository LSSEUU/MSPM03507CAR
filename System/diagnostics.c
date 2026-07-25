#include "diagnostics.h"

#include "app_line.h"
#include "app_remote.h"
#include "control_config.h"
#include "control_protocol.h"
#include "encoder.h"
#include "grayscale.h"
#include "gyro.h"
#include "heading.h"
#include "ir_receiver.h"
#include "led.h"
#include "m6_telemetry.h"
#include "motor.h"
#include "oled.h"
#include "parameter_store.h"
#include "safety.h"
#include "scheduler.h"
#include "system_state.h"
#include "system_time.h"
#include "uart_debug.h"
#include "vehicle_config.h"
#include "wheel_speed.h"

#include <stdbool.h>
#include <string.h>

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

typedef enum {
    HEADING_DIAG_IDLE = 0,
    HEADING_DIAG_ARMED,
    HEADING_DIAG_RUNNING,
    HEADING_DIAG_RAMP_DOWN
} HeadingDiagState_t;

typedef enum {
    HEADING_DIAG_HOLD = 0,
    HEADING_DIAG_TURN_POSITIVE,
    HEADING_DIAG_TURN_NEGATIVE
} HeadingDiagKind_t;

#define SPEED_DIAG_LEFT_MASK  (1U << WHEEL_SPEED_LEFT)
#define SPEED_DIAG_RIGHT_MASK (1U << WHEEL_SPEED_RIGHT)

static DiagReport_t reports[DIAG_PERIPHERAL_COUNT];
static bool motor_test_passed;
static bool oled_visual_passed;
static bool gray_contrast_observed;
static bool gray_high_observed;
static bool gray_low_observed;
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
static bool infrared_report_pending;
static uint8_t infrared_report_line;
static IrReceiverStats_t infrared_report_receiver;
static AppRemoteSnapshot_t infrared_report_app;
static bool safety_report_pending;
static uint8_t safety_report_line;
static SystemState_t safety_report_system_state;
static SafetySnapshot_t safety_report_snapshot;
static SchedulerStats_t safety_report_scheduler;
static SpeedDiagState_t speed_diag_state;
static uint8_t speed_diag_wheel_mask;
static uint32_t speed_diag_started_ms;
static uint32_t speed_diag_deadline_ms;
static float speed_diag_target_mm_s;
static float speed_diag_error_sum[WHEEL_SPEED_COUNT];
static float speed_diag_previous_error[WHEEL_SPEED_COUNT];
static uint32_t speed_diag_samples[WHEEL_SPEED_COUNT];
static uint16_t speed_diag_sign_changes[WHEEL_SPEED_COUNT];
static HeadingDiagState_t heading_diag_state;
static HeadingDiagKind_t heading_diag_kind;
static uint32_t heading_diag_started_ms;
static uint32_t heading_diag_deadline_ms;
static float heading_diag_final_error_deg;
static bool heading_diag_completed;
static float heading_diag_hold_speed_mm_s;
static uint32_t heading_diag_hold_duration_ms;
static AppLineState_t line_previous_state;

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
        (gray.error_count > 0U) ? "address scan timeout" :
            (gray_contrast_observed ? "digital high/low response observed" :
                ((gray.scan_count > 0U) ? "waiting for both digital levels" :
                                          "waiting for scheduled GPIO scan")));
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

static const char *Diagnostics_HeadingModeName(HeadingMode_t mode)
{
    if (mode == HEADING_MODE_HOLD) return "HOLD";
    if (mode == HEADING_MODE_TURN) return "TURN";
    return "IDLE";
}

static void Diagnostics_PrintHeading(void)
{
    HeadingSnapshot_t s;
    Heading_GetSnapshot(&s);
    debug_printf("HEADING valid=%u calibrated=%u calibrating=%u stationary=%u count=%lu rejects=%lu span_x100=%ld std_x100=%ld mode=%s zero_x100=%ld yaw_x100=%ld rate_x100=%ld reference_x100=%ld target_x100=%ld error_x100=%ld complete=%u age=%lu\r\n",
        s.valid, s.calibrated, s.calibrating,
        s.calibration_stationary,
        (unsigned long)s.calibration_count,
        (unsigned long)s.calibration_rejects,
        (long)Diagnostics_Round(s.calibration_span_deg * 100.0f),
        (long)Diagnostics_Round(s.calibration_stddev_deg * 100.0f),
        Diagnostics_HeadingModeName(s.mode),
        (long)Diagnostics_Round(s.zero_offset_deg * 100.0f),
        (long)Diagnostics_Round(s.yaw_deg * 100.0f),
        (long)Diagnostics_Round(s.yaw_rate_deg_s * 100.0f),
        (long)Diagnostics_Round(s.reference_deg * 100.0f),
        (long)Diagnostics_Round(s.target_deg * 100.0f),
        (long)Diagnostics_Round(s.error_deg * 100.0f), s.complete,
        (unsigned long)(SystemTime_Now() - s.last_update_ms));
}

static void Diagnostics_PrintGraySnapshot(void)
{
    GrayscaleSnapshot_t s;
    Grayscale_GetSnapshot(&s);
    if (s.high_mask != 0U) gray_high_observed = true;
    if (s.high_mask != 0xFFU) gray_low_observed = true;
    gray_contrast_observed = gray_high_observed && gray_low_observed;
    debug_printf("GRAY digital=[%u,%u,%u,%u,%u,%u,%u,%u] high_mask=0x%02X scans=%lu errors=%lu pending=%u\r\n",
        s.level[0], s.level[1], s.level[2], s.level[3],
        s.level[4], s.level[5], s.level[6], s.level[7],
        (unsigned int)s.high_mask, (unsigned long)s.scan_count,
        (unsigned long)s.error_count, Grayscale_IsScanPending());
    Diagnostics_RefreshReports();
}

static const char *Diagnostics_LinePolarityName(LinePolarity_t polarity)
{
    return (polarity == LINE_POLARITY_ACTIVE_HIGH) ? "HIGH" : "LOW";
}

static const char *Diagnostics_LineStateName(LineFollowState_t state)
{
    if (state == LINE_FOLLOW_TRACKING) return "TRACK";
    if (state == LINE_FOLLOW_SEARCHING) return "SEARCH";
    if (state == LINE_FOLLOW_LOST_TIMEOUT) return "TIMEOUT";
    return "IDLE";
}

static void Diagnostics_PrintLine(void)
{
    AppLineSnapshot_t app;
    AppLine_GetSnapshot(&app);
    debug_printf("LINE app=%u state=%s polarity=%s valid=%u pos_x1000=%ld conf_x1000=%lu correction=%ld target=%ld,%ld lost_ms=%lu age=%lu\r\n",
        (unsigned int)app.state,
        Diagnostics_LineStateName(app.control.state),
        Diagnostics_LinePolarityName(app.control.polarity),
        app.control.line_valid,
        (long)Diagnostics_Round(app.control.position * 1000.0f),
        (unsigned long)Diagnostics_ScalePositive(
            app.control.confidence, 1000.0f),
        (long)Diagnostics_Round(app.control.correction_mm_s),
        (long)Diagnostics_Round(app.control.left_target_mm_s),
        (long)Diagnostics_Round(app.control.right_target_mm_s),
        (unsigned long)((app.control.lost_since_ms == 0U) ? 0U :
            (SystemTime_Now() - app.control.lost_since_ms)),
        (unsigned long)(SystemTime_Now() - app.control.last_update_ms));
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

static void Diagnostics_QueueInfraredReport(void)
{
    IrReceiver_GetStats(&infrared_report_receiver);
    AppRemote_GetSnapshot(&infrared_report_app);
    infrared_report_line = 0U;
    infrared_report_pending = true;
}

static void Diagnostics_ServiceInfraredReport(void)
{
    if (!infrared_report_pending) return;
    if (infrared_report_line == 0U) {
        debug_printf("IR full=%lu repeat=%lu checksum=%lu timing=%lu overflow=%lu queued=%lu last=0x%02X/0x%02X raw=0x%08lX\r\n",
            (unsigned long)infrared_report_receiver.nec.full_frames,
            (unsigned long)infrared_report_receiver.nec.repeat_frames,
            (unsigned long)infrared_report_receiver.nec.checksum_errors,
            (unsigned long)infrared_report_receiver.nec.timing_errors,
            (unsigned long)infrared_report_receiver.queue_overflow,
            (unsigned long)infrared_report_receiver.queued_events,
            infrared_report_receiver.nec.last_address,
            infrared_report_receiver.nec.last_command,
            (unsigned long)infrared_report_receiver.nec.last_raw);
    } else if (infrared_report_line == 1U) {
        debug_printf("IR EDGE count=%lu phase_high=%lu phase_low=%lu ticks=%u/%u/%u gpio_pin=%u\r\n",
            (unsigned long)infrared_report_receiver.capture_edges,
            (unsigned long)infrared_report_receiver.capture_high_edges,
            (unsigned long)infrared_report_receiver.capture_low_edges,
            infrared_report_receiver.capture_min_ticks,
            infrared_report_receiver.capture_max_ticks,
            infrared_report_receiver.capture_last_ticks,
            infrared_report_receiver.pin_high);
    } else {
        debug_printf("REMOTE enabled=%u state=%u direction=%u speed=%u addr_reject=%lu unknown=%lu action_reject=%lu heartbeat_stop=%lu captured=%lu\r\n",
            infrared_report_app.actions_enabled,
            (unsigned int)infrared_report_app.state,
            (unsigned int)infrared_report_app.direction,
            infrared_report_app.speed_mm_s,
            (unsigned long)infrared_report_app.address_rejects,
            (unsigned long)infrared_report_app.unknown_commands,
            (unsigned long)infrared_report_app.action_rejects,
            (unsigned long)infrared_report_app.heartbeat_stops,
            (unsigned long)infrared_report_app.captured_keys);
        infrared_report_pending = false;
        return;
    }
    infrared_report_line++;
}

static void Diagnostics_QueueSafetyReport(void)
{
    safety_report_system_state = SystemState_Get();
    Safety_GetSnapshot(&safety_report_snapshot);
    Scheduler_GetStats(&safety_report_scheduler);
    safety_report_line = 0U;
    safety_report_pending = true;
}

static bool Diagnostics_ActionTelemetryActive(void)
{
    AppLineSnapshot_t app;
    AppLine_GetSnapshot(&app);
    return (speed_diag_state == SPEED_DIAG_RUNNING) ||
        (speed_diag_state == SPEED_DIAG_RAMP_DOWN) ||
        (heading_diag_state == HEADING_DIAG_RUNNING) ||
        (heading_diag_state == HEADING_DIAG_RAMP_DOWN) ||
        (app.state == APP_LINE_RUNNING);
}

static void Diagnostics_ServiceSafetyReport(void)
{
    if (!safety_report_pending || Diagnostics_ActionTelemetryActive()) return;

    if (safety_report_line == 0U) {
        debug_printf("SAFETY state=%s faults=0x%08lX first=%lu last=%lu action=%u deadline=%lu pwm=%d,%d\r\n",
            SystemState_Name(safety_report_system_state),
            (unsigned long)safety_report_snapshot.fault_bits,
            (unsigned long)safety_report_snapshot.first_fault_ms,
            (unsigned long)safety_report_snapshot.last_fault_ms,
            safety_report_snapshot.action_active,
            (unsigned long)safety_report_snapshot.action_deadline_ms,
            safety_report_snapshot.applied_motor_a,
            safety_report_snapshot.applied_motor_b);
    } else if (safety_report_line == 1U) {
        debug_printf("CONTEXT target=%d,%d speed=%d,%d requested_pwm=%d,%d\r\n",
            safety_report_snapshot.target_speed_left_mm_s,
            safety_report_snapshot.target_speed_right_mm_s,
            safety_report_snapshot.measured_speed_left_mm_s,
            safety_report_snapshot.measured_speed_right_mm_s,
            safety_report_snapshot.requested_motor_a,
            safety_report_snapshot.requested_motor_b);
    } else {
        debug_printf("SCHED release=%lu/%lu/%lu/%lu miss=%lu/%lu/%lu/%lu pending=0x%02lX\r\n",
            (unsigned long)safety_report_scheduler.release_count[0],
            (unsigned long)safety_report_scheduler.release_count[1],
            (unsigned long)safety_report_scheduler.release_count[2],
            (unsigned long)safety_report_scheduler.release_count[3],
            (unsigned long)safety_report_scheduler.miss_count[0],
            (unsigned long)safety_report_scheduler.miss_count[1],
            (unsigned long)safety_report_scheduler.miss_count[2],
            (unsigned long)safety_report_scheduler.miss_count[3],
            (unsigned long)safety_report_scheduler.pending_mask);
        safety_report_pending = false;
        return;
    }
    safety_report_line++;
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
    AppLineSnapshot_t app_line_snapshot;
    AppLine_GetSnapshot(&app_line_snapshot);
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
        (motor_diag_state != MOTOR_DIAG_IDLE) ||
        (app_line_snapshot.state != APP_LINE_IDLE)) {
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

static void Diagnostics_AdjustSpeedTarget(float delta, uint32_t now_ms)
{
    float target = ControlParams_Staged()->wheel.diag_target_mm_s + delta;
    (void)now_ms;
    if (speed_diag_state == SPEED_DIAG_RAMP_DOWN) {
        debug_println("M3 tuning rejected while stopping.");
        return;
    }
    if (target < 50.0f) target = 50.0f;
    if (target > 400.0f) target = 400.0f;
    if ((ControlParams_Set("wheel.diag_target", target) != CONTROL_PARAMS_OK) ||
        !ControlProtocol_ApplyLegacyStaged()) {
        (void)ControlParams_Revert();
        debug_println("M3 tuning rejected by parameter state gate.");
        return;
    }
    debug_printf("M3 test target=%ld mm/s\r\n",
        (long)Diagnostics_Round(target));
}

static void Diagnostics_AdjustSpeedGain(bool proportional, float delta,
                                        uint32_t now_ms)
{
    uint32_t i;
    bool changed = false;
    (void)now_ms;
    if (speed_diag_state == SPEED_DIAG_RAMP_DOWN) {
        debug_println("M3 tuning rejected while stopping.");
        return;
    }
    for (i = 0U; i < (uint32_t)WHEEL_SPEED_COUNT; i++) {
        float kp;
        float ki;
        if ((speed_diag_wheel_mask & (1U << i)) == 0U) continue;
        const ControlParams_t *staged = ControlParams_Staged();
        const char *key;
        if (i == (uint32_t)WHEEL_SPEED_LEFT) {
            kp = staged->wheel.left_kp;
            ki = staged->wheel.left_ki;
        } else {
            kp = staged->wheel.right_kp;
            ki = staged->wheel.right_ki;
        }
        if (proportional) kp += delta;
        else ki += delta;
        if (kp < 0.0f) kp = 0.0f;
        if (kp > 3.0f) kp = 3.0f;
        if (ki < 0.0f) ki = 0.0f;
        if (ki > 10.0f) ki = 10.0f;
        if (proportional)
            key = (i == (uint32_t)WHEEL_SPEED_LEFT) ?
                "wheel.left.kp" : "wheel.right.kp";
        else
            key = (i == (uint32_t)WHEEL_SPEED_LEFT) ?
                "wheel.left.ki" : "wheel.right.ki";
        if (ControlParams_Set(key, proportional ? kp : ki) ==
            CONTROL_PARAMS_OK) changed = true;
    }
    if (!changed || !ControlProtocol_ApplyLegacyStaged()) {
        (void)ControlParams_Revert();
        debug_println("M3 tuning rejected by parameter state gate.");
        return;
    }
    Diagnostics_PrintWheelSpeed();
}

static const char *Diagnostics_HeadingKindName(void)
{
    if (heading_diag_kind == HEADING_DIAG_HOLD) return "straight hold";
    if (heading_diag_kind == HEADING_DIAG_TURN_POSITIVE) return "+90 turn";
    return "-90 turn";
}

static bool Diagnostics_ArmHeadingTest(HeadingDiagKind_t kind,
                                       float hold_speed_mm_s,
                                       uint32_t hold_duration_ms,
                                       uint32_t now_ms)
{
    HeadingSnapshot_t heading_snapshot;
    AppLineSnapshot_t app_line_snapshot;
    Heading_GetSnapshot(&heading_snapshot);
    AppLine_GetSnapshot(&app_line_snapshot);
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
        (motor_diag_state != MOTOR_DIAG_IDLE) ||
        (speed_diag_state != SPEED_DIAG_IDLE) ||
        (heading_diag_state != HEADING_DIAG_IDLE) ||
        (app_line_snapshot.state != APP_LINE_IDLE)) {
        debug_println("M4 test rejected: clear faults and return IDLE.");
        return false;
    }
    if (!heading_snapshot.calibrated) {
        debug_println("M4 test rejected: keep car still and wait for calibration.");
        return false;
    }
    heading_diag_kind = kind;
    heading_diag_hold_speed_mm_s = hold_speed_mm_s;
    heading_diag_hold_duration_ms = hold_duration_ms;
    heading_diag_state = HEADING_DIAG_ARMED;
    heading_diag_deadline_ms = now_ms + DIAGNOSTIC_ARM_TIMEOUT_MS;
    debug_printf("Place car safely for %s; type uppercase A within 10 s.\r\n",
        Diagnostics_HeadingKindName());
    return true;
}

static bool Diagnostics_StartHeadingTest(uint32_t now_ms)
{
    bool started;
    if ((heading_diag_state != HEADING_DIAG_ARMED) ||
        SystemTime_Reached(now_ms, heading_diag_deadline_ms)) {
        heading_diag_state = HEADING_DIAG_IDLE;
        debug_println("M4 arm expired or not requested.");
        return false;
    }
    if (!SystemState_RequestRun()) {
        heading_diag_state = HEADING_DIAG_IDLE;
        debug_println("M4 test rejected: system is not IDLE.");
        return false;
    }
    if (heading_diag_kind == HEADING_DIAG_HOLD)
        started = Heading_StartHold(heading_diag_hold_speed_mm_s);
    else
        started = Heading_StartTurn(
            (heading_diag_kind == HEADING_DIAG_TURN_POSITIVE) ? 90.0f : -90.0f);
    if (!started) {
        (void)SystemState_RequestIdle();
        heading_diag_state = HEADING_DIAG_IDLE;
        debug_println("M4 start rejected: heading sample is not ready.");
        return false;
    }
    Safety_BeginAction(now_ms, HEADING_ACTION_TIMEOUT_MS,
        SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO));
    WheelSpeed_Enable();
    heading_diag_started_ms = now_ms;
    heading_diag_deadline_ms = now_ms +
        ((heading_diag_kind == HEADING_DIAG_HOLD) ?
            heading_diag_hold_duration_ms : HEADING_TURN_TIMEOUT_MS);
    heading_diag_state = HEADING_DIAG_RUNNING;
    debug_println("CSV: M4,time_ms,phase,mode,yaw_x100,rate_x100,target_x100,error_x100,targetL,targetR,rampL,rampR,speedL,speedR,pwmL,pwmR,faults");
    debug_println("M4 test running; S/space is immediate emergency stop.");
    return true;
}

static void Diagnostics_FinishHeadingTest(void)
{
    WheelSpeed_Disable();
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    debug_printf("M4 RESULT kind=%s elapsed=%lu error_x100=%ld complete=%u\r\n",
        Diagnostics_HeadingKindName(),
        (unsigned long)(SystemTime_Now() - heading_diag_started_ms),
        (long)Diagnostics_Round(heading_diag_final_error_deg * 100.0f),
        heading_diag_completed);
    Heading_Stop();
    heading_diag_state = HEADING_DIAG_IDLE;
}

static void Diagnostics_ServiceHeadingTest(uint32_t now_ms)
{
    HeadingSnapshot_t heading_snapshot;
    WheelSpeedSnapshot_t speed_snapshot;
    if (heading_diag_state == HEADING_DIAG_ARMED) {
        if (SystemTime_Reached(now_ms, heading_diag_deadline_ms)) {
            heading_diag_state = HEADING_DIAG_IDLE;
            debug_println("M4 test arm timed out.");
        }
        return;
    }
    if ((heading_diag_state != HEADING_DIAG_RUNNING) &&
        (heading_diag_state != HEADING_DIAG_RAMP_DOWN)) return;
    if (Safety_HasFault()) {
        Heading_Stop();
        WheelSpeed_Disable();
        heading_diag_state = HEADING_DIAG_IDLE;
        debug_println("M4 test aborted by safety fault.");
        return;
    }
    Heading_GetSnapshot(&heading_snapshot);
    if ((heading_diag_state == HEADING_DIAG_RUNNING) &&
        (((heading_diag_kind != HEADING_DIAG_HOLD) &&
          heading_snapshot.complete) ||
         SystemTime_Reached(now_ms, heading_diag_deadline_ms))) {
        heading_diag_final_error_deg = heading_snapshot.error_deg;
        heading_diag_completed = heading_snapshot.complete;
        Heading_Stop();
        WheelSpeed_SetTarget(0.0f, 0.0f);
        Safety_BeginAction(now_ms, SPEED_DIAGNOSTIC_STOP_TIMEOUT_MS,
            SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO));
        heading_diag_state = HEADING_DIAG_RAMP_DOWN;
        heading_diag_deadline_ms = now_ms + SPEED_DIAGNOSTIC_STOP_TIMEOUT_MS;
        return;
    }
    if (heading_diag_state != HEADING_DIAG_RAMP_DOWN) return;
    WheelSpeed_GetSnapshot(&speed_snapshot);
    if (((Diagnostics_Abs(speed_snapshot.ramped_mm_s[WHEEL_SPEED_LEFT]) < 0.5f) &&
         (Diagnostics_Abs(speed_snapshot.ramped_mm_s[WHEEL_SPEED_RIGHT]) < 0.5f) &&
         (Diagnostics_Abs(speed_snapshot.filtered_mm_s[WHEEL_SPEED_LEFT]) < 20.0f) &&
         (Diagnostics_Abs(speed_snapshot.filtered_mm_s[WHEEL_SPEED_RIGHT]) < 20.0f)) ||
        SystemTime_Reached(now_ms, heading_diag_deadline_ms)) {
        Diagnostics_FinishHeadingTest();
    }
}

void Diagnostics_Init(void)
{
    memset(reports, 0, sizeof(reports));
    motor_test_passed = false;
    oled_visual_passed = false;
    gray_contrast_observed = false;
    gray_high_observed = false;
    gray_low_observed = false;
    boot_reported = false;
    boot_started_ms = SystemTime_Now();
    motor_diag_state = MOTOR_DIAG_IDLE;
    gray_print_pending = false;
    report_pending = false;
    report_line = 0U;
    infrared_report_pending = false;
    infrared_report_line = 0U;
    safety_report_pending = false;
    safety_report_line = 0U;
    speed_diag_state = SPEED_DIAG_IDLE;
    speed_diag_wheel_mask = SPEED_DIAG_LEFT_MASK |
                            SPEED_DIAG_RIGHT_MASK;
    speed_diag_target_mm_s = SPEED_DIAGNOSTIC_DEFAULT_MM_S;
    heading_diag_state = HEADING_DIAG_IDLE;
    heading_diag_final_error_deg = 0.0f;
    heading_diag_completed = false;
    heading_diag_hold_speed_mm_s = HEADING_STRAIGHT_TEST_SPEED_MM_S;
    heading_diag_hold_duration_ms = HEADING_STRAIGHT_TEST_DURATION_MS;
    line_previous_state = APP_LINE_IDLE;
    Diagnostics_ResetSpeedMetrics();
#ifdef M2_FAULT_INJECTION_TEST
    fault_injection_stop_watchdog = false;
#endif
    Safety_StopMotor();
}

void Diagnostics_RunBootSelfTest(void)
{
    ParameterStoreInfo_t store;

    ParameterStore_GetInfo(&store);
    debug_println("\r\n=== MSPM0 CAR M6 RUNTIME TUNING ===");
    debug_printf("M6 READY fw=%s schema=%u revision=%lu source=%s "
                 "storage=%s generation=%lu\r\n",
        CONTROL_PROTOCOL_FIRMWARE_ID, CONTROL_PARAMS_SCHEMA_VERSION,
        (unsigned long)ControlParams_Revision(), ParameterStore_SourceName(),
        (store.health == PARAMETER_STORE_HEALTH_OK) ? "OK" : "DEGRADED",
        (unsigned long)store.generation);
#ifdef M2_FAULT_INJECTION_TEST
    debug_println("*** FAULT INJECTION BUILD - ZERO PWM TESTS ONLY ***");
#endif
    debug_println("Boot self-test settling; motor output is safety-locked.");
    Diagnostics_PrintHelp();
}

void Diagnostics_PrintHelp(void)
{
    debug_println("Commands: h=help s=status d=all e=encoder y=gyro q=grayscale t=IR capture");
    debug_println("          o=OLED pattern p/f=confirm l=LED c=comm m=motor A=arm");
    debug_println("          j=left k=right b=both speed test; n=wheel snapshot");
    debug_println("          +/- target, [/]=Kp, ,/.=Ki for selected wheel(s)");
    debug_println("          g=heading z=calibrate u=+90 i=-90 w=straight; A=arm");
    debug_println("          r=line 2=active-level 3=arm line A=start 0=stop");
    debug_println("          v=safety x=clear fault; S or space=immediate e-stop");
    debug_println("M6 transaction: :id,get/set/apply/revert/load/defaults/save/log,...");
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
        Diagnostics_QueueSafetyReport();
        report_pending = false;
        return;
    }
    report_line++;
}

static void Diagnostics_ServiceLineEvents(void)
{
    AppLineSnapshot_t app;
    AppLine_GetSnapshot(&app);
    if ((line_previous_state == APP_LINE_RUNNING) &&
               (app.state == APP_LINE_IDLE)) {
        debug_printf("M5 RESULT elapsed=%lu state=%s valid=%u pos_x1000=%ld conf_x1000=%lu\r\n",
            (unsigned long)(SystemTime_Now() - app.started_ms),
            Diagnostics_LineStateName(app.control.state),
            app.control.line_valid,
            (long)Diagnostics_Round(app.control.position * 1000.0f),
            (unsigned long)Diagnostics_ScalePositive(
                app.control.confidence, 1000.0f));
        Diagnostics_QueueSafetyReport();
    }
    line_previous_state = app.state;
}

bool Diagnostics_ActionIsIdle(void)
{
    AppLineSnapshot_t app;
    AppLine_GetSnapshot(&app);
    return (motor_diag_state == MOTOR_DIAG_IDLE) &&
        (speed_diag_state == SPEED_DIAG_IDLE) &&
        (heading_diag_state == HEADING_DIAG_IDLE) &&
        (app.state == APP_LINE_IDLE);
}

bool Diagnostics_WheelSpeedDiagnosticActive(void)
{
    return speed_diag_state != SPEED_DIAG_IDLE;
}

void Diagnostics_OnControlParamsCommitted(const ControlParamsCommit_t *commit)
{
    const ControlParams_t *params = ControlParams_Active();
    float left_target;
    float right_target;

    if (commit == NULL) {
        speed_diag_target_mm_s = params->wheel.diag_target_mm_s;
        return;
    }
    if ((commit->change_mask &
         (CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS |
          CONTROL_PARAMS_CHANGE_WHEEL_RIGHT_GAINS |
          CONTROL_PARAMS_CHANGE_WHEEL_DIAG_TARGET)) == 0U) return;

    speed_diag_target_mm_s = params->wheel.diag_target_mm_s;
    Diagnostics_ResetSpeedMetrics();
    if (speed_diag_state == SPEED_DIAG_RUNNING) {
        left_target = ((speed_diag_wheel_mask & SPEED_DIAG_LEFT_MASK) != 0U) ?
            speed_diag_target_mm_s : 0.0f;
        right_target = ((speed_diag_wheel_mask & SPEED_DIAG_RIGHT_MASK) != 0U) ?
            speed_diag_target_mm_s : 0.0f;
        WheelSpeed_SetTarget(left_target, right_target);
        speed_diag_started_ms = SystemTime_Now();
        speed_diag_deadline_ms = speed_diag_started_ms +
            SPEED_DIAGNOSTIC_HOLD_MS;
        Safety_BeginAction(speed_diag_started_ms,
            SPEED_DIAGNOSTIC_ACTION_TIMEOUT_MS, 0U);
    }
}

bool Diagnostics_ActionArm(DiagnosticsAction_t action,
                           float straight_speed_mm_s,
                           uint32_t straight_duration_ms,
                           uint32_t now_ms)
{
    if ((straight_speed_mm_s <= 0.0f) ||
        (straight_duration_ms == 0U)) return false;

    if (action == DIAGNOSTICS_ACTION_LINE) {
        if (!Diagnostics_ActionIsIdle() || Safety_HasFault() ||
            (SystemState_Get() != SYSTEM_STATE_IDLE)) return false;
        return AppLine_Arm(now_ms);
    }
    if (action == DIAGNOSTICS_ACTION_STRAIGHT)
        return Diagnostics_ArmHeadingTest(HEADING_DIAG_HOLD,
            straight_speed_mm_s, straight_duration_ms, now_ms);
    if (action == DIAGNOSTICS_ACTION_TURN_POSITIVE)
        return Diagnostics_ArmHeadingTest(HEADING_DIAG_TURN_POSITIVE,
            straight_speed_mm_s, straight_duration_ms, now_ms);
    if (action == DIAGNOSTICS_ACTION_TURN_NEGATIVE)
        return Diagnostics_ArmHeadingTest(HEADING_DIAG_TURN_NEGATIVE,
            straight_speed_mm_s, straight_duration_ms, now_ms);
    return false;
}

bool Diagnostics_ActionConfirm(uint32_t now_ms)
{
    AppLineSnapshot_t app;
    AppLine_GetSnapshot(&app);
    if (app.state == APP_LINE_ARMED) return AppLine_ConfirmStart(now_ms);
    if (heading_diag_state == HEADING_DIAG_ARMED)
        return Diagnostics_StartHeadingTest(now_ms);
    return false;
}

bool Diagnostics_ActionCancel(uint32_t now_ms)
{
    AppLineSnapshot_t app;
    bool active;
    AppLine_GetSnapshot(&app);
    active = (motor_diag_state != MOTOR_DIAG_IDLE) ||
        (speed_diag_state != SPEED_DIAG_IDLE) ||
        (heading_diag_state != HEADING_DIAG_IDLE) ||
        (app.state != APP_LINE_IDLE);
    if (!active || Safety_HasFault()) return false;

    WheelSpeed_Disable();
    Heading_Stop();
    (void)AppLine_Stop(now_ms);
    Safety_EndAction();
    (void)SystemState_RequestIdle();
    motor_diag_state = MOTOR_DIAG_IDLE;
    speed_diag_state = SPEED_DIAG_IDLE;
    heading_diag_state = HEADING_DIAG_IDLE;
    return true;
}

void Diagnostics_Poll(void)
{
    uint32_t now_ms = SystemTime_Now();
    GrayscaleSnapshot_t gray;
    Diagnostics_ServiceMotorTest(now_ms);
    Diagnostics_ServiceSpeedTest(now_ms);
    Diagnostics_ServiceHeadingTest(now_ms);
    Diagnostics_ServiceLineEvents();
    ControlProtocol_Service();

    if (gray_print_pending) {
        Grayscale_GetSnapshot(&gray);
        if (gray.scan_count != gray_requested_scan_count) {
            gray_print_pending = false;
            Diagnostics_PrintGraySnapshot();
        }
    }

    if (!debug_kbhit()) return;
    {
    char command = debug_getc();
    if (ControlProtocol_ProcessByte(command, now_ms)) return;
    switch (command) {
        case 'h': case 'H': Diagnostics_PrintHelp(); break;
        case 's':
            Diagnostics_QueueInfraredReport();
            Diagnostics_PrintReport();
            break;
        case 'd': case 'D':
            Diagnostics_PrintEncoder(); Diagnostics_PrintGyro();
            Diagnostics_PrintGraySnapshot(); Diagnostics_PrintCommunication();
            Diagnostics_PrintWheelSpeed(); Diagnostics_QueueInfraredReport();
            Diagnostics_QueueSafetyReport(); break;
        case 'e': case 'E': Diagnostics_PrintEncoder(); break;
        case 'y': case 'Y': Diagnostics_PrintGyro(); break;
        case 'g': Diagnostics_PrintHeading(); break;
        case 'r': case 'R': {
            AppLineSnapshot_t app;
            AppLine_GetSnapshot(&app);
            if (app.state == APP_LINE_RUNNING)
                debug_println("M5 running: use CSV telemetry; stop before r.");
            else
                Diagnostics_PrintLine();
            break;
        }
        case '1':
            debug_println("M5 qualification disabled: digital inputs use fixed 0/1 levels.");
            break;
        case '2':
            if (AppLine_TogglePolarity()) Diagnostics_PrintLine();
            else debug_println("M5 active-level change rejected: return to IDLE.");
            break;
        case '3':
            if ((motor_diag_state != MOTOR_DIAG_IDLE) ||
                (speed_diag_state != SPEED_DIAG_IDLE) ||
                (heading_diag_state != HEADING_DIAG_IDLE)) {
                debug_println("M5 arm rejected: another action is active.");
            } else if (AppLine_Arm(now_ms)) {
                debug_println("Place car on line; type uppercase A within 10 s.");
            } else {
                debug_println("M5 arm rejected: acquire the line first.");
            }
            break;
        case '0':
            if (AppRemote_NormalStop(now_ms))
                debug_println("Action cancelled normally; state=IDLE.");
            else debug_println("No non-fault action is active.");
            break;
        case 't':
            if (AppRemote_StartCapture(now_ms))
                debug_println("IR key capture active for 30 s; full frames only, vehicle actions disabled.");
            else debug_println("IR key capture rejected: clear faults and return IDLE.");
            break;
        case 'z':
            if ((SystemState_Get() == SYSTEM_STATE_IDLE) &&
                !Safety_HasFault() &&
                (heading_diag_state == HEADING_DIAG_IDLE)) {
                Heading_StartCalibration();
                debug_println("M4 calibration started: keep car still for 20 samples.");
            } else debug_println("M4 calibration rejected: return to IDLE.");
            break;
        case 'u': case 'U':
            (void)Diagnostics_ArmHeadingTest(HEADING_DIAG_TURN_POSITIVE,
                HEADING_STRAIGHT_TEST_SPEED_MM_S,
                HEADING_STRAIGHT_TEST_DURATION_MS, now_ms); break;
        case 'i': case 'I':
            (void)Diagnostics_ArmHeadingTest(HEADING_DIAG_TURN_NEGATIVE,
                HEADING_STRAIGHT_TEST_SPEED_MM_S,
                HEADING_STRAIGHT_TEST_DURATION_MS, now_ms); break;
        case 'w':
            (void)Diagnostics_ArmHeadingTest(HEADING_DIAG_HOLD,
                HEADING_STRAIGHT_TEST_SPEED_MM_S,
                HEADING_STRAIGHT_TEST_DURATION_MS, now_ms); break;
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
        case 'v': case 'V': Diagnostics_QueueSafetyReport(); break;
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
        case 'm': case 'M': {
            AppLineSnapshot_t app_line_snapshot;
            AppLine_GetSnapshot(&app_line_snapshot);
            if ((SystemState_Get() != SYSTEM_STATE_IDLE) ||
                Safety_HasFault() ||
                (speed_diag_state != SPEED_DIAG_IDLE) ||
                (heading_diag_state != HEADING_DIAG_IDLE) ||
                (app_line_snapshot.state != APP_LINE_IDLE)) {
                debug_println("Motor test rejected: clear faults and return IDLE.");
            } else {
                motor_diag_state = MOTOR_DIAG_ARMED;
                motor_diag_deadline_ms = now_ms + DIAGNOSTIC_ARM_TIMEOUT_MS;
                debug_println("Lift wheels, then type uppercase A within 10 s.");
            }
            break;
        }
        case 'A':
            {
                AppLineSnapshot_t app_line_snapshot;
                AppLine_GetSnapshot(&app_line_snapshot);
                if (app_line_snapshot.state == APP_LINE_ARMED) {
                    if (AppLine_ConfirmStart(now_ms)) {
                        debug_println("CSV: M5,time_ms,phase,state,polarity,position_x1000,confidence_x1000,correction,targetL,targetR,rampL,rampR,speedL,speedR,pwmL,pwmR,faults");
                        debug_println("M5 line following running; 0=normal stop, S/space=e-stop.");
                    } else debug_println("M5 start rejected: arm expired or line unavailable.");
                } else if (heading_diag_state == HEADING_DIAG_ARMED)
                (void)Diagnostics_StartHeadingTest(now_ms);
                else if (speed_diag_state == SPEED_DIAG_ARMED)
                Diagnostics_StartSpeedTest(now_ms);
                else
                Diagnostics_StartMotorTest(now_ms);
            }
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
                Heading_Stop();
                heading_diag_state = HEADING_DIAG_IDLE;
                motor_diag_state = MOTOR_DIAG_IDLE;
                (void)AppLine_Stop(now_ms);
                debug_println("Faults cleared; state=IDLE.");
            } else {
                debug_println("Fault clear rejected.");
            }
            break;
        case '\r': case '\n': break;
        default: debug_println("Unknown command; h=help."); break;
    }
    }
}

void Diagnostics_Task100ms(void)
{
    uint32_t now_ms = SystemTime_Now();
    bool legacy_csv_enabled = !M6Telemetry_IsLogging();
    if (!boot_reported &&
        ((uint32_t)(now_ms - boot_started_ms) >= BOOT_SELF_TEST_SETTLE_MS)) {
        boot_reported = true;
        Diagnostics_PrintReport();
        if (!Safety_HasFault()) (void)SystemState_CompleteSelfTest(true);
    }
    if (infrared_report_pending)
        Diagnostics_ServiceInfraredReport();
    else if (report_pending)
        Diagnostics_ServiceReport();
    else
        Diagnostics_ServiceSafetyReport();
    if ((speed_diag_state == SPEED_DIAG_RUNNING) ||
        (speed_diag_state == SPEED_DIAG_RAMP_DOWN)) {
        WheelSpeedSnapshot_t wheel_snapshot;
        uint32_t i;
        WheelSpeed_GetSnapshot(&wheel_snapshot);
        if (legacy_csv_enabled) {
            SafetySnapshot_t safety_snapshot;
            uint32_t saturated;
            Safety_GetSnapshot(&safety_snapshot);
            saturated =
                (wheel_snapshot.saturated[WHEEL_SPEED_LEFT] ? 1U : 0U) |
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
        }

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
    if (legacy_csv_enabled &&
        ((heading_diag_state == HEADING_DIAG_RUNNING) ||
         (heading_diag_state == HEADING_DIAG_RAMP_DOWN))) {
        HeadingSnapshot_t heading_snapshot;
        SafetySnapshot_t safety_snapshot;
        Heading_GetSnapshot(&heading_snapshot);
        Safety_GetSnapshot(&safety_snapshot);
        WheelSpeedSnapshot_t wheel_snapshot;
        WheelSpeed_GetSnapshot(&wheel_snapshot);
        debug_printf("M4,%lu,%s,%s,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,0x%08lX\r\n",
                (unsigned long)now_ms,
                (heading_diag_state == HEADING_DIAG_RUNNING) ? "RUN" : "STOP",
                Diagnostics_HeadingModeName(heading_snapshot.mode),
                (long)Diagnostics_Round(heading_snapshot.yaw_deg * 100.0f),
                (long)Diagnostics_Round(
                    heading_snapshot.yaw_rate_deg_s * 100.0f),
                (long)Diagnostics_Round(heading_snapshot.target_deg * 100.0f),
                (long)Diagnostics_Round(heading_snapshot.error_deg * 100.0f),
                (long)Diagnostics_Round(heading_snapshot.left_target_mm_s),
                (long)Diagnostics_Round(heading_snapshot.right_target_mm_s),
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
            (unsigned long)safety_snapshot.fault_bits);
    }
    if (legacy_csv_enabled) {
        AppLineSnapshot_t app;
        AppLine_GetSnapshot(&app);
        if (app.state == APP_LINE_RUNNING) {
            SafetySnapshot_t safety_snapshot;
            WheelSpeedSnapshot_t wheel_snapshot;
            Safety_GetSnapshot(&safety_snapshot);
            WheelSpeed_GetSnapshot(&wheel_snapshot);
            debug_printf("M5,%lu,RUN,%s,%s,%ld,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,0x%08lX\r\n",
                (unsigned long)now_ms,
                Diagnostics_LineStateName(app.control.state),
                Diagnostics_LinePolarityName(app.control.polarity),
                (long)Diagnostics_Round(app.control.position * 1000.0f),
                (unsigned long)Diagnostics_ScalePositive(
                    app.control.confidence, 1000.0f),
                (long)Diagnostics_Round(app.control.correction_mm_s),
                (long)Diagnostics_Round(app.control.left_target_mm_s),
                (long)Diagnostics_Round(app.control.right_target_mm_s),
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
                (unsigned long)safety_snapshot.fault_bits);
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
