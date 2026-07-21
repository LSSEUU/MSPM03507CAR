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

#include <stdbool.h>
#include <string.h>

#define GRAY_ADC_RESPONSE_DELTA 128U

typedef enum {
    MOTOR_DIAG_IDLE = 0,
    MOTOR_DIAG_ARMED,
    MOTOR_DIAG_RUNNING
} MotorDiagState_t;

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
    Safety_StopMotor();
}

void Diagnostics_RunBootSelfTest(void)
{
    debug_println("\r\n=== MSPM0 CAR M2 SCHEDULER + SAFETY ===");
    debug_println("Boot self-test settling; motor output is safety-locked.");
    Diagnostics_PrintHelp();
}

void Diagnostics_PrintHelp(void)
{
    debug_println("Commands: h=help s=status d=all e=encoder y=gyro q=grayscale");
    debug_println("          o=OLED pattern p/f=confirm l=LED c=comm m=motor A=arm");
    debug_println("          v=safety x=clear fault; S or space=immediate e-stop");
}

void Diagnostics_PrintReport(void)
{
    Diagnostics_RefreshReports();
    debug_println("--- SELF TEST ---");
    for (uint8_t i = 0U; i < (uint8_t)DIAG_PERIPHERAL_COUNT; i++) {
        debug_printf("%-10s %-8s samples=%lu errors=%lu %s\r\n",
            Diag_PeripheralName((DiagPeripheral_t)i),
            Diag_StatusName(reports[i].status),
            (unsigned long)reports[i].sample_count,
            (unsigned long)reports[i].error_count, reports[i].detail);
    }
    debug_printf("OVERALL    %s\r\n", Diag_StatusName(
        Diag_WorstStatus(reports, (uint8_t)DIAG_PERIPHERAL_COUNT)));
    Diagnostics_PrintSafety();
}

void Diagnostics_Poll(void)
{
    uint32_t now_ms = SystemTime_Now();
    GrayscaleSnapshot_t gray;
    Diagnostics_ServiceMotorTest(now_ms);

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
            Diagnostics_PrintSafety(); break;
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
                debug_println("OLED refresh queued; wait about 7 s, then type p=pass or f=fail.");
            } else debug_println("OLED FAULT: no ACK.");
            break;
        case 'p': oled_visual_passed = true; break;
        case 'f': case 'F': oled_visual_passed = false; break;
        case 'l': case 'L':
            debug_printf("LED configured=%u errors=%lu\r\n",
                LED_GetConfiguredCount(), (unsigned long)LED_GetErrorCount()); break;
        case 'c': case 'C': Diagnostics_PrintCommunication(); break;
        case 'v': case 'V': Diagnostics_PrintSafety(); break;
        case 'm': case 'M':
            if (SystemState_Get() != SYSTEM_STATE_IDLE || Safety_HasFault()) {
                debug_println("Motor test rejected: clear faults and return IDLE.");
            } else {
                motor_diag_state = MOTOR_DIAG_ARMED;
                motor_diag_deadline_ms = now_ms + DIAGNOSTIC_ARM_TIMEOUT_MS;
                debug_println("Lift wheels, then type uppercase A within 10 s.");
            }
            break;
        case 'A': Diagnostics_StartMotorTest(now_ms); break;
        case 'x': case 'X':
            debug_println(Safety_ClearFaults(now_ms) ?
                "Faults cleared; state=IDLE." : "Fault clear rejected."); break;
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
}

const DiagReport_t *Diagnostics_GetReports(void)
{
    return reports;
}
