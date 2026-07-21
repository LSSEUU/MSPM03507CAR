#include "diagnostics.h"
#include "board.h"
#include "uart_debug.h"
#include "motor.h"
#include "encoder.h"
#include "gyro.h"
#include "grayscale.h"
#include "oled.h"
#include "led.h"

#define MOTOR_TEST_PWM              250
#define MOTOR_TEST_PHASE_MS         500U
#define OPERATOR_TIMEOUT_MS         10000U

static DiagReport_t reports[DIAG_PERIPHERAL_COUNT];
static bool motor_test_passed;
static bool oled_visual_passed;

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
        encoder.edge_count_a + encoder.edge_count_b, encoder.error_count,
        (encoder.edge_count_a > 0U && encoder.edge_count_b > 0U) ?
            "both channels active" : "rotate both wheels or run motor test");

    Diagnostics_Set(DIAG_GYRO,
        (gyro.valid_frames > 0U) ? DIAG_STATUS_OK : DIAG_STATUS_FAULT,
        gyro.valid_frames, gyro.checksum_errors + gyro.frame_errors,
        (gyro.valid_frames > 0U) ? "valid JY61P angle frames" : "no valid 0x55 0x53 frame");

    Diagnostics_Set(DIAG_GRAYSCALE, DIAG_STATUS_DEGRADED,
        gray.scan_count, gray.error_count, "raw scan works; optical response needs operator");

    Diagnostics_Set(DIAG_OLED,
        !oled.present ? DIAG_STATUS_FAULT :
            ((oled.ack_errors > 0U || !oled_visual_passed) ?
                DIAG_STATUS_DEGRADED : DIAG_STATUS_OK),
        oled.refresh_count, oled.ack_errors,
        !oled.present ? "no ACK at 0x3C/0x3D" :
            (oled.ack_errors > 0U ? "I2C NACK observed" :
                (oled_visual_passed ? "pattern confirmed" : "ACK received; visual check pending")));

    Diagnostics_Set(DIAG_LED, DIAG_STATUS_DEGRADED, 0U, LED_GetErrorCount(),
        "no MCU-controlled LED populated; external LED not configured");

    Diagnostics_Set(DIAG_COMMUNICATION,
        (debug.rx_bytes > 0U) ? DIAG_STATUS_OK : DIAG_STATUS_DEGRADED,
        debug.rx_bytes + debug.tx_bytes, debug.format_errors,
        (debug.rx_bytes > 0U) ? "UART0 RX/TX active" : "TX active; waiting for RX command");
}

static bool Diagnostics_WaitForChar(char pass, char fail, uint32_t timeout_ms)
{
    for (uint32_t elapsed = 0U; elapsed < timeout_ms; elapsed += 10U) {
        if (debug_kbhit()) {
            char c = debug_getc();
            if (c == pass) return true;
            if (c == fail || c == 'S' || c == 's' || c == ' ') return false;
        }
        delay_ms(10U);
    }
    return false;
}

static bool Diagnostics_RunMotorPhase(int16_t a, int16_t b, uint32_t duration_ms)
{
    Motor_SetPWM(a, b);
    for (uint32_t elapsed = 0U; elapsed < duration_ms; elapsed += 10U) {
        if (debug_kbhit()) {
            char c = debug_getc();
            if (c == 'S' || c == 's' || c == ' ') {
                Motor_Stop();
                return false;
            }
        }
        delay_ms(10U);
    }
    Motor_Stop();
    delay_ms(150U);
    return true;
}

static void Diagnostics_MotorTest(void)
{
    EncoderSnapshot_t before, after;
    debug_println("MOTOR TEST: lift wheels; S/space is emergency stop.");
    debug_println("Type uppercase A within 10 s to arm.");
    Motor_Stop();
    if (!Diagnostics_WaitForChar('A', 'x', OPERATOR_TIMEOUT_MS)) {
        Motor_Stop();
        debug_println("Motor test cancelled.");
        return;
    }

    Encoder_GetSnapshot(&before);
    debug_println("A forward, A reverse, B forward, B reverse...");
    if (!Diagnostics_RunMotorPhase(MOTOR_TEST_PWM, 0, MOTOR_TEST_PHASE_MS) ||
        !Diagnostics_RunMotorPhase(-MOTOR_TEST_PWM, 0, MOTOR_TEST_PHASE_MS) ||
        !Diagnostics_RunMotorPhase(0, MOTOR_TEST_PWM, MOTOR_TEST_PHASE_MS) ||
        !Diagnostics_RunMotorPhase(0, -MOTOR_TEST_PWM, MOTOR_TEST_PHASE_MS)) {
        Motor_Stop();
        debug_println("EMERGENCY STOP: motor test aborted.");
        return;
    }
    Encoder_GetSnapshot(&after);
    motor_test_passed = (after.edge_count_a > before.edge_count_a) &&
                        (after.edge_count_b > before.edge_count_b);
    debug_printf("Motor test %s: dEdgeA=%lu dEdgeB=%lu\r\n",
        motor_test_passed ? "OK" : "DEGRADED",
        (unsigned long)(after.edge_count_a - before.edge_count_a),
        (unsigned long)(after.edge_count_b - before.edge_count_b));
    Diagnostics_RefreshReports();
}

static void Diagnostics_OLEDTest(void)
{
    if (!OLED_IsPresent()) {
        debug_println("OLED FAULT: no ACK at address 0x3C or 0x3D.");
        return;
    }
    OLED_ShowTestPattern();
    debug_println("OLED checker pattern sent. Type p=visible/pass, f=fail.");
    oled_visual_passed = Diagnostics_WaitForChar('p', 'f', OPERATOR_TIMEOUT_MS);
    debug_println(oled_visual_passed ? "OLED OK." : "OLED visual test not confirmed.");
    Diagnostics_RefreshReports();
}

static void Diagnostics_PrintEncoder(void)
{
    EncoderSnapshot_t s;
    Encoder_GetSnapshot(&s);
    debug_printf("ENC countA=%ld countB=%ld edgeA=%lu edgeB=%lu errors=%lu\r\n",
        (long)s.count_a, (long)s.count_b, (unsigned long)s.edge_count_a,
        (unsigned long)s.edge_count_b, (unsigned long)s.error_count);
}

static void Diagnostics_PrintGyro(void)
{
    GyroSnapshot_t s;
    Gyro_GetSnapshot(&s);
    debug_printf("GYRO yaw=%.2f pitch=%.2f roll=%.2f valid=%lu bytes=%lu checksum=%lu frame=%lu\r\n",
        (double)s.sample.yaw, (double)s.sample.pitch, (double)s.sample.roll,
        (unsigned long)s.valid_frames, (unsigned long)s.rx_bytes,
        (unsigned long)s.checksum_errors, (unsigned long)s.frame_errors);
}

static void Diagnostics_PrintGray(void)
{
    GrayscaleSnapshot_t s;
    Grayscale_Update();
    Grayscale_GetSnapshot(&s);
    debug_printf("GRAY raw=0x%02X filtered=0x%02X position=%ld scans=%lu errors=%lu\r\n",
        s.raw_state, s.filtered_state, (long)s.position,
        (unsigned long)s.scan_count, (unsigned long)s.error_count);
}

static void Diagnostics_PrintCommunication(void)
{
    DebugStats_t s;
    debug_get_stats(&s);
    debug_printf("COMM tx=%lu rx=%lu format_errors=%lu baud=115200 8N1\r\n",
        (unsigned long)s.tx_bytes, (unsigned long)s.rx_bytes,
        (unsigned long)s.format_errors);
}

void Diagnostics_Init(void)
{
    memset(reports, 0, sizeof(reports));
    motor_test_passed = false;
    oled_visual_passed = false;
    Motor_Stop();
}

void Diagnostics_RunBootSelfTest(void)
{
    Motor_Stop();
    debug_println("\r\n=== MSPM0 CAR M1 DIAGNOSTICS ===");
    debug_println("Boot self-test: motor output is locked.");
    for (uint16_t i = 0U; i < 75U; i++) {
        if ((i % 2U) == 0U) Grayscale_Update();
        delay_ms(10U);
    }
    Diagnostics_RefreshReports();
    Diagnostics_PrintReport();
    Diagnostics_PrintHelp();
}

void Diagnostics_PrintHelp(void)
{
    debug_println("Commands: h=help s=status d=all raw e=encoder y=gyro");
    debug_println("          q=grayscale o=OLED test l=LED info c=comm m=motor test");
    debug_println("          S or space=stop motor at any time");
}

void Diagnostics_PrintReport(void)
{
    Diagnostics_RefreshReports();
    debug_println("--- SELF TEST ---");
    for (uint8_t i = 0U; i < (uint8_t)DIAG_PERIPHERAL_COUNT; i++) {
        debug_printf("%-10s %-8s samples=%lu errors=%lu %s\r\n",
            Diag_PeripheralName((DiagPeripheral_t)i), Diag_StatusName(reports[i].status),
            (unsigned long)reports[i].sample_count, (unsigned long)reports[i].error_count,
            reports[i].detail);
    }
    debug_printf("OVERALL    %s\r\n", Diag_StatusName(
        Diag_WorstStatus(reports, (uint8_t)DIAG_PERIPHERAL_COUNT)));
}

void Diagnostics_Poll(void)
{
    if (!debug_kbhit()) return;
    char command = debug_getc();
    switch (command) {
        case 'h': case 'H': Diagnostics_PrintHelp(); break;
        case 's': Diagnostics_PrintReport(); break;
        case 'd': case 'D':
            Diagnostics_PrintEncoder(); Diagnostics_PrintGyro();
            Diagnostics_PrintGray(); Diagnostics_PrintCommunication(); break;
        case 'e': case 'E': Diagnostics_PrintEncoder(); break;
        case 'y': case 'Y': Diagnostics_PrintGyro(); break;
        case 'q': case 'Q': Diagnostics_PrintGray(); break;
        case 'o': case 'O': Diagnostics_OLEDTest(); break;
        case 'l': case 'L':
            debug_printf("LED configured=%u errors=%lu; carrier LEDs are power-only.\r\n",
                LED_GetConfiguredCount(), (unsigned long)LED_GetErrorCount()); break;
        case 'c': case 'C': Diagnostics_PrintCommunication(); break;
        case 'm': case 'M': Diagnostics_MotorTest(); break;
        case 'S': case ' ': Motor_Stop(); debug_println("EMERGENCY STOP"); break;
        case '\r': case '\n': break;
        default: debug_printf("Unknown command 0x%02X; h=help\r\n", (unsigned char)command); break;
    }
}

const DiagReport_t *Diagnostics_GetReports(void)
{
    return reports;
}
