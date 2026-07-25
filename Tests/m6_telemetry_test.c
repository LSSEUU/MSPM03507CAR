#include "m6_telemetry.h"

#include "app_line.h"
#include "control_params.h"
#include "heading.h"
#include "safety.h"
#include "scheduler.h"
#include "uart_debug.h"
#include "wheel_speed.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static WheelSpeedSnapshot_t wheel;
static HeadingSnapshot_t heading;
static AppLineSnapshot_t app_line;
static SafetySnapshot_t safety;
static SchedulerStats_t scheduler;
static DebugStats_t debug_stats;
static bool uart_accept;
static char uart_output[8192];
static size_t uart_length;

void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot) { *snapshot = wheel; }
void Heading_GetSnapshot(HeadingSnapshot_t *snapshot) { *snapshot = heading; }
void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot) { *snapshot = app_line; }
void Safety_GetSnapshot(SafetySnapshot_t *snapshot) { *snapshot = safety; }
void Scheduler_GetStats(SchedulerStats_t *stats) { *stats = scheduler; }
void debug_get_stats(DebugStats_t *stats) { *stats = debug_stats; }

int debug_try_write(const char *data, size_t length)
{
    if (!uart_accept) return 0;
    assert((uart_length + length) < sizeof(uart_output));
    memcpy(uart_output + uart_length, data, length);
    uart_length += length;
    uart_output[uart_length] = '\0';
    return 1;
}

static unsigned comma_count(const char *start, const char *end)
{
    unsigned count = 0U;
    while (start < end) {
        if (*start == ',') count++;
        start++;
    }
    return count;
}

int main(void)
{
    M6TelemetryStats_t stats;
    const char *header_end;
    const char *row_start;
    const char *row_end;
    uint32_t index;

    ControlParams_Init();
    memset(&wheel, 0, sizeof(wheel));
    memset(&heading, 0, sizeof(heading));
    memset(&app_line, 0, sizeof(app_line));
    memset(&safety, 0, sizeof(safety));
    memset(&scheduler, 0, sizeof(scheduler));
    memset(&debug_stats, 0, sizeof(debug_stats));
    wheel.requested_mm_s[0] = 160.0f;
    wheel.requested_mm_s[1] = 160.0f;
    wheel.actual_dt_us = 5000U;
    heading.yaw_deg = 1.25f;
    heading.yaw_rate_deg_s = -2.5f;
    heading.rate_source = HEADING_RATE_GYRO_FRAME;
    app_line.control.active_mask = 0x18U;
    app_line.control.active_count = 2U;
    app_line.control.cluster_count = 1U;
    app_line.control.dynamic_base_mm_s = 160.0f;

    M6Telemetry_Init();
    assert(!M6Telemetry_IsLogging());
    M6Telemetry_SetLogging(true, 20U);
    assert(M6Telemetry_IsLogging());
    for (index = 0U; index < 10U; index++)
        M6Telemetry_Capture20ms(100U + (index * 20U));
    M6Telemetry_GetStats(&stats);
    assert(stats.captured == 7U);
    assert(stats.formatted == 0U);
    assert(stats.dropped == 3U);
    assert(stats.queued == 7U);

    uart_accept = false;
    M6Telemetry_Service();
    M6Telemetry_GetStats(&stats);
    assert(stats.queued == 7U);
    assert(stats.transmitted == 0U);

    uart_accept = true;
    M6Telemetry_Service();
    M6Telemetry_Capture20ms(400U);
    M6Telemetry_GetStats(&stats);
    assert(stats.formatted == 1U);
    assert(stats.transmitted == 1U);

    uart_accept = false;
    for (index = 0U; index < 20U; index++)
        M6Telemetry_Service();
    M6Telemetry_GetStats(&stats);
    assert(stats.formatted == 2U);
    assert(stats.transmitted == 1U);
    assert(stats.queued == 7U);

    uart_accept = true;
    M6Telemetry_Service();
    M6Telemetry_GetStats(&stats);
    assert(stats.formatted == 2U);
    assert(stats.transmitted == 2U);
    for (index = 0U; index < 10U; index++)
        M6Telemetry_Service();
    M6Telemetry_GetStats(&stats);
    assert(stats.captured == 8U);
    assert(stats.formatted == 8U);
    assert(stats.transmitted == 8U);
    assert(stats.dropped == 3U);
    assert(stats.queued == 0U);

    assert(strncmp(uart_output, "M6,time_ms,revision", 19U) == 0);
    header_end = strstr(uart_output, "\r\n");
    assert(header_end != NULL);
    assert(comma_count(uart_output, header_end) == 41U);
    row_start = header_end + 2;
    row_end = strstr(row_start, "\r\n");
    assert(row_end != NULL);
    assert(comma_count(row_start, row_end) == 41U);
    assert(strstr(uart_output, ",3\r\n") != NULL);

    M6Telemetry_StopFromISR();
    assert(!M6Telemetry_IsLogging());
    M6Telemetry_Capture20ms(500U);
    M6Telemetry_GetStats(&stats);
    assert(!stats.enabled);
    assert(stats.captured == 8U);
    return 0;
}
