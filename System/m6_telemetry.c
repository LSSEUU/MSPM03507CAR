#include "m6_telemetry.h"

#include "app_line.h"
#include "control_params.h"
#include "heading.h"
#include "safety.h"
#include "scheduler.h"
#include "uart_debug.h"
#include "wheel_speed.h"

#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define M6_TELEMETRY_QUEUE_SIZE 8U

typedef struct {
    uint32_t time_ms;
    uint32_t revision;
    int32_t request[2];
    int32_t ramp[2];
    int32_t raw[2];
    int32_t filtered[2];
    int32_t error[2];
    int32_t proportional[2];
    int32_t integral[2];
    int32_t feedforward[2];
    int32_t pwm[2];
    uint32_t dt_us;
    int32_t encoder_delta[2];
    uint32_t saturation_ms[2];
    int32_t yaw_x100;
    int32_t rate_x100;
    uint32_t rate_source;
    uint32_t gray_mask;
    uint32_t line_info;
    int32_t position_raw_x1000;
    int32_t position_filtered_x1000;
    int32_t derivative_x1000;
    int32_t dynamic_base;
    int32_t correction;
    uint32_t saturation_bits;
    uint32_t fault_bits;
    uint32_t missed_tasks;
    uint32_t rx_dropped;
    uint32_t tx_dropped;
    uint32_t telemetry_dropped;
} M6TelemetrySample_t;

static M6TelemetrySample_t s_queue[M6_TELEMETRY_QUEUE_SIZE];
static uint8_t s_read;
static uint8_t s_write;
static volatile bool s_enabled;
static uint16_t s_period_ms;
static uint32_t s_last_capture_ms;
static uint32_t s_captured;
static uint32_t s_formatted;
static uint32_t s_transmitted;
static uint32_t s_dropped;
static bool s_header_pending;
static char s_formatted_line[640];
static size_t s_formatted_length;
static bool s_formatted_pending;

static const char k_header[] =
    "M6,time_ms,revision,request_l,request_r,ramp_l,ramp_r,raw_l,raw_r,"
    "filter_l,filter_r,error_l,error_r,p_l,p_r,i_l,i_r,ff_l,ff_r,pwm_l,"
    "pwm_r,dt_us,delta_l,delta_r,sat_ms_l,sat_ms_r,yaw_x100,rate_x100,"
    "rate_source,gray_mask,line_info,pos_raw_x1000,pos_filter_x1000,"
    "derivative_x1000,dynamic_base,correction,saturation_bits,fault_bits,"
    "missed_tasks,rx_dropped,tx_dropped,telemetry_dropped\r\n";

static uint8_t next_index(uint8_t index)
{
    return (uint8_t)((index + 1U) % M6_TELEMETRY_QUEUE_SIZE);
}

static int32_t rounded(float value, float scale)
{
    float scaled = value * scale;
    if (scaled >= (float)INT32_MAX) return INT32_MAX;
    if (scaled <= (float)INT32_MIN) return INT32_MIN;
    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

void M6Telemetry_Init(void)
{
    memset(s_queue, 0, sizeof(s_queue));
    s_read = s_write = 0U;
    s_enabled = false;
    s_period_ms = 0U;
    s_last_capture_ms = 0U;
    s_captured = 0U;
    s_formatted = 0U;
    s_transmitted = 0U;
    s_dropped = 0U;
    s_header_pending = false;
    s_formatted_length = 0U;
    s_formatted_pending = false;
}

void M6Telemetry_SetLogging(bool enabled, uint16_t period_ms)
{
    if (enabled && ((period_ms < 20U) || (period_ms > 1000U) ||
                    ((period_ms % 20U) != 0U))) return;
    s_enabled = enabled;
    s_period_ms = enabled ? period_ms : 0U;
    s_last_capture_ms = 0U;
    s_read = s_write = 0U;
    s_header_pending = enabled;
    s_formatted_length = 0U;
    s_formatted_pending = false;
}

void M6Telemetry_StopFromISR(void)
{
    s_enabled = false;
}

bool M6Telemetry_IsLogging(void)
{
    return s_enabled;
}

void M6Telemetry_Capture20ms(uint32_t now_ms)
{
    WheelSpeedSnapshot_t wheel;
    HeadingSnapshot_t heading;
    AppLineSnapshot_t line;
    SafetySnapshot_t safety;
    SchedulerStats_t scheduler;
    DebugStats_t debug;
    M6TelemetrySample_t *sample;
    uint8_t next;
    uint32_t missed = 0U;
    uint32_t index;

    if (!s_enabled) return;
    if ((s_last_capture_ms != 0U) &&
        ((uint32_t)(now_ms - s_last_capture_ms) < s_period_ms)) return;
    s_last_capture_ms = now_ms;
    next = next_index(s_write);
    if (next == s_read) {
        s_dropped++;
        return;
    }

    WheelSpeed_GetSnapshot(&wheel);
    Heading_GetSnapshot(&heading);
    AppLine_GetSnapshot(&line);
    Safety_GetSnapshot(&safety);
    Scheduler_GetStats(&scheduler);
    debug_get_stats(&debug);
    for (index = 0U; index < (uint32_t)SCHEDULER_TASK_COUNT; index++)
        missed += scheduler.miss_count[index];

    sample = &s_queue[s_write];
    memset(sample, 0, sizeof(*sample));
    sample->time_ms = now_ms;
    sample->revision = ControlParams_Revision();
    for (index = 0U; index < 2U; index++) {
        sample->request[index] = rounded(wheel.requested_mm_s[index], 1.0f);
        sample->ramp[index] = rounded(wheel.ramped_mm_s[index], 1.0f);
        sample->raw[index] = rounded(wheel.raw_mm_s[index], 1.0f);
        sample->filtered[index] = rounded(wheel.filtered_mm_s[index], 1.0f);
        sample->error[index] = rounded(wheel.error_mm_s[index], 1.0f);
        sample->proportional[index] = rounded(wheel.proportional_pwm[index], 1.0f);
        sample->integral[index] = rounded(wheel.integral_pwm[index], 1.0f);
        sample->feedforward[index] = rounded(wheel.feedforward_pwm[index], 1.0f);
        sample->pwm[index] = wheel.pwm[index];
        sample->encoder_delta[index] = wheel.encoder_delta[index];
        sample->saturation_ms[index] = wheel.saturation_ms[index];
    }
    sample->dt_us = wheel.actual_dt_us;
    sample->yaw_x100 = rounded(heading.yaw_deg, 100.0f);
    sample->rate_x100 = rounded(heading.yaw_rate_deg_s, 100.0f);
    sample->rate_source = (uint32_t)heading.rate_source;
    sample->gray_mask = line.control.active_mask;
    sample->line_info = ((uint32_t)line.control.active_count << 4U) |
                        (uint32_t)(line.control.cluster_count & 0x0FU);
    sample->position_raw_x1000 = rounded(line.control.raw_position, 1000.0f);
    sample->position_filtered_x1000 = rounded(line.control.position, 1000.0f);
    sample->derivative_x1000 = rounded(line.control.filtered_derivative, 1000.0f);
    sample->dynamic_base = rounded(line.control.dynamic_base_mm_s, 1.0f);
    sample->correction = rounded(line.control.correction_mm_s, 1.0f);
    sample->saturation_bits =
        (wheel.saturated[WHEEL_SPEED_LEFT] ? 1U : 0U) |
        (wheel.saturated[WHEEL_SPEED_RIGHT] ? 2U : 0U) |
        ((line.control.raw_correction_mm_s != line.control.correction_mm_s) ?
            4U : 0U) |
        ((line.control.left_target_mm_s >=
          CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S ||
          line.control.right_target_mm_s >=
          CONTROL_PARAMS_WHEEL_TARGET_HARD_LIMIT_MM_S) ? 8U : 0U);
    sample->fault_bits = safety.fault_bits;
    sample->missed_tasks = missed;
    sample->rx_dropped = debug.rx_dropped;
    sample->tx_dropped = debug.tx_dropped;
    sample->telemetry_dropped = s_dropped;
    s_write = next;
    s_captured++;
}

void M6Telemetry_Service(void)
{
    M6TelemetrySample_t *s;
    int length;

    if (s_header_pending) {
        if (!debug_try_write(k_header, sizeof(k_header) - 1U)) return;
        s_header_pending = false;
    }
    if (!s_formatted_pending) {
        if (s_read == s_write) return;
        s = &s_queue[s_read];
        length = snprintf(s_formatted_line, sizeof(s_formatted_line),
            "M6,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
            "%ld,%ld,%ld,%ld,%ld,%ld,%lu,%ld,%ld,%lu,%lu,%ld,%ld,%lu,%lu,"
            "%lu,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
            (unsigned long)s->time_ms, (unsigned long)s->revision,
            (long)s->request[0], (long)s->request[1],
            (long)s->ramp[0], (long)s->ramp[1],
            (long)s->raw[0], (long)s->raw[1],
            (long)s->filtered[0], (long)s->filtered[1],
            (long)s->error[0], (long)s->error[1],
            (long)s->proportional[0], (long)s->proportional[1],
            (long)s->integral[0], (long)s->integral[1],
            (long)s->feedforward[0], (long)s->feedforward[1],
            (long)s->pwm[0], (long)s->pwm[1],
            (unsigned long)s->dt_us,
            (long)s->encoder_delta[0], (long)s->encoder_delta[1],
            (unsigned long)s->saturation_ms[0],
            (unsigned long)s->saturation_ms[1],
            (long)s->yaw_x100, (long)s->rate_x100,
            (unsigned long)s->rate_source, (unsigned long)s->gray_mask,
            (unsigned long)s->line_info, (long)s->position_raw_x1000,
            (long)s->position_filtered_x1000, (long)s->derivative_x1000,
            (long)s->dynamic_base, (long)s->correction,
            (unsigned long)s->saturation_bits, (unsigned long)s->fault_bits,
            (unsigned long)s->missed_tasks, (unsigned long)s->rx_dropped,
            (unsigned long)s->tx_dropped,
            (unsigned long)s->telemetry_dropped);
        s_formatted++;
        if ((length <= 0) || (length >= (int)sizeof(s_formatted_line))) {
            s_dropped++;
            s_read = next_index(s_read);
            return;
        }
        s_formatted_length = (size_t)length;
        s_formatted_pending = true;
    }
    if (!debug_try_write(s_formatted_line, s_formatted_length)) return;
    s_formatted_pending = false;
    s_formatted_length = 0U;
    s_read = next_index(s_read);
    s_transmitted++;
}

void M6Telemetry_GetStats(M6TelemetryStats_t *stats)
{
    if (stats == NULL) return;
    stats->enabled = s_enabled;
    stats->period_ms = s_period_ms;
    stats->captured = s_captured;
    stats->formatted = s_formatted;
    stats->transmitted = s_transmitted;
    stats->dropped = s_dropped;
    stats->queued = (s_write >= s_read) ? (uint8_t)(s_write - s_read) :
        (uint8_t)(M6_TELEMETRY_QUEUE_SIZE - s_read + s_write);
}
