#include "app_h2_panel.h"

#include "app_line.h"
#include "control_config.h"
#include "grayscale_calibration.h"
#include "heading.h"
#include "key.h"
#include "oled.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "vehicle_config.h"
#include "wheel_speed.h"

#include <stddef.h>
#include <string.h>

#define APP_H2_KEY_DEBOUNCE_SAMPLES                 3U
#define APP_H2_STATIONARY_SPEED_MM_S               5.0f
#define APP_H2_TIME_MAX_CENTISECONDS             99999U

typedef struct {
    AppH2PanelSnapshot_t snapshot;
    bool raw_pressed;
    uint8_t stable_samples;
    bool display_dirty;
    bool calibration_was_active;
} AppH2PanelState_t;

static AppH2PanelState_t panel;

static float AppH2Panel_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static AppH2StartStatus_t AppH2Panel_Preflight(uint32_t now_ms)
{
    AppLineSnapshot_t line;
    HeadingSnapshot_t heading;
    WheelSpeedSnapshot_t wheel;

    AppLine_GetSnapshot(&line);
    Heading_GetSnapshot(&heading);
    WheelSpeed_GetSnapshot(&wheel);
    if (Safety_HasFault()) return APP_H2_START_FAULT;
    if ((SystemState_Get() != SYSTEM_STATE_IDLE) ||
        (line.state != APP_LINE_IDLE))
        return APP_H2_START_NOT_IDLE;
    if (GrayscaleCalibration_IsActive())
        return APP_H2_START_CALIBRATION;
    if (!wheel.sample_valid || wheel.enabled ||
        (AppH2Panel_Abs(wheel.requested_mm_s[WHEEL_SPEED_LEFT]) > 0.5f) ||
        (AppH2Panel_Abs(wheel.requested_mm_s[WHEEL_SPEED_RIGHT]) > 0.5f) ||
        (AppH2Panel_Abs(wheel.ramped_mm_s[WHEEL_SPEED_LEFT]) > 0.5f) ||
        (AppH2Panel_Abs(wheel.ramped_mm_s[WHEEL_SPEED_RIGHT]) > 0.5f) ||
        (AppH2Panel_Abs(wheel.filtered_mm_s[WHEEL_SPEED_LEFT]) >
         APP_H2_STATIONARY_SPEED_MM_S) ||
        (AppH2Panel_Abs(wheel.filtered_mm_s[WHEEL_SPEED_RIGHT]) >
         APP_H2_STATIONARY_SPEED_MM_S))
        return APP_H2_START_MOVING;
    if (!line.control.calibration.valid || !line.control.line_valid ||
        ((uint32_t)(now_ms - line.control.last_update_ms) >
         GRAYSCALE_TIMEOUT_MS))
        return APP_H2_START_LINE_INVALID;
    if (line.control.ambiguous || (line.control.active_mask != 0x18U))
        return APP_H2_START_NOT_CENTERED;
    if (!heading.valid || !heading.calibrated ||
        (heading.mode != HEADING_MODE_IDLE) ||
        (heading.rate_source != HEADING_RATE_GYRO_FRAME) ||
        ((uint32_t)(now_ms - heading.last_rate_update_ms) >
         LINE_YAW_ASSIST_GYRO_FRESH_MS))
        return APP_H2_START_GYRO_INVALID;
    return APP_H2_START_READY;
}

static const char *AppH2Panel_StatusText(AppH2StartStatus_t status)
{
    switch (status) {
        case APP_H2_START_FAULT: return "FAULT ACTIVE";
        case APP_H2_START_NOT_IDLE: return "SYSTEM BUSY";
        case APP_H2_START_CALIBRATION: return "CAL ACTIVE";
        case APP_H2_START_MOVING: return "CAR MOVING";
        case APP_H2_START_LINE_INVALID: return "LINE WAIT";
        case APP_H2_START_NOT_CENTERED: return "CENTER LINE";
        case APP_H2_START_GYRO_INVALID: return "GYRO WAIT";
        case APP_H2_START_REJECTED: return "START REJECT";
        default: return "READY";
    }
}

static void AppH2Panel_ShowTime(uint32_t elapsed_ms)
{
    uint32_t centiseconds = elapsed_ms / 10U;
    uint32_t seconds;
    uint32_t fraction;
    uint8_t x = 30U;
    if (centiseconds > APP_H2_TIME_MAX_CENTISECONDS)
        centiseconds = APP_H2_TIME_MAX_CENTISECONDS;
    seconds = centiseconds / 100U;
    fraction = centiseconds % 100U;
    OLED_ShowString(0U, 4U, "TIME");
    OLED_ShowChar(x, 4U, (char)('0' + ((seconds / 100U) % 10U)));
    x = (uint8_t)(x + 6U);
    OLED_ShowChar(x, 4U, (char)('0' + ((seconds / 10U) % 10U)));
    x = (uint8_t)(x + 6U);
    OLED_ShowChar(x, 4U, (char)('0' + (seconds % 10U)));
    x = (uint8_t)(x + 6U);
    OLED_ShowChar(x, 4U, ':');
    x = (uint8_t)(x + 6U);
    OLED_ShowChar(x, 4U, (char)('0' + (fraction / 10U)));
    x = (uint8_t)(x + 6U);
    OLED_ShowChar(x, 4U, (char)('0' + (fraction % 10U)));
}

static void AppH2Panel_Render(AppH2PanelDisplay_t display,
                              AppH2StartStatus_t status,
                              uint32_t elapsed_ms)
{
    if (!OLED_IsPresent()) return;
    OLED_Clear();
    if (display == APP_H2_PANEL_RUNNING) {
        OLED_ShowString(0U, 0U, "H2 RUNNING");
        OLED_ShowString(0U, 2U, "AUTO STOP ON");
        AppH2Panel_ShowTime(elapsed_ms);
        OLED_ShowString(0U, 6U, "WAIT FOR LINE");
    } else if (display == APP_H2_PANEL_FINISHED) {
        OLED_ShowString(0U, 0U, "H2 FINISHED");
        OLED_ShowString(0U, 2U, "POINT ON LINE");
        AppH2Panel_ShowTime(elapsed_ms);
        OLED_ShowString(0U, 6U, "K1 NEXT LAP");
    } else if (display == APP_H2_PANEL_STOPPED) {
        OLED_ShowString(0U, 0U, "H2 STOPPED");
        OLED_ShowString(0U, 2U, "CHECK SAFETY");
        AppH2Panel_ShowTime(elapsed_ms);
    } else if (display == APP_H2_PANEL_READY) {
        OLED_ShowString(0U, 0U, "H2 READY");
        OLED_ShowString(0U, 2U, "K1 START");
        OLED_ShowString(0U, 4U, "LINE CENTER");
        OLED_ShowString(0U, 6U, "AUTO STOP ON");
    } else {
        OLED_ShowString(0U, 0U, "H2 WAIT");
        OLED_ShowString(0U, 2U, AppH2Panel_StatusText(status));
        OLED_ShowString(0U, 4U, "K1 WHEN READY");
    }
    OLED_RequestRefresh();
}

void AppH2Panel_Init(uint32_t now_ms)
{
    (void)now_ms;
    memset(&panel, 0, sizeof(panel));
    panel.raw_pressed = Key1_IsPressed();
    panel.snapshot.debounced_pressed = panel.raw_pressed;
    panel.snapshot.release_armed = !panel.raw_pressed;
    panel.stable_samples = 1U;
    panel.snapshot.display = APP_H2_PANEL_WAIT;
    panel.snapshot.start_status = APP_H2_START_LINE_INVALID;
    panel.display_dirty = true;
}

void AppH2Panel_Task20ms(uint32_t now_ms)
{
    bool pressed = Key1_IsPressed();
    AppH2StartStatus_t status;
    if (pressed == panel.raw_pressed) {
        if (panel.stable_samples < APP_H2_KEY_DEBOUNCE_SAMPLES)
            panel.stable_samples++;
    } else {
        panel.raw_pressed = pressed;
        panel.stable_samples = 1U;
    }
    if ((panel.stable_samples < APP_H2_KEY_DEBOUNCE_SAMPLES) ||
        (pressed == panel.snapshot.debounced_pressed))
        return;
    panel.snapshot.debounced_pressed = pressed;
    if (!pressed) {
        panel.snapshot.release_armed = true;
        return;
    }
    if (!panel.snapshot.release_armed) return;
    panel.snapshot.release_armed = false;
    panel.snapshot.key_presses++;
    status = AppH2Panel_Preflight(now_ms);
    if ((status == APP_H2_START_READY) && AppLine_StartH2(now_ms)) {
        panel.snapshot.starts++;
        panel.snapshot.display = APP_H2_PANEL_RUNNING;
        panel.snapshot.start_status = APP_H2_START_READY;
        panel.snapshot.displayed_elapsed_ms = 0U;
    } else {
        if (status == APP_H2_START_READY)
            status = APP_H2_START_REJECTED;
        panel.snapshot.rejects++;
        panel.snapshot.display = APP_H2_PANEL_WAIT;
        panel.snapshot.start_status = status;
    }
    panel.display_dirty = true;
}

void AppH2Panel_Task100ms(uint32_t now_ms)
{
    AppLineSnapshot_t line;
    AppH2PanelDisplay_t display;
    AppH2StartStatus_t status;
    uint32_t elapsed_ms = 0U;
    uint32_t elapsed_centiseconds;
    uint32_t displayed_centiseconds;
    bool active_calibration = GrayscaleCalibration_IsActive();

    if (active_calibration) {
        panel.calibration_was_active = true;
        return;
    }
    if (panel.calibration_was_active) {
        panel.calibration_was_active = false;
        panel.display_dirty = true;
    }
    AppLine_GetSnapshot(&line);
    status = AppH2Panel_Preflight(now_ms);
    if ((line.state == APP_LINE_RUNNING) && line.h2_finish_enabled) {
        display = APP_H2_PANEL_RUNNING;
        elapsed_ms = (uint32_t)(now_ms - line.started_ms);
        status = APP_H2_START_NOT_IDLE;
    } else if (line.finish_complete) {
        display = APP_H2_PANEL_FINISHED;
        elapsed_ms = line.elapsed_ms;
        status = APP_H2_START_READY;
    } else if (panel.snapshot.display == APP_H2_PANEL_RUNNING) {
        display = APP_H2_PANEL_STOPPED;
        elapsed_ms = (uint32_t)(now_ms - line.started_ms);
        status = Safety_HasFault() ? APP_H2_START_FAULT :
            APP_H2_START_REJECTED;
    } else {
        display = (status == APP_H2_START_READY) ?
            APP_H2_PANEL_READY : APP_H2_PANEL_WAIT;
    }
    elapsed_centiseconds = elapsed_ms / 10U;
    displayed_centiseconds = panel.snapshot.displayed_elapsed_ms / 10U;
    if (panel.display_dirty || (display != panel.snapshot.display) ||
        (status != panel.snapshot.start_status) ||
        ((display == APP_H2_PANEL_RUNNING) &&
         (elapsed_centiseconds != displayed_centiseconds)) ||
        ((display == APP_H2_PANEL_FINISHED) &&
         (elapsed_ms != panel.snapshot.displayed_elapsed_ms))) {
        AppH2Panel_Render(display, status, elapsed_ms);
        panel.snapshot.display = display;
        panel.snapshot.start_status = status;
        panel.snapshot.displayed_elapsed_ms = elapsed_ms;
        panel.display_dirty = false;
    }
}

void AppH2Panel_GetSnapshot(AppH2PanelSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = panel.snapshot;
}
