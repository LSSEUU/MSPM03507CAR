#include "app_h2_panel.h"

#include "app_line.h"
#include "grayscale_calibration.h"
#include "heading.h"
#include "key.h"
#include "oled.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "wheel_speed.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define TEST_SCREEN_COLUMNS 21U

static bool test_key_pressed;
static bool test_key2_pressed;
static bool test_key3_pressed;
static bool test_fault;
static bool test_calibration_active;
static bool test_start_allowed;
static uint32_t test_start_calls;
static uint32_t test_refreshes;
static SystemState_t test_system_state;
static AppLineSnapshot_t test_line;
static HeadingSnapshot_t test_heading;
static WheelSpeedSnapshot_t test_wheel;
static char test_screen[OLED_PAGES][TEST_SCREEN_COLUMNS + 1U];

static void clear_screen_model(void)
{
    uint32_t page;
    for (page = 0U; page < OLED_PAGES; page++) {
        memset(test_screen[page], ' ', TEST_SCREEN_COLUMNS);
        test_screen[page][TEST_SCREEN_COLUMNS] = '\0';
    }
}

static void reset_fixture(bool key_held)
{
    memset(&test_line, 0, sizeof(test_line));
    memset(&test_heading, 0, sizeof(test_heading));
    memset(&test_wheel, 0, sizeof(test_wheel));
    test_key_pressed = key_held;
    test_key2_pressed = false;
    test_key3_pressed = false;
    test_fault = false;
    test_calibration_active = false;
    test_start_allowed = true;
    test_start_calls = 0U;
    test_refreshes = 0U;
    test_system_state = SYSTEM_STATE_IDLE;
    test_line.state = APP_LINE_IDLE;
    test_line.control.calibration.valid = true;
    test_line.control.line_valid = true;
    test_line.control.active_mask = 0x18U;
    test_line.control.last_update_ms = 1000U;
    test_heading.valid = true;
    test_heading.calibrated = true;
    test_heading.mode = HEADING_MODE_IDLE;
    test_heading.rate_source = HEADING_RATE_GYRO_FRAME;
    test_heading.last_rate_update_ms = 1000U;
    test_wheel.sample_valid = true;
    clear_screen_model();
    HMission_Init();
    AppH2Panel_Init(1000U);
}

static void stable_key(bool pressed, uint32_t *now_ms)
{
    uint32_t sample;
    test_key_pressed = pressed;
    for (sample = 0U; sample < 3U; sample++) {
        *now_ms += 20U;
        test_line.control.last_update_ms = *now_ms;
        test_heading.last_rate_update_ms = *now_ms;
        AppH2Panel_Task20ms(*now_ms);
    }
}

static void stable_mode_key(bool key2, bool key3, uint32_t *now_ms)
{
    uint32_t sample;
    test_key2_pressed = key2;
    test_key3_pressed = key3;
    for (sample = 0U; sample < 3U; sample++) {
        *now_ms += 20U;
        test_line.control.last_update_ms = *now_ms;
        test_heading.last_rate_update_ms = *now_ms;
        AppH2Panel_Task20ms(*now_ms);
    }
}

bool Key1_IsPressed(void) { return test_key_pressed; }
bool Key2_IsPressed(void) { return test_key2_pressed; }
bool Key3_IsPressed(void) { return test_key3_pressed; }
bool Safety_HasFault(void) { return test_fault; }
bool GrayscaleCalibration_IsActive(void)
{ return test_calibration_active; }
SystemState_t SystemState_Get(void) { return test_system_state; }
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms)
{ return (int32_t)(now_ms - deadline_ms) >= 0; }

void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot)
{ *snapshot = test_line; }

bool AppLine_StartH2(uint32_t now_ms)
{
    test_start_calls++;
    if (!test_start_allowed) return false;
    test_line.state = APP_LINE_RUNNING;
    test_line.h2_finish_enabled = true;
    test_line.finish_complete = false;
    test_line.started_ms = now_ms;
    test_system_state = SYSTEM_STATE_RUN;
    return true;
}

bool AppLine_StartMission(HMissionMode_t mode, uint32_t now_ms)
{
    HMission_Start(mode, now_ms);
    return AppLine_StartH2(now_ms);
}

void Heading_GetSnapshot(HeadingSnapshot_t *snapshot)
{ *snapshot = test_heading; }
void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot)
{ *snapshot = test_wheel; }

bool OLED_IsPresent(void) { return true; }
void OLED_Clear(void) { clear_screen_model(); }
void OLED_ShowChar(uint8_t x, uint8_t y, char c)
{
    uint8_t column = (uint8_t)(x / 6U);
    if ((y < OLED_PAGES) && (column < TEST_SCREEN_COLUMNS))
        test_screen[y][column] = c;
}
void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        OLED_ShowChar(x, y, *text++);
        x = (uint8_t)(x + 6U);
    }
}
void OLED_RequestRefresh(void) { test_refreshes++; }

static AppH2PanelSnapshot_t snapshot(void)
{
    AppH2PanelSnapshot_t value;
    AppH2Panel_GetSnapshot(&value);
    return value;
}

static void test_boot_with_held_key_requires_release(void)
{
    uint32_t now_ms = 1000U;
    reset_fixture(true);
    stable_key(true, &now_ms);
    assert(test_start_calls == 0U);
    stable_key(false, &now_ms);
    stable_key(true, &now_ms);
    assert(test_start_calls == 1U);
    assert(snapshot().starts == 1U);
    stable_key(true, &now_ms);
    assert(test_start_calls == 1U);
}

static void test_preflight_rejects_bad_center_gyro_motion_and_fault(void)
{
    uint32_t now_ms = 1000U;
    AppH2PanelSnapshot_t value;
    reset_fixture(false);

    test_line.control.active_mask = 0x08U;
    stable_key(true, &now_ms);
    value = snapshot();
    assert(value.start_status == APP_H2_START_NOT_CENTERED);
    stable_key(false, &now_ms);

    test_line.control.active_mask = 0x18U;
    test_heading.rate_source = HEADING_RATE_ANGLE_DIFFERENCE;
    stable_key(true, &now_ms);
    assert(snapshot().start_status == APP_H2_START_GYRO_INVALID);
    stable_key(false, &now_ms);

    test_heading.rate_source = HEADING_RATE_GYRO_FRAME;
    test_heading.last_rate_update_ms = now_ms;
    test_wheel.filtered_mm_s[WHEEL_SPEED_LEFT] = 6.0f;
    stable_key(true, &now_ms);
    assert(snapshot().start_status == APP_H2_START_MOVING);
    stable_key(false, &now_ms);

    test_wheel.filtered_mm_s[WHEEL_SPEED_LEFT] = 0.0f;
    test_fault = true;
    stable_key(true, &now_ms);
    assert(snapshot().start_status == APP_H2_START_FAULT);
    assert(snapshot().rejects == 4U);
    assert(test_start_calls == 0U);
}

static void test_successful_press_and_oled_timing(void)
{
    uint32_t now_ms = 1000U;
    uint32_t refreshes;
    reset_fixture(false);
    AppH2Panel_Task100ms(now_ms);
    assert(strncmp(test_screen[0], "H2 READY", 8U) == 0);
    refreshes = test_refreshes;
    AppH2Panel_Task100ms(now_ms + 100U);
    assert(test_refreshes == refreshes);

    stable_key(true, &now_ms);
    assert(snapshot().starts == 1U);
    now_ms = test_line.started_ms + 12345U;
    AppH2Panel_Task100ms(now_ms);
    assert(strncmp(test_screen[0], "H2 RUNNING", 10U) == 0);
    assert(strncmp(test_screen[4], "TIME 012:34", 11U) == 0);

    test_line.state = APP_LINE_IDLE;
    test_line.finish_complete = true;
    test_line.elapsed_ms = 19148U;
    test_system_state = SYSTEM_STATE_IDLE;
    AppH2Panel_Task100ms(now_ms + 100U);
    assert(strncmp(test_screen[0], "H2 FINISHED", 11U) == 0);
    assert(strncmp(test_screen[4], "TIME 019:14", 11U) == 0);
    assert(snapshot().displayed_elapsed_ms == 19148U);
}

static void test_calibration_temporarily_owns_oled(void)
{
    uint32_t refreshes;
    reset_fixture(false);
    AppH2Panel_Task100ms(1000U);
    refreshes = test_refreshes;
    test_calibration_active = true;
    AppH2Panel_Task100ms(1100U);
    assert(test_refreshes == refreshes);
    test_calibration_active = false;
    AppH2Panel_Task100ms(1200U);
    assert(test_refreshes == refreshes + 1U);
}

static void test_k2_k3_switch_only_while_idle(void)
{
    uint32_t now_ms = 1000U;
    reset_fixture(false);
    stable_mode_key(true, false, &now_ms);
    assert(snapshot().selected_mode == H_MISSION_MODE_H5);
    assert(snapshot().mode_changes == 1U);
    stable_mode_key(false, false, &now_ms);
    stable_mode_key(false, true, &now_ms);
    assert(snapshot().selected_mode == H_MISSION_MODE_H2);
    assert(snapshot().mode_changes == 2U);
    stable_mode_key(false, false, &now_ms);
    test_system_state = SYSTEM_STATE_RUN;
    test_line.state = APP_LINE_RUNNING;
    stable_mode_key(false, true, &now_ms);
    assert(snapshot().selected_mode == H_MISSION_MODE_H2);
    assert(snapshot().mode_changes == 2U);
}

int main(void)
{
    test_boot_with_held_key_requires_release();
    test_preflight_rejects_bad_center_gyro_motion_and_fault();
    test_successful_press_and_oled_timing();
    test_calibration_temporarily_owns_oled();
    test_k2_k3_switch_only_while_idle();
    return 0;
}
