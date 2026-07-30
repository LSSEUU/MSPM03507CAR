#include "grayscale_calibration.h"

#include "app_line.h"
#include "app_manual_drive.h"
#include "app_remote.h"
#include "control_params.h"
#include "diagnostics.h"
#include "line_follow.h"
#include "motor.h"
#include "oled.h"
#include "parameter_store.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "wheel_speed.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint32_t test_now;
static uint32_t test_generation;
static SystemState_t test_system_state;
static bool test_fault;
static bool test_save_should_fail;
static uint32_t test_save_calls;
static uint32_t test_apply_calls;
static LineCalibrationValues_t test_active_calibration;
static LineCalibrationValues_t test_saved_calibration;

static void reset_fixture(uint32_t now_ms)
{
    uint32_t channel;
    test_now = now_ms;
    test_generation = 5U;
    test_system_state = SYSTEM_STATE_IDLE;
    test_fault = false;
    test_save_should_fail = false;
    test_save_calls = 0U;
    test_apply_calls = 0U;
    memset(&test_active_calibration, 0, sizeof(test_active_calibration));
    memset(&test_saved_calibration, 0, sizeof(test_saved_calibration));
    for (channel = 0U; channel < GRAYSCALE_CALIBRATION_CHANNEL_COUNT;
         channel++) {
        test_active_calibration.white_adc[channel] = 4094U;
        test_active_calibration.black_adc[channel] = 1U;
    }
    GrayscaleCalibration_Init(now_ms);
}

static GrayscaleCalibrationStatus_t status(void)
{
    GrayscaleCalibrationStatus_t value;
    GrayscaleCalibration_GetStatus(&value);
    return value;
}

static void start_and_capture_white(uint32_t now_ms)
{
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_START, test_generation, now_ms) ==
        GRAYSCALE_CAL_RESULT_OK);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CAPTURE_WHITE, test_generation, now_ms) ==
        GRAYSCALE_CAL_RESULT_OK);
}

static void send_scan(uint32_t scan_count, uint32_t now_ms,
                      uint16_t base, uint32_t error_count, bool ok)
{
    GrayscaleSnapshot_t scan;
    uint32_t channel;
    memset(&scan, 0, sizeof(scan));
    scan.scan_count = scan_count;
    scan.last_update_ms = now_ms;
    scan.error_count = error_count;
    scan.last_scan_ok = ok;
    for (channel = 0U; channel < GRAYSCALE_CALIBRATION_CHANNEL_COUNT;
         channel++)
        scan.adc[channel] = base;
    GrayscaleCalibration_ProcessScan(&scan);
}

static void send_quality_scan(uint32_t scan_count, uint32_t now_ms,
                              bool white_stage)
{
    GrayscaleSnapshot_t scan;
    uint32_t channel;
    memset(&scan, 0, sizeof(scan));
    scan.scan_count = scan_count;
    scan.last_update_ms = now_ms;
    scan.last_scan_ok = true;
    for (channel = 0U; channel < GRAYSCALE_CALIBRATION_CHANNEL_COUNT;
         channel++)
        scan.adc[channel] = white_stage ? 3000U : 800U;
    if (white_stage) {
        if (scan_count == 1U) scan.adc[0] = 4095U;
        scan.adc[3] = (scan_count & 1U) ? 2400U : 3600U;
    } else {
        scan.adc[1] = 3100U;
        scan.adc[2] = 2600U;
    }
    GrayscaleCalibration_ProcessScan(&scan);
}

static void test_valid_capture_is_atomic_and_persistent(void)
{
    GrayscaleCalibrationStatus_t value;
    uint32_t index;
    reset_fixture(100U);
    start_and_capture_white(100U);
    send_scan(1U, 101U, 4094U, 0U, true);
    send_scan(1U, 102U, 4094U, 0U, true);
    assert(status().progress == 1U);
    for (index = 2U; index <= 64U; index++)
        send_scan(index, 100U + index, 4094U, 0U, true);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_WAIT_BLACK);
    assert(test_apply_calls == 0U);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CAPTURE_BLACK, test_generation, 200U) ==
        GRAYSCALE_CAL_RESULT_OK);
    for (index = 65U; index <= 128U; index++)
        send_scan(index, 200U + index, 1U, 0U, true);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_REVIEW);
    assert(value.valid_mask == 0xFFU);
    assert(value.candidate.white_adc[0] == 4094U);
    assert(value.candidate.black_adc[0] == 1U);
    assert(value.span[0] == 4093U);
    assert(value.max_noise[0] == 0U);
    assert(test_save_calls == 0U && test_apply_calls == 0U);
    assert(test_active_calibration.white_adc[0] == 4094U);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CONFIRM, test_generation, 400U) ==
        GRAYSCALE_CAL_RESULT_OK);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_COMPLETE);
    assert(value.owner == GRAYSCALE_CAL_OWNER_NONE);
    assert(value.generation == 6U);
    assert(test_save_calls == 1U && test_apply_calls == 1U);
    assert(test_saved_calibration.white_adc[0] == 4094U);
    assert(test_active_calibration.black_adc[0] == 1U);
    assert(test_system_state == SYSTEM_STATE_IDLE);
}

static void test_quality_masks_and_retry_timeout(void)
{
    GrayscaleCalibrationStatus_t value;
    uint32_t index;
    reset_fixture(0U);
    start_and_capture_white(0U);
    for (index = 1U; index <= 64U; index++)
        send_quality_scan(index, index, true);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CAPTURE_BLACK, test_generation, 100U) ==
        GRAYSCALE_CAL_RESULT_OK);
    for (index = 65U; index <= 128U; index++)
        send_quality_scan(index, 100U + index, false);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_FAILED);
    assert((value.rail_fail_mask & 0x01U) != 0U);
    assert((value.polarity_fail_mask & 0x02U) != 0U);
    assert((value.span_fail_mask & 0x06U) == 0x06U);
    assert((value.noise_fail_mask & 0x08U) != 0U);
    assert(test_apply_calls == 0U && test_save_calls == 0U);
    GrayscaleCalibration_Task(120228U);
    assert(status().state == GRAYSCALE_CAL_STATE_CANCELLED);
    assert(test_system_state == SYSTEM_STATE_IDLE);
}

static void test_scan_errors_capture_timeout_and_clock_wrap(void)
{
    GrayscaleCalibrationStatus_t value;
    uint32_t start = UINT32_MAX - 50U;
    uint32_t deadline = start + 120000U;
    reset_fixture(start);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_START, test_generation, start) ==
        GRAYSCALE_CAL_RESULT_OK);
    GrayscaleCalibration_Task(deadline - 1U);
    assert(GrayscaleCalibration_IsActive());
    GrayscaleCalibration_Task(deadline);
    assert(status().result == GRAYSCALE_CAL_RESULT_TIMEOUT);
    assert(!GrayscaleCalibration_IsActive());

    reset_fixture(0U);
    start_and_capture_white(0U);
    send_scan(1U, 1U, 3000U, 0U, true);
    send_scan(2U, 2U, 3000U, 1U, true);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_FAILED);
    assert(value.scan_error && value.failed_stage == 1U);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CAPTURE_WHITE, test_generation, 10U) ==
        GRAYSCALE_CAL_RESULT_OK);
    GrayscaleCalibration_Task(5010U);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_FAILED);
    assert(value.result == GRAYSCALE_CAL_RESULT_TIMEOUT);
}

static void test_generation_and_storage_failures_preserve_runtime(void)
{
    uint32_t index;
    GrayscaleCalibrationStatus_t value;
    reset_fixture(0U);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_START, 4U, 0U) ==
        GRAYSCALE_CAL_RESULT_STALE_GENERATION);
    start_and_capture_white(0U);
    for (index = 1U; index <= 64U; index++)
        send_scan(index, index, 3000U, 0U, true);
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CAPTURE_BLACK, test_generation, 100U) ==
        GRAYSCALE_CAL_RESULT_OK);
    for (index = 65U; index <= 128U; index++)
        send_scan(index, 100U + index, 800U, 0U, true);
    test_save_should_fail = true;
    assert(GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST, 77U,
        GRAYSCALE_CAL_COMMAND_CONFIRM, test_generation, 300U) ==
        GRAYSCALE_CAL_RESULT_STORAGE);
    value = status();
    assert(value.state == GRAYSCALE_CAL_STATE_FAILED);
    assert(value.failed_stage == 3U);
    assert(test_apply_calls == 0U);
    assert(test_active_calibration.white_adc[0] == 4094U);
}

void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state = APP_LINE_IDLE;
}
bool AppManualDrive_IsIdle(void) { return true; }
void AppRemote_GetSnapshot(AppRemoteSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state = APP_REMOTE_IDLE;
}
bool ControlParams_HasPendingApply(void) { return false; }
bool Diagnostics_ActionIsIdle(void) { return true; }
void WheelSpeed_GetSnapshot(WheelSpeedSnapshot_t *snapshot)
{ memset(snapshot, 0, sizeof(*snapshot)); }
void Safety_GetSnapshot(SafetySnapshot_t *snapshot)
{ memset(snapshot, 0, sizeof(*snapshot)); }
bool Safety_HasFault(void) { return test_fault; }
void Motor_GetStats(MotorStats_t *stats) { memset(stats, 0, sizeof(*stats)); }
bool OLED_IsPresent(void) { return false; }
void OLED_Clear(void) {}
void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{ (void)x; (void)y; (void)text; }
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t number, uint8_t length)
{ (void)x; (void)y; (void)number; (void)length; }
void OLED_RequestRefresh(void) {}
uint32_t SystemTime_Now(void) { return test_now; }
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms)
{ return (int32_t)(now_ms - deadline_ms) >= 0; }
SystemState_t SystemState_Get(void) { return test_system_state; }
bool SystemState_BeginCalibration(void)
{
    if (test_system_state != SYSTEM_STATE_IDLE) return false;
    test_system_state = SYSTEM_STATE_CALIBRATION;
    return true;
}
bool SystemState_EndCalibration(void)
{
    if (test_system_state != SYSTEM_STATE_CALIBRATION) return false;
    test_system_state = SYSTEM_STATE_IDLE;
    return true;
}
bool ParameterStore_GetLoadedCalibration(LineCalibrationValues_t *calibration)
{ (void)calibration; return false; }
void ParameterStore_GetInfo(ParameterStoreInfo_t *info)
{
    memset(info, 0, sizeof(*info));
    info->generation = test_generation;
}
ParameterStoreResult_t ParameterStore_SaveCalibration(
    const LineCalibrationValues_t *calibration, uint32_t expected_generation)
{
    test_save_calls++;
    if (expected_generation != test_generation)
        return PARAMETER_STORE_ERR_STALE_GENERATION;
    if (test_save_should_fail) return PARAMETER_STORE_ERR_VERIFY;
    test_saved_calibration = *calibration;
    test_generation++;
    return PARAMETER_STORE_OK;
}
bool LineFollow_ApplyCalibration(const LineCalibrationValues_t *calibration,
                                 LineCalibrationSource_t source)
{
    (void)source;
    test_apply_calls++;
    test_active_calibration = *calibration;
    return true;
}
void LineFollow_GetCalibrationValues(LineCalibrationValues_t *calibration,
                                     LineCalibrationSource_t *source)
{
    if (calibration != NULL) *calibration = test_active_calibration;
    if (source != NULL) *source = LINE_CALIBRATION_SOURCE_COMPILED;
}
void LineFollow_GetSnapshot(LineFollowSnapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->calibration.source = LINE_CALIBRATION_SOURCE_COMPILED;
    snapshot->calibration.valid_mask = 0xFFU;
    snapshot->calibration.valid = true;
}

int main(void)
{
    test_valid_capture_is_atomic_and_persistent();
    test_quality_masks_and_retry_timeout();
    test_scan_errors_capture_timeout_and_clock_wrap();
    test_generation_and_storage_failures_preserve_runtime();
    return 0;
}
