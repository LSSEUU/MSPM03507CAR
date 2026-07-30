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

#include <stddef.h>
#include <string.h>

#define GRAYSCALE_CAL_WAIT_TIMEOUT_MS 120000UL
#define GRAYSCALE_CAL_CAPTURE_TIMEOUT_MS 5000UL
#define GRAYSCALE_CAL_ADC_MAX 4095U
#define GRAYSCALE_CAL_MIN_SPAN 512U
#define GRAYSCALE_CAL_NOISE_RATIO 8U

typedef struct {
    uint32_t sum[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t minimum[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t maximum[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t mean[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t noise[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint8_t rail_mask;
    uint16_t count;
} StageAccumulator_t;

static GrayscaleCalibrationStatus_t s_status;
static StageAccumulator_t s_white;
static StageAccumulator_t s_black;
static uint32_t s_owner_id;
static uint32_t s_deadline_ms;
static uint32_t s_last_scan_count;
static uint32_t s_error_count_at_capture;
static bool s_error_count_initialized;

static uint16_t minimum_candidate_span(void)
{
    uint16_t minimum = UINT16_MAX;
    uint32_t i;
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        if (s_status.span[i] < minimum) minimum = s_status.span[i];
    }
    return minimum == UINT16_MAX ? 0U : minimum;
}

static void oled_status(void)
{
    if (!OLED_IsPresent()) return;
    OLED_Clear();
    OLED_ShowString(0U, 0U, "IR CHECK");
    if (s_status.state == GRAYSCALE_CAL_STATE_WAIT_WHITE)
        OLED_ShowString(0U, 2U, "BG OK=START");
    else if (s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE)
        OLED_ShowString(0U, 2U, "BG CAPTURE");
    else if (s_status.state == GRAYSCALE_CAL_STATE_WAIT_BLACK)
        OLED_ShowString(0U, 2U, "LINE OK=START");
    else if (s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_BLACK)
        OLED_ShowString(0U, 2U, "LINE CAPTURE");
    else if (s_status.state == GRAYSCALE_CAL_STATE_REVIEW)
        OLED_ShowString(0U, 2U, "CHECK OK=SAVE");
    else if (s_status.state == GRAYSCALE_CAL_STATE_COMPLETE)
        OLED_ShowString(0U, 2U, "FLASH SAVED");
    else if (s_status.state == GRAYSCALE_CAL_STATE_FAILED)
        OLED_ShowString(0U, 2U, "FAILED OK=RETRY");
    else if (s_status.state == GRAYSCALE_CAL_STATE_CANCELLED)
        OLED_ShowString(0U, 2U, "CANCELLED");
    OLED_ShowString(0U, 4U, "N=");
    OLED_ShowNum(12U, 4U, s_status.progress, 2U);
    OLED_ShowString(30U, 4U, "/64 MASK=");
    OLED_ShowNum(84U, 4U,
        (int32_t)(s_status.polarity_fail_mask | s_status.span_fail_mask |
                  s_status.noise_fail_mask | s_status.rail_fail_mask), 3U);
    OLED_ShowString(0U, 6U, "MIN=");
    OLED_ShowNum(24U, 6U, minimum_candidate_span(), 4U);
    OLED_ShowString(54U, 6U, "G=");
    OLED_ShowNum(66U, 6U, (int32_t)s_status.generation, 8U);
    OLED_RequestRefresh();
}

static bool safe_to_begin(void)
{
    AppLineSnapshot_t line;
    AppRemoteSnapshot_t remote;
    WheelSpeedSnapshot_t wheel;
    SafetySnapshot_t safety;
    MotorStats_t motor;
    AppLine_GetSnapshot(&line);
    AppRemote_GetSnapshot(&remote);
    WheelSpeed_GetSnapshot(&wheel);
    Safety_GetSnapshot(&safety);
    Motor_GetStats(&motor);
    return (SystemState_Get() == SYSTEM_STATE_IDLE) && !Safety_HasFault() &&
        (line.state == APP_LINE_IDLE) && AppManualDrive_IsIdle() &&
        (remote.state == APP_REMOTE_IDLE) && Diagnostics_ActionIsIdle() &&
        !ControlParams_HasPendingApply() && !wheel.enabled &&
        !safety.action_active && (safety.requested_motor_a == 0) &&
        (safety.requested_motor_b == 0) && (safety.applied_motor_a == 0) &&
        (safety.applied_motor_b == 0) && (motor.last_a == 0) &&
        (motor.last_b == 0);
}

static void stage_reset(StageAccumulator_t *stage)
{
    uint32_t i;
    memset(stage, 0, sizeof(*stage));
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++)
        stage->minimum[i] = UINT16_MAX;
}

static void stage_finish(StageAccumulator_t *stage)
{
    uint32_t i;
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        stage->mean[i] = (uint16_t)((stage->sum[i] +
            (GRAYSCALE_CALIBRATION_SAMPLE_COUNT / 2U)) /
            GRAYSCALE_CALIBRATION_SAMPLE_COUNT);
        stage->noise[i] = (uint16_t)(stage->maximum[i] - stage->minimum[i]);
    }
}

static void finish_calibration_state(GrayscaleCalibrationState_t state,
                                     GrayscaleCalibrationResult_t result)
{
    s_status.state = state;
    s_status.result = result;
    s_status.owner = GRAYSCALE_CAL_OWNER_NONE;
    s_owner_id = 0U;
    if (SystemState_Get() == SYSTEM_STATE_CALIBRATION)
        (void)SystemState_EndCalibration();
    oled_status();
}

static void evaluate_candidate(void)
{
    uint32_t i;
    s_status.valid_mask = 0U;
    s_status.polarity_fail_mask = 0U;
    s_status.span_fail_mask = 0U;
    s_status.noise_fail_mask = 0U;
    s_status.rail_fail_mask = (uint8_t)(s_white.rail_mask | s_black.rail_mask);
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        uint8_t bit = (uint8_t)(1U << i);
        uint16_t white = s_white.mean[i];
        uint16_t black = s_black.mean[i];
        uint16_t noise = s_white.noise[i] > s_black.noise[i] ?
            s_white.noise[i] : s_black.noise[i];
        uint16_t span = white > black ? (uint16_t)(white - black) : 0U;
        s_status.candidate.white_adc[i] = white;
        s_status.candidate.black_adc[i] = black;
        s_status.span[i] = span;
        s_status.max_noise[i] = noise;
        if (white <= black) s_status.polarity_fail_mask |= bit;
        if (span < GRAYSCALE_CAL_MIN_SPAN) s_status.span_fail_mask |= bit;
        if ((uint32_t)span < ((uint32_t)GRAYSCALE_CAL_NOISE_RATIO * noise))
            s_status.noise_fail_mask |= bit;
        if (((s_status.polarity_fail_mask | s_status.span_fail_mask |
              s_status.noise_fail_mask | s_status.rail_fail_mask) & bit) == 0U)
            s_status.valid_mask |= bit;
    }
    s_status.progress = GRAYSCALE_CALIBRATION_SAMPLE_COUNT;
    if (s_status.valid_mask == 0xFFU) {
        s_status.state = GRAYSCALE_CAL_STATE_REVIEW;
        s_status.result = GRAYSCALE_CAL_RESULT_OK;
        s_status.failed_stage = 0U;
        s_deadline_ms = SystemTime_Now() + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
    } else {
        s_status.state = GRAYSCALE_CAL_STATE_FAILED;
        s_status.result = GRAYSCALE_CAL_RESULT_REJECTED;
        s_status.failed_stage = 2U;
        s_deadline_ms = SystemTime_Now() + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
    }
    oled_status();
}

void GrayscaleCalibration_Init(uint32_t now_ms)
{
    LineCalibrationValues_t persisted;
    LineFollowSnapshot_t line;
    ParameterStoreInfo_t store;
    (void)now_ms;
    memset(&s_status, 0, sizeof(s_status));
    s_status.required = GRAYSCALE_CALIBRATION_SAMPLE_COUNT;
    ParameterStore_GetInfo(&store);
    s_status.generation = store.generation;
    if (ParameterStore_GetLoadedCalibration(&persisted) &&
        LineFollow_ApplyCalibration(&persisted,
                                    LINE_CALIBRATION_SOURCE_PERSISTED))
        s_status.active_source = LINE_CALIBRATION_SOURCE_PERSISTED;
    else
        s_status.active_source = LINE_CALIBRATION_SOURCE_COMPILED;
    LineFollow_GetSnapshot(&line);
    LineFollow_GetCalibrationValues(&s_status.candidate, NULL);
    s_status.active_source = (uint8_t)line.calibration.source;
    {
        uint32_t i;
        for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++)
            s_status.span[i] = s_status.candidate.white_adc[i] >
                s_status.candidate.black_adc[i] ? (uint16_t)(
                    s_status.candidate.white_adc[i] -
                    s_status.candidate.black_adc[i]) : 0U;
        s_status.valid_mask = line.calibration.valid_mask;
    }
}

GrayscaleCalibrationResult_t GrayscaleCalibration_Command(
    GrayscaleCalibrationOwner_t owner, uint32_t owner_id,
    GrayscaleCalibrationCommand_t command, uint32_t expected_generation,
    uint32_t now_ms)
{
    ParameterStoreInfo_t store;
    ParameterStoreResult_t saved;
    if ((owner == GRAYSCALE_CAL_OWNER_NONE) || (owner_id == 0U))
        return GRAYSCALE_CAL_RESULT_WRONG_OWNER;
    ParameterStore_GetInfo(&store);
    if (command == GRAYSCALE_CAL_COMMAND_START) {
        if (s_status.owner != GRAYSCALE_CAL_OWNER_NONE &&
            (s_status.owner != owner || s_owner_id != owner_id))
            return GRAYSCALE_CAL_RESULT_BUSY;
        if (expected_generation != store.generation)
            return GRAYSCALE_CAL_RESULT_STALE_GENERATION;
        if (!safe_to_begin() || !SystemState_BeginCalibration())
            return GRAYSCALE_CAL_RESULT_WRONG_STATE;
        memset(&s_status, 0, sizeof(s_status));
        s_status.state = GRAYSCALE_CAL_STATE_WAIT_WHITE;
        s_status.owner = owner;
        s_status.required = GRAYSCALE_CALIBRATION_SAMPLE_COUNT;
        s_status.generation = store.generation;
        s_status.expected_generation = expected_generation;
        {
            LineCalibrationSource_t source;
            LineFollow_GetCalibrationValues(&s_status.candidate, &source);
            s_status.active_source = (uint8_t)source;
        }
        s_owner_id = owner_id;
        s_deadline_ms = now_ms + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
        return GRAYSCALE_CAL_RESULT_OK;
    }
    if ((s_status.owner != owner) || (s_owner_id != owner_id))
        return GRAYSCALE_CAL_RESULT_WRONG_OWNER;
    if (command == GRAYSCALE_CAL_COMMAND_CANCEL) {
        finish_calibration_state(GRAYSCALE_CAL_STATE_CANCELLED,
                                 GRAYSCALE_CAL_RESULT_CANCELLED);
        return GRAYSCALE_CAL_RESULT_OK;
    }
    if (expected_generation != s_status.expected_generation)
        return GRAYSCALE_CAL_RESULT_STALE_GENERATION;
    if (command == GRAYSCALE_CAL_COMMAND_CAPTURE_WHITE) {
        if ((s_status.state != GRAYSCALE_CAL_STATE_WAIT_WHITE) &&
            !((s_status.state == GRAYSCALE_CAL_STATE_FAILED) &&
              (s_status.failed_stage == 1U)))
            return GRAYSCALE_CAL_RESULT_WRONG_STATE;
        stage_reset(&s_white);
        stage_reset(&s_black);
        s_status.state = GRAYSCALE_CAL_STATE_CAPTURE_WHITE;
        s_status.result = GRAYSCALE_CAL_RESULT_OK;
        s_status.progress = 0U;
        s_status.failed_stage = 0U;
        s_status.scan_error = false;
        s_last_scan_count = UINT32_MAX;
        s_error_count_at_capture = 0U;
        s_error_count_initialized = false;
        s_deadline_ms = now_ms + GRAYSCALE_CAL_CAPTURE_TIMEOUT_MS;
        oled_status();
        return GRAYSCALE_CAL_RESULT_OK;
    }
    if (command == GRAYSCALE_CAL_COMMAND_CAPTURE_BLACK) {
        if ((s_status.state != GRAYSCALE_CAL_STATE_WAIT_BLACK) &&
            !((s_status.state == GRAYSCALE_CAL_STATE_FAILED) &&
              (s_status.failed_stage == 2U)))
            return GRAYSCALE_CAL_RESULT_WRONG_STATE;
        stage_reset(&s_black);
        s_status.state = GRAYSCALE_CAL_STATE_CAPTURE_BLACK;
        s_status.result = GRAYSCALE_CAL_RESULT_OK;
        s_status.progress = 0U;
        s_status.failed_stage = 0U;
        s_status.scan_error = false;
        s_last_scan_count = UINT32_MAX;
        s_error_count_at_capture = 0U;
        s_error_count_initialized = false;
        s_deadline_ms = now_ms + GRAYSCALE_CAL_CAPTURE_TIMEOUT_MS;
        oled_status();
        return GRAYSCALE_CAL_RESULT_OK;
    }
    if (command != GRAYSCALE_CAL_COMMAND_CONFIRM ||
        (s_status.state != GRAYSCALE_CAL_STATE_REVIEW &&
         !((s_status.state == GRAYSCALE_CAL_STATE_FAILED) &&
           (s_status.failed_stage == 3U))))
        return GRAYSCALE_CAL_RESULT_WRONG_STATE;
    s_status.state = GRAYSCALE_CAL_STATE_SAVING;
    oled_status();
    saved = ParameterStore_SaveCalibration(&s_status.candidate,
                                           s_status.expected_generation);
    if (saved == PARAMETER_STORE_ERR_STALE_GENERATION) {
        s_status.state = GRAYSCALE_CAL_STATE_FAILED;
        s_status.result = GRAYSCALE_CAL_RESULT_STALE_GENERATION;
        s_status.failed_stage = 3U;
        s_deadline_ms = now_ms + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
        return s_status.result;
    }
    if (saved != PARAMETER_STORE_OK ||
        !LineFollow_ApplyCalibration(&s_status.candidate,
                                     LINE_CALIBRATION_SOURCE_PERSISTED)) {
        s_status.state = GRAYSCALE_CAL_STATE_FAILED;
        s_status.result = GRAYSCALE_CAL_RESULT_STORAGE;
        s_status.failed_stage = 3U;
        s_deadline_ms = now_ms + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
        return s_status.result;
    }
    ParameterStore_GetInfo(&store);
    s_status.generation = store.generation;
    s_status.active_source = LINE_CALIBRATION_SOURCE_PERSISTED;
    finish_calibration_state(GRAYSCALE_CAL_STATE_COMPLETE,
                             GRAYSCALE_CAL_RESULT_OK);
    return GRAYSCALE_CAL_RESULT_OK;
}

void GrayscaleCalibration_ProcessScan(const GrayscaleSnapshot_t *snapshot)
{
    StageAccumulator_t *stage;
    uint32_t i;
    if (snapshot == NULL ||
        (s_status.state != GRAYSCALE_CAL_STATE_CAPTURE_WHITE &&
         s_status.state != GRAYSCALE_CAL_STATE_CAPTURE_BLACK) ||
        snapshot->scan_count == s_last_scan_count) return;
    s_last_scan_count = snapshot->scan_count;
    if (!snapshot->last_scan_ok) {
        bool white = s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE;
        s_status.scan_error = true;
        s_status.state = GRAYSCALE_CAL_STATE_FAILED;
        s_status.result = GRAYSCALE_CAL_RESULT_SCAN_ERROR;
        s_status.failed_stage = white ? 1U : 2U;
        s_deadline_ms = snapshot->last_update_ms +
            GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
        return;
    }
    if (!s_error_count_initialized) {
        s_error_count_at_capture = snapshot->error_count;
        s_error_count_initialized = true;
    } else if (snapshot->error_count != s_error_count_at_capture) {
        bool white = s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE;
        s_status.scan_error = true;
        s_status.state = GRAYSCALE_CAL_STATE_FAILED;
        s_status.result = GRAYSCALE_CAL_RESULT_SCAN_ERROR;
        s_status.failed_stage = white ? 1U : 2U;
        s_deadline_ms = snapshot->last_update_ms +
            GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
        return;
    }
    stage = (s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE) ?
        &s_white : &s_black;
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        uint16_t value = snapshot->adc[i];
        stage->sum[i] += value;
        if (value < stage->minimum[i]) stage->minimum[i] = value;
        if (value > stage->maximum[i]) stage->maximum[i] = value;
        if ((value == 0U) || (value == GRAYSCALE_CAL_ADC_MAX))
            stage->rail_mask |= (uint8_t)(1U << i);
    }
    stage->count++;
    s_status.progress = stage->count;
    if ((stage->count & 7U) == 0U) oled_status();
    if (stage->count < GRAYSCALE_CALIBRATION_SAMPLE_COUNT) return;
    stage_finish(stage);
    if (s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE) {
        s_status.state = GRAYSCALE_CAL_STATE_WAIT_BLACK;
        s_status.progress = 0U;
        s_deadline_ms = snapshot->last_update_ms + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
    } else {
        evaluate_candidate();
    }
}

void GrayscaleCalibration_Task(uint32_t now_ms)
{
    if (s_status.owner == GRAYSCALE_CAL_OWNER_NONE) return;
    if (Safety_HasFault() || (SystemState_Get() == SYSTEM_STATE_FAULT)) {
        finish_calibration_state(GRAYSCALE_CAL_STATE_CANCELLED,
                                 GRAYSCALE_CAL_RESULT_CANCELLED);
        return;
    }
    if (SystemTime_Reached(now_ms, s_deadline_ms) &&
        (s_status.state == GRAYSCALE_CAL_STATE_WAIT_WHITE ||
         s_status.state == GRAYSCALE_CAL_STATE_WAIT_BLACK ||
         s_status.state == GRAYSCALE_CAL_STATE_REVIEW ||
         s_status.state == GRAYSCALE_CAL_STATE_FAILED)) {
        finish_calibration_state(GRAYSCALE_CAL_STATE_CANCELLED,
                                 GRAYSCALE_CAL_RESULT_TIMEOUT);
    } else if (SystemTime_Reached(now_ms, s_deadline_ms) &&
        (s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE ||
         s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_BLACK)) {
        bool white = s_status.state == GRAYSCALE_CAL_STATE_CAPTURE_WHITE;
        s_status.state = GRAYSCALE_CAL_STATE_FAILED;
        s_status.result = GRAYSCALE_CAL_RESULT_TIMEOUT;
        s_status.failed_stage = white ? 1U : 2U;
        s_deadline_ms = now_ms + GRAYSCALE_CAL_WAIT_TIMEOUT_MS;
        oled_status();
    }
}

void GrayscaleCalibration_CancelHost(uint32_t owner_id, uint32_t now_ms)
{
    if ((s_status.owner == GRAYSCALE_CAL_OWNER_HOST) &&
        (s_owner_id == owner_id))
        (void)GrayscaleCalibration_Command(GRAYSCALE_CAL_OWNER_HOST,
            owner_id, GRAYSCALE_CAL_COMMAND_CANCEL,
            s_status.expected_generation, now_ms);
}

bool GrayscaleCalibration_IsActive(void)
{
    return s_status.owner != GRAYSCALE_CAL_OWNER_NONE;
}

void GrayscaleCalibration_GetStatus(GrayscaleCalibrationStatus_t *status)
{
    if (status != NULL) *status = s_status;
}
