#ifndef LEVER_CALIBRATION_H
#define LEVER_CALIBRATION_H

#include "lever_calibration_types.h"

#include <stdbool.h>
#include <stdint.h>

#define LEVER_CALIBRATION_REFERENCE_SAMPLES 3U
#define LEVER_CALIBRATION_POINT_SAMPLES     10U
#define LEVER_CALIBRATION_REPEAT_COUNT      3U
#define LEVER_CALIBRATION_TOTAL_POINTS      54U

#define LEVER_CAL_START_PHONE_ZERO_VERIFIED (1U << 0)
#define LEVER_CAL_START_FORCE_FULL           (1U << 1)

typedef enum {
    LEVER_CAL_STATE_IDLE = 0,
    LEVER_CAL_STATE_WAIT_REFERENCE = 1,
    LEVER_CAL_STATE_WAIT_HORIZONTAL = 2,
    LEVER_CAL_STATE_CAPTURE_POINTS = 3,
    LEVER_CAL_STATE_REVIEW = 4,
    LEVER_CAL_STATE_SAVING = 5,
    LEVER_CAL_STATE_COMPLETE = 6,
    LEVER_CAL_STATE_FAILED = 7,
    LEVER_CAL_STATE_CANCELLED = 8
} LeverCalibrationState_t;

typedef enum {
    LEVER_CAL_OWNER_NONE = 0,
    LEVER_CAL_OWNER_HOST = 1
} LeverCalibrationOwner_t;

typedef enum {
    LEVER_CAL_COMMAND_START = 1,
    LEVER_CAL_COMMAND_CAPTURE_REFERENCE = 2,
    LEVER_CAL_COMMAND_CAPTURE_HORIZONTAL = 3,
    LEVER_CAL_COMMAND_CAPTURE_POINT = 4,
    LEVER_CAL_COMMAND_REVIEW = 5,
    LEVER_CAL_COMMAND_COMMIT = 6,
    LEVER_CAL_COMMAND_CANCEL = 7
} LeverCalibrationCommand_t;

typedef enum {
    LEVER_CAL_DIRECTION_INCREASING = 0,
    LEVER_CAL_DIRECTION_DECREASING = 1
} LeverCalibrationDirection_t;

typedef enum {
    LEVER_CAL_RESULT_OK = 0,
    LEVER_CAL_RESULT_BUSY,
    LEVER_CAL_RESULT_WRONG_STATE,
    LEVER_CAL_RESULT_WRONG_OWNER,
    LEVER_CAL_RESULT_STALE_GENERATION,
    LEVER_CAL_RESULT_INVALID_REQUEST,
    LEVER_CAL_RESULT_TIMEOUT,
    LEVER_CAL_RESULT_MOTOR_NOT_DISABLED,
    LEVER_CAL_RESULT_MOTOR_PROTECTION,
    LEVER_CAL_RESULT_STATUS_STALE,
    LEVER_CAL_RESULT_CONFIG,
    LEVER_CAL_RESULT_UNSTABLE,
    LEVER_CAL_RESULT_NON_MONOTONIC,
    LEVER_CAL_RESULT_REPEATABILITY,
    LEVER_CAL_RESULT_BACKLASH,
    LEVER_CAL_RESULT_SLOPE,
    LEVER_CAL_RESULT_STORAGE,
    LEVER_CAL_RESULT_CANCELLED
} LeverCalibrationResult_t;

typedef struct {
    LeverCalibrationState_t state;
    LeverCalibrationOwner_t owner;
    LeverCalibrationResult_t result;
    uint32_t generation;
    uint32_t expected_generation;
    uint32_t owner_id;
    uint32_t candidate_crc32;
    uint32_t reject_detail;
    int32_t last_capture_counts;
    uint16_t progress;
    uint16_t required;
    uint8_t expected_point;
    uint8_t expected_direction;
    uint8_t expected_repeat;
    uint8_t sample_count;
    bool capture_pending;
    bool persisted_valid;
    bool candidate_valid;
    bool session_zero_valid;
    bool rearm_only;
    LeverCalibrationValues_t candidate;
} LeverCalibrationStatus_t;

void LeverCalibration_Init(uint32_t now_ms);
void LeverCalibration_Task(uint32_t now_ms);
LeverCalibrationResult_t LeverCalibration_Command(
    LeverCalibrationOwner_t owner, uint32_t owner_id,
    LeverCalibrationCommand_t command, uint32_t expected_generation,
    uint8_t point, LeverCalibrationDirection_t direction, uint8_t repeat,
    int32_t angle_mdeg, uint16_t flags, uint32_t now_ms);
void LeverCalibration_CancelHost(uint32_t owner_id, uint32_t now_ms);
void LeverCalibration_GetStatus(LeverCalibrationStatus_t *status);
bool LeverCalibration_IsActive(void);
bool LeverCalibration_GetActive(LeverCalibrationValues_t *values);
bool LeverCalibration_GetSessionReference(uint16_t *reference_encoder,
                                          int32_t *horizontal_counts);
void LeverCalibration_InvalidateDriverConfig(uint32_t actual_fingerprint);

/* Pure validation helper used by the state machine and host tests. */
LeverCalibrationResult_t LeverCalibration_Evaluate(
    const int32_t angles_mdeg[LEVER_CALIBRATION_POINT_COUNT],
    const int16_t captures[2][LEVER_CALIBRATION_REPEAT_COUNT]
                          [LEVER_CALIBRATION_POINT_COUNT],
    uint16_t reference_encoder, int32_t horizontal_offset_counts,
    uint32_t driver_config_fingerprint,
    LeverCalibrationValues_t *candidate);

#endif
