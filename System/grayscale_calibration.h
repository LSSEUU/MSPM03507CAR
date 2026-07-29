#ifndef GRAYSCALE_CALIBRATION_H
#define GRAYSCALE_CALIBRATION_H

#include "grayscale.h"
#include "grayscale_calibration_types.h"

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE_CALIBRATION_SAMPLE_COUNT 64U

typedef enum {
    GRAYSCALE_CAL_STATE_IDLE = 0,
    GRAYSCALE_CAL_STATE_WAIT_WHITE = 1,
    GRAYSCALE_CAL_STATE_CAPTURE_WHITE = 2,
    GRAYSCALE_CAL_STATE_WAIT_BLACK = 3,
    GRAYSCALE_CAL_STATE_CAPTURE_BLACK = 4,
    GRAYSCALE_CAL_STATE_REVIEW = 5,
    GRAYSCALE_CAL_STATE_SAVING = 6,
    GRAYSCALE_CAL_STATE_COMPLETE = 7,
    GRAYSCALE_CAL_STATE_FAILED = 8,
    GRAYSCALE_CAL_STATE_CANCELLED = 9
} GrayscaleCalibrationState_t;

typedef enum {
    GRAYSCALE_CAL_OWNER_NONE = 0,
    GRAYSCALE_CAL_OWNER_HOST = 1,
    GRAYSCALE_CAL_OWNER_REMOTE = 2
} GrayscaleCalibrationOwner_t;

typedef enum {
    GRAYSCALE_CAL_COMMAND_START = 1,
    GRAYSCALE_CAL_COMMAND_CAPTURE_WHITE = 2,
    GRAYSCALE_CAL_COMMAND_CAPTURE_BLACK = 3,
    GRAYSCALE_CAL_COMMAND_CONFIRM = 4,
    GRAYSCALE_CAL_COMMAND_CANCEL = 5
} GrayscaleCalibrationCommand_t;

typedef enum {
    GRAYSCALE_CAL_RESULT_OK = 0,
    GRAYSCALE_CAL_RESULT_BUSY,
    GRAYSCALE_CAL_RESULT_WRONG_STATE,
    GRAYSCALE_CAL_RESULT_WRONG_OWNER,
    GRAYSCALE_CAL_RESULT_STALE_GENERATION,
    GRAYSCALE_CAL_RESULT_TIMEOUT,
    GRAYSCALE_CAL_RESULT_SCAN_ERROR,
    GRAYSCALE_CAL_RESULT_REJECTED,
    GRAYSCALE_CAL_RESULT_STORAGE,
    GRAYSCALE_CAL_RESULT_CANCELLED
} GrayscaleCalibrationResult_t;

typedef struct {
    GrayscaleCalibrationState_t state;
    GrayscaleCalibrationOwner_t owner;
    uint8_t active_source;
    GrayscaleCalibrationResult_t result;
    uint16_t progress;
    uint16_t required;
    uint32_t generation;
    uint32_t expected_generation;
    uint8_t valid_mask;
    uint8_t polarity_fail_mask;
    uint8_t span_fail_mask;
    uint8_t noise_fail_mask;
    uint8_t rail_fail_mask;
    uint8_t failed_stage;
    bool scan_error;
    LineCalibrationValues_t candidate;
    uint16_t span[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t max_noise[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
} GrayscaleCalibrationStatus_t;

void GrayscaleCalibration_Init(uint32_t now_ms);
GrayscaleCalibrationResult_t GrayscaleCalibration_Command(
    GrayscaleCalibrationOwner_t owner, uint32_t owner_id,
    GrayscaleCalibrationCommand_t command, uint32_t expected_generation,
    uint32_t now_ms);
void GrayscaleCalibration_ProcessScan(const GrayscaleSnapshot_t *snapshot);
void GrayscaleCalibration_Task(uint32_t now_ms);
void GrayscaleCalibration_CancelHost(uint32_t owner_id, uint32_t now_ms);
bool GrayscaleCalibration_IsActive(void);
void GrayscaleCalibration_GetStatus(GrayscaleCalibrationStatus_t *status);

#endif
