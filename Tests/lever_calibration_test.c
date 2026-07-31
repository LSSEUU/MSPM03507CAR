#include "lever_calibration.h"

#include "parameter_store.h"
#include "safety.h"
#include "system_state.h"
#include "zdt_stepper.h"
#include "zdt_stepper_test.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static ZdtStepperSnapshot_t device;
static SystemState_t system_state;
static ParameterStoreResult_t save_result;
static uint32_t generation;
static uint32_t status_reads;
static uint32_t config_reads;
static uint32_t encoder_reads;

bool ParameterStore_GetLoadedLeverCalibration(LeverCalibrationValues_t *value)
{
    (void)value;
    return false;
}

void ParameterStore_GetInfo(ParameterStoreInfo_t *info)
{
    memset(info, 0, sizeof(*info));
    info->generation = generation;
}

ParameterStoreResult_t ParameterStore_SaveLeverCalibration(
    const LeverCalibrationValues_t *value, uint32_t expected_generation)
{
    assert(value != NULL);
    assert(expected_generation == generation);
    if (save_result == PARAMETER_STORE_OK) generation++;
    return save_result;
}

uint32_t ParameterStore_Crc32(const void *data, size_t length)
{
    const uint8_t *bytes = data;
    uint32_t crc = 0xFFFFFFFFUL;
    size_t index;
    uint8_t bit;
    for (index = 0U; index < length; index++) {
        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return crc ^ 0xFFFFFFFFUL;
}

SystemState_t SystemState_Get(void) { return system_state; }
bool SystemState_BeginCalibration(void)
{
    if (system_state != SYSTEM_STATE_IDLE) return false;
    system_state = SYSTEM_STATE_CALIBRATION;
    return true;
}
bool SystemState_EndCalibration(void)
{
    if (system_state != SYSTEM_STATE_CALIBRATION) return false;
    system_state = SYSTEM_STATE_IDLE;
    return true;
}
bool Safety_HasFault(void) { return false; }

void ZdtStepperTest_GetSnapshot(ZdtStepperTestSnapshot_t *snapshot,
                                uint32_t now_ms)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->time_ms = now_ms;
    snapshot->state = ZDT_TEST_STATE_IDLE;
}

void ZdtStepper_GetSnapshot(ZdtStepperSnapshot_t *snapshot)
{
    *snapshot = device;
}

ZdtStepperResult_t ZdtStepper_ReadMotorStatus(void)
{
    status_reads++;
    return ZDT_STEPPER_OK;
}

ZdtStepperResult_t ZdtStepper_ReadDriverConfig(void)
{
    config_reads++;
    return ZDT_STEPPER_OK;
}

ZdtStepperResult_t ZdtStepper_ReadEncoder(void)
{
    encoder_reads++;
    return ZDT_STEPPER_OK;
}

int32_t ZdtStepper_CircularDelta(uint16_t value, uint16_t reference)
{
    uint16_t wrapped = (uint16_t)(value - reference);
    return wrapped < 0x8000U ? (int32_t)wrapped : (int32_t)wrapped - 65536L;
}

static void reset_fixture(void)
{
    memset(&device, 0, sizeof(device));
    system_state = SYSTEM_STATE_IDLE;
    save_result = PARAMETER_STORE_OK;
    generation = 7U;
    status_reads = config_reads = encoder_reads = 0U;
    LeverCalibration_Init(0U);
}

static void test_idle_status_tracks_storage_generation(void)
{
    LeverCalibrationStatus_t status;
    reset_fixture();
    LeverCalibration_GetStatus(&status);
    assert(status.generation == 7U);
    assert(status.expected_generation == 7U);
    generation = 8U;
    LeverCalibration_GetStatus(&status);
    assert(status.generation == 8U);
    assert(status.expected_generation == 8U);
}

static void respond_capture(const uint16_t *samples, uint8_t count,
                            uint32_t now_ms)
{
    uint8_t index;
    device.motor_status_valid = true;
    device.motor_status = 0U;
    device.motor_status_sequence++;
    LeverCalibration_Task(now_ms);
    assert(config_reads != 0U);
    device.driver_config_valid = true;
    device.driver_config_fingerprint = 0x12345678UL;
    device.driver_config_sequence++;
    LeverCalibration_Task(now_ms);
    LeverCalibration_Task(now_ms + 10U);
    assert(encoder_reads != 0U);
    for (index = 0U; index < count; index++) {
        device.encoder_valid = true;
        device.encoder_raw = samples[index];
        device.encoder_sequence++;
        LeverCalibration_Task(now_ms);
    }
}

static void test_evaluate_rejections_and_inset(void)
{
    int32_t angles[LEVER_CALIBRATION_POINT_COUNT];
    int16_t captures[2][LEVER_CALIBRATION_REPEAT_COUNT]
                    [LEVER_CALIBRATION_POINT_COUNT];
    LeverCalibrationValues_t candidate;
    uint8_t direction, repeat, point;
    for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
        angles[point] = -8000 + (int32_t)point * 2000;
    for (direction = 0U; direction < 2U; direction++)
        for (repeat = 0U; repeat < LEVER_CALIBRATION_REPEAT_COUNT; repeat++)
            for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
                captures[direction][repeat][point] = (int16_t)(
                    -800 + (int32_t)point * 200 + direction * 2 + repeat - 1);
    assert(LeverCalibration_Evaluate(angles, captures, 10U, 500,
        0x12345678UL, &candidate) == LEVER_CAL_RESULT_OK);
    assert(candidate.soft_min_angle_mdeg == -7200);
    assert(candidate.soft_max_angle_mdeg == 7200);
    assert(candidate.repeatability_mdeg <= 200U);
    assert(candidate.backlash_mdeg <= 300U);

    for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
        angles[point] = -4000 + (int32_t)point * 2000;
    assert(LeverCalibration_Evaluate(angles, captures, 10U, 500,
        0x12345678UL, &candidate) == LEVER_CAL_RESULT_OK);
    assert(candidate.soft_min_angle_mdeg == -3500);
    assert(candidate.soft_max_angle_mdeg == 10800);
    for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
        angles[point] = -8000 + (int32_t)point * 2000;

    captures[0][0][4] = captures[0][0][3];
    captures[0][1][4] = captures[0][1][3];
    captures[0][2][4] = captures[0][2][3];
    assert(LeverCalibration_Evaluate(angles, captures, 10U, 500,
        0x12345678UL, &candidate) == LEVER_CAL_RESULT_NON_MONOTONIC);

    for (direction = 0U; direction < 2U; direction++)
        for (repeat = 0U; repeat < LEVER_CALIBRATION_REPEAT_COUNT; repeat++) {
            uint8_t changed;
            captures[direction][repeat][4] =
                (int16_t)(captures[direction][repeat][3] + 450);
            for (changed = 5U; changed < LEVER_CALIBRATION_POINT_COUNT;
                 changed++)
                captures[direction][repeat][changed] = (int16_t)(
                    captures[direction][repeat][changed - 1U] + 200);
        }
    assert(LeverCalibration_Evaluate(angles, captures, 10U, 500,
        0x12345678UL, &candidate) == LEVER_CAL_RESULT_SLOPE);

    for (direction = 0U; direction < 2U; direction++)
        for (repeat = 0U; repeat < LEVER_CALIBRATION_REPEAT_COUNT; repeat++)
            for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
                captures[direction][repeat][point] = (int16_t)(
                    -800 + (int32_t)point * 200 + direction * 50);
    assert(LeverCalibration_Evaluate(angles, captures, 10U, 500,
        0x12345678UL, &candidate) == LEVER_CAL_RESULT_BACKLASH);
}

static void test_protection_rejects_capture(void)
{
    LeverCalibrationStatus_t status;
    reset_fixture();
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 12U,
        LEVER_CAL_COMMAND_START, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0,
        LEVER_CAL_START_PHONE_ZERO_VERIFIED |
        LEVER_CAL_START_FORCE_FULL, 0U) == LEVER_CAL_RESULT_OK);
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 12U,
        LEVER_CAL_COMMAND_CAPTURE_REFERENCE, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, 1U) ==
        LEVER_CAL_RESULT_OK);
    device.motor_status_valid = true;
    device.motor_status = ZDT_STEPPER_MOTOR_LEFT_LIMIT;
    device.motor_status_sequence++;
    LeverCalibration_Task(1U);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_FAILED);
    assert(status.result == LEVER_CAL_RESULT_MOTOR_PROTECTION);
    assert(config_reads == 0U && encoder_reads == 0U);
}

static void test_state_machine_and_storage_failure(void)
{
    LeverCalibrationStatus_t status;
    uint16_t reference_samples[] = {65530U, 2U, 65534U};
    uint16_t samples[LEVER_CALIBRATION_POINT_SAMPLES];
    uint16_t reference;
    uint8_t index;
    reset_fixture();
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
        LEVER_CAL_COMMAND_START, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 100,
        LEVER_CAL_START_PHONE_ZERO_VERIFIED |
        LEVER_CAL_START_FORCE_FULL, 0U) == LEVER_CAL_RESULT_OK);
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
        LEVER_CAL_COMMAND_CAPTURE_POINT, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, -8000, 0U, 1U) ==
        LEVER_CAL_RESULT_INVALID_REQUEST);
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
        LEVER_CAL_COMMAND_CAPTURE_REFERENCE, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, 1U) ==
        LEVER_CAL_RESULT_OK);
    respond_capture(reference_samples, 3U, 1U);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_WAIT_HORIZONTAL);
    reference = status.candidate.reference_encoder;

    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
        LEVER_CAL_COMMAND_CAPTURE_HORIZONTAL, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, 2U) ==
        LEVER_CAL_RESULT_OK);
    for (index = 0U; index < LEVER_CALIBRATION_POINT_SAMPLES; index++)
        samples[index] = (uint16_t)(reference + 500 + (index & 1U));
    respond_capture(samples, LEVER_CALIBRATION_POINT_SAMPLES, 2U);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_CAPTURE_POINTS);

    while (status.progress < LEVER_CALIBRATION_TOTAL_POINTS) {
        int32_t angle = -8000 + (int32_t)status.expected_point * 2000;
        int32_t counts = -800 + (int32_t)status.expected_point * 200 +
                         status.expected_direction * 2 +
                         (int32_t)status.expected_repeat - 1;
        assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
            LEVER_CAL_COMMAND_CAPTURE_POINT, generation,
            status.expected_point,
            (LeverCalibrationDirection_t)status.expected_direction,
            status.expected_repeat, angle, 0U, 3U) == LEVER_CAL_RESULT_OK);
        for (index = 0U; index < LEVER_CALIBRATION_POINT_SAMPLES; index++)
            samples[index] = (uint16_t)(reference + counts + (index & 1U));
        /* A repeated Task call with no new sequence must not consume a sample. */
        device.motor_status_sequence++;
        LeverCalibration_Task(3U);
        LeverCalibration_Task(3U);
        device.driver_config_sequence++;
        LeverCalibration_Task(3U);
        LeverCalibration_Task(13U);
        for (index = 0U; index < LEVER_CALIBRATION_POINT_SAMPLES; index++) {
            device.encoder_raw = samples[index];
            device.encoder_sequence++;
            LeverCalibration_Task(3U);
        }
        LeverCalibration_GetStatus(&status);
    }
    assert(status.progress == 54U);
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
        LEVER_CAL_COMMAND_REVIEW, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, 4U) ==
        LEVER_CAL_RESULT_OK);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_REVIEW);
    save_result = PARAMETER_STORE_ERR_IO;
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 99U,
        LEVER_CAL_COMMAND_COMMIT, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, 5U) ==
        LEVER_CAL_RESULT_STORAGE);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_FAILED);
}

static void test_timeout_and_cancel(void)
{
    LeverCalibrationStatus_t status;
    reset_fixture();
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 10U,
        LEVER_CAL_COMMAND_START, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0,
        LEVER_CAL_START_PHONE_ZERO_VERIFIED |
        LEVER_CAL_START_FORCE_FULL, 0U) == LEVER_CAL_RESULT_OK);
    LeverCalibration_Task(120001U);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_FAILED);

    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 10U,
        LEVER_CAL_COMMAND_START, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0,
        LEVER_CAL_START_PHONE_ZERO_VERIFIED |
        LEVER_CAL_START_FORCE_FULL, 120002U) == LEVER_CAL_RESULT_OK);
    assert(LeverCalibration_Command(LEVER_CAL_OWNER_HOST, 10U,
        LEVER_CAL_COMMAND_CANCEL, generation, 0U,
        LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, 120003U) ==
        LEVER_CAL_RESULT_OK);
    LeverCalibration_GetStatus(&status);
    assert(status.state == LEVER_CAL_STATE_CANCELLED);
}

int main(void)
{
    test_idle_status_tracks_storage_generation();
    test_evaluate_rejections_and_inset();
    test_state_machine_and_storage_failure();
    test_timeout_and_cancel();
    test_protection_rejects_capture();
    return 0;
}
