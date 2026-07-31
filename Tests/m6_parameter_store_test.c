#include "control_params.h"
#include "parameter_store.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void test_crc(void)
{
    static const char input[] = "123456789";
    assert(ParameterStore_Crc32(input, sizeof(input) - 1U) == 0xCBF43926UL);
}

static void test_record_validation(void)
{
    uint8_t record[PARAMETER_STORE_RECORD_SIZE];
    uint8_t damaged[PARAMETER_STORE_RECORD_SIZE];
    ParameterStoreDecoded_t decoded;

    ParameterStore_BuildRecord(record, ControlParams_Defaults(), 7U, 11U,
                               true);
    assert(ParameterStore_DecodeRecord(record, sizeof(record), &decoded));
    assert(decoded.valid);
    assert(decoded.generation == 7U);
    assert(decoded.parameter_revision == 11U);
    assert(decoded.params.wheel.left_kp ==
           ControlParams_Defaults()->wheel.left_kp);

    memcpy(damaged, record, sizeof(damaged));
    damaged[32] ^= 0x01U;
    assert(!ParameterStore_DecodeRecord(damaged, sizeof(damaged), &decoded));

    memcpy(damaged, record, sizeof(damaged));
    damaged[4] = 2U;
    assert(!ParameterStore_DecodeRecord(damaged, sizeof(damaged), &decoded));

    ParameterStore_BuildRecord(damaged, ControlParams_Defaults(), 8U, 12U,
                               false);
    assert(!ParameterStore_DecodeRecord(damaged, sizeof(damaged), &decoded));
    assert(!ParameterStore_DecodeRecord(record,
                                        PARAMETER_STORE_RECORD_V1_SIZE - 1U,
                                        &decoded));
}

static void test_v2_calibration_round_trip(void)
{
    uint8_t record[PARAMETER_STORE_RECORD_SIZE];
    uint8_t damaged[PARAMETER_STORE_RECORD_SIZE];
    LineCalibrationValues_t calibration = {
        {3000U, 3010U, 3020U, 3030U, 3040U, 3050U, 3060U, 3070U},
        { 500U,  510U,  520U,  530U,  540U,  550U,  560U,  570U}
    };
    ParameterStoreDecoded_t decoded;

    ParameterStore_BuildRecordV2(record, ControlParams_Defaults(),
        &calibration, true, 12U, 9U, true);
    assert(ParameterStore_DecodeRecord(record, sizeof(record), &decoded));
    assert(decoded.valid && decoded.calibration_valid);
    assert(decoded.generation == 12U && decoded.parameter_revision == 9U);
    assert(memcmp(&decoded.calibration, &calibration,
                  sizeof(calibration)) == 0);
    assert(!ParameterStore_DecodeRecord(record,
                                        PARAMETER_STORE_RECORD_V2_SIZE - 1U,
                                        &decoded));

    memcpy(damaged, record, sizeof(damaged));
    damaged[180] ^= 1U;
    assert(!ParameterStore_DecodeRecord(damaged, sizeof(damaged), &decoded));
    calibration.white_adc[3] = calibration.black_adc[3];
    ParameterStore_BuildRecordV2(damaged, ControlParams_Defaults(),
        &calibration, true, 13U, 9U, true);
    assert(ParameterStore_DecodeRecord(damaged, sizeof(damaged), &decoded));
    assert(!decoded.calibration_valid);
}

static void fill_valid_lever(LeverCalibrationValues_t *lever)
{
    uint32_t i;
    memset(lever, 0, sizeof(*lever));
    lever->data_version = LEVER_CALIBRATION_DATA_VERSION;
    lever->driver_config_fingerprint = 0x12345678UL;
    lever->reference_encoder = 65000U;
    lever->horizontal_offset_counts = 500;
    for (i = 0U; i < LEVER_CALIBRATION_POINT_COUNT; i++) {
        lever->angle_mdeg[i] = -8000 + (int32_t)i * 2000;
        lever->increasing_counts[i] = -800 + (int32_t)i * 200;
        lever->decreasing_counts[i] = -790 + (int32_t)i * 200;
    }
    lever->soft_min_angle_mdeg = -7200;
    lever->soft_max_angle_mdeg = 7200;
    lever->soft_min_counts = -640;
    lever->soft_max_counts = 640;
    lever->repeatability_mdeg = 180U;
    lever->backlash_mdeg = 250U;
    lever->reference_spread_counts = 20U;
    lever->point_spread_counts = 12U;
}

static void test_v3_lever_round_trip_and_old_records_stay_unarmed(void)
{
    uint8_t record[PARAMETER_STORE_RECORD_SIZE];
    uint8_t damaged[PARAMETER_STORE_RECORD_SIZE];
    LineCalibrationValues_t line = {
        {3000U, 3010U, 3020U, 3030U, 3040U, 3050U, 3060U, 3070U},
        { 500U,  510U,  520U,  530U,  540U,  550U,  560U,  570U}
    };
    LeverCalibrationValues_t lever;
    ParameterStoreDecoded_t decoded;

    fill_valid_lever(&lever);
    ParameterStore_BuildRecordV3(record, ControlParams_Defaults(), &line,
        true, &lever, true, 14U, 10U, true);
    assert(ParameterStore_DecodeRecord(record, sizeof(record), &decoded));
    assert(decoded.calibration_valid && decoded.lever_calibration_valid);
    assert(memcmp(&decoded.lever_calibration, &lever, sizeof(lever)) == 0);
    assert(!ParameterStore_DecodeRecord(record,
        PARAMETER_STORE_RECORD_SIZE - 1U, &decoded));

    memcpy(damaged, record, sizeof(damaged));
    damaged[240] ^= 1U;
    assert(!ParameterStore_DecodeRecord(damaged, sizeof(damaged), &decoded));

    ParameterStore_BuildRecordV2(record, ControlParams_Defaults(), &line,
        true, 15U, 10U, true);
    assert(ParameterStore_DecodeRecord(record,
        PARAMETER_STORE_RECORD_V2_SIZE, &decoded));
    assert(decoded.calibration_valid && !decoded.lever_calibration_valid);
}

static void test_invalid_payload_is_rejected_even_with_valid_crc(void)
{
    uint8_t record[PARAMETER_STORE_RECORD_SIZE];
    ControlParams_t params = *ControlParams_Defaults();
    ParameterStoreDecoded_t decoded;

    params.line.speed_min_mm_s = 300.0f;
    params.line.speed_max_mm_s = 100.0f;
    ParameterStore_BuildRecord(record, &params, 1U, 1U, true);
    assert(!ParameterStore_DecodeRecord(record, sizeof(record), &decoded));

    params = *ControlParams_Defaults();
    params.wheel.left_kp = NAN;
    ParameterStore_BuildRecord(record, &params, 1U, 1U, true);
    assert(!ParameterStore_DecodeRecord(record, sizeof(record), &decoded));
}

static void test_generation_selection(void)
{
    ParameterStoreDecoded_t a;
    ParameterStoreDecoded_t b;

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.valid = true;
    a.generation = 4U;
    a.parameter_revision = 1U;
    b.valid = true;
    b.generation = 5U;
    b.parameter_revision = 1U;

    assert(ParameterStore_Select(&a, &b) == PARAMETER_STORE_SLOT_B);
    a.generation = b.generation;
    assert(ParameterStore_Select(&a, &b) == PARAMETER_STORE_SLOT_A);
    a.valid = false;
    assert(ParameterStore_Select(&a, &b) == PARAMETER_STORE_SLOT_B);
    b.valid = false;
    assert(ParameterStore_Select(&a, &b) == PARAMETER_STORE_SLOT_NONE);

    a.valid = true;
    b.valid = true;
    a.generation = 1U;
    b.generation = UINT32_MAX;
    assert(ParameterStore_Select(&a, &b) == PARAMETER_STORE_SLOT_A);
}

int main(void)
{
    ControlParams_Init();
    assert(PARAMETER_STORE_RECORD_V1_SIZE == 168U);
    assert(PARAMETER_STORE_RECORD_V2_SIZE == 208U);
    assert(PARAMETER_STORE_RECORD_SIZE == 368U);
    test_crc();
    test_record_validation();
    test_invalid_payload_is_rejected_even_with_valid_crc();
    test_v2_calibration_round_trip();
    test_v3_lever_round_trip_and_old_records_stay_unarmed();
    test_generation_selection();
    return 0;
}
