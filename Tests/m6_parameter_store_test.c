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
    assert(!ParameterStore_DecodeRecord(record, sizeof(record) - 1U,
                                        &decoded));
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
    assert(PARAMETER_STORE_RECORD_SIZE == 168U);
    test_crc();
    test_record_validation();
    test_invalid_payload_is_rejected_even_with_valid_crc();
    test_generation_selection();
    return 0;
}
