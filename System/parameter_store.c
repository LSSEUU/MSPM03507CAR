#include "parameter_store.h"

#include <stddef.h>
#include <string.h>

#ifndef PARAMETER_STORE_HOST_TEST
#include "line_follow.h"
#endif

#ifndef PARAMETER_STORE_HOST_TEST
#include "ti_msp_dl_config.h"
#include "reset_diag.h"

#define PARAMETER_STORE_MAIN_SECTOR(address) \
    ((uint32_t)(address) / (uint32_t)PARAMETER_STORE_SLOT_SIZE)
#define PARAMETER_STORE_WEPROTB_BIT(address) \
    (1UL << ((PARAMETER_STORE_MAIN_SECTOR(address) - 32U) / 8U))
#define PARAMETER_STORE_FLASH_POLL_LIMIT 16000000UL

_Static_assert(PARAMETER_STORE_MAIN_SECTOR(PARAMETER_STORE_SLOT_A_ADDRESS) >= 32U,
               "parameter sectors must use CMDWEPROTB");
_Static_assert(PARAMETER_STORE_WEPROTB_BIT(PARAMETER_STORE_SLOT_A_ADDRESS) ==
               PARAMETER_STORE_WEPROTB_BIT(PARAMETER_STORE_SLOT_B_ADDRESS),
               "parameter sectors must share one protection group");
_Static_assert(PARAMETER_STORE_WEPROTB_BIT(PARAMETER_STORE_SLOT_A_ADDRESS) ==
               (1UL << 11U),
               "parameter slot A must use CMDWEPROTB bit 11");
_Static_assert(PARAMETER_STORE_WEPROTB_BIT(PARAMETER_STORE_SLOT_B_ADDRESS) ==
               (1UL << 11U),
               "parameter slot B must use CMDWEPROTB bit 11");
#endif

typedef struct {
    float wheel_left_kp;
    float wheel_left_ki;
    float wheel_right_kp;
    float wheel_right_ki;
    float wheel_filter_alpha;
    float wheel_accel;
    float wheel_decel;
    float wheel_jerk;
    float wheel_left_ks;
    float wheel_left_kv;
    float wheel_right_ks;
    float wheel_right_kv;
    float wheel_diag_target;
    float heading_straight_kp;
    float heading_straight_kd;
    float heading_turn_kp;
    float heading_turn_kd;
    float heading_rate_alpha;
    float heading_correction_limit;
    float heading_turn_speed;
    float heading_smooth_zone;
    float line_speed_min;
    float line_speed_max;
    float line_position_alpha;
    float line_derivative_alpha;
    float line_kp;
    float line_kd;
    float line_correction_limit;
    float line_correction_slew;
    float line_curve_gain;
    float line_lost_search_speed;
    float remote_speed_low;
    float remote_speed_medium;
    float remote_speed_high;
    uint32_t remote_heading_hold;
} ParameterPayloadV1_t;

typedef struct {
    ParameterPayloadV1_t controls;
    uint16_t grayscale_white[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t grayscale_black[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint32_t grayscale_valid;
    uint32_t reserved;
} ParameterPayloadV2_t;

typedef struct {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t payload_length;
    uint32_t generation;
    uint32_t parameter_revision;
    ParameterPayloadV1_t payload;
    uint32_t crc32;
    uint32_t commit_marker;
    uint32_t commit_reserved;
} ParameterRecordV1_t;

typedef struct {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t payload_length;
    uint32_t generation;
    uint32_t parameter_revision;
    ParameterPayloadV2_t payload;
    uint32_t crc32;
    uint32_t commit_marker;
    uint32_t commit_reserved;
} ParameterRecordV2_t;

#define PARAMETER_STORE_SCHEMA_V1 1U
#define PARAMETER_STORE_SCHEMA_V2 2U

_Static_assert(sizeof(ParameterPayloadV1_t) == 140U,
               "parameter payload V1 layout changed");
_Static_assert(sizeof(ParameterPayloadV2_t) == 180U,
               "parameter payload V2 layout changed");
_Static_assert(offsetof(ParameterRecordV1_t, crc32) == 156U,
               "parameter CRC coverage changed");
_Static_assert(offsetof(ParameterRecordV1_t, commit_marker) == 160U,
               "commit marker must begin the final flash word");
_Static_assert(sizeof(ParameterRecordV1_t) == PARAMETER_STORE_RECORD_V1_SIZE,
               "parameter record V1 layout changed");
_Static_assert((sizeof(ParameterRecordV1_t) % 8U) == 0U,
               "record must use complete 64-bit flash words");
_Static_assert(offsetof(ParameterRecordV2_t, crc32) == 196U,
               "parameter V2 CRC coverage changed");
_Static_assert(offsetof(ParameterRecordV2_t, commit_marker) == 200U,
               "parameter V2 commit offset changed");
_Static_assert(sizeof(ParameterRecordV2_t) == PARAMETER_STORE_RECORD_SIZE,
               "parameter record V2 layout changed");
_Static_assert((sizeof(ParameterRecordV2_t) % 8U) == 0U,
               "V2 record must use complete 64-bit flash words");

static ParameterStoreInfo_t s_info;
static LineCalibrationValues_t s_loaded_calibration;
static bool s_loaded_calibration_valid;

static bool calibration_values_valid(const LineCalibrationValues_t *values)
{
    uint32_t i;
    if (values == NULL) return false;
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        if ((values->white_adc[i] <= values->black_adc[i]) ||
            ((uint16_t)(values->white_adc[i] - values->black_adc[i]) < 512U))
            return false;
    }
    return true;
}

static void encode_payload(ParameterPayloadV1_t *payload,
                           const ControlParams_t *params)
{
    payload->wheel_left_kp = params->wheel.left_kp;
    payload->wheel_left_ki = params->wheel.left_ki;
    payload->wheel_right_kp = params->wheel.right_kp;
    payload->wheel_right_ki = params->wheel.right_ki;
    payload->wheel_filter_alpha = params->wheel.filter_alpha;
    payload->wheel_accel = params->wheel.accel_mm_s2;
    payload->wheel_decel = params->wheel.decel_mm_s2;
    payload->wheel_jerk = params->wheel.jerk_mm_s3;
    payload->wheel_left_ks = params->wheel.left_ks;
    payload->wheel_left_kv = params->wheel.left_kv;
    payload->wheel_right_ks = params->wheel.right_ks;
    payload->wheel_right_kv = params->wheel.right_kv;
    payload->wheel_diag_target = params->wheel.diag_target_mm_s;
    payload->heading_straight_kp = params->heading.straight_kp;
    payload->heading_straight_kd = params->heading.straight_kd;
    payload->heading_turn_kp = params->heading.turn_kp;
    payload->heading_turn_kd = params->heading.turn_kd;
    payload->heading_rate_alpha = params->heading.rate_alpha;
    payload->heading_correction_limit = params->heading.correction_limit_mm_s;
    payload->heading_turn_speed = params->heading.turn_speed_mm_s;
    payload->heading_smooth_zone = params->heading.smooth_zone_deg;
    payload->line_speed_min = params->line.speed_min_mm_s;
    payload->line_speed_max = params->line.speed_max_mm_s;
    payload->line_position_alpha = params->line.position_alpha;
    payload->line_derivative_alpha = params->line.derivative_alpha;
    payload->line_kp = params->line.kp;
    payload->line_kd = params->line.kd;
    payload->line_correction_limit = params->line.correction_limit_mm_s;
    payload->line_correction_slew = params->line.correction_slew_mm_s2;
    payload->line_curve_gain = params->line.curve_gain;
    payload->line_lost_search_speed = params->line.lost_search_speed_mm_s;
    payload->remote_speed_low = params->remote.speed_low_mm_s;
    payload->remote_speed_medium = params->remote.speed_medium_mm_s;
    payload->remote_speed_high = params->remote.speed_high_mm_s;
    payload->remote_heading_hold = (params->remote.heading_hold != 0U) ? 1U : 0U;
}

static void decode_payload(ControlParams_t *params,
                           const ParameterPayloadV1_t *payload)
{
    memset(params, 0, sizeof(*params));
    params->wheel.left_kp = payload->wheel_left_kp;
    params->wheel.left_ki = payload->wheel_left_ki;
    params->wheel.right_kp = payload->wheel_right_kp;
    params->wheel.right_ki = payload->wheel_right_ki;
    params->wheel.filter_alpha = payload->wheel_filter_alpha;
    params->wheel.accel_mm_s2 = payload->wheel_accel;
    params->wheel.decel_mm_s2 = payload->wheel_decel;
    params->wheel.jerk_mm_s3 = payload->wheel_jerk;
    params->wheel.left_ks = payload->wheel_left_ks;
    params->wheel.left_kv = payload->wheel_left_kv;
    params->wheel.right_ks = payload->wheel_right_ks;
    params->wheel.right_kv = payload->wheel_right_kv;
    params->wheel.diag_target_mm_s = payload->wheel_diag_target;
    params->heading.straight_kp = payload->heading_straight_kp;
    params->heading.straight_kd = payload->heading_straight_kd;
    params->heading.turn_kp = payload->heading_turn_kp;
    params->heading.turn_kd = payload->heading_turn_kd;
    params->heading.rate_alpha = payload->heading_rate_alpha;
    params->heading.correction_limit_mm_s = payload->heading_correction_limit;
    params->heading.turn_speed_mm_s = payload->heading_turn_speed;
    params->heading.smooth_zone_deg = payload->heading_smooth_zone;
    params->line.speed_min_mm_s = payload->line_speed_min;
    params->line.speed_max_mm_s = payload->line_speed_max;
    params->line.position_alpha = payload->line_position_alpha;
    params->line.derivative_alpha = payload->line_derivative_alpha;
    params->line.kp = payload->line_kp;
    params->line.kd = payload->line_kd;
    params->line.correction_limit_mm_s = payload->line_correction_limit;
    params->line.correction_slew_mm_s2 = payload->line_correction_slew;
    params->line.curve_gain = payload->line_curve_gain;
    params->line.lost_search_speed_mm_s = payload->line_lost_search_speed;
    params->remote.speed_low_mm_s = payload->remote_speed_low;
    params->remote.speed_medium_mm_s = payload->remote_speed_medium;
    params->remote.speed_high_mm_s = payload->remote_speed_high;
    params->remote.heading_hold = payload->remote_heading_hold;
}

uint32_t ParameterStore_Crc32(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    size_t index;
    uint8_t bit;

    if ((data == NULL) && (length != 0U))
        return 0U;
    for (index = 0U; index < length; index++) {
        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return crc ^ 0xFFFFFFFFUL;
}

void ParameterStore_BuildRecord(void *destination,
                                const ControlParams_t *params,
                                uint32_t generation,
                                uint32_t parameter_revision,
                                bool committed)
{
    ParameterRecordV1_t *record = (ParameterRecordV1_t *)destination;

    if ((record == NULL) || (params == NULL))
        return;
    memset(record, 0xFF, sizeof(*record));
    record->magic = PARAMETER_STORE_MAGIC;
    record->schema_version = PARAMETER_STORE_SCHEMA_V1;
    record->payload_length = (uint16_t)sizeof(record->payload);
    record->generation = generation;
    record->parameter_revision = parameter_revision;
    encode_payload(&record->payload, params);
    record->crc32 = ParameterStore_Crc32(record,
        offsetof(ParameterRecordV1_t, crc32));
    record->commit_marker = committed ? PARAMETER_STORE_COMMIT_MARKER :
                                        0xFFFFFFFFUL;
    record->commit_reserved = 0xFFFFFFFFUL;
}

void ParameterStore_BuildRecordV2(void *destination,
                                  const ControlParams_t *params,
                                  const LineCalibrationValues_t *calibration,
                                  bool calibration_valid,
                                  uint32_t generation,
                                  uint32_t parameter_revision,
                                  bool committed)
{
    ParameterRecordV2_t *record = (ParameterRecordV2_t *)destination;
    uint32_t i;
    if ((record == NULL) || (params == NULL)) return;
    if (calibration_valid && !calibration_values_valid(calibration))
        calibration_valid = false;
    memset(record, 0xFF, sizeof(*record));
    record->magic = PARAMETER_STORE_MAGIC;
    record->schema_version = PARAMETER_STORE_SCHEMA_V2;
    record->payload_length = (uint16_t)sizeof(record->payload);
    record->generation = generation;
    record->parameter_revision = parameter_revision;
    encode_payload(&record->payload.controls, params);
    for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
        record->payload.grayscale_white[i] = calibration_valid ?
            calibration->white_adc[i] : 0U;
        record->payload.grayscale_black[i] = calibration_valid ?
            calibration->black_adc[i] : 0U;
    }
    record->payload.grayscale_valid = calibration_valid ? 1U : 0U;
    record->payload.reserved = 0xFFFFFFFFUL;
    record->crc32 = ParameterStore_Crc32(record,
        offsetof(ParameterRecordV2_t, crc32));
    record->commit_marker = committed ? PARAMETER_STORE_COMMIT_MARKER :
                                        0xFFFFFFFFUL;
    record->commit_reserved = 0xFFFFFFFFUL;
}

bool ParameterStore_DecodeRecord(const void *record_data,
                                 size_t available,
                                 ParameterStoreDecoded_t *decoded)
{
    const ParameterRecordV1_t *record =
        (const ParameterRecordV1_t *)record_data;
    ControlParams_t candidate;

    if (decoded != NULL)
        memset(decoded, 0, sizeof(*decoded));
    if ((record == NULL) || (decoded == NULL) ||
        (available < sizeof(*record))) return false;
    if ((record->magic != PARAMETER_STORE_MAGIC) ||
        (record->schema_version != PARAMETER_STORE_SCHEMA_V1 &&
         record->schema_version != PARAMETER_STORE_SCHEMA_V2)) return false;
    if (record->schema_version == PARAMETER_STORE_SCHEMA_V1) {
        if ((record->payload_length != sizeof(record->payload)) ||
            (record->commit_marker != PARAMETER_STORE_COMMIT_MARKER) ||
            (record->commit_reserved != 0xFFFFFFFFUL) ||
            (record->crc32 != ParameterStore_Crc32(record,
                offsetof(ParameterRecordV1_t, crc32)))) return false;
        decode_payload(&candidate, &record->payload);
    } else {
        const ParameterRecordV2_t *record_v2 =
            (const ParameterRecordV2_t *)record_data;
        uint32_t i;
        if ((available < sizeof(*record_v2)) ||
            (record_v2->payload_length != sizeof(record_v2->payload)) ||
            (record_v2->commit_marker != PARAMETER_STORE_COMMIT_MARKER) ||
            (record_v2->commit_reserved != 0xFFFFFFFFUL) ||
            (record_v2->payload.reserved != 0xFFFFFFFFUL) ||
            (record_v2->payload.grayscale_valid > 1U) ||
            (record_v2->crc32 != ParameterStore_Crc32(record_v2,
                offsetof(ParameterRecordV2_t, crc32)))) return false;
        decode_payload(&candidate, &record_v2->payload.controls);
        if (record_v2->payload.grayscale_valid != 0U) {
            for (i = 0U; i < GRAYSCALE_CALIBRATION_CHANNEL_COUNT; i++) {
                decoded->calibration.white_adc[i] =
                    record_v2->payload.grayscale_white[i];
                decoded->calibration.black_adc[i] =
                    record_v2->payload.grayscale_black[i];
            }
            if (!calibration_values_valid(&decoded->calibration)) return false;
            decoded->calibration_valid = true;
        }
    }
    if (ControlParams_Validate(&candidate) != CONTROL_PARAMS_OK)
        return false;
    decoded->valid = true;
    decoded->generation = record->generation;
    decoded->parameter_revision = record->parameter_revision;
    decoded->params = candidate;
    return true;
}

ParameterStoreSlot_t ParameterStore_Select(
    const ParameterStoreDecoded_t *slot_a,
    const ParameterStoreDecoded_t *slot_b)
{
    if ((slot_a != NULL) && slot_a->valid &&
        (slot_b != NULL) && slot_b->valid) {
        if (slot_a->generation == slot_b->generation)
            return PARAMETER_STORE_SLOT_A;
        return ((int32_t)(slot_a->generation - slot_b->generation) > 0) ?
            PARAMETER_STORE_SLOT_A : PARAMETER_STORE_SLOT_B;
    }
    if ((slot_a != NULL) && slot_a->valid)
        return PARAMETER_STORE_SLOT_A;
    if ((slot_b != NULL) && slot_b->valid)
        return PARAMETER_STORE_SLOT_B;
    return PARAMETER_STORE_SLOT_NONE;
}

#ifndef PARAMETER_STORE_HOST_TEST
static void read_slots(ParameterStoreDecoded_t *a, ParameterStoreDecoded_t *b)
{
    s_info.slot_a_valid = ParameterStore_DecodeRecord(
        (const void *)PARAMETER_STORE_SLOT_A_ADDRESS,
        PARAMETER_STORE_SLOT_SIZE, a);
    s_info.slot_b_valid = ParameterStore_DecodeRecord(
        (const void *)PARAMETER_STORE_SLOT_B_ADDRESS,
        PARAMETER_STORE_SLOT_SIZE, b);
}

/*
 * MSPM0 reapplies maximum write/erase protection after every Flash command.
 * On a single-bank MSPM0G3507, CMDWEPROTB bit 0 starts at main sector 32;
 * sectors 126 and 127 therefore share group bit 11. Clear-status reapplies
 * maximum protection, so that group must be unprotected again before each
 * 64-bit program command. Keep these direct register operations in the
 * RAM-only path; a DriverLib call would introduce
 * a Flash-resident dependency between consecutive program operations.
 */
__attribute__((always_inline)) static inline void
clear_flash_status_from_ram(void)
{
    FLASHCTL->GEN.CMDTYPE = DL_FLASHCTL_COMMAND_TYPE_CLEAR_STATUS;
    FLASHCTL->GEN.CMDEXEC = FLASHCTL_CMDEXEC_VAL_EXECUTE;
    while ((FLASHCTL->GEN.STATCMD & FLASHCTL_STATCMD_CMDINPROGRESS_MASK) ==
           FLASHCTL_STATCMD_CMDINPROGRESS_STATINPROGRESS) {
        WWDT0->WWDTCNTRST = 0x00A7U;
    }
}

__attribute__((always_inline)) static inline void
unprotect_parameter_group_from_ram(void)
{
    FLASHCTL->GEN.CMDWEPROTB &=
        ~PARAMETER_STORE_WEPROTB_BIT(PARAMETER_STORE_SLOT_A_ADDRESS);
}

__attribute__((section(".ramfunc"), noinline))
static bool write_sector_from_ram(uint32_t address,
                                  const ParameterRecordV2_t *record)
{
    const uint32_t *words = (const uint32_t *)record;
    uint32_t word_index;
    uint32_t status;
    uint32_t polls;

    FLASHCTL->GEN.CMDTYPE =
        DL_FLASHCTL_COMMAND_TYPE_ERASE | DL_FLASHCTL_COMMAND_SIZE_SECTOR;
    FLASHCTL->GEN.CMDADDR = address;
    FLASHCTL->GEN.CMDEXEC = FLASHCTL_CMDEXEC_VAL_EXECUTE;
    polls = PARAMETER_STORE_FLASH_POLL_LIMIT;
    do {
        WWDT0->WWDTCNTRST = 0x00A7U;
        status = FLASHCTL->GEN.STATCMD &
            (FLASHCTL_STATCMD_CMDDONE_MASK | FLASHCTL_STATCMD_CMDPASS_MASK |
             FLASHCTL_STATCMD_CMDINPROGRESS_MASK);
        if ((status == (uint32_t)DL_FLASHCTL_COMMAND_STATUS_PASSED) ||
            (status == (uint32_t)DL_FLASHCTL_COMMAND_STATUS_FAILED)) break;
        polls--;
    } while (polls != 0U);
    if ((polls == 0U) &&
        (status == (uint32_t)DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS)) {
        /* Stay in SRAM and deliberately stop feeding WWDT; a wedged Flash
         * controller must reset rather than return into unavailable Flash. */
        do {
            status = FLASHCTL->GEN.STATCMD &
                (FLASHCTL_STATCMD_CMDDONE_MASK |
                 FLASHCTL_STATCMD_CMDPASS_MASK |
                 FLASHCTL_STATCMD_CMDINPROGRESS_MASK);
        } while (status ==
                 (uint32_t)DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS);
    }
    if (status != (uint32_t)DL_FLASHCTL_COMMAND_STATUS_PASSED)
        return false;

    /* Program all covered bytes first; the final 64-bit commit word is last. */
    for (word_index = 0U;
         word_index < (sizeof(ParameterRecordV2_t) / 8U);
         word_index++) {
        uint32_t source_index = word_index * 2U;
        clear_flash_status_from_ram();
        unprotect_parameter_group_from_ram();
        FLASHCTL->GEN.CMDTYPE =
            DL_FLASHCTL_COMMAND_TYPE_PROGRAM |
            DL_FLASHCTL_COMMAND_SIZE_ONE_WORD;
        FLASHCTL->GEN.CMDADDR = address + (word_index * 8U);
        FLASHCTL->GEN.CMDBYTEN = DL_FLASHCTL_PROGRAM_64_WITH_ECC;
        FLASHCTL->GEN.CMDDATA0 = words[source_index];
        FLASHCTL->GEN.CMDDATA1 = words[source_index + 1U];
        FLASHCTL->GEN.CMDEXEC = FLASHCTL_CMDEXEC_VAL_EXECUTE;
        polls = PARAMETER_STORE_FLASH_POLL_LIMIT;
        do {
            WWDT0->WWDTCNTRST = 0x00A7U;
            status = FLASHCTL->GEN.STATCMD &
                (FLASHCTL_STATCMD_CMDDONE_MASK |
                 FLASHCTL_STATCMD_CMDPASS_MASK |
                 FLASHCTL_STATCMD_CMDINPROGRESS_MASK);
            if ((status == (uint32_t)DL_FLASHCTL_COMMAND_STATUS_PASSED) ||
                (status == (uint32_t)DL_FLASHCTL_COMMAND_STATUS_FAILED)) break;
            polls--;
        } while (polls != 0U);
        if ((polls == 0U) &&
            (status == (uint32_t)DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS)) {
            /* See erase timeout above: never execute from Flash while busy. */
            do {
                status = FLASHCTL->GEN.STATCMD &
                    (FLASHCTL_STATCMD_CMDDONE_MASK |
                     FLASHCTL_STATCMD_CMDPASS_MASK |
                     FLASHCTL_STATCMD_CMDINPROGRESS_MASK);
            } while (status ==
                     (uint32_t)DL_FLASHCTL_COMMAND_STATUS_IN_PROGRESS);
        }
        if (status != (uint32_t)DL_FLASHCTL_COMMAND_STATUS_PASSED)
            return false;
    }
    return true;
}
#endif

void ParameterStore_Init(void)
{
    memset(&s_info, 0, sizeof(s_info));
    memset(&s_loaded_calibration, 0, sizeof(s_loaded_calibration));
    s_loaded_calibration_valid = false;
#ifdef PARAMETER_STORE_HOST_TEST
    s_info.health = PARAMETER_STORE_HEALTH_DEGRADED;
    return;
#else
    ParameterStoreDecoded_t a;
    ParameterStoreDecoded_t b;
    ParameterStoreSlot_t selected;

    read_slots(&a, &b);
    selected = ParameterStore_Select(&a, &b);
    s_info.selected_slot = selected;
    s_info.health = (selected == PARAMETER_STORE_SLOT_NONE) ?
        PARAMETER_STORE_HEALTH_DEGRADED : PARAMETER_STORE_HEALTH_OK;
    if (selected == PARAMETER_STORE_SLOT_A) {
        s_info.generation = a.generation;
        if (a.calibration_valid) {
            s_loaded_calibration = a.calibration;
            s_loaded_calibration_valid = true;
        }
        (void)ControlParams_BootRestore(&a.params, a.parameter_revision,
                                        CONTROL_PARAMS_SOURCE_PERSISTED);
    } else if (selected == PARAMETER_STORE_SLOT_B) {
        s_info.generation = b.generation;
        if (b.calibration_valid) {
            s_loaded_calibration = b.calibration;
            s_loaded_calibration_valid = true;
        }
        (void)ControlParams_BootRestore(&b.params, b.parameter_revision,
                                        CONTROL_PARAMS_SOURCE_PERSISTED);
    }
#endif
}

ParameterStoreResult_t ParameterStore_LoadStaged(void)
{
#ifdef PARAMETER_STORE_HOST_TEST
    return PARAMETER_STORE_ERR_IO;
#else
    ParameterStoreDecoded_t a;
    ParameterStoreDecoded_t b;
    ParameterStoreSlot_t selected;
    const ControlParams_t *params;
    ControlParamsSource_t source;

    read_slots(&a, &b);
    selected = ParameterStore_Select(&a, &b);
    if (selected == PARAMETER_STORE_SLOT_A)
        params = &a.params;
    else if (selected == PARAMETER_STORE_SLOT_B)
        params = &b.params;
    else
        params = ControlParams_Defaults();
    source = (selected == PARAMETER_STORE_SLOT_NONE) ?
        CONTROL_PARAMS_SOURCE_DEFAULT : CONTROL_PARAMS_SOURCE_PERSISTED;
    s_info.selected_slot = selected;
    s_info.health = (selected == PARAMETER_STORE_SLOT_NONE) ?
        PARAMETER_STORE_HEALTH_DEGRADED : PARAMETER_STORE_HEALTH_OK;
    s_info.generation = (selected == PARAMETER_STORE_SLOT_A) ? a.generation :
        ((selected == PARAMETER_STORE_SLOT_B) ? b.generation : 0U);
    return (ControlParams_StageExternal(params, source) == CONTROL_PARAMS_OK) ?
        PARAMETER_STORE_OK : PARAMETER_STORE_ERR_PARAMS;
#endif
}

static ParameterStoreResult_t save_active_with_calibration(
    const LineCalibrationValues_t *calibration, bool calibration_valid,
    uint32_t expected_generation, bool enforce_generation)
{
#ifdef PARAMETER_STORE_HOST_TEST
    (void)calibration;
    (void)calibration_valid;
    (void)expected_generation;
    (void)enforce_generation;
    return PARAMETER_STORE_ERR_IO;
#else
    ParameterStoreDecoded_t a;
    ParameterStoreDecoded_t b;
    ParameterStoreDecoded_t verification;
    ParameterRecordV2_t record;
    ParameterStoreSlot_t selected;
    ParameterStoreSlot_t target;
    uint32_t target_address;
    uint32_t next_generation;
    uint32_t current_generation;
    uint32_t primask;
    bool write_ok;

    if (ControlParams_Validate(ControlParams_Active()) != CONTROL_PARAMS_OK)
        return PARAMETER_STORE_ERR_PARAMS;
    if (calibration_valid && !calibration_values_valid(calibration))
        return PARAMETER_STORE_ERR_PARAMS;
    read_slots(&a, &b);
    selected = ParameterStore_Select(&a, &b);
    current_generation = (selected == PARAMETER_STORE_SLOT_A) ? a.generation :
        ((selected == PARAMETER_STORE_SLOT_B) ? b.generation : 0U);
    if (enforce_generation && (current_generation != expected_generation))
        return PARAMETER_STORE_ERR_STALE_GENERATION;
    target = (selected == PARAMETER_STORE_SLOT_A) ?
        PARAMETER_STORE_SLOT_B : PARAMETER_STORE_SLOT_A;
    target_address = (target == PARAMETER_STORE_SLOT_A) ?
        PARAMETER_STORE_SLOT_A_ADDRESS : PARAMETER_STORE_SLOT_B_ADDRESS;
    next_generation = (selected == PARAMETER_STORE_SLOT_A) ?
        (a.generation + 1U) : ((selected == PARAMETER_STORE_SLOT_B) ?
        (b.generation + 1U) : 1U);
    ParameterStore_BuildRecordV2(&record, ControlParams_Active(), calibration,
        calibration_valid, next_generation, ControlParams_Revision(), true);

    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_PARAMETER_FLASH);
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, target_address,
                                DL_FLASHCTL_REGION_SELECT_MAIN);
    primask = __get_PRIMASK();
    __disable_irq();
    write_ok = write_sector_from_ram(target_address, &record);
    DL_FlashCTL_protectSector(FLASHCTL, PARAMETER_STORE_SLOT_A_ADDRESS,
                              DL_FLASHCTL_REGION_SELECT_MAIN);
    DL_FlashCTL_protectSector(FLASHCTL, PARAMETER_STORE_SLOT_B_ADDRESS,
                              DL_FLASHCTL_REGION_SELECT_MAIN);
    if (primask == 0U)
        __enable_irq();
    if (!write_ok) {
        s_info.save_failures++;
        return PARAMETER_STORE_ERR_IO;
    }
    if (!ParameterStore_DecodeRecord((const void *)target_address,
                                     PARAMETER_STORE_SLOT_SIZE,
                                     &verification) ||
        (verification.generation != next_generation) ||
        (verification.parameter_revision != ControlParams_Revision()) ||
        (verification.calibration_valid != calibration_valid) ||
        (calibration_valid &&
         memcmp(&verification.calibration, calibration,
                sizeof(*calibration)) != 0)) {
        s_info.save_failures++;
        return PARAMETER_STORE_ERR_VERIFY;
    }
    s_info.selected_slot = target;
    s_info.health = PARAMETER_STORE_HEALTH_OK;
    s_info.generation = next_generation;
    s_info.slot_a_valid = (target == PARAMETER_STORE_SLOT_A) ? true : a.valid;
    s_info.slot_b_valid = (target == PARAMETER_STORE_SLOT_B) ? true : b.valid;
    if (calibration_valid) {
        s_loaded_calibration = *calibration;
        s_loaded_calibration_valid = true;
    } else {
        memset(&s_loaded_calibration, 0, sizeof(s_loaded_calibration));
        s_loaded_calibration_valid = false;
    }
    return PARAMETER_STORE_OK;
#endif
}

ParameterStoreResult_t ParameterStore_SaveActive(void)
{
#ifdef PARAMETER_STORE_HOST_TEST
    return PARAMETER_STORE_ERR_IO;
#else
    LineCalibrationValues_t calibration;
    LineFollowSnapshot_t line;
    LineFollow_GetSnapshot(&line);
    LineFollow_GetCalibrationValues(&calibration, NULL);
    return save_active_with_calibration(&calibration,
                                        line.calibration.valid, 0U, false);
#endif
}

ParameterStoreResult_t ParameterStore_SaveCalibration(
    const LineCalibrationValues_t *calibration,
    uint32_t expected_generation)
{
    return save_active_with_calibration(calibration, true,
                                        expected_generation, true);
}

bool ParameterStore_GetLoadedCalibration(LineCalibrationValues_t *calibration)
{
    if ((calibration == NULL) || !s_loaded_calibration_valid) return false;
    *calibration = s_loaded_calibration;
    return true;
}

void ParameterStore_GetInfo(ParameterStoreInfo_t *info)
{
    if (info != NULL)
        *info = s_info;
}

const char *ParameterStore_SourceName(void)
{
    if (s_info.selected_slot == PARAMETER_STORE_SLOT_A)
        return "flash-a";
    if (s_info.selected_slot == PARAMETER_STORE_SLOT_B)
        return "flash-b";
    return "defaults";
}
