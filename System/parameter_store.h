#ifndef PARAMETER_STORE_H
#define PARAMETER_STORE_H

#include "control_params.h"
#include "grayscale_calibration_types.h"
#include "lever_calibration_types.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PARAMETER_STORE_SLOT_SIZE        1024U
#define PARAMETER_STORE_SLOT_A_ADDRESS   0x0001F800UL
#define PARAMETER_STORE_SLOT_B_ADDRESS   0x0001FC00UL
#define PARAMETER_STORE_RECORD_V1_SIZE   168U
#define PARAMETER_STORE_RECORD_V2_SIZE   208U
#define PARAMETER_STORE_RECORD_SIZE      368U
#define PARAMETER_STORE_MAGIC            0x4D365052UL
#define PARAMETER_STORE_COMMIT_MARKER    0x434F4D54UL

typedef enum {
    PARAMETER_STORE_SLOT_NONE = 0,
    PARAMETER_STORE_SLOT_A,
    PARAMETER_STORE_SLOT_B
} ParameterStoreSlot_t;

typedef enum {
    PARAMETER_STORE_HEALTH_OK = 0,
    PARAMETER_STORE_HEALTH_DEGRADED
} ParameterStoreHealth_t;

typedef enum {
    PARAMETER_STORE_OK = 0,
    PARAMETER_STORE_ERR_ARGUMENT,
    PARAMETER_STORE_ERR_PARAMS,
    PARAMETER_STORE_ERR_STALE_GENERATION,
    PARAMETER_STORE_ERR_IO,
    PARAMETER_STORE_ERR_VERIFY
} ParameterStoreResult_t;

typedef struct {
    bool valid;
    uint32_t generation;
    uint32_t parameter_revision;
    ControlParams_t params;
    LineCalibrationValues_t calibration;
    bool calibration_valid;
    LeverCalibrationValues_t lever_calibration;
    bool lever_calibration_valid;
} ParameterStoreDecoded_t;

typedef struct {
    ParameterStoreSlot_t selected_slot;
    ParameterStoreHealth_t health;
    uint32_t generation;
    bool slot_a_valid;
    bool slot_b_valid;
    uint32_t save_failures;
} ParameterStoreInfo_t;

uint32_t ParameterStore_Crc32(const void *data, size_t length);
void ParameterStore_BuildRecord(void *destination,
                                const ControlParams_t *params,
                                uint32_t generation,
                                uint32_t parameter_revision,
                                bool committed);
void ParameterStore_BuildRecordV2(void *destination,
                                  const ControlParams_t *params,
                                  const LineCalibrationValues_t *calibration,
                                  bool calibration_valid,
                                  uint32_t generation,
                                  uint32_t parameter_revision,
                                  bool committed);
void ParameterStore_BuildRecordV3(void *destination,
                                  const ControlParams_t *params,
                                  const LineCalibrationValues_t *calibration,
                                  bool calibration_valid,
                                  const LeverCalibrationValues_t *lever,
                                  bool lever_valid,
                                  uint32_t generation,
                                  uint32_t parameter_revision,
                                  bool committed);
bool ParameterStore_DecodeRecord(const void *record,
                                 size_t available,
                                 ParameterStoreDecoded_t *decoded);
ParameterStoreSlot_t ParameterStore_Select(
    const ParameterStoreDecoded_t *slot_a,
    const ParameterStoreDecoded_t *slot_b);

/* Call after ControlParams_Init and before scheduled control starts. */
void ParameterStore_Init(void);
ParameterStoreResult_t ParameterStore_LoadStaged(void);
ParameterStoreResult_t ParameterStore_SaveActive(void);
ParameterStoreResult_t ParameterStore_SaveCalibration(
    const LineCalibrationValues_t *calibration,
    uint32_t expected_generation);
ParameterStoreResult_t ParameterStore_SaveLeverCalibration(
    const LeverCalibrationValues_t *calibration,
    uint32_t expected_generation);
bool ParameterStore_GetLoadedCalibration(LineCalibrationValues_t *calibration);
bool ParameterStore_GetLoadedLeverCalibration(
    LeverCalibrationValues_t *calibration);
void ParameterStore_GetInfo(ParameterStoreInfo_t *info);
const char *ParameterStore_SourceName(void);

#endif
